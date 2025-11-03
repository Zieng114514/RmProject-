// 云台模块：负责 yaw/pitch 两轴的初始化与任务控制
#include "gimbal.h"
// 机器人参数宏：尺寸、安装偏置、角度单位转换等
#include "robot_def.h"
#include "robot_cmd.h"
// DJI 电机（GM6020/M3508/M2006）统一接口
#include "dji_motor.h"
// 达妙电机（DM4310/DM 系列）接口 - 增强版
#include "dmmotor.h"
// 姿态解算任务接口：提供 IMU 姿态（角度/角速度）数据
#include "ins_task.h"
// 发布订阅消息中心：云台与上层 cmd 模块通信
#include "message_center.h"
// 通用宏/类型
#include "general_def.h"
// BMI088 传感器接口（云台常用 IMU）
#include "bmi088.h"

/**
 * @brief DM电机使用说明
 *
 * 本模块使用增强版DM电机接口，支持以下功能：
 * 1. 统一的measure结构体数据访问
 * 2. 完善的电机状态监控
 * 3. 温度保护和错误处理
 * 4. 多种控制模式支持（MIT模式、位置控制等）
 * 5. 完整的位置、速度、电流三环PID控制
 * 6. 支持外部反馈源（如IMU数据）
 *
 * 主要API：
 * - DMMotorGetMeasure(): 获取电机测量数据结构指针
 * - DMMotorGetStatus(): 获取电机状态
 * - DMMotorSetPosition(): 设置目标位置
 * - DMMotorSetCurrent(): 设置目标电流
 * - DMMotorSetRef(): 设置PID参考值
 * - DMMotorEnable/Disable/Stop(): 电机控制
 * - DMMotorOuterLoop(): 设置外环控制类型
 *
 * PID控制说明：
 * - 支持位置环、速度环的串级控制（已去掉电流环）
 * - 支持电机自身反馈和外部反馈（如IMU）
 * - 支持正反转和反馈反向设置
 * - 自动进行输出限幅和死区处理
 */

// 云台 IMU 数据指针（Yaw / Pitch / Gyro[] 等）
// 注意：无人机云台使用单圈角度Yaw，不使用多圈YawTotalAngle
static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor;
static DMMotorInstance *pitch_motor;

/**
 * @brief 清空PID控制器状态（误差、积分等）
 * @param pid PID控制器实例指针
 */
static void ClearPIDState(PIDInstance *pid)
{
    if (pid == NULL)
        return;
    pid->Err = 0;
    pid->Last_Err = 0;
    pid->Iout = 0;
    pid->ITerm = 0;
    pid->Last_ITerm = 0;
    pid->Last_Measure = pid->Measure; // 保存当前测量值作为新的起始点
    pid->Last_Output = 0;
    pid->Last_Dout = 0;
    pid->Dout = 0;
    pid->Output = 0;
    pid->Pout = 0;
}

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

// 缓启动参数
#define GIMBAL_RAMP_RATE_YAW 0.03f    // Yaw轴缓启动速度(rad/任务周期)，约10 rad/s = 1.6圈/s
#define GIMBAL_RAMP_RATE_PITCH 0.04f  // Pitch轴缓启动速度(rad/任务周期)，约6 rad/s
static gimbal_mode_e last_gimbal_mode = GIMBAL_ZERO_FORCE; // 上一次的云台模式
static float ramp_yaw_ref = 0;        // 缓启动时的yaw目标值
static float ramp_pitch_ref = 0;      // 缓启动时的pitch目标值
static uint8_t ramp_active = 0;      // 缓启动激活标志

//static BMI088Instance *bmi088; // 云台IMU
void GimbalInit()
{
    // 1) 初始化 IMU 姿态解算，获取姿态数据指针（Yaw/Pitch/Gyro[]）
    gimba_IMU_data = INS_Init();
	// 初始化Pitch展开角度
    // 2) YAW 轴（GM6020）：四环串级控制（IMU角度-IMU角速度-电机位置-电机速度）
    //    IMU反馈：YawTotalAngle作为角度反馈，Gyro[2]作为角速度反馈
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            // IMU角度环PID配置（四环串级控制的最外环）
            .imu_angle_PID = {
                .Kp = 1.5,
                .Ki = 2,
                .Kd = 0,
                .MaxOut = 500,
                .DeadBand = 0.0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
            },
            // IMU角速度环PID配置（第二环）
            .imu_speed_PID = {
                .Kp = 2,  // IMU角速度环PID，可根据需要调整
                .Ki = 1,
                .Kd = 0,
                .MaxOut = 300,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
            },
            // 电机位置环PID（第三环）
            .angle_PID = {
                .Kp = 35, // 8
                .Ki = 20,
                .Kd = 0.00,
                .DeadBand = 0.0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 50,
                .MaxOut = 450,
            },
            // 电机速度环PID（最内环）
            .speed_PID = {
                .Kp = 35,  // 50
                .Ki = 50, // 200
                .Kd = 0.00,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 1000,
                .MaxOut = 7000,
            },
            // IMU反馈指针
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,  // IMU角度反馈
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],        // IMU角速度反馈(Z轴，yaw轴)
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,       // 修改为反转模式，修正拨杆左移时电机右转的问题
            .imu_reverse_flag = MOTOR_DIRECTION_REVERSE,          // IMU反馈方向标志（NORMAL=正向, REVERSE=取反）
        },
        .motor_type = GM6020};
    // 3) PITCH 轴（DM4310）：使用 MIT 固件，MIT 模式控制
    //    CAN ID = 0x002，MIT 模式支持位置、速度、电流控制
    Motor_Init_Config_s dm_pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x002,  // DM电机使用不同的CAN ID
            .rx_id = 0x001,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.405,  // 角度环增益，根据实际调试调整
                .Ki = 0.02,   // 积分项暂时关闭，避免漂移
                .Kd = 0.0,  // 微分项
                .DeadBand = 0.00,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 5,
                .MaxOut = 10.25,
            },
            .speed_PID = {
                .Kp = 0.05,  // 速度环增益
                .Ki = 0.1,   // 积分项暂时关闭，避免漂移
                .Kd = 0.0,  // 微分项
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2,
                .MaxOut = 0.1,  // 限制最大输出（Nm）
            },
			.other_angle_feedback_ptr = &gimba_IMU_data->Roll,  // 使用展开后的pitch角度反馈，避免±180°跳变
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[1], // 使用pitch轴角速度反馈(Y轴)
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,  // 使用IMU反馈
            .speed_feedback_source = OTHER_FEED,   // 使用IMU反馈
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,  // 位置+速度双环控制
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = MOTOR_TYPE_NONE};  // DM电机类型，使用通用类型

    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DMMotorInit(&dm_pitch_config);

    // DM4310 增强版使用任务式控制循环，需启动其控制任务
    DMMotorControlInit();


    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/**
 * @brief 无人机云台遥控器控制说明
 *
 * ## 项目类型
 * 无人机二轴云台（Yaw + Pitch），不包含底盘控制
 *
 * ## 控制数据流向
 * 遥控器 → robot_cmd 模块 → gimbal_cmd 消息 → GimbalTask() → 电机控制
 *
 * ## 遥控器通道映射
 * - 左摇杆 X 轴（rocker_l_）：yaw 轴控制（左右转动）
 * - 左摇杆 Y 轴（rocker_l1）：pitch 轴控制（上下俯仰）
 * - 右侧三段开关：云台工作模式切换
 *   * [上] → GIMBAL_ZERO_FORCE（急停）
 *   * [中] → GIMBAL_GYRO_MODE（正常工作）
 *   * [下] → GIMBAL_GYRO_MODE（正常工作）
 *
 * ## 坐标系定义
 * - Yaw 轴：逆时针为正方向（从上方俯视）
 * - Pitch 轴：向上为正方向（从侧面看）
 * - 角度单位：弧度（rad），在 robot_cmd 中会进行增量累加
 * - 速度单位：弧度/秒（rad/s）
 *
 * ## 控制模式
 * - GIMBAL_GYRO_MODE：陀螺仪模式，使用 IMU 反馈进行姿态控制
 * - GIMBAL_ZERO_FORCE：急停模式，所有电机停止输出
 * - GIMBAL_FREE_MODE：自由模式（暂未使用）
 *
 * ## 电机配置
 * - Yaw 轴：GM6020，角度+速度双闭环，反馈来自 IMU，CAN ID = 0x001
 * - Pitch 轴：DM4310，位置+速度双环控制，反馈来自 IMU，CAN ID = 0x002
 *
 * ## 关键优化点
 * - Pitch电机必须在GYRO模式下调用 DMMotorEnable() 才能正常工作
 * - DM电机的反馈源在初始化时设置，运行时无需修改
 * - 云台角度增量在 robot_cmd 中累加，确保连续控制
 * - 建议添加pitch角度软件限位，防止云台翻转
 */

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据（可在上层加入"未收到数据"的保护策略）
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
   // 说明：推荐统一使用 IMU 的姿态数据作为反馈源
   // yaw 的编码器 offset 仅用于与底盘的相对关系（底盘跟随/解耦）；视觉模式在 robot_cmd 中已处理为 yaw/pitch 期望
    switch (gimbal_cmd_recv.gimbal_mode) {
        // 停止
        case GIMBAL_ZERO_FORCE:
            DJIMotorStop(yaw_motor);
            // DM4310 MIT 模式停止控制（设置为零电流）
            if (pitch_motor != NULL) {
                DMMotorStop(pitch_motor);
            }
            // 清空yaw轴所有PID控制器的状态
            if (yaw_motor != NULL) {
                ClearPIDState(&yaw_motor->motor_controller.current_PID);
                ClearPIDState(&yaw_motor->motor_controller.speed_PID);
                ClearPIDState(&yaw_motor->motor_controller.angle_PID);
                ClearPIDState(&yaw_motor->motor_controller.imu_angle_PID);
                ClearPIDState(&yaw_motor->motor_controller.imu_speed_PID);
            }
            // 清空pitch轴所有PID控制器的状态
            if (pitch_motor != NULL) {
                ClearPIDState(&pitch_motor->current_PID);
                ClearPIDState(&pitch_motor->speed_PID);
                ClearPIDState(&pitch_motor->angle_PID);
            }
            // 重置缓启动标志
            ramp_active = 0;
            break;
            // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用ß
        case GIMBAL_GYRO_MODE: // 后续只保留此模式
            // 检测是否从ZERO_FORCE切换到GYRO_MODE，如果是则启动缓启动
            if (last_gimbal_mode == GIMBAL_ZERO_FORCE && gimba_IMU_data != NULL)
            {
                // 记录当前IMU角度作为缓启动起始点
                ramp_yaw_ref = gimba_IMU_data->YawTotalAngle;
                ramp_pitch_ref = gimba_IMU_data->Roll; // Pitch使用Roll轴（根据初始化配置）
                ramp_active = 1; // 激活缓启动
            }
            
            // 缓启动逻辑：从起始角度逐渐过渡到目标角度
            if (ramp_active)
            {
                float target_yaw = gimbal_cmd_recv.yaw;
                float target_pitch = gimbal_cmd_recv.pitch;
                
                // Yaw轴缓启动
                float yaw_diff = target_yaw - ramp_yaw_ref;
                if (fabsf(yaw_diff) > GIMBAL_RAMP_RATE_YAW)
                {
                    // 限制单次变化量
                    if (yaw_diff > 0)
                        ramp_yaw_ref += GIMBAL_RAMP_RATE_YAW;
                    else
                        ramp_yaw_ref -= GIMBAL_RAMP_RATE_YAW;
                }
                else
                {
                    ramp_yaw_ref = target_yaw;
                }
                
                // Pitch轴缓启动
                float pitch_diff = target_pitch - ramp_pitch_ref;
                if (fabsf(pitch_diff) > GIMBAL_RAMP_RATE_PITCH)
                {
                    // 限制单次变化量
                    if (pitch_diff > 0)
                        ramp_pitch_ref += GIMBAL_RAMP_RATE_PITCH;
                    else
                        ramp_pitch_ref -= GIMBAL_RAMP_RATE_PITCH;
                }
                else
                {
                    ramp_pitch_ref = target_pitch;
                    // 如果yaw也到达目标，关闭缓启动
                    if (fabsf(target_yaw - ramp_yaw_ref) <= GIMBAL_RAMP_RATE_YAW)
                        ramp_active = 0;
                }
            }
            else
            {
                // 缓启动完成，直接使用目标值
                ramp_yaw_ref = gimbal_cmd_recv.yaw;
                ramp_pitch_ref = gimbal_cmd_recv.pitch;
            }
            
            // YAW轴控制 - 四环串级控制（IMU角度-IMU角速度-电机位置-电机速度）
            // 控制流程在dji_motor.c的DJIMotorControl()中自动完成
            DJIMotorEnable(yaw_motor);
            DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, MOTOR_FEED);
            DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, MOTOR_FEED);
            // 使用缓启动后的目标角度
            DJIMotorSetRef(yaw_motor, ramp_yaw_ref);

            // PITCH轴控制 - DM4310电机使用位置+速度双环控制
            // 注意：DM电机的反馈源在初始化时已设置为OTHER_FEED（IMU），运行时无需修改
            DMMotorEnable(pitch_motor);  // 启用pitch电机
            DMMotorOuterLoop(pitch_motor, ANGLE_LOOP);  // 设置外环为角度环控制
            DMMotorSetRef(pitch_motor, ramp_pitch_ref);  // 使用缓启动后的目标角度
            break;
            // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
        default:
            break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;



    // 更新上一次的模式状态
    last_gimbal_mode = gimbal_cmd_recv.gimbal_mode;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}