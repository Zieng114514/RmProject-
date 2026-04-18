// 云台模块：负责 yaw/pitch 两轴的初始化与任务控制
#include "gimbal.h"
#include <math.h>
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
#include "sp_vision/sp_vision.h"
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

static Publisher_t *gimbal_pub;                    // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                   // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data;  // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;          // 来自cmd的控制信息
static SP_Vision_Recv_s *sp_vision_cmd_recv = NULL; // 来自sp_vision上位机的视觉控制命令

// 视觉控制目标
static float vision_yaw_ref = 0;   // 视觉目标yaw（下位机坐标系，角度制）
static float vision_pitch_ref = 0; // 视觉目标pitch（下位机坐标系，角度制）

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
            // 电机位置环PID（第三环）
            .angle_PID = {
                .Kp = 1.0f, // 8
                .Ki = 0,
                .Kd = 0.00f,
                .DeadBand = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 50,
                .MaxOut = 8450,
            },
            // 电机速度环PID（最内环）
            .speed_PID = {
                .Kp =6000,  // 50
                .Ki = 200, // 200
                .Kd = 0.00f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 1000,
                .MaxOut = 13000,
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
                .Kp = 0.850f,  // 角度环增益，根据实际调试调整
                .Ki = 0.1000f,   // 积分项暂时关闭，避免漂移
                .Kd = 0.0f,  // 微分项
                .DeadBand = 0.00f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 5,
                .MaxOut = 10.25f,
            },
            .speed_PID = {
                .Kp = 0.04f,  // 速度环增益
                .Ki = 0.0f,   // 积分项暂时关闭，避免漂移
                .Kd = 0.0f,  // 微分项
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2,
                .MaxOut = 1.5f,  // 限制最大输出（Nm）
            },
			.other_angle_feedback_ptr = &gimba_IMU_data->Roll,  // 使用展开后的pitch角度反馈，避免±180°跳变
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[1], // 使用pitch轴角速度反馈(Y轴)
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,  // 使用IMU反馈
            .speed_feedback_source = OTHER_FEED,   // 使用IMU反馈
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,  // 位置+速度双环控制
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = MOTOR_TYPE_NONE};  // DM电机类型，使用通用类型

    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DMMotorInit(&dm_pitch_config);

    // DM4310 增强版使用任务式控制循环，需启动其控制任务
    DMMotorControlInit();


    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    sp_vision_cmd_recv = SP_VisionGetRecvData();
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
    uint8_t vision_control_enabled = 0;

    // 获取云台控制数据（可在上层加入"未收到数据"的保护策略）
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    if (sp_vision_online && sp_vision_cmd_recv != NULL && sp_vision_cmd_recv->mode != 0u)
    {
        if (isfinite(sp_vision_cmd_recv->yaw) && isfinite(sp_vision_cmd_recv->pitch))
        {
            // sp_vision发送的是弧度制，下位机云台控制链内部使用角度制
            vision_yaw_ref = sp_vision_cmd_recv->yaw * 57.295779513f;
            vision_pitch_ref = -sp_vision_cmd_recv->pitch * 57.295779513f;
            vision_control_enabled = 1u;
        }
    }
   // 说明：推荐统一使用 IMU 的姿态数据作为反馈源
   // yaw 的编码器 offset 仅用于与底盘的相对关系（底盘跟随/解耦）；视觉模式在 robot_cmd 中已处理为 yaw/pitch 期望
    switch (gimbal_cmd_recv.gimbal_mode) {
        case GIMBAL_ZERO_FORCE:
            DJIMotorStop(yaw_motor);
            if (pitch_motor != NULL) {
                DMMotorStop(pitch_motor);
            }
            if (yaw_motor != NULL) {
                ClearPIDState(&yaw_motor->motor_controller.current_PID);
                ClearPIDState(&yaw_motor->motor_controller.speed_PID);
                ClearPIDState(&yaw_motor->motor_controller.angle_PID);
            }
            if (pitch_motor != NULL) {
                ClearPIDState(&pitch_motor->current_PID);
                ClearPIDState(&pitch_motor->speed_PID);
                ClearPIDState(&pitch_motor->angle_PID);
            }
            break;

        case GIMBAL_GYRO_MODE:
        case GIMBAL_FREE_MODE:
            DJIMotorEnable(yaw_motor);
            DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);

            DMMotorEnable(pitch_motor);
            DMMotorOuterLoop(pitch_motor, ANGLE_LOOP);

            if (vision_control_enabled)
            {
                // sp_vision侧的yaw/pitch均为弧度制，且与当前下位机机械坐标系方向相反
                DJIMotorSetRef(yaw_motor, -vision_yaw_ref);
                DMMotorSetRef(pitch_motor, -vision_pitch_ref);
            }
            else if (gimbal_cmd_recv.gimbal_mode == GIMBAL_GYRO_MODE)
            {
                DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
                DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
            }
            else if (gimba_IMU_data != NULL)
            {
                DJIMotorSetRef(yaw_motor, gimba_IMU_data->YawTotalAngle);
                DMMotorSetRef(pitch_motor, gimba_IMU_data->Roll);
            }
            break;

        default:
            break;
    }

    SP_Vision_Set_Mode(SP_VISION_MODE_AIM);

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    SP_Vision_Set_BulletSpeed(23.0f);
    SP_Vision_Set_BulletCount(0u);
    SP_Vision_Set_EnemyColor(COLOR_RED);

    if (gimba_IMU_data != NULL)
    {
        float q[4] = {QEKF_INS.q[0], QEKF_INS.q[1], QEKF_INS.q[2], QEKF_INS.q[3]};
        SP_Vision_Set_GimbalAttitude(
            gimba_IMU_data->YawTotalAngle / 180.0f * PI,
            gimba_IMU_data->Gyro[2],
            gimba_IMU_data->Roll / 180.0f * PI,
            gimba_IMU_data->Gyro[1],
            q);
    }



    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
