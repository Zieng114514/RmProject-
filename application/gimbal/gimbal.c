// 云台模块：负责 yaw/pitch 两轴的初始化与任务控制
#include "gimbal.h"
// 机器人参数宏：尺寸、安装偏置、角度单位转换等
#include "robot_def.h"
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
 * 
 * 主要API：
 * - DMMotorGetMeasure(): 获取电机测量数据结构指针
 * - DMMotorGetStatus(): 获取电机状态
 * - DMMotorSetPosition(): 设置目标位置
 * - DMMotorSetCurrent(): 设置目标电流
 * - DMMotorEnable/Disable/Stop(): 电机控制
 */

// 云台 IMU 数据指针（YawTotalAngle / Pitch / Gyro[] 等）
static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor;
static DMMotorInstance *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static BMI088Instance *bmi088; // 云台IMU
void GimbalInit()
{   
    // 1) 初始化 IMU 姿态解算，获取姿态数据指针（YawTotalAngle/Pitch/Gyro[]）
    gimba_IMU_data = INS_Init();
    // 2) YAW 轴（GM6020）：外环角度、内环速度，反馈来自 IMU（OTHER_FEED）
    //    YawTotalAngle 作为角度反馈，Gyro[2] 作为角速度反馈（注意坐标轴方向）
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8, // 8
                .Ki = 0,
                .Kd = 0,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,

                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 5,  // 50
                .Ki = 2, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 10000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};
    // 3) PITCH 轴（DM4310）：使用 MIT 固件，MIT 模式控制
    //    CAN ID = 0x302，MIT 模式支持位置、速度、电流控制
    DMMotor_Init_Config_s dm_pitch_config = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x302,  // DM4310 MIT 固件的 CAN ID
            .rx_id = 0x01,
        },
        .motor_type = DM4310,
        .control_method = MIT_MODE,       // 使用 MIT 模式，支持位置/速度/电流控制
        .angle_pid_config = NULL,         // MIT 模式不使用内置 PID
        .velocity_pid_config = NULL,      // MIT 模式不使用内置 PID
        .current_pid_config = NULL,       // MIT 模式不使用内置 PID
        .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,    // IMU pitch角度反馈
        .other_velocity_feedback_ptr = &gimba_IMU_data->Gyro[0], // IMU pitch角速度反馈
    };
    // 4) 电机初始化：YAW 使用 GM6020 闭环，PITCH 使用 DM4310 增强版控制
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DMMotorInit(&dm_pitch_config);
    
    // 检查DM电机初始化是否成功
    if (pitch_motor == NULL) {
        // DM电机初始化失败，可以添加错误处理
        // 例如：LOGERROR("[gimbal] DM motor initialization failed");
    }
    
    // DM4310 增强版使用任务式控制循环，需启动其控制任务
    DMMotorControlInit();


    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/**
 * @brief 云台遥控器控制说明
 * 
 * ## 控制数据流向
 * 遥控器 → robot_cmd 模块 → gimbal_cmd 消息 → GimbalTask() → 电机控制
 * 
 * ## 遥控器通道映射（以常见配置为例）
 * - 右摇杆 X 轴：yaw 轴控制（左右转动）
 * - 右摇杆 Y 轴：pitch 轴控制（上下俯仰）
 * - 左摇杆 X 轴：底盘 yaw 跟随/解耦控制
 * - 左摇杆 Y 轴：底盘前后移动
 * - 左摇杆 X 轴：底盘左右移动
 * 
 * ## 坐标系定义
 * - Yaw 轴：逆时针为正方向（从上方俯视）
 * - Pitch 轴：向上为正方向（从侧面看）
 * - 角度单位：度（°），在 robot_cmd 中会转换为弧度
 * - 速度单位：度/秒（°/s）
 * 
 * ## 控制模式
 * - GIMBAL_GYRO_MODE：陀螺仪模式，使用 IMU 反馈，底盘跟随云台
 * - GIMBAL_FREE_MODE：自由模式，云台独立控制，底盘不跟随
 * - GIMBAL_ZERO_FORCE：急停模式，所有电机停止
 * 
 * ## 电机配置
 * - Yaw 轴：GM6020，角度+速度双闭环，反馈来自 IMU，CAN ID = 0x205
 * - Pitch 轴：DM4310 MIT 固件，MIT 模式位置控制，CAN ID = 0x302
 * 
 * ## 注意事项
 * - 遥控器死区处理在 robot_cmd 模块完成
 * - 多圈角度处理在 robot_cmd 模块完成
 * - 视觉模式下的目标跟踪也在 robot_cmd 模块处理
 * - 重力补偿前馈力矩可在此处添加
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
                DMMotorSetCurrent(pitch_motor, 0.0f);
                DMMotorStop(pitch_motor);  // 确保电机停止
            }
            break;
            // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
        case GIMBAL_GYRO_MODE: // 后续只保留此模式
            DJIMotorEnable(yaw_motor);
            DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
            // DM4310 MIT 模式使用位置控制
            DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
            if (pitch_motor != NULL) {
                DMMotorEnable(pitch_motor);  // 确保电机使能
                DMMotorSetPosition(pitch_motor, gimbal_cmd_recv.pitch);  // pitch 作为位置控制
            }
            break;
            // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
        case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
            DJIMotorEnable(yaw_motor);
            DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
            // DM4310 MIT 模式使用位置控制
            DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
            if (pitch_motor != NULL) {
                DMMotorEnable(pitch_motor);  // 确保电机使能
                DMMotorSetPosition(pitch_motor, gimbal_cmd_recv.pitch);  // pitch 作为位置控制
            }
            break;
        default:
            break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
    
    // DM电机状态监控和反馈数据设置
    if (pitch_motor != NULL) {
        // 检查DM电机状态
        Motor_Status_e pitch_status = DMMotorGetStatus(pitch_motor);
        switch (pitch_status) {
            case MOTOR_ERROR:
                // 电机错误，尝试清除错误
                DMMotorClearError(pitch_motor);
                break;
            case MOTOR_OFFLINE:
                // 电机离线，尝试重新连接
                DMMotorEnable(pitch_motor);
                break;
            case MOTOR_ONLINE:
                // 电机正常在线，可以获取数据
                break;
            default:
                break;
        }
        
        // 获取DM电机的测量数据
        DM_Motor_Measure_s *pitch_measure = DMMotorGetMeasure(pitch_motor);
        if (pitch_measure != NULL) {
            // 可以在这里添加pitch电机的反馈数据到gimbal_feedback_data中
            // 例如：gimbal_feedback_data.pitch_motor_angle = pitch_measure->position;
            // 例如：gimbal_feedback_data.pitch_motor_velocity = pitch_measure->velocity;
            // 例如：gimbal_feedback_data.pitch_motor_torque = pitch_measure->torque;
            // 例如：gimbal_feedback_data.pitch_motor_temperature = pitch_measure->temperature_mos;
            
            // 温度保护：如果电机温度过高，停止控制
            if (pitch_measure->temperature_mos > 80.0f) {  // 80度保护阈值
                DMMotorStop(pitch_motor);
            }
        }
        
        // 临时调试：测试DM电机数据解析
        // 取消注释下面的行来启用调试
        static int debug_count = 0;
        if (debug_count++ < 10) {  // 只打印前10次
            // 强制数据处理，绕过ID检查
            Data_Process_Force(pitch_motor);
            DMMotorTestInGimbal(pitch_motor);
        }
    }

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}