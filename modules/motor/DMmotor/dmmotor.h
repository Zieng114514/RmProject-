/**
 * @file dmmotor.h
 * @brief 达妙电机控制模块头文件 - 基于DMPower.c功能增强版
 * @author Enhanced from DMPower.c
 * @version 2.0
 * @date 2024-12-24
 */

#ifndef DMMOTOR_H
#define DMMOTOR_H

#include "stdint.h"
#include "bsp_can.h"
#include "daemon.h"
#include "controller.h"
#include "cmsis_os.h"

// 电机类型枚举
typedef enum
{
    DM4310 = 0,
    DM6006,
    DM8006
} DMMotor_Type_e;

// 控制模式枚举
typedef enum
{
    MIT_MODE = 0,
    POSITION_VELOCITY_MODE,
    VELOCITY_MODE,
    CURRENT_MODE
} DMMotor_Control_Method_e;

// 电机状态枚举
typedef enum
{
    MOTOR_OFFLINE = 0,
    MOTOR_ONLINE,
    MOTOR_ENABLED,
    MOTOR_DISABLED,
    MOTOR_ERROR
} Motor_Status_e;

// 一拖四固件电机ID枚举
typedef enum
{
    Motor_DM_ID_0x301 = 1,
    Motor_DM_ID_0x302,
    Motor_DM_ID_0x303,
    Motor_DM_ID_0x304,
    Motor_DM_ID_0x305,
    Motor_DM_ID_0x306,
    Motor_DM_ID_0x307,
    Motor_DM_ID_0x308
} Enum_Motor_DM_Motor_ID_1_To_4;

// 电机参数定义
#define P_MIN   -12.5f    // 位置最小值
#define P_MAX   12.5f     // 位置最大值
#define V_MIN   -30.0f   // 速度最小值
#define V_MAX   30.0f    // 速度最大值
#define KP_MIN  0.0f     // Kp最小值
#define KP_MAX  500.0f   // Kp最大值
#define KD_MIN  0.0f     // Kd最小值
#define KD_MAX  5.0f     // Kd最大值
#define T_MIN   -10.0f   // 扭矩最小值
#define T_MAX   10.0f    // 扭矩最大值

// DM6006参数
#define P_MIN_6006   -12.5f
#define P_MAX_6006   12.5f
#define V_MIN_6006   -45.0f
#define V_MAX_6006   45.0f
#define T_MIN_6006   -12.0f
#define T_MAX_6006   12.0f

// DM8006参数
#define P_MIN_8006   -12.5f
#define P_MAX_8006   12.5f
#define V_MIN_8006   -25.0f
#define V_MAX_8006   25.0f
#define T_MIN_8006   -20.0f
#define T_MAX_8006   20.0f

// DM电机测量数据结构
typedef struct
{
    // 编码器相关
    uint16_t last_encoder;        // 上一次读取的编码器值
    uint16_t encoder;             // 当前编码器值
    int32_t total_encoder;        // 总编码器值
    int32_t total_round;          // 总圈数
    
    // 位置和角度
    float position;               // 当前位置
    float angle_single_round;     // 单圈角度
    
    // 速度和扭矩
    float velocity;               // 当前速度
    float torque;                 // 当前扭矩
    float current;                // 当前电流
    
    // 温度
    float temperature_mos;         // MOS管温度
    float temperature_rotor;       // 转子温度
    
    // 时间相关
    uint32_t feed_cnt;            // 反馈计数
    float dt;                     // 时间间隔
} DM_Motor_Measure_s;

// 初始化配置结构体
typedef struct
{
    // CAN配置
    CAN_Init_Config_s can_config;
    
    // 电机类型
    DMMotor_Type_e motor_type;
    
    // 控制模式
    DMMotor_Control_Method_e control_method;
    
    // PID配置
    PID_Init_Config_s *angle_pid_config;
    PID_Init_Config_s *velocity_pid_config;
    PID_Init_Config_s *current_pid_config;
    
    // 反馈指针
    float *other_angle_feedback_ptr;
    float *other_velocity_feedback_ptr;
} DMMotor_Init_Config_s;

// 电机数据结构
typedef struct
{
    // 电机测量值
    DM_Motor_Measure_s measure;
    
    // 电机基本信息
    DMMotor_Type_e motor_type;
    DMMotor_Control_Method_e control_method;
    Motor_Status_e motor_status;
    
    // 目标值
    float target_position;
    float target_velocity;
    float target_torque;
    float target_current;
    
    // 参数范围
    float angle_max;
    float velocity_max;
    float torque_max;
    float current_max;
    
    // PID参数
    float kp, ki, kd;
    
    // PID控制器
    PIDInstance angle_pid;
    PIDInstance velocity_pid;
    PIDInstance current_pid;
    
    // 反馈指针
    float *other_angle_feedback_ptr;
    float *other_velocity_feedback_ptr;
    
    // 系统相关
    CANInstance *can_instance;
    DaemonInstance *motor_daemon;
    uint32_t flag;
    
    // 配置信息
    DMMotor_Init_Config_s config;
} DMMotorInstance;

// 函数声明
DMMotorInstance* DMMotorInit(DMMotor_Init_Config_s *config);
void DMMotorControlInit(void);
void DMMotorControlTask(void const *argument);

// 控制函数
void DMMotorSetPosition(DMMotorInstance *motor, float position);
void DMMotorSetVelocity(DMMotorInstance *motor, float velocity);
void DMMotorSetTorque(DMMotorInstance *motor, float torque);
void DMMotorSetCurrent(DMMotorInstance *motor, float current);
void DMMotorSetPID(DMMotorInstance *motor, float kp, float ki, float kd);

// 状态控制函数
void DMMotorEnable(DMMotorInstance *motor);
void DMMotorDisable(DMMotorInstance *motor);
void DMMotorStop(DMMotorInstance *motor);
void DMMotorClearError(DMMotorInstance *motor);
void DMMotorSaveZero(DMMotorInstance *motor);

// 状态获取函数
Motor_Status_e DMMotorGetStatus(DMMotorInstance *motor);
float DMMotorGetPosition(DMMotorInstance *motor);
float DMMotorGetVelocity(DMMotorInstance *motor);
float DMMotorGetTorque(DMMotorInstance *motor);
float DMMotorGetTemperature(DMMotorInstance *motor);

// 测量数据获取函数
DM_Motor_Measure_s* DMMotorGetMeasure(DMMotorInstance *motor);
float DMMotorGetCurrent(DMMotorInstance *motor);
float DMMotorGetRotorTemperature(DMMotorInstance *motor);
uint16_t DMMotorGetEncoder(DMMotorInstance *motor);
int32_t DMMotorGetTotalEncoder(DMMotorInstance *motor);
int32_t DMMotorGetTotalRound(DMMotorInstance *motor);

// 数据处理函数
void CAN_Rx_Callback(CANInstance* instance);
void Data_Process(DMMotorInstance *motor);
void Data_Process_Force(DMMotorInstance *motor);  // 强制数据处理，用于调试

// 调试函数
void DMMotorDebugPrint(DMMotorInstance *motor);

// 测试函数
void DMMotorTestDataParsing(void);
void DMMotorTestInGimbal(DMMotorInstance *pitch_motor);

#endif // DMMOTOR_H