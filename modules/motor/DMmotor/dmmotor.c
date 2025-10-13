/**
 * @file dmmotor.c
 * @brief 达妙电机控制模块 - 基于DMPower.c功能增强版
 * @author Enhanced from DMPower.c
 * @version 2.0
 * @date 2024-12-24
 * 
 * 功能特性：
 * - 支持多种控制模式（MIT、位置速度、速度、电流）
 * - 支持多种电机型号（DM4310、DM6006、DM8006）
 * - 完善的错误处理和状态监控
 * - 一拖四固件支持
 * - 实时数据反馈
 */

#include "dmmotor.h"
#include "stdlib.h"
#include "string.h"
#include "stdint.h"
#include "math.h"
#include "stdio.h"
#include "bsp_can.h"
#include "daemon.h"
#include "cmsis_os.h"
#include "bsp_dwt.h"

// 常量定义
#define DM_MOTOR_CNT 8                    // 最大支持电机数量
#define RPM_TO_RADPS (2.0f * PI / 60.0f) // RPM转弧度/秒
#define DEG_TO_RAD (PI / 180.0f)         // 度转弧度
#define CELSIUS_TO_KELVIN (273.15f)      // 摄氏度转开氏度
#ifndef PI
#define PI (3.14159265358979323846f)     // 圆周率
#endif

// 数学工具函数
static float constrain_float(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

static int float_to_int(float value, float min_val, float max_val, int bits)
{
    float span = max_val - min_val;
    float offset = min_val;
    return (int)((value - offset) * ((float)((1 << bits) - 1)) / span);
}

static float int_to_float(int value, float min_val, float max_val, int bits)
{
    float span = max_val - min_val;
    float offset = min_val;
    return ((float)value) * span / ((float)((1 << bits) - 1)) + offset;
}

// 专门用于DM电机的数据转换函数
static float dm_int_to_float(int value, float min_val, float max_val, int bits)
{
    float span = max_val - min_val;
    float offset = min_val;
    return ((float)value) * span / ((float)((1 << bits) - 1)) + offset;
}

// 简单的整数转字符串函数
static void int_to_string(int value, char *str, int base)
{
    if (value == 0) {
        str[0] = '0';
        str[1] = '\0';
        return;
    }
    
    int i = 0;
    int is_negative = 0;
    
    if (value < 0) {
        is_negative = 1;
        value = -value;
    }
    
    while (value > 0) {
        str[i++] = '0' + (value % base);
        value /= base;
    }
    
    if (is_negative) {
        str[i++] = '-';
    }
    
    str[i] = '\0';
    
    // 反转字符串
    int len = i;
    for (int j = 0; j < len / 2; j++) {
        char temp = str[j];
        str[j] = str[len - 1 - j];
        str[len - 1 - j] = temp;
    }
}

// 全局变量
static DMMotorInstance *dm_motor_instances[DM_MOTOR_CNT];
static uint8_t motor_count = 0;
static osThreadId dm_task_handle[DM_MOTOR_CNT];

// CAN数据缓冲区（一拖四固件）
static uint8_t CAN1_0x3fe_Tx_Data[8];
static uint8_t CAN2_0x3fe_Tx_Data[8];
static uint8_t CAN1_0x4fe_Tx_Data[8];
static uint8_t CAN2_0x4fe_Tx_Data[8];

// 电机控制命令
static const uint8_t DM_Motor_CAN_Message_Clear_Error[8] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb
};

static const uint8_t DM_Motor_CAN_Message_Enter[8] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc
};

static const uint8_t DM_Motor_CAN_Message_Exit[8] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd
};

static const uint8_t DM_Motor_CAN_Message_Save_Zero[8] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe
};

/**
 * @brief 一拖四固件数据分配函数
 * @param hcan CAN句柄
 * @param motor_id 电机ID
 * @return 数据缓冲区指针
 */
static uint8_t* allocate_tx_data(CAN_HandleTypeDef* hcan, Enum_Motor_DM_Motor_ID_1_To_4 motor_id)
{
    uint8_t* tmp_tx_data_ptr = NULL;
    
    if (hcan->Instance == CAN1)
    {
        switch (motor_id)
        {
        case Motor_DM_ID_0x301:
            tmp_tx_data_ptr = &(CAN1_0x3fe_Tx_Data[0]);
            break;
        case Motor_DM_ID_0x302:
            tmp_tx_data_ptr = &(CAN1_0x3fe_Tx_Data[2]);
            break;
        case Motor_DM_ID_0x303:
            tmp_tx_data_ptr = &(CAN1_0x3fe_Tx_Data[4]);
            break;
        case Motor_DM_ID_0x304:
            tmp_tx_data_ptr = &(CAN1_0x3fe_Tx_Data[6]);
            break;
        case Motor_DM_ID_0x305:
            tmp_tx_data_ptr = &(CAN1_0x4fe_Tx_Data[0]);
            break;
        case Motor_DM_ID_0x306:
            tmp_tx_data_ptr = &(CAN1_0x4fe_Tx_Data[2]);
            break;
        case Motor_DM_ID_0x307:
            tmp_tx_data_ptr = &(CAN1_0x4fe_Tx_Data[4]);
            break;
        case Motor_DM_ID_0x308:
            tmp_tx_data_ptr = &(CAN1_0x4fe_Tx_Data[6]);
            break;
        default:
            break;
        }
    }
    else if (hcan->Instance == CAN2)
    {
        // CAN2 支持（如果需要）
        switch (motor_id)
        {
        case Motor_DM_ID_0x301:
            tmp_tx_data_ptr = &(CAN2_0x3fe_Tx_Data[0]);
            break;
        case Motor_DM_ID_0x302:
            tmp_tx_data_ptr = &(CAN2_0x3fe_Tx_Data[2]);
            break;
        case Motor_DM_ID_0x303:
            tmp_tx_data_ptr = &(CAN2_0x3fe_Tx_Data[4]);
            break;
        case Motor_DM_ID_0x304:
            tmp_tx_data_ptr = &(CAN2_0x3fe_Tx_Data[6]);
            break;
        case Motor_DM_ID_0x305:
            tmp_tx_data_ptr = &(CAN2_0x4fe_Tx_Data[0]);
            break;
        case Motor_DM_ID_0x306:
            tmp_tx_data_ptr = &(CAN2_0x4fe_Tx_Data[2]);
            break;
        case Motor_DM_ID_0x307:
            tmp_tx_data_ptr = &(CAN2_0x4fe_Tx_Data[4]);
            break;
        case Motor_DM_ID_0x308:
            tmp_tx_data_ptr = &(CAN2_0x4fe_Tx_Data[6]);
            break;
        default:
            break;
        }
    }
    
    return tmp_tx_data_ptr;
}

/**
 * @brief 电机丢失回调函数
 * @param motor_ptr 电机实例指针
 */
static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    if (motor != NULL)
    {
        motor->motor_status = MOTOR_OFFLINE;
        // 可以添加更多错误处理逻辑
    }
}

/**
 * @brief 数据转换函数：uint转float
 * @param x_int 整数值
 * @param x_min 最小值
 * @param x_max 最大值
 * @param bits 位数
 * @return 转换后的浮点值
 */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief 数据转换函数：float转uint
 * @param x 浮点值
 * @param x_min 最小值
 * @param x_max 最大值
 * @param bits 位数
 * @return 转换后的整数值
 */
static int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief MIT模式控制函数
 * @param motor 电机实例
 * @param pos 位置目标
 * @param vel 速度目标
 * @param kp 位置增益
 * @param kd 速度增益
 * @param torque 扭矩目标
 */
static void MIT_ControlMotor(DMMotorInstance *motor, float pos, float vel, float kp, float kd, float torque)
{
    if (motor == NULL || motor->can_instance == NULL) return;
    
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint8_t *tx_buff = motor->can_instance->tx_buff;
    
    // 根据电机类型选择参数范围
    float p_min, p_max, v_min, v_max, t_min, t_max;
    
    switch (motor->motor_type)
    {
    case DM4310:
        p_min = P_MIN; p_max = P_MAX;
        v_min = V_MIN; v_max = V_MAX;
        t_min = T_MIN; t_max = T_MAX;
        break;
    case DM6006:
        p_min = P_MIN_6006; p_max = P_MAX_6006;
        v_min = V_MIN_6006; v_max = V_MAX_6006;
        t_min = T_MIN_6006; t_max = T_MAX_6006;
        break;
    case DM8006:
        p_min = P_MIN_8006; p_max = P_MAX_8006;
        v_min = V_MIN_8006; v_max = V_MAX_8006;
        t_min = T_MIN_8006; t_max = T_MAX_8006;
        break;
    default:
        return;
    }
    
    // 数据转换
    pos_tmp = float_to_int(pos, p_min, p_max, 16);
    vel_tmp = float_to_int(vel, v_min, v_max, 12);
    kp_tmp = float_to_int(kp, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_int(kd, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_int(torque, t_min, t_max, 12);
    
    // 数据打包
    tx_buff[0] = (pos_tmp >> 8);
    tx_buff[1] = pos_tmp;
    tx_buff[2] = (vel_tmp >> 4);
    tx_buff[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    tx_buff[4] = kp_tmp;
    tx_buff[5] = (kd_tmp >> 4);
    tx_buff[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    tx_buff[7] = tor_tmp;
    
    CANTransmit(motor->can_instance, 1);
}

/**
 * @brief 位置速度模式控制函数
 * @param motor 电机实例
 * @param pos 位置目标
 * @param vel 速度目标
 */
static void PosSpeed_ControlMotor(DMMotorInstance *motor, float pos, float vel)
{
    if (motor == NULL || motor->can_instance == NULL) return;
    
    uint8_t *tx_buff = motor->can_instance->tx_buff;
    uint8_t *pbuf = (uint8_t*)&pos;
    uint8_t *vbuf = (uint8_t*)&vel;
    
    tx_buff[0] = pbuf[0];
    tx_buff[1] = pbuf[1];
    tx_buff[2] = pbuf[2];
    tx_buff[3] = pbuf[3];
    tx_buff[4] = vbuf[0];
    tx_buff[5] = vbuf[1];
    tx_buff[6] = vbuf[2];
    tx_buff[7] = vbuf[3];
    
    CANTransmit(motor->can_instance, 1);
}

/**
 * @brief 速度模式控制函数
 * @param motor 电机实例
 * @param vel 速度目标
 */
static void Speed_ControlMotor(DMMotorInstance *motor, float vel)
{
    if (motor == NULL || motor->can_instance == NULL) return;
    
    uint8_t *tx_buff = motor->can_instance->tx_buff;
    uint8_t *vbuf = (uint8_t*)&vel;
    
    tx_buff[0] = vbuf[0];
    tx_buff[1] = vbuf[1];
    tx_buff[2] = vbuf[2];
    tx_buff[3] = vbuf[3];
    tx_buff[4] = 0;
    tx_buff[5] = 0;
    tx_buff[6] = 0;
    tx_buff[7] = 0;
    
    CANTransmit(motor->can_instance, 1);
}

/**
 * @brief 电流模式控制函数
 * @param motor 电机实例
 * @param current 电流目标
 */
static void Current_ControlMotor(DMMotorInstance *motor, float current)
{
    if (motor == NULL || motor->can_instance == NULL) return;
    
    uint8_t *tx_buff = motor->can_instance->tx_buff;
    uint8_t *cbuf = (uint8_t*)&current;
    
    tx_buff[0] = cbuf[0];
    tx_buff[1] = cbuf[1];
    tx_buff[2] = cbuf[2];
    tx_buff[3] = cbuf[3];
    tx_buff[4] = 0;
    tx_buff[5] = 0;
    tx_buff[6] = 0;
    tx_buff[7] = 0;
    
    CANTransmit(motor->can_instance, 1);
}

/**
 * @brief CAN接收回调函数
 * @param instance CAN实例
 */
void CAN_Rx_Callback(CANInstance* instance)
{
    if (instance == NULL || instance->id == NULL) return;
    
    DMMotorInstance *motor = (DMMotorInstance *)instance->id;
    
    // 更新在线状态
    motor->flag += 1;
    motor->motor_status = MOTOR_ONLINE;
    
    // 临时调试：强制处理数据，绕过ID检查
    // 取消注释下面的行来启用强制数据处理
    Data_Process_Force(motor);
    
    // 数据处理
    Data_Process(motor);
}

/**
 * @brief DM电机调试函数 - 用于验证数据解析
 * @param motor 电机实例
 */
void DMMotorDebugPrint(DMMotorInstance *motor)
{
    if (motor == NULL) return;
    
    // 打印原始CAN数据
    uint8_t *rx_buff = motor->can_instance->rx_buff;
    printf("DM Motor Raw Data: ");
    for (int i = 0; i < 8; i++) {
        printf("%d ", rx_buff[i]);
    }
    printf("\n");
    
    // 打印解析后的数据
    printf("DM Motor Parsed: ID=%d, Encoder=%d, Omega=%d, Torque=%d, Temp_MOS=%d, Temp_Rotor=%d\n",
           rx_buff[0], (rx_buff[1] << 8) | rx_buff[2], 
           (rx_buff[3] << 4) | (rx_buff[4] >> 4),
           ((rx_buff[4] & 0x0f) << 8) | rx_buff[5],
           rx_buff[6], rx_buff[7]);
    
    // 打印measure结构体数据
    printf("DM Motor Measure: Pos=%.4f, Vel=%.4f, Torque=%.4f, Temp_MOS=%.1f, Temp_Rotor=%.1f\n",
           motor->measure.position, motor->measure.velocity, motor->measure.torque,
           motor->measure.temperature_mos, motor->measure.temperature_rotor);
}

/**
 * @brief 强制数据处理函数 - 用于调试，绕过ID检查
 * @param motor 电机实例
 */
void Data_Process_Force(DMMotorInstance *motor)
{
    if (motor == NULL || motor->can_instance == NULL) return;
    
    int32_t delta_encoder;
    uint16_t tmp_encoder, tmp_omega, tmp_torque;
    uint8_t *rx_buff = motor->can_instance->rx_buff;
    
    printf("DM Motor Force Process: Raw data = ");
    for (int i = 0; i < 8; i++) {
        printf("%d ", rx_buff[i]);
    }
    printf("\n");
    
    // 更新守护进程
    DaemonReload(motor->motor_daemon);
    motor->measure.dt = DWT_GetDeltaT(&motor->measure.feed_cnt);
    
    // 解析数据 - 根据DM电机协议格式
    tmp_encoder = (rx_buff[1] << 8) | rx_buff[2];
    tmp_omega = (rx_buff[3] << 4) | (rx_buff[4] >> 4);
    tmp_torque = ((rx_buff[4] & 0x0f) << 8) | rx_buff[5];
    
    printf("DM Motor Force Parse: ID=%d, Encoder=%d, Omega=%d, Torque=%d, Temp_MOS=%d, Temp_Rotor=%d\n",
           rx_buff[0], tmp_encoder, tmp_omega, tmp_torque, rx_buff[6], rx_buff[7]);
    
    // 保存上一次编码器值
    motor->measure.last_encoder = motor->measure.encoder;
    motor->measure.encoder = tmp_encoder;
    
    // 计算圈数
    delta_encoder = tmp_encoder - motor->measure.last_encoder;
    if (delta_encoder < -(1 << 15))
    {
        motor->measure.total_round++;
    }
    else if (delta_encoder > (1 << 15))
    {
        motor->measure.total_round--;
    }
    
    motor->measure.total_encoder = motor->measure.total_round * (1 << 16) + tmp_encoder - ((1 << 15) - 1);
    
    // 计算实际值
    motor->measure.position = (float)(motor->measure.total_encoder) / (float)((1 << 16) - 1) * motor->angle_max * 2.0f;
    motor->measure.angle_single_round = (float)tmp_encoder / (float)((1 << 16) - 1) * 360.0f;
    
    // 使用正确的数据转换函数
    motor->measure.velocity = dm_int_to_float(tmp_omega, -motor->velocity_max, motor->velocity_max, 12);
    motor->measure.torque = dm_int_to_float(tmp_torque, -motor->torque_max, motor->torque_max, 12);
    
    // 温度数据直接使用，不需要转换
    motor->measure.temperature_mos = (float)rx_buff[6];
    motor->measure.temperature_rotor = (float)rx_buff[7];
    
    printf("DM Motor Force Result: Pos=%.4f, Vel=%.4f, Torque=%.4f, Temp_MOS=%.1f, Temp_Rotor=%.1f\n",
           motor->measure.position, motor->measure.velocity, motor->measure.torque,
           motor->measure.temperature_mos, motor->measure.temperature_rotor);
}

/**
 * @brief 数据处理函数
 * @param motor 电机实例
 */
void Data_Process(DMMotorInstance *motor)
{
    if (motor == NULL || motor->can_instance == NULL) return;
    
    int32_t delta_encoder;
    uint16_t tmp_encoder, tmp_omega, tmp_torque;
    uint8_t *rx_buff = motor->can_instance->rx_buff;
    
    // 检查电机ID匹配 - 添加调试信息
    // printf("DM Motor ID Check: rx_id=%d, expected_id=%d\n", rx_buff[0], (motor->can_instance->tx_id & 0x0f));
    if (rx_buff[0] != (motor->can_instance->tx_id & 0x0f))
    {
        // printf("DM Motor ID Mismatch: rx_id=%d, expected_id=%d\n", rx_buff[0], (motor->can_instance->tx_id & 0x0f));
        return;
    }
    
    // 更新守护进程
    DaemonReload(motor->motor_daemon);
    motor->measure.dt = DWT_GetDeltaT(&motor->measure.feed_cnt);
    
    // 解析数据 - 根据DM电机协议格式
    tmp_encoder = (rx_buff[1] << 8) | rx_buff[2];
    tmp_omega = (rx_buff[3] << 4) | (rx_buff[4] >> 4);
    tmp_torque = ((rx_buff[4] & 0x0f) << 8) | rx_buff[5];
    
    // 调试信息 - 可以临时添加用于调试
    // printf("DM Motor Debug: ID=%d, Encoder=%d, Omega=%d, Torque=%d, Temp_MOS=%d, Temp_Rotor=%d\n",
    //        rx_buff[0], tmp_encoder, tmp_omega, tmp_torque, rx_buff[6], rx_buff[7]);
    
    // 保存上一次编码器值
    motor->measure.last_encoder = motor->measure.encoder;
    motor->measure.encoder = tmp_encoder;
    
    // 计算圈数
    delta_encoder = tmp_encoder - motor->measure.last_encoder;
    if (delta_encoder < -(1 << 15))
    {
        motor->measure.total_round++;
    }
    else if (delta_encoder > (1 << 15))
    {
        motor->measure.total_round--;
    }
    
    motor->measure.total_encoder = motor->measure.total_round * (1 << 16) + tmp_encoder - ((1 << 15) - 1);
    
    // 计算实际值
    motor->measure.position = (float)(motor->measure.total_encoder) / (float)((1 << 16) - 1) * motor->angle_max * 2.0f;
    motor->measure.angle_single_round = (float)tmp_encoder / (float)((1 << 16) - 1) * 360.0f;
    
    // 使用正确的数据转换函数
    // 注意：DM电机的数据范围可能不同，需要根据实际协议调整
    motor->measure.velocity = dm_int_to_float(tmp_omega, -motor->velocity_max, motor->velocity_max, 12);
    motor->measure.torque = dm_int_to_float(tmp_torque, -motor->torque_max, motor->torque_max, 12);
    
    // 如果上述转换不正确，可以尝试直接使用原始数据
    // motor->measure.velocity = (float)tmp_omega * 0.01f;  // 根据实际比例调整
    // motor->measure.torque = (float)tmp_torque * 0.01f;   // 根据实际比例调整
    
    // 温度数据直接使用，不需要转换
    motor->measure.temperature_mos = (float)rx_buff[6];
    motor->measure.temperature_rotor = (float)rx_buff[7];
    
    // 调试输出 - 可以临时启用用于调试
    // printf("DM Motor Parsed: Pos=%.4f, Vel=%.4f, Torque=%.4f, Temp_MOS=%.1f, Temp_Rotor=%.1f\n",
    //        motor->measure.position, motor->measure.velocity, motor->measure.torque,
    //        motor->measure.temperature_mos, motor->measure.temperature_rotor);
    
    // 临时调试输出 - 用于验证数据解析
    // DMMotorDebugPrint(motor);
}

/**
 * @brief 电机初始化函数
 * @param config 初始化配置
 * @return 电机实例指针
 */
DMMotorInstance* DMMotorInit(DMMotor_Init_Config_s *config)
{
    if (config == NULL || motor_count >= DM_MOTOR_CNT) return NULL;
    
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    if (motor == NULL) return NULL;
    
    // 清零初始化
    memset(motor, 0, sizeof(DMMotorInstance));
    
    // 初始化measure结构体
    motor->measure.last_encoder = 0;
    motor->measure.encoder = 0;
    motor->measure.total_encoder = 0;
    motor->measure.total_round = 0;
    motor->measure.position = 0.0f;
    motor->measure.angle_single_round = 0.0f;
    motor->measure.velocity = 0.0f;
    motor->measure.torque = 0.0f;
    motor->measure.current = 0.0f;
    motor->measure.temperature_mos = 0.0f;
    motor->measure.temperature_rotor = 0.0f;
    motor->measure.feed_cnt = 0;
    motor->measure.dt = 0.0f;
    
    // 复制配置
    motor->config = *config;
    motor->motor_type = config->motor_type;
    motor->control_method = config->control_method;
    
    // 设置参数范围
    switch (motor->motor_type)
    {
    case DM4310:
        motor->angle_max = 12.5f;
        motor->velocity_max = 30.0f;
        motor->torque_max = 10.0f;
        motor->current_max = 10.0f;
        break;
    case DM6006:
        motor->angle_max = 12.5f;
        motor->velocity_max = 45.0f;
        motor->torque_max = 12.0f;
        motor->current_max = 12.0f;
        break;
    case DM8006:
        motor->angle_max = 12.5f;
        motor->velocity_max = 25.0f;
        motor->torque_max = 20.0f;
        motor->current_max = 20.0f;
        break;
    default:
        motor->angle_max = 12.5f;
        motor->velocity_max = 30.0f;
        motor->torque_max = 10.0f;
        motor->current_max = 10.0f;
        break;
    }
    
    // 初始化PID控制器
    if (config->angle_pid_config != NULL)
    {
        PIDInit(&motor->angle_pid, config->angle_pid_config);
    }
    if (config->velocity_pid_config != NULL)
    {
        PIDInit(&motor->velocity_pid, config->velocity_pid_config);
    }
    if (config->current_pid_config != NULL)
    {
        PIDInit(&motor->current_pid, config->current_pid_config);
    }
    
    // 设置CAN回调
    config->can_config.can_module_callback = CAN_Rx_Callback;
    config->can_config.id = motor;
    
    // 注册CAN
    motor->can_instance = CANRegister(&config->can_config);
    if (motor->can_instance == NULL)
    {
        free(motor);
        return NULL;
    }
    
    // 注册看门狗
    Daemon_Init_Config_s daemon_conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&daemon_conf);
    
    // 设置反馈指针
    motor->other_angle_feedback_ptr = config->other_angle_feedback_ptr;
    motor->other_velocity_feedback_ptr = config->other_velocity_feedback_ptr;
    
    // 使能电机
    DMMotorEnable(motor);
    DWT_Delay(0.1);
    
    // 保存实例
    dm_motor_instances[motor_count] = motor;
    motor_count++;
    
    return motor;
}

/**
 * @brief 电机控制任务
 * @param argument 电机实例指针
 */
void DMMotorControlTask(void const *argument)
{
    DMMotorInstance *motor = (DMMotorInstance *)argument;
    if (motor == NULL) return;
    
    while (1)
    {
        if (motor->motor_status == MOTOR_ONLINE)
        {
            // 电机在线，正常控制
            // 约束控制值
            motor->target_position = constrain_float(motor->target_position, -motor->angle_max, motor->angle_max);
            motor->target_velocity = constrain_float(motor->target_velocity, -motor->velocity_max, motor->velocity_max);
            motor->target_torque = constrain_float(motor->target_torque, -motor->torque_max, motor->torque_max);
            motor->target_current = constrain_float(motor->target_current, -motor->current_max, motor->current_max);
            
            switch (motor->control_method)
            {
            case MIT_MODE:
                MIT_ControlMotor(motor, motor->target_position, motor->target_velocity, 
                               motor->kp, motor->kd, motor->target_torque);
                break;
            case POSITION_VELOCITY_MODE:
                PosSpeed_ControlMotor(motor, motor->target_position, motor->target_velocity);
                break;
            case VELOCITY_MODE:
                Speed_ControlMotor(motor, motor->target_velocity);
                break;
            case CURRENT_MODE:
                Current_ControlMotor(motor, motor->target_current);
                break;
            default:
                break;
            }
        }
        else if (motor->motor_status == MOTOR_OFFLINE)
        {
            // 电机离线，尝试重新连接
            DMMotorEnable(motor);
        }
        else
        {
            // 电机错误，清除错误
            DMMotorClearError(motor);
        }
        
        osDelay(2);
    }
}

/**
 * @brief 电机控制初始化
 */
void DMMotorControlInit()
{
    char task_name[10] = "dm_motor";
    
    for (uint8_t i = 0; i < motor_count; i++)
    {
        char id_buff[3] = {0};
        int_to_string(i, id_buff, 10);
        strcat(task_name, id_buff);
        
        osThreadDef(task_name, DMMotorControlTask, osPriorityNormal, 0, 128);
        dm_task_handle[i] = osThreadCreate(osThread(task_name), dm_motor_instances[i]);
    }
}

/**
 * @brief 设置电机目标位置
 * @param motor 电机实例
 * @param position 目标位置
 */
void DMMotorSetPosition(DMMotorInstance *motor, float position)
{
    if (motor != NULL)
    {
        motor->target_position = position;
    }
}

/**
 * @brief 设置电机目标速度
 * @param motor 电机实例
 * @param velocity 目标速度
 */
void DMMotorSetVelocity(DMMotorInstance *motor, float velocity)
{
    if (motor != NULL)
    {
        motor->target_velocity = velocity;
    }
}

/**
 * @brief 设置电机目标扭矩
 * @param motor 电机实例
 * @param torque 目标扭矩
 */
void DMMotorSetTorque(DMMotorInstance *motor, float torque)
{
    if (motor != NULL)
    {
        motor->target_torque = torque;
    }
}

/**
 * @brief 设置电机目标电流
 * @param motor 电机实例
 * @param current 目标电流
 */
void DMMotorSetCurrent(DMMotorInstance *motor, float current)
{
    if (motor != NULL)
    {
        motor->target_current = current;
    }
}

/**
 * @brief 设置PID参数
 * @param motor 电机实例
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
void DMMotorSetPID(DMMotorInstance *motor, float kp, float ki, float kd)
{
    if (motor != NULL)
    {
        motor->kp = kp;
        motor->ki = ki;
        motor->kd = kd;
    }
}

/**
 * @brief 使能电机
 * @param motor 电机实例
 */
void DMMotorEnable(DMMotorInstance *motor)
{
    if (motor != NULL && motor->can_instance != NULL)
    {
        memcpy(motor->can_instance->tx_buff, DM_Motor_CAN_Message_Enter, 8);
        CANTransmit(motor->can_instance, 1);
        motor->motor_status = MOTOR_ENABLED;
    }
}

/**
 * @brief 失能电机
 * @param motor 电机实例
 */
void DMMotorDisable(DMMotorInstance *motor)
{
    if (motor != NULL && motor->can_instance != NULL)
    {
        memcpy(motor->can_instance->tx_buff, DM_Motor_CAN_Message_Exit, 8);
        CANTransmit(motor->can_instance, 1);
        motor->motor_status = MOTOR_DISABLED;
    }
}

/**
 * @brief 停止电机
 * @param motor 电机实例
 */
void DMMotorStop(DMMotorInstance *motor)
{
    if (motor != NULL)
    {
        motor->target_position = 0.0f;
        motor->target_velocity = 0.0f;
        motor->target_torque = 0.0f;
        motor->target_current = 0.0f;
    }
}

/**
 * @brief 清除电机错误
 * @param motor 电机实例
 */
void DMMotorClearError(DMMotorInstance *motor)
{
    if (motor != NULL && motor->can_instance != NULL)
    {
        memcpy(motor->can_instance->tx_buff, DM_Motor_CAN_Message_Clear_Error, 8);
        CANTransmit(motor->can_instance, 1);
    }
}

/**
 * @brief 保存零点
 * @param motor 电机实例
 */
void DMMotorSaveZero(DMMotorInstance *motor)
{
    if (motor != NULL && motor->can_instance != NULL)
    {
        memcpy(motor->can_instance->tx_buff, DM_Motor_CAN_Message_Save_Zero, 8);
        CANTransmit(motor->can_instance, 1);
    }
}

/**
 * @brief 获取电机状态
 * @param motor 电机实例
 * @return 电机状态
 */
Motor_Status_e DMMotorGetStatus(DMMotorInstance *motor)
{
    if (motor != NULL)
    {
        return motor->motor_status;
    }
    return MOTOR_ERROR;
}

/**
 * @brief 获取电机位置
 * @param motor 电机实例
 * @return 电机位置
 */
float DMMotorGetPosition(DMMotorInstance *motor)
{
    if (motor != NULL)
    {
        return motor->measure.position;
    }
    return 0.0f;
}

/**
 * @brief 获取电机速度
 * @param motor 电机实例
 * @return 电机速度
 */
float DMMotorGetVelocity(DMMotorInstance *motor)
{
    if (motor != NULL)
    {
        return motor->measure.velocity;
    }
    return 0.0f;
}

/**
 * @brief 获取电机扭矩
 * @param motor 电机实例
 * @return 电机扭矩
 */
float DMMotorGetTorque(DMMotorInstance *motor)
{
    if (motor != NULL)
    {
        return motor->measure.torque;
    }
    return 0.0f;
}

/**
 * @brief 获取电机温度
 * @param motor 电机实例
 * @return 电机温度
 */
float DMMotorGetTemperature(DMMotorInstance *motor)
{
    if (motor != NULL)
    {
        return motor->measure.temperature_mos;
    }
    return 0.0f;
}

/**
 * @brief 获取电机测量数据结构
 * @param motor 电机实例
 * @return 测量数据结构指针
 */
DM_Motor_Measure_s* DMMotorGetMeasure(DMMotorInstance *motor)
{
    if (motor != NULL)
    {
        return &motor->measure;
    }
    return NULL;
}

/**
 * @brief 获取电机电流
 * @param motor 电机实例
 * @return 电机电流
 */
float DMMotorGetCurrent(DMMotorInstance *motor)
{
    if (motor != NULL)
    {
        return motor->measure.current;
    }
    return 0.0f;
}

/**
 * @brief 获取电机转子温度
 * @param motor 电机实例
 * @return 转子温度
 */
float DMMotorGetRotorTemperature(DMMotorInstance *motor)
{
    if (motor != NULL)
    {
        return motor->measure.temperature_rotor;
    }
    return 0.0f;
}

/**
 * @brief 获取电机编码器值
 * @param motor 电机实例
 * @return 编码器值
 */
uint16_t DMMotorGetEncoder(DMMotorInstance *motor)
{
    if (motor != NULL)
    {
        return motor->measure.encoder;
    }
    return 0;
}

/**
 * @brief 获取电机总编码器值
 * @param motor 电机实例
 * @return 总编码器值
 */
int32_t DMMotorGetTotalEncoder(DMMotorInstance *motor)
{
    if (motor != NULL)
    {
        return motor->measure.total_encoder;
    }
    return 0;
}

/**
 * @brief 获取电机总圈数
 * @param motor 电机实例
 * @return 总圈数
 */
int32_t DMMotorGetTotalRound(DMMotorInstance *motor)
{
    if (motor != NULL)
    {
        return motor->measure.total_round;
    }
    return 0;
}