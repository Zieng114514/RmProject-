/**
 * @file example_usage.c
 * @brief DM电机使用示例 - 展示如何使用新的measure结构体
 * @author Enhanced from DMPower.c
 * @version 2.0
 * @date 2024-12-24
 */

#include "dmmotor.h"

/**
 * @brief DM电机使用示例
 * 展示如何使用新的measure结构体获取电机数据
 */
void DMMotorExampleUsage()
{
    // 假设已经初始化了电机
    DMMotorInstance *motor = NULL; // 这里应该是通过DMMotorInit()初始化的电机实例
    
    if (motor != NULL)
    {
        // 方法1: 使用传统的获取函数
        float position = DMMotorGetPosition(motor);
        float velocity = DMMotorGetVelocity(motor);
        float torque = DMMotorGetTorque(motor);
        float temperature = DMMotorGetTemperature(motor);
        
        // 方法2: 直接访问measure结构体（推荐）
        DM_Motor_Measure_s *measure = DMMotorGetMeasure(motor);
        if (measure != NULL)
        {
            // 访问编码器相关数据
            uint16_t encoder = measure->encoder;
            uint16_t last_encoder = measure->last_encoder;
            int32_t total_encoder = measure->total_encoder;
            int32_t total_round = measure->total_round;
            
            // 访问位置和角度数据
            float position = measure->position;
            float angle_single_round = measure->angle_single_round;
            
            // 访问速度和扭矩数据
            float velocity = measure->velocity;
            float torque = measure->torque;
            float current = measure->current;
            
            // 访问温度数据
            float temperature_mos = measure->temperature_mos;
            float temperature_rotor = measure->temperature_rotor;
            
            // 访问时间相关数据
            uint32_t feed_cnt = measure->feed_cnt;
            float dt = measure->dt;
        }
        
        // 方法3: 使用新的专用获取函数
        float current = DMMotorGetCurrent(motor);
        float rotor_temp = DMMotorGetRotorTemperature(motor);
        uint16_t encoder = DMMotorGetEncoder(motor);
        int32_t total_encoder = DMMotorGetTotalEncoder(motor);
        int32_t total_round = DMMotorGetTotalRound(motor);
    }
}

/**
 * @brief 电机数据监控示例
 * 展示如何监控电机的各种状态
 */
void DMMotorMonitorExample()
{
    DMMotorInstance *motor = NULL; // 这里应该是通过DMMotorInit()初始化的电机实例
    
    if (motor != NULL)
    {
        DM_Motor_Measure_s *measure = DMMotorGetMeasure(motor);
        if (measure != NULL)
        {
            // 检查电机状态
            Motor_Status_e status = DMMotorGetStatus(motor);
            
            if (status == MOTOR_ONLINE)
            {
                // 电机在线，可以安全访问数据
                float position = measure->position;
                float velocity = measure->velocity;
                float torque = measure->torque;
                
                // 检查温度是否过高
                if (measure->temperature_mos > 80.0f) // 80度阈值
                {
                    // 电机过热，采取保护措施
                    DMMotorStop(motor);
                }
                
                // 检查编码器是否正常
                if (measure->encoder == measure->last_encoder)
                {
                    // 编码器值没有变化，可能存在问题
                }
            }
            else if (status == MOTOR_OFFLINE)
            {
                // 电机离线，尝试重新连接
                DMMotorEnable(motor);
            }
            else if (status == MOTOR_ERROR)
            {
                // 电机错误，清除错误
                DMMotorClearError(motor);
            }
        }
    }
}