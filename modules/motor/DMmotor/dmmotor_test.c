/**
 * @file dmmotor_test.c
 * @brief DM电机数据解析测试函数
 * @author Enhanced from DMPower.c
 * @version 2.0
 * @date 2024-12-24
 */

#include "dmmotor.h"
#include "stdio.h"

/**
 * @brief 测试DM电机数据解析
 * 使用您提供的实际CAN数据进行测试
 */
void DMMotorTestDataParsing()
{
    // 模拟您的CAN数据: 18 158 175 127 231 255 31 30
    uint8_t test_rx_data[8] = {18, 158, 175, 127, 231, 255, 31, 30};
    
    printf("=== DM Motor Data Parsing Test ===\n");
    printf("Raw CAN Data: ");
    for (int i = 0; i < 8; i++) {
        printf("%d ", test_rx_data[i]);
    }
    printf("\n");
    
    // 解析数据
    uint16_t tmp_encoder = (test_rx_data[1] << 8) | test_rx_data[2];
    uint16_t tmp_omega = (test_rx_data[3] << 4) | (test_rx_data[4] >> 4);
    uint16_t tmp_torque = ((test_rx_data[4] & 0x0f) << 8) | test_rx_data[5];
    
    printf("Parsed Data:\n");
    printf("  ID: %d\n", test_rx_data[0]);
    printf("  Encoder: %d (0x%04X)\n", tmp_encoder, tmp_encoder);
    printf("  Omega: %d (0x%04X)\n", tmp_omega, tmp_omega);
    printf("  Torque: %d (0x%04X)\n", tmp_torque, tmp_torque);
    printf("  Temp_MOS: %d\n", test_rx_data[6]);
    printf("  Temp_Rotor: %d\n", test_rx_data[7]);
    
    // 测试数据转换
    float velocity_max = 30.0f;
    float torque_max = 10.0f;
    
    // 方法1: 使用标准转换
    float velocity1 = ((float)tmp_omega) * (2.0f * velocity_max) / ((float)((1 << 12) - 1)) - velocity_max;
    float torque1 = ((float)tmp_torque) * (2.0f * torque_max) / ((float)((1 << 12) - 1)) - torque_max;
    
    // 方法2: 直接比例转换
    float velocity2 = (float)tmp_omega * 0.01f;
    float torque2 = (float)tmp_torque * 0.01f;
    
    printf("\nData Conversion Test:\n");
    printf("  Velocity (Method 1): %.4f\n", velocity1);
    printf("  Velocity (Method 2): %.4f\n", velocity2);
    printf("  Torque (Method 1): %.4f\n", torque1);
    printf("  Torque (Method 2): %.4f\n", torque2);
    
    // 位置计算测试
    int32_t total_encoder = tmp_encoder;
    float position = (float)total_encoder / (float)((1 << 16) - 1) * 12.5f * 2.0f;
    printf("  Position: %.4f\n", position);
    
    printf("=== Test Complete ===\n");
}

/**
 * @brief 在gimbal.c中调用的测试函数
 * 可以在GimbalTask()中调用此函数来测试数据解析
 */
void DMMotorTestInGimbal(DMMotorInstance *pitch_motor)
{
    if (pitch_motor == NULL) {
        printf("DM Motor Test: pitch_motor is NULL\n");
        return;
    }
    
    printf("=== DM Motor Status Test ===\n");
    printf("Motor Status: %d\n", DMMotorGetStatus(pitch_motor));
    
    DM_Motor_Measure_s *measure = DMMotorGetMeasure(pitch_motor);
    if (measure != NULL) {
        printf("Measure Data:\n");
        printf("  Position: %.4f\n", measure->position);
        printf("  Velocity: %.4f\n", measure->velocity);
        printf("  Torque: %.4f\n", measure->torque);
        printf("  Temp_MOS: %.1f\n", measure->temperature_mos);
        printf("  Temp_Rotor: %.1f\n", measure->temperature_rotor);
        printf("  Encoder: %d\n", measure->encoder);
        printf("  Total_Encoder: %d\n", measure->total_encoder);
    } else {
        printf("Measure data is NULL\n");
    }
    
    // 调用调试函数
    DMMotorDebugPrint(pitch_motor);
}
