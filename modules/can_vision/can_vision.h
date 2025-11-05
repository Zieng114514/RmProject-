/**
 ******************************************************************************
 * @file    can_vision.h
 * @brief   CAN视觉通信模块，用于与上位机进行CAN通信
 *          发送IMU四元数数据（ID: 0x001），接收上位机控制命令（ID: 0xff）
 ******************************************************************************
 */
#ifndef CAN_VISION_H
#define CAN_VISION_H

#include "stdint.h"
#include "bsp_can.h"

/* CAN ID定义 */
#define CAN_VISION_QUATERNION_ID 0x001  // 四元数发送ID
#define CAN_VISION_COMMAND_ID 0xff      // 命令接收ID

/**
 * @brief 上位机命令数据结构体
 *        对应上位机Command结构，用于接收控制指令
 */
typedef struct
{
    uint8_t control;           // 控制标志位，1表示开启控制，0表示关闭
    uint8_t shoot;            // 射击标志位，1表示射击，0表示不射击
    float yaw;                // 偏航角度（弧度）
    float pitch;              // 俯仰角度（弧度）
    float horizon_distance;   // 水平距离（米）
} CanVisionCommand_t;

/**
 * @brief 初始化CAN视觉通信模块
 * @note 需要在RobotCMDInit()中调用
 */
void CanVisionInit(void);

/**
 * @brief CAN视觉通信任务函数
 * @note 用于定期发送四元数数据，建议在100Hz任务中调用
 */
void CanVisionTask(void);

/**
 * @brief 获取最新的上位机命令数据
 * @return 返回命令数据结构体指针，如果未收到数据则返回NULL
 */
CanVisionCommand_t *CanVisionGetCommand(void);

#endif // CAN_VISION_H

