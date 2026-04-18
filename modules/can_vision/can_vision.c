/**
 ******************************************************************************
 * @file    can_vision.c
 * @brief   CAN视觉通信模块实现
 *          发送IMU四元数数据，接收上位机控制命令
 ******************************************************************************
 */
#include "can_vision.h"
#include "QuaternionEKF.h"
#include "can.h"
#include "bsp_log.h"
#include "stddef.h"
#include "message_center.h"
#include "math.h"

/* CAN实例指针 */
static CANInstance *can_vision_instance = NULL;

/* 接收到的命令数据 */
static CanVisionCommand_t vision_command;
static uint8_t command_received = 0; // 标志位，表示是否收到新命令

/* 消息发布者，用于发布上位机命令 */
static Publisher_t *vision_cmd_pub = NULL;

static float can_vision_bullet_speed_mps = 25.0f;
static uint8_t can_vision_mode = 1;
static uint8_t can_vision_shoot_mode = 0;
static float can_vision_ft_angle_rad = 0.0f;

static void normalize_quat(float *w, float *x, float *y, float *z)
{
    float n = sqrtf((*w) * (*w) + (*x) * (*x) + (*y) * (*y) + (*z) * (*z));
    if (n > 0.0f)
    {
        *w /= n;
        *x /= n;
        *y /= n;
        *z /= n;
    }
}

void CanVisionSetBulletSpeed(float speed_mps)
{
    can_vision_bullet_speed_mps = speed_mps;
}

/**
 * @brief CAN接收回调函数
 * @param instance CAN实例指针
 */
static void CanVisionRxCallback(CANInstance *instance)
{
    if (instance->rx_id == CAN_VISION_COMMAND_ID && instance->rx_len == 8)
    {
        // 解析控制标志位
        vision_command.control = instance->rx_buff[0];
        vision_command.shoot = instance->rx_buff[1];

        // 解析偏航角度（int16_t，高字节在前，除以1e4得到浮点数）
        int16_t yaw_int = (int16_t)((instance->rx_buff[2] << 8) | instance->rx_buff[3]);
        vision_command.yaw = (float)yaw_int / 1e4f;

        // 解析俯仰角度
        int16_t pitch_int = (int16_t)((instance->rx_buff[4] << 8) | instance->rx_buff[5]);
        vision_command.pitch = (float)pitch_int / 1e4f;

        // 解析水平距离
        int16_t distance_int = (int16_t)((instance->rx_buff[6] << 8) | instance->rx_buff[7]);
        vision_command.horizon_distance = (float)distance_int / 1e4f;

        command_received = 1; // 标记收到新命令
        
        // 发布命令到消息中心
        if (vision_cmd_pub != NULL)
        {
            PubPushMessage(vision_cmd_pub, (void*)&vision_command);
        }
    }
}

/**
 * @brief 初始化CAN视觉通信模块
 */
void CanVisionInit(void)
{
    // CAN初始化配置
    CAN_Init_Config_s can_config = {
        .can_handle = &hcan2,              // 使用CAN2，避免与双板通信冲突
        .tx_id = CAN_VISION_QUATERNION_ID,  // 发送ID：四元数
        .rx_id = CAN_VISION_COMMAND_ID,     // 接收ID：命令
        .can_module_callback = CanVisionRxCallback,
        .id = NULL,                         // 不需要模块ID
    };

    // 注册CAN实例
    can_vision_instance = CANRegister(&can_config);
    
    // 设置发送数据长度为8字节
    CANSetDLC(can_vision_instance, 8);

    // 初始化命令数据
    vision_command.control = 0;
    vision_command.shoot = 0;
    vision_command.yaw = 0.0f;
    vision_command.pitch = 0.0f;
    vision_command.horizon_distance = 0.0f;
    command_received = 0;

    // 注册消息发布者，发布上位机命令
    vision_cmd_pub = PubRegister("vision_cmd", sizeof(CanVisionCommand_t));

  //  LOGINFO("[can_vision] CAN Vision Module Initialized");
}

/**
 * @brief CAN视觉通信任务函数
 * @note 用于定期发送四元数数据，建议在100Hz任务中调用
 */
void CanVisionTask(void)
{
    if (can_vision_instance == NULL)
    {
        return; // 未初始化，直接返回
    }

    float q_w = QEKF_INS.q[0];
    float q_x = QEKF_INS.q[1];
    float q_y = QEKF_INS.q[2];
    float q_z = QEKF_INS.q[3];
    normalize_quat(&q_w, &q_x, &q_y, &q_z);

    int16_t x_int = (int16_t)(q_x * 1e4f);
    int16_t y_int = (int16_t)(q_y * 1e4f);
    int16_t z_int = (int16_t)(q_z * 1e4f);
    int16_t w_int = (int16_t)(q_w * 1e4f);

    // 打包数据到CAN发送缓冲区（大端序，高字节在前）
    can_vision_instance->tx_buff[0] = (uint8_t)(x_int >> 8);
    can_vision_instance->tx_buff[1] = (uint8_t)(x_int & 0xFF);
    can_vision_instance->tx_buff[2] = (uint8_t)(y_int >> 8);
    can_vision_instance->tx_buff[3] = (uint8_t)(y_int & 0xFF);
    can_vision_instance->tx_buff[4] = (uint8_t)(z_int >> 8);
    can_vision_instance->tx_buff[5] = (uint8_t)(z_int & 0xFF);
    can_vision_instance->tx_buff[6] = (uint8_t)(w_int >> 8);
    can_vision_instance->tx_buff[7] = (uint8_t)(w_int & 0xFF);

    // 发送CAN数据，超时时间1ms
    can_vision_instance->txconf.StdId = CAN_VISION_QUATERNION_ID;
    CANTransmit(can_vision_instance, 1.0f);

    int16_t bullet_speed_int = (int16_t)(can_vision_bullet_speed_mps * 1e2f);
    int16_t ft_angle_int = (int16_t)(can_vision_ft_angle_rad * 1e4f);
    can_vision_instance->tx_buff[0] = (uint8_t)(bullet_speed_int >> 8);
    can_vision_instance->tx_buff[1] = (uint8_t)(bullet_speed_int & 0xFF);
    can_vision_instance->tx_buff[2] = can_vision_mode;
    can_vision_instance->tx_buff[3] = can_vision_shoot_mode;
    can_vision_instance->tx_buff[4] = (uint8_t)(ft_angle_int >> 8);
    can_vision_instance->tx_buff[5] = (uint8_t)(ft_angle_int & 0xFF);
    can_vision_instance->tx_buff[6] = 0;
    can_vision_instance->tx_buff[7] = 0;
    can_vision_instance->txconf.StdId = CAN_VISION_BULLET_SPEED_ID;
    CANTransmit(can_vision_instance, 1.0f);
}

/**
 * @brief 获取最新的上位机命令数据
 * @return 返回命令数据结构体指针，如果未收到数据则返回NULL
 */
CanVisionCommand_t *CanVisionGetCommand(void)
{
    if (command_received)
    {
        command_received = 0; // 清除标志位
        return &vision_command;
    }
    return NULL;
}

void CanVisionSetMode(uint8_t mode)
{
    can_vision_mode = mode;
}

void CanVisionSetShootMode(uint8_t shoot_mode)
{
    can_vision_shoot_mode = shoot_mode;
}

void CanVisionSetFeedforwardAngle(float ft_angle_rad)
{
    can_vision_ft_angle_rad = ft_angle_rad;
}
