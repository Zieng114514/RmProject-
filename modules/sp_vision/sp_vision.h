#ifndef SP_VISION_H
#define SP_VISION_H

#include "stdint.h"

#include "bsp_usart.h"

#pragma pack(1)

//define帧头 head[2]={'S','P'}
#define SP_VISION_FRAME_HEADER   {'S','P'}

typedef enum
{
	SP_VISION_MODE_IDLE, // 空闲
	SP_VISION_MODE_AIM,  // 自瞄
	SP_VISION_MODE_SMALL_SIG, // 小符
	SP_VISION_MODE_BIG_SIG,   // 大符
} SP_Vision_mode_e;


typedef struct
{
	uint8_t head[2];
	uint8_t mode; // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
	float yaw;
	float yaw_vel;
	float yaw_acc;
	float pitch;
	float pitch_vel;
	float pitch_acc;
	uint16_t crc16;

} SP_Vision_Recv_s;

typedef struct
{
	uint8_t head[2];
	uint8_t mode; // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
	float q[4];	  // wxyz顺序
	float yaw;
	float yaw_vel;
	float pitch;
	float pitch_vel;
	float bullet_speed;
	uint16_t bullet_count; // 子弹累计发送次数
	uint16_t crc16;
} SP_Vision_Send_s;


#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
SP_Vision_Recv_s *SP_VisionInit();

/**
 * @brief 获取最近一次收到的视觉命令结构体指针
 *
 * @note 该接口不会触发初始化，仅返回模块内部接收缓存地址。
 */
SP_Vision_Recv_s *SP_VisionGetRecvData(void);

/**
 * @brief 发送视觉数据
 *
 */
void SP_VisionSend();

void SP_Vision_Set_GimbalAttitude(float yaw, float yaw_vel, float pitch, float pitch_vel, float q[4]);

void SP_Vision_Set_BulletSpeed(float speed);

void SP_Vision_Set_BulletCount(uint16_t count);

void SP_Vision_Set_EnemyColor(uint8_t color);

void SP_Vision_Set_Mode(SP_Vision_mode_e mode);

extern uint8_t sp_vision_online;

#endif // !SP_VISION_H
