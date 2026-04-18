
#include "sp_vision.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"


static SP_Vision_Recv_s recv_data;
static SP_Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;
uint8_t sp_vision_online = 0;

const uint16_t CRC16_INIT = 0xffff;
const uint16_t CRC16_TABLE[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
  0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
  0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
  0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
  0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
  0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
  0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
  0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
  0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
  0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
  0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
  0x3de3, 0x2c6a, 0x1ef1, 0x0f78};


static uint16_t get_crc16(const uint8_t * data, uint32_t len)
{
  uint16_t crc16 = CRC16_INIT;
  uint8_t byte;
  uint8_t i;

  while (len--) {
    byte = *data++;
    i = (crc16 ^ byte) & 0x00ff;
    crc16 = (crc16 >> 8) ^ CRC16_TABLE[i];
  }

  return crc16;
}

static uint8_t check_crc16(const uint8_t * data, uint32_t len)
{
  uint16_t crc16 = (data[len - 1] << 8) | data[len - 2];
  return get_crc16(data, len - 2) == crc16;
}


void SP_Vision_Set_GimbalAttitude(float yaw, float yaw_vel, float pitch, float pitch_vel, float q[4])
{
    send_data.yaw = yaw;
    send_data.yaw_vel = yaw_vel;
    send_data.pitch = pitch;
    send_data.pitch_vel = pitch_vel;
    send_data.q[0] = q[0];
    send_data.q[1] = q[1];
    send_data.q[2] = q[2];
    send_data.q[3] = q[3];

}

void SP_Vision_Set_BulletSpeed(float speed)
{
    send_data.bullet_speed = speed;
}

void SP_Vision_Set_BulletCount(uint16_t count)
{
    send_data.bullet_count = count;
}

void SP_Vision_Set_Mode(SP_Vision_mode_e mode)
{
    send_data.mode = (uint8_t)mode;
}

void SP_Vision_Set_EnemyColor(uint8_t color)
{
    (void)color;
}

SP_Vision_Recv_s *SP_VisionGetRecvData(void)
{
    return &recv_data;
}

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
    sp_vision_online = 0;
    LOGWARNING("[vision] vision offline!");

}

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

static void DecodeVision(uint16_t recv_len)
{
    sp_vision_online = 1;
    DaemonReload(vision_daemon_instance); // 刷新daemon计数器
    if(recv_len != sizeof(SP_Vision_Recv_s))
    {
        LOGERROR("[vision] recv len err %d", recv_len);
        return;
    }
    if(vis_recv_buff[0] != 'S' || vis_recv_buff[1] != 'P')
    {
        LOGERROR("[vision] recv head error %02X %02X", vis_recv_buff[0], vis_recv_buff[1]);
        return;
    }
    if(!check_crc16(vis_recv_buff, sizeof(SP_Vision_Recv_s)))
    {
        LOGERROR("[vision] recv crc err");
        return;
    }
    memcpy(&recv_data, vis_recv_buff, sizeof(SP_Vision_Recv_s));
    // LOGDEBUG("[vision] recv ok");
}

/* 视觉通信初始化 */
SP_Vision_Recv_s *SP_VisionInit()
{
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);
    send_data.head[0] = 'S';
    send_data.head[1] = 'P';
    send_data.mode = 1;

    return &recv_data;
}

void SP_VisionSend()
{
    static uint8_t send_buffs[2][sizeof(SP_Vision_Send_s)];
    static uint16_t tx_len = sizeof(SP_Vision_Send_s);
    static uint8_t active_buf_idx = 0;
    uint8_t next_buf_idx = active_buf_idx ^ 1u;
    SP_Vision_Send_s frame;

    __disable_irq();
    frame = send_data;
    __enable_irq();

    frame.crc16 = get_crc16((uint8_t *)&frame, sizeof(SP_Vision_Send_s) - 2);
    memcpy(send_buffs[next_buf_idx], &frame, sizeof(SP_Vision_Send_s));

    if (USBTransmit(send_buffs[next_buf_idx], tx_len) == USBD_OK)
    {
        active_buf_idx = next_buf_idx;
    }
}

#endif // VISION_USE_VCP
