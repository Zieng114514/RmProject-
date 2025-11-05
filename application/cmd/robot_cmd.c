// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回


static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;

// 无人机云台控制标志



void RobotCMDInit()
{
   rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个


    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    
    // 初始化云台目标角度为0，实际角度会在RobotCMDTask首次运行时同步
    gimbal_cmd_send.yaw = 0.0f;
    gimbal_cmd_send.pitch = 0.0f;
    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
    
    // 初始化发射控制参数
    shoot_cmd_send.shoot_mode = SHOOT_ON; // 启用发射模式
    
    // 初始化CAN视觉通信模块

}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

static void RemoteControlSet()
{
    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],底盘跟随云台
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;

    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        gimbal_cmd_send.yaw += 0.005f * (float)rc_data[TEMP].rc.rocker_l_;
        gimbal_cmd_send.pitch += 0.0005f * (float)rc_data[TEMP].rc.rocker_l1;
        LIMIT_MIN_MAX(gimbal_cmd_send.pitch, pitch_limit_up, pitch_limit_down);
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;

    }
        ; // 弹舱舵机控制,待添加servo_motor模块,关闭

    // 摩擦轮和拨弹控制,使用左侧三段拨杆
    if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧拨杆中位,开启摩擦轮
    {
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 左侧拨杆低位,保持摩擦轮开启并连发
    {
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    }
    else // 左侧拨杆高位或其他情况,关闭摩擦轮和拨弹
    {
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
    // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
    shoot_cmd_send.shoot_rate = 8;
}


/**
 * @brief 无人机云台核心控制任务，200Hz频率运行
 * @note  简化版本：只使用遥控器控制，暂不支持键鼠和视觉
 */
void RobotCMDTask()
{
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data); // gimbal发布者获取遥控器数据
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data); //shoot发布者获取遥控器数据
    
    // if (switch_is_up(c.remote_left))
    // {
    //    // AutoControlSet();
    //
    // }
    // else if (switch_is_mid(c.remote_left))
    // {
        RemoteControlSet();
    // }
    // else
    // {

        //  MouseKeySet();
    //}
   // union_data();


    //HAL_UART_Transmit(&huart1,send_vision,4,10);//
    PubPushMessage(shoot_cmd_pub, (void*)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void*)&gimbal_cmd_send);
}
