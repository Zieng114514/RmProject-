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
#include "elrs/elrs.h"
#include "sp_vision/sp_vision.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"


/*
    空中无人机robot_cmd.c说明
    --------------------------------------------------
    该任务负责整个机器人的遥控器输入操作命令处理

    实现了SBUS和ELRS接收机的兼容
    如果使用SBUS接收机只需要去robot_def中修改
    如果使用ELRS接收机则

    //#define UART1_NORMAL
    #define USE_CRSF
    //#define USE_SBUS

    默认ELRS接收机接到了UART1上 如果接到其他接口需要修改串口波特率为

    huart1.Init.BaudRate = 420000; // ELRS 正常波特率
    huart1.Init.BaudRate = 115200; // 示例波特率

    如果需要使用官方的DT7或者SBUS则使用

    #define UART1_NORMAL
    //#define USE_CRSF
    #define USE_SBUS

    如果UART1上需要接其他外设则

    #define UART1_NORMAL
    //#define USE_CRSF
    #define USE_SBUS

*/



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

#if defined USE_SBUS
static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
 #elif defined USE_CRSF
static ELRS_Data *elrs_data_ptr; // ELRS数据指针
#endif


static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

// 视觉命令缓存（sp_vision协议）
static SP_Vision_Recv_s *vision_cmd_recv = NULL; // 上位机视觉命令缓存

// 视觉单发控制节流/保持时间
#define VISION_SHOT_INTERVAL_MS 80u           // 两次单发的最小间隔(毫秒)
#define VISION_SINGLE_SHOT_HOLD_MS 80u        // 单发命令保持时间(毫秒)，确保角度环执行完成
static uint32_t vision_last_shot_ms = 0;       // 上一次触发单发的时间戳
static uint32_t vision_single_shot_until = 0;  // 当前单发保持截止时间戳
static uint8_t  vision_single_shot_active = 0; // 是否处于单发执行期
 


BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;

// 无人机云台控制标志



void RobotCMDInit()
{

#if defined USE_SBUS
    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#elif defined USE_CRSF
    elrs_data_ptr = ELRS_Init(&huart1); // 初始化ELRS并获取数据指针
#endif



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
    
    // 初始化sp_vision视觉通信模块接收缓存
    vision_cmd_recv = SP_VisionGetRecvData();
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

#if defined USE_SBUS


    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],底盘跟随云台
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;

    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        gimbal_cmd_send.yaw += 0.0005f * (float)rc_data[TEMP].rc.rocker_l_;
        gimbal_cmd_send.pitch += 0.0005f * (float)rc_data[TEMP].rc.rocker_l1;
        LIMIT_MIN_MAX(gimbal_cmd_send.pitch, pitch_limit_up, pitch_limit_down);
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],底盘和云台分离,底盘保持不转动
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;

    }
        ; // 弹舱舵机控制,待添加servo_motor模块,关闭

    // 摩擦轮开关由右侧三段拨杆控制：中/下=开启，其余=关闭
    if (switch_is_down(rc_data[TEMP].rc.switch_right) || switch_is_mid(rc_data[TEMP].rc.switch_right))
        shoot_cmd_send.friction_mode = FRICTION_ON;
    else
        shoot_cmd_send.friction_mode = FRICTION_OFF;

    // 拨弹逻辑仍由左侧拨杆控制发射模式（与摩擦轮解耦）
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 左侧拨杆低位 -> 连发
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    else
        shoot_cmd_send.load_mode = LOAD_STOP; // 其他情况不转动拨盘
    // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
    shoot_cmd_send.shoot_rate = 8;
#elif defined USE_CRSF

#endif
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

    // 视觉开火单发逻辑：
    // 在限定的最小间隔内仅触发一次，将拨盘以角度环+45°实现单发；
    // 触发后保持角度环命令一段时间，避免下一周期被立即STOP覆盖。
    {
        const uint32_t now_ms = DWT_GetTimeline_ms();
        const uint8_t allow_vision = (shoot_cmd_send.friction_mode == FRICTION_ON) &&
                                     (gimbal_cmd_send.gimbal_mode == GIMBAL_FREE_MODE ||
                                      gimbal_cmd_send.gimbal_mode == GIMBAL_GYRO_MODE) &&
                                     sp_vision_online &&
                                     (vision_cmd_recv != NULL);
        const uint8_t vision_fire = allow_vision && (vision_cmd_recv->mode == 2u);

        if (vision_single_shot_active) {
            if (!allow_vision) {
                vision_single_shot_active = 0; // 不允许视觉时立即取消保持，避免误转动
            }
            else if (now_ms < vision_single_shot_until) {
                shoot_cmd_send.load_mode = LOAD_1_BULLET; // 保持角度环
            } else {
                vision_single_shot_active = 0;
            }
        }

        if (!vision_single_shot_active && vision_fire) {
            if (now_ms - vision_last_shot_ms >= VISION_SHOT_INTERVAL_MS) {
                vision_single_shot_active = 1;
                vision_last_shot_ms = now_ms;
                vision_single_shot_until = now_ms + VISION_SINGLE_SHOT_HOLD_MS;
                shoot_cmd_send.load_mode = LOAD_1_BULLET; // 触发单发
            }
        }

        // 若未由视觉触发单发且无其他发射请求，维持原有RC逻辑的load_mode；
        // 这里不强制改为STOP，避免覆盖RC的连发/反转等模式。
    }

    //HAL_UART_Transmit(&huart1,send_vision,4,10);//
    PubPushMessage(shoot_cmd_pub, (void*)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void*)&gimbal_cmd_send);
}
