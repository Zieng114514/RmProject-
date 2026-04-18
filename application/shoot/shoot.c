#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "can_vision/can_vision.h"


/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l;
static DJIMotorInstance *friction_r;
static DJIMotorInstance *loader; // 拨盘电机
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息
static Shoot_Ctrl_Cmd_s shoot_cmd_send ;

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

// 摩擦轮缓启动参数
#define FRICTION_TARGET_SPEED 34000      // 摩擦轮目标速度
#define FRICTION_RAMP_RATE 200.0f        // 摩擦轮缓启动加速度(每个任务周期增加的速度值)
#define FRICTION_STOP_RATE 500.0f        // 摩擦轮急停加速度(比缓启动更快)

static float current_friction_ref_l = 0; // 左摩擦轮当前设定值
static float current_friction_ref_r = 0; // 右摩擦轮当前设定值

static float shoot_bullet_speed_mps = 19.40f;

// 卡弹检测参数




// 卡弹检测状态机
typedef enum {
    JAM_NORMAL,     // 正常状态
    JAM_DETECTING,  // 检测到可能卡弹
    JAM_REVERSING   // 正在反转
} JamState_e;

static JamState_e jam_state = JAM_NORMAL;
static uint32_t jam_detect_start_time = 0;
static uint32_t jam_reverse_start_time = 0;



void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20, // 20
                .Ki = 3, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 40000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 40000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 2,

    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 3;
   // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 4,

        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 688, // 10
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 10000,
            },
            .speed_PID = {
                .Kp = 10, // 10
                .Ki = 0.5, // 1
                .Kd = 0.00,
                .Improve = PID_Integral_Limit ,
                .IntegralLimit = 5000,
                .MaxOut = 16000,
            },
            .current_PID = {
                .Kp = 0.5, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 16000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    CanVisionSetBulletSpeed(shoot_bullet_speed_mps);
}

/* 卡弹检测与处理函数 */
static void FrictionQuickStop(void)
{
    // 使用更快的减速度停止摩擦轮
    float stop_rate = FRICTION_STOP_RATE * 2.0f;

    if (current_friction_ref_l > 0) {
        current_friction_ref_l -= stop_rate;
        if (current_friction_ref_l < 0) current_friction_ref_l = 0;
    } else if (current_friction_ref_l < 0) {
        current_friction_ref_l += stop_rate;
        if (current_friction_ref_l > 0) current_friction_ref_l = 0;
    }

    if (current_friction_ref_r > 0) {
        current_friction_ref_r -= stop_rate;
        if (current_friction_ref_r < 0) current_friction_ref_r = 0;
    } else if (current_friction_ref_r < 0) {
        current_friction_ref_r += stop_rate;
        if (current_friction_ref_r > 0) current_friction_ref_r = 0;
    }

    // 设置摩擦轮电机参考值
    DJIMotorSetRef(friction_l, current_friction_ref_l);
    DJIMotorSetRef(friction_r, current_friction_ref_r);

    // 如果速度已经接近0，直接设置为0
    if (fabsf(current_friction_ref_l) < 50.0f && fabsf(current_friction_ref_r) < 50.0f) {
        current_friction_ref_l = 0;
        current_friction_ref_r = 0;
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);

        current_friction_ref_l = 0;
        current_friction_ref_r = 0;
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }

    // 拨弹盘控制逻辑
    switch (shoot_cmd_recv.load_mode)
    {
        // 停止拨盘
        case LOAD_STOP:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, 0);
            break;

        // 单发模式
        case LOAD_1_BULLET:
            DJIMotorOuterLoop(loader, ANGLE_LOOP);
            DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE);
            hibernate_time = DWT_GetTimeline_ms();
            dead_time = 150;
            break;

        // 三连发
        case LOAD_3_BULLET:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, 40);
            break;

        // 连发模式
        case LOAD_BURSTFIRE:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            // 正常发射用负数速度（假设负数是正转发射方向）
            DJIMotorSetRef(loader,11000);
            break;

        // 反转模式（卡弹处理）
        case LOAD_REVERSE:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            // 反转用正数速度（与正常发射方向相反）
            DJIMotorSetRef(loader, 6000);
            break;

        default:
            break;
    }

    // 确定是否开启摩擦轮
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        float target_friction_speed = FRICTION_TARGET_SPEED;

        // 摩擦轮缓启动: 线性插值逐渐增加到目标速度
        if (current_friction_ref_l < target_friction_speed)
        {
            current_friction_ref_l += FRICTION_RAMP_RATE;
            if (current_friction_ref_l > target_friction_speed)
                current_friction_ref_l = target_friction_speed;
        }
        else if (current_friction_ref_l > target_friction_speed)
        {
            current_friction_ref_l -= FRICTION_RAMP_RATE;
            if (current_friction_ref_l < target_friction_speed)
                current_friction_ref_l = target_friction_speed;
        }

        if (current_friction_ref_r < target_friction_speed)
        {
            current_friction_ref_r += FRICTION_RAMP_RATE;
            if (current_friction_ref_r > target_friction_speed)
                current_friction_ref_r = target_friction_speed;
        }
        else if (current_friction_ref_r > target_friction_speed)
        {
            current_friction_ref_r -= FRICTION_RAMP_RATE;
            if (current_friction_ref_r < target_friction_speed)
                current_friction_ref_r = target_friction_speed;
        }

        // 设置摩擦轮电机参考值
        DJIMotorSetRef(friction_l, current_friction_ref_l);
        DJIMotorSetRef(friction_r, current_friction_ref_r);
    }
    else // 关闭摩擦轮
    {
        FrictionQuickStop();
    }

    // 更新反馈数据
    shoot_feedback_data.friction_l_speed_aps = friction_l->measure.speed_aps;
    shoot_feedback_data.friction_r_speed_aps = friction_r->measure.speed_aps;
    shoot_feedback_data.friction_l_real_current = friction_l->measure.real_current;
    shoot_feedback_data.friction_r_real_current = friction_r->measure.real_current;
    shoot_feedback_data.loader_speed_aps = loader->measure.speed_aps;
    shoot_feedback_data.loader_real_current = loader->measure.real_current;

    // 反馈数据
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}


