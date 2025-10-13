// 基础板级支持包初始化，完成时钟、外设等底层初始化
#include "bsp_init.h"
// 机器人应用层入口与调度
#include "robot.h"
// 机器人机体物理/装配参数与宏定义（底盘、云台尺寸偏置、轮距等）
#include "robot_def.h"
// 系统任务创建/调度入口（FreeRTOS 任务）
#include "robot_task.h"

// 编译期提醒：未正确配置 `robot_def.h` 会导致运行期严重错误
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !ROBOT_DEF_PARAM_WARNING

// 当目标板包含底盘控制能力（单板或底盘板）时引入底盘应用
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
#include "chassis.h"
#endif

// 当目标板包含云台/发射控制能力（单板或云台板）时引入云台、发射与指令分发
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
#include "gimbal.h"
#include "shoot.h"
#include "robot_cmd.h"
#endif


// 机器人应用初始化：仅在系统启动时调用一次
void RobotInit()
{  
    // 1) 关闭全局中断，避免初始化过程中被抢占（例如外设中断访问未完成初始化的数据结构）
    // 注意：初始化阶段禁止使用会依赖 tick 或中断的延时；如确需延时，仅允许使用 DWT_Delay()
    __disable_irq();

    // 2) 板级外设初始化（时钟、GPIO、DMA、CAN/UART/SPI/I2C、定时器、USB、DWT、日志等）
    BSPInit();

    // 3) 应用子系统初始化（按硬件编译宏划分职责）
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    // 指令分发模块：负责接收遥控器/上位机/视觉等源的控制命令并发布给各应用
    RobotCMDInit();
    // 云台子系统：本项目配置为 yaw=GM6020（DJI 闭环）、pitch=DM4310（力控开环）
    GimbalInit();
    // 发射子系统：如需启用，请打开 ShootInit()
    // ShootInit();
#endif

    // 底盘子系统：负责底盘运动学解算与功率受限输出
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisInit();
#endif

    // 4) 创建系统基础任务（调度各模块周期性逻辑，如电机控制/功率管理等）
    OSTaskInit();

    // 5) 初始化完成，重新开启全局中断
    __enable_irq();
}

// 机器人应用主任务：由调度器周期性调用，串联各子系统的核心任务
void RobotTask()
{
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    // 指令分发核心任务：消费外部输入并发布到各子系统
    RobotCMDTask();
    // 云台核心任务：根据模式（陀螺/自由）与期望值驱动 yaw/pitch
    GimbalTask();
    // 发射核心任务：若启用发射功能，则开启
    // ShootTask();
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    // 底盘核心任务：订阅控制命令、进行坐标变换与麦轮解算、功率限制与输出
    ChassisTask();
#endif
}