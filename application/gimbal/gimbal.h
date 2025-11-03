#ifndef GIMBAL_H
#define GIMBAL_H

/**
 * @brief 初始化云台,会被RobotInit()调用
 *
 * 注意：云台控制配置（灵敏度、死区、模式绑定等）在 robot_cmd.h 中定义
 *       可通过 RobotCMDGetGimbalConfig() 获取配置进行修改
 */
void GimbalInit();

/**
 * @brief 云台任务
 *
 */
void GimbalTask();

#endif // GIMBAL_H