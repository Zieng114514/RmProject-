#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H
#include <stdbool.h>

#include "robot_def.h"



/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 *
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 *
 */
void RobotCMDTask();
typedef struct
{
    uint16_t shoot_speed;
    uint16_t heat_limit;
    uint8_t close_shoot;
    uint8_t gimbal_disenable;
}send_a;

typedef enum
{
    RC_MODE,
    AUTO_MODE,
    STOP_MODE = 2,
}mode_enum;

typedef struct
{
    float remote_right_x; //遥控器右摇杆的水平值
    float remote_right_y; //遥控器右摇杆的垂直值
    float c_type_yaw; //由于阿头是A板控制yaw轴电机且使用C板的陀螺仪进行闭环，所以需要把C板陀螺仪yaw轴的总角度向下传输
    float c_type_speed_angle; //C板的角速度用于yaw轴电机闭环
    float remote_left_x; //遥控器左摇杆的水平值，用于控制yaw轴电机
    float rx;
    float ry;
    float vx;
    float vy;
    bool top_mode;
    float yaw;
    bool tracking;
    friction_mode_e e_e; //摩擦轮是否开启标志
    lid_mode_e sup; //弹仓是否开启标志
    uint8_t version_flag; //自瞄开启的标志
    uint8_t remote_right; //遥控器右边的开关，如果是鼠标操控的话则是ctrl键按下次数与2取余(主要用处是开启小陀螺，使用遥控器时可以急停，具体请看A板底盘控制)
    uint8_t remote_left;
    bool Follow;
} rev_c;
extern mode_enum mode_enum_C;
#endif // !ROBOT_CMD_H