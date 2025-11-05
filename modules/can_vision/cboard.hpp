#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "io/command.hpp"
#include "io/socketcan.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
/**
 * @brief 模式枚举类
 * 
 * 定义了游戏中可能存在的不同模式状态
 */
enum Mode
{
  idle,        // 空闲模式，无特殊操作状态
  auto_aim,    // 自动瞄准模式，可能用于辅助瞄准功能
  small_buff,  // 小buff增益模式，获取小型增益效果
  big_buff,    // 大buff增益模式，获取大型增益效果
  outpost      // 前哨站模式，可能与占领或控制据点相关
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

// 哨兵专有
/**
 * @brief 射击模式枚举类
 * 
 * 该枚举类定义了不同的射击模式，用于控制射击行为。
 */
enum ShootMode
{
  left_shoot,  // 仅左侧射击
  right_shoot, // 仅右侧射击
  both_shoot   // 左右两侧同时射击
};
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

/**
 * @brief CBoard类，用于管理无人机控制板的相关功能
 * 
 * 该类包含无人机控制所需的参数、通信接口和数据处理方法
 */
class CBoard
{
public:
  double bullet_speed;    // 弹道速度参数，用于控制射击速度
  Mode mode;              // 工作模式枚举，定义无人机当前的工作状态
  ShootMode shoot_mode;   // 射击模式枚举，定义射击方式
  double ft_angle;  //无人机专有角度参数，可能是俯仰角或其他姿态角

  /**
   * @brief 构造函数，初始化控制板
   * @param config_path 配置文件路径，用于加载初始参数
   */
  CBoard(const std::string & config_path);

  /**
   * @brief 获取指定时间点的IMU姿态四元数
   * @param timestamp 需要查询的时间点
   * @return 返回该时间点的姿态四元数
   */
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  /**
   * @brief 发送命令到控制板
   * @param command 要发送的命令结构体
   */
  void send(Command command) const;

private:
  /**
   * @brief IMU数据结构体，存储姿态四元数和时间戳
   */
  struct IMUData
  {
    Eigen::Quaterniond q;  // 姿态四元数
    std::chrono::steady_clock::time_point timestamp;  // 数据采集时间点
  };

  tools::ThreadSafeQueue<IMUData> queue_;  // 必须在can_之前初始化，否则存在死锁的可能
  SocketCAN can_;
  IMUData data_ahead_;
  IMUData data_behind_;

  int quaternion_canid_, bullet_speed_canid_, send_canid_;

  void callback(const can_frame & frame);

  std::string read_yaml(const std::string & config_path);
};

}  // namespace io

#endif  // IO__CBOARD_HPP    // 从CAN帧数据中解析出子弹速度、模式、射击模式和云台角度
