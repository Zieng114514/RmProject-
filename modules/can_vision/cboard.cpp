#include "cboard.hpp"

#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
/**
 * @brief CBoard类的构造函数，用于初始化棋盘对象
 * @param config_path 配置文件的路径，用于读取相关配置信息
 */
CBoard::CBoard(const std::string & config_path)
: mode(Mode::idle),              // 初始化模式为idle
  shoot_mode(ShootMode::left_shoot),  // 初始化射击模式为左射击
  bullet_speed(0),               // 初始化子弹速度为0
  queue_(5000),                  // 初始化队列，容量为5000
  can_(read_yaml(config_path), std::bind(&CBoard::callback, this, std::placeholders::_1))  // 初始化CAN总线，配置来自yaml文件，并绑定回调函数
// 注意: callback的运行会早于Cboard构造函数的完成
{
  tools::logger()->info("[Cboard] Waiting for q...");  // 记录日志，等待队列数据
  queue_.pop(data_ahead_);       // 从队列中弹出前方数据
  queue_.pop(data_behind_);      // 从队列中弹出后方数据
  tools::logger()->info("[Cboard] Opened.");           // 记录日志，表示棋盘已打开
}

/**
 * @brief 根据时间戳获取对应的IMU姿态四元数
 * @param timestamp 需要查询的时间点
 * @return 对应时间点的姿态四元数
 */
Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  // 如果当前数据队列尾部的时间戳小于查询时间，则将尾部数据提前到前一个位置
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  // 循环遍历数据队列，找到包含查询时间戳的数据范围
  while (true) {
    queue_.pop(data_behind_);  // 从队列中弹出数据到尾部
    if (data_behind_.timestamp > timestamp) break;  // 找到超出查询时间的数据时停止
    data_ahead_ = data_behind_;  // 更新前一个数据点
  }



  // 获取前后两个数据点的四元数和时间戳
  Eigen::Quaterniond q_a = data_ahead_.q.normalized();  // 前一个数据点的四元数（归一化）
  Eigen::Quaterniond q_b = data_behind_.q.normalized();  // 后一个数据点的四元数（归一化）
  auto t_a = data_ahead_.timestamp;  // 前一个数据点的时间戳
  auto t_b = data_behind_.timestamp;  // 后一个数据点的时间戳
  auto t_c = timestamp;  // 查询时间戳
  std::chrono::duration<double> t_ab = t_b - t_a;  // 前后数据点的时间差
  std::chrono::duration<double> t_ac = t_c - t_a;  // 前数据点与查询时间的时间差

  // 四元数插值
  auto k = t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

/**
 * @brief 发送控制命令到CAN总线
 * @param command 包含控制指令的结构体，包含控制、射击、偏航、俯仰和水平距离等参数
 * @note 该函数将控制命令打包成CAN帧并通过CAN总线发送
 */
void CBoard::send(Command command) const
{
  can_frame frame;  // 定义CAN帧结构体
  frame.can_id = send_canid_;  // 设置CAN帧ID
  frame.can_dlc = 8;  // 设置CAN帧数据长度为8字节

  
  // 将控制命令转换为CAN帧数据
  frame.data[0] = (command.control) ? 1 : 0;  // 控制标志位，1表示开启控制，0表示关闭
  frame.data[1] = (command.shoot) ? 1 : 0;  // 射击标志位，1表示射击，0表示不射击

  
  // 将偏航角度转换为16位整数，并拆分为高低字节
  // 乘以1e4将浮点数转换为整数，保留4位小数精度
  frame.data[2] = (int16_t)(command.yaw * 1e4) >> 8;  // 偏航角度高字节
  frame.data[3] = (int16_t)(command.yaw * 1e4);  // 偏航角度低字节

  
  // 将俯仰角度转换为16位整数，并拆分为高低字节
  frame.data[4] = (int16_t)(command.pitch * 1e4) >> 8;  // 俯仰角度高字节
  frame.data[5] = (int16_t)(command.pitch * 1e4);  // 俯仰角度低字节

  
  // 将水平距离转换为16位整数，并拆分为高低字节
  frame.data[6] = (int16_t)(command.horizon_distance * 1e4) >> 8;  // 水平距离高字节
  frame.data[7] = (int16_t)(command.horizon_distance * 1e4);  // 水平距离低字节

  try {
    can_.write(&frame);  // 尝试通过CAN总线发送数据
  } catch (const std::exception & e) {
    tools::logger()->warn("{}", e.what());  // 捕获并记录异常信息
  }
}

/**
 * @brief CBoard类的回调函数，处理接收到的CAN帧数据
 * @param frame 接收到的CAN帧数据结构体
 */
void CBoard::callback(const can_frame & frame)
{
  // 获取当前时间戳
  auto timestamp = std::chrono::steady_clock::now();

  // 处理四元数数据帧
  if (frame.can_id == quaternion_canid_) {
    // 从CAN帧数据中解析出四元数的四个分量，并进行缩放处理
    auto x = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e4;
    auto y = (int16_t)(frame.data[2] << 8 | frame.data[3]) / 1e4;
    auto z = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;
    auto w = (int16_t)(frame.data[6] << 8 | frame.data[7]) / 1e4;

    // 验证四元数是否有效（单位四元数的模应接近1）
    if (std::abs(x * x + y * y + z * z + w * w - 1) > 1e-2) {
      tools::logger()->warn("Invalid q: {} {} {} {}", w, x, y, z);
      return;
    }

    // 将有效的四元数和时间戳存入队列
    queue_.push({{w, x, y, z}, timestamp});
  }

  // 处理子弹速度数据帧
  else if (frame.can_id == bullet_speed_canid_) {
    bullet_speed = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e2;
    mode = Mode(frame.data[2]);
    shoot_mode = ShootMode(frame.data[3]);
    ft_angle = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;

    // 限制日志输出频率为1Hz
    static auto last_log_time = std::chrono::steady_clock::time_point::min();
    auto now = std::chrono::steady_clock::now();

    if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) {
      tools::logger()->info(
        "[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, Shoot mode: {}, FT angle: {:.2f} rad",
        bullet_speed, MODES[mode], SHOOT_MODES[shoot_mode], ft_angle);
      last_log_time = now;
    }
  }
}


std::string CBoard::read_yaml(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  quaternion_canid_ = tools::read<int>(yaml, "quaternion_canid");
  bullet_speed_canid_ = tools::read<int>(yaml, "bullet_speed_canid");
  send_canid_ = tools::read<int>(yaml, "send_canid");

  if (!yaml["can_interface"]) {
    throw std::runtime_error("Missing 'can_interface' in YAML configuration.");
  }

  return yaml["can_interface"].as<std::string>();
}

} 