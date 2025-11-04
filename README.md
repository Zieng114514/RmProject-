# 无人机云台控制系统

本文档介绍无人机云台控制系统的代码结构、功能实现以及参数调试方法。

## 项目概述

本项目是一个基于STM32F407的无人机二轴云台控制系统，支持Yaw（偏航）和Pitch（俯仰）两轴独立控制。系统采用IMU（惯性测量单元）作为主要反馈源，实现高精度的姿态控制。

### 主要特性

- 四环串级PID控制（Yaw轴）：IMU角度环 -> IMU角速度环 -> 电机位置环 -> 电机速度环
- 双环PID控制（Pitch轴）：IMU角度环 -> IMU角速度环
- 急停模式支持：GIMBAL_ZERO_FORCE模式下清空所有PID状态，确保安全
- 缓启动功能：从急停模式恢复时平滑过渡，避免突然的角度跳跃
- 实时控制：200Hz任务频率，保证控制的实时性和响应速度

## 系统架构

### 控制数据流

```
遥控器输入 -> robot_cmd模块 -> gimbal_cmd消息 -> GimbalTask() -> 电机控制
```

### 硬件配置

- **主控芯片**: STM32F407IGHx
- **Yaw轴电机**: GM6020（CAN ID: 0x001）
- **Pitch轴电机**: DM4310（CAN ID: 0x002）
- **IMU传感器**: BMI088（通过姿态解算模块提供角度和角速度反馈）
- **CAN总线**: CAN1（用于电机通信）

## 代码结构

### 核心文件

- `application/gimbal/gimbal.c`: 云台主控制文件
- `application/gimbal/gimbal.h`: 云台头文件
- `application/cmd/robot_cmd.c`: 遥控器指令处理，生成云台控制命令
- `application/robot_def.h`: 机器人全局参数定义

### 关键模块

1. **GimbalInit()**: 云台初始化函数
   - 初始化IMU姿态解算
   - 配置Yaw轴GM6020电机（四环PID）
   - 配置Pitch轴DM4310电机（双环PID）
   - 注册消息订阅/发布

2. **GimbalTask()**: 云台控制任务（200Hz）
   - 接收控制命令
   - 根据模式执行相应控制逻辑
   - 处理缓启动
   - 更新反馈数据

3. **ClearPIDState()**: PID状态清空函数
   - 清空所有PID控制器的误差和积分项
   - 在急停模式下调用，确保状态干净

## 控制模式

### GIMBAL_GYRO_MODE（正常工作模式）

在此模式下，云台使用IMU反馈进行姿态控制：

- **Yaw轴控制**: 
  - 使用IMU的YawTotalAngle作为角度反馈
  - 使用IMU的Gyro[2]（Z轴角速度）作为角速度反馈
  - 四环串级控制确保高精度和快速响应

- **Pitch轴控制**:
  - 使用IMU的Roll作为角度反馈（注意：Roll对应Pitch轴的姿态）
  - 使用IMU的Gyro[1]（Y轴角速度）作为角速度反馈
  - 双环控制，输出单位为扭矩（Nm）

### GIMBAL_ZERO_FORCE（急停模式）

在此模式下：

- 所有电机停止输出（电流置零）
- 清空所有PID控制器的状态（Err、Iout等）
- 重置缓启动标志
- 用于紧急停止或系统保护

## PID参数配置

### Yaw轴（GM6020）PID参数

位置：`application/gimbal/gimbal.c` 的 `GimbalInit()` 函数中

#### IMU角度环（最外环）

```c
.imu_angle_PID = {
    .Kp = 1.5,     // 比例系数
    .Ki = 2,       // 积分系数
    .Kd = 0,       // 微分系数（未使用）
    .MaxOut = 500, // 最大输出限制
    .IntegralLimit = 100, // 积分限幅
}
```

**调试方法**:
- Kp过小：响应慢，跟踪滞后明显
- Kp过大：可能引起震荡，系统不稳定
- Ki过小：稳态误差无法消除
- Ki过大：积分饱和，响应变慢，可能过冲
- 建议从Kp开始调试，Ki保持较小值

#### IMU角速度环（第二环）

```c
.imu_speed_PID = {
    .Kp = 2,
    .Ki = 1,
    .Kd = 0,
    .MaxOut = 300,
    .IntegralLimit = 100,
}
```

**调试方法**:
- 此环主要控制IMU角速度的响应速度
- Kp影响角速度跟踪的快速性
- Ki用于消除角速度稳态误差
- 注意MaxOut不能太大，防止影响角度环的控制

#### 电机位置环（第三环）

```c
.angle_PID = {
    .Kp = 35,
    .Ki = 20,
    .Kd = 0.00,
    .MaxOut = 450,
    .IntegralLimit = 50,
}
```

**调试方法**:
- 此环控制电机编码器位置
- Kp通常较大，确保位置快速跟踪
- Ki用于消除位置稳态误差
- MaxOut限制电机的最大位置指令

#### 电机速度环（最内环）

```c
.speed_PID = {
    .Kp = 35,
    .Ki = 50,
    .Kd = 0.00,
    .MaxOut = 7000,
    .IntegralLimit = 1000,
}
```

**调试方法**:
- 最内环，控制电机实际转速
- Kp和Ki通常都较大，确保快速响应
- MaxOut对应电机最大电流（15000对应满电流）
- 此环性能直接影响整个系统的动态响应

### Pitch轴（DM4310）PID参数

#### 角度环（外环）

```c
.angle_PID = {
    .Kp = 0.405,
    .Ki = 0.02,
    .Kd = 0.0,
    .MaxOut = 10.25, // 单位：Nm（牛米）
    .IntegralLimit = 5,
}
```

**调试方法**:
- DM电机的角度环输出为扭矩，单位是Nm
- Kp通常较小（0.1-1.0范围），因为扭矩输出对角度影响大
- Ki保持很小值，避免积分饱和
- MaxOut限制最大输出扭矩，防止过载

#### 速度环（内环）

```c
.speed_PID = {
    .Kp = 0.05,
    .Ki = 0.1,
    .Kd = 0.0,
    .MaxOut = 0.1, // 单位：Nm
    .IntegralLimit = 2,
}
```

**调试方法**:
- 速度环的输出也是扭矩（Nm）
- Kp和Ki都很小，因为这是内环
- MaxOut需要小于角度环的MaxOut
- 调整时注意观察电机是否有震荡

### PID调试流程

1. **从内环到外环逐步调试**:
   - 先调最内环（速度环），确保速度跟踪良好
   - 再调位置环，确保位置跟踪
   - 最后调IMU角度环和角速度环

2. **参数调整原则**:
   - 先增大Kp，直到出现轻微震荡，然后减小10-20%
   - 再逐步加入Ki，观察稳态误差消除情况
   - Kd一般先设为0，如果超调严重再考虑加入

3. **观察指标**:
   - 响应速度：阶跃响应时间
   - 超调量：不应超过10-20%
   - 稳态误差：最终应接近零
   - 震荡：系统不应有明显震荡

4. **调试工具**:
   - 使用Ozone调试器观察PID各环的输出和误差
   - 使用上位机实时绘制曲线
   - 通过串口打印关键变量值

## 缓启动参数配置

### 参数位置

位置：`application/gimbal/gimbal.c` 文件顶部

```c
#define GIMBAL_RAMP_RATE_YAW 0.03f    // Yaw轴缓启动速度(rad/任务周期)
#define GIMBAL_RAMP_RATE_PITCH 0.04f  // Pitch轴缓启动速度(rad/任务周期)
```

### 工作原理

当从`GIMBAL_ZERO_FORCE`模式切换到`GIMBAL_GYRO_MODE`时：
1. 系统检测到模式切换
2. 记录当前IMU角度作为起始点
3. 使用线性插值，每个任务周期（5ms）逐渐从起始角度过渡到目标角度
4. 到达目标后自动关闭缓启动，恢复正常响应

### 调试方法

- **GIMBAL_RAMP_RATE_YAW**:
  - 当前值：0.03 rad/周期 ≈ 6 rad/s
  - 增大：缓启动更快，但可能不够平滑
  - 减小：缓启动更慢，更平滑但可能感觉迟缓
  - 建议范围：0.01 - 0.1

- **GIMBAL_RAMP_RATE_PITCH**:
  - 当前值：0.04 rad/周期 ≈ 8 rad/s
  - 调整原则与Yaw相同
  - Pitch轴通常需要稍快的响应

### 计算说明

任务频率为200Hz，即每个周期5ms：
- 实际角速度 = RAMP_RATE × 200 rad/s
- 例如：0.03 rad/周期 = 0.03 × 200 = 6 rad/s ≈ 1圈/秒

## 机械参数配置

### 位置参数

位置：`application/robot_def.h`

```c
// Pitch轴限位
#define pitch_limit_up -24.0f   // 向上最大角度（度）
#define pitch_limit_down 35.0f  // 向下最大角度（度）

// 云台对齐参数
#define YAW_CHASSIS_ALIGN_ECD 2711      // Yaw轴与底盘对齐时的编码器值
#define YAW_ECD_GREATER_THAN_4096 0     // 对齐编码器值是否大于4096
#define PITCH_HORIZON_ECD 3412          // Pitch轴水平时的编码器值
```

### 配置方法

1. **Pitch限位**:
   - 根据机械结构设置上下限
   - 单位：度（转换为弧度使用）
   - 防止云台翻转或碰撞

2. **对齐编码器值**:
   - 将云台调整到对齐位置
   - 读取电机编码器值
   - 填入`YAW_CHASSIS_ALIGN_ECD`或`PITCH_HORIZON_ECD`
   - 如果对齐值大于4096，设置`YAW_ECD_GREATER_THAN_4096 = 1`

## 遥控器配置

### 通道映射

在`application/cmd/robot_cmd.c`的`RemoteControlSet()`函数中：

- **左摇杆X轴（rocker_l_）**: Yaw轴控制（左右转动）
- **左摇杆Y轴（rocker_l1）**: Pitch轴控制（上下俯仰）
- **右侧三段开关**:
  - [上档]: GIMBAL_ZERO_FORCE（急停）
  - [中档]: GIMBAL_GYRO_MODE（正常工作）
  - [下档]: GIMBAL_GYRO_MODE（正常工作）

### 控制灵敏度

Yaw轴灵敏度在`robot_cmd.c`中：
```c
gimbal_cmd_send.yaw += 0.005f * (float)rc_data[TEMP].rc.rocker_l_;
```

Pitch轴灵敏度：
```c
gimbal_cmd_send.pitch += 0.0005f * (float)rc_data[TEMP].rc.rocker_l1;
```

调整系数可以改变摇杆的控制灵敏度。

## 坐标系定义

- **Yaw轴**: 逆时针为正方向（从上方俯视）
- **Pitch轴**: 向上为正方向（从侧面看）
- **角度单位**: 弧度（rad）
- **速度单位**: 弧度/秒（rad/s）

## 常见问题排查

### 1. 云台不响应控制

- 检查CAN总线通信是否正常
- 确认电机ID配置正确（Yaw: 0x001, Pitch: 0x002）
- 检查IMU数据是否正常更新
- 确认`GimbalTask()`是否被正确调用（200Hz）

### 2. 云台震荡

- 减小外环（角度环）的Kp和Ki
- 检查内环（速度环）是否稳定
- 降低MaxOut限制
- 检查机械连接是否松动

### 3. 响应过慢

- 增大外环Kp
- 检查内环是否响应正常
- 确认缓启动参数是否过大

### 4. 稳态误差大

- 增大Ki值（但要注意积分饱和）
- 检查IntegralLimit是否设置合理
- 确认死区（DeadBand）是否过大

### 5. 急停后恢复时跳动

- 确认`ClearPIDState()`正确清空了PID状态
- 检查缓启动是否正常激活
- 调整缓启动速度参数

### 6. Pitch轴控制异常

- 确认DM电机是否正确初始化
- 检查`DMMotorEnable()`是否被调用
- 验证IMU的Roll轴数据是否对应Pitch轴
- 确认反馈源设置为`OTHER_FEED`

## 调试建议

1. **分步调试**:
   - 先单独调试Yaw轴，确保正常工作
   - 再单独调试Pitch轴
   - 最后进行两轴联调

2. **参数记录**:
   - 记录每次参数修改和效果
   - 建立参数文档，方便后续参考
   - 使用版本控制跟踪参数变更

3. **安全措施**:
   - 调试时设置合理的限位
   - 初始参数设置较小值，逐步增大
   - 准备急停开关，随时可以停止

4. **测试方法**:
   - 小幅度阶跃响应测试
   - 正弦跟踪测试
   - 长时间稳定性测试

## 文件结构

```
application/gimbal/
├── gimbal.c      # 主控制文件
├── gimbal.h      # 头文件
└── gimbal.md     # 模块说明文档

application/cmd/
├── robot_cmd.c   # 遥控器指令处理
└── robot_cmd.h   # 指令处理头文件

application/
└── robot_def.h   # 全局参数定义
```

## 版本信息

- 当前版本支持的功能：
  - Yaw轴四环串级控制
  - Pitch轴双环控制
  - 急停模式及PID状态清空
  - 缓启动功能
  - IMU反馈控制

## 注意事项

1. 修改PID参数后必须重新编译并烧录程序
2. 调试时建议先使用保守的参数值
3. 注意观察电机温度，避免过载
4. 确保IMU数据更新频率足够高（建议>=200Hz）
5. CAN总线通信质量直接影响控制性能

## 技术支持

如有问题，请参考：
- 代码注释中的详细说明
- `gimbal.md`模块文档
- 相关电机驱动模块的文档

---

最后更新：基于当前代码版本编写，参数值以实际代码为准。
