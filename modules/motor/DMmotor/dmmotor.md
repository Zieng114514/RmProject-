# DM电机模块修改说明

## 修改概述

本次修改将DM电机模块的结构体设计改为与DJI电机模块类似，添加了 `measure` 成员来统一管理电机的测量数据。

## 主要修改内容

### 1. 新增测量数据结构体

```c
typedef struct
{
    // 编码器相关
    uint16_t last_encoder;        // 上一次读取的编码器值
    uint16_t encoder;             // 当前编码器值
    int32_t total_encoder;        // 总编码器值
    int32_t total_round;          // 总圈数
    
    // 位置和角度
    float position;               // 当前位置
    float angle_single_round;     // 单圈角度
    
    // 速度和扭矩
    float velocity;               // 当前速度
    float torque;                 // 当前扭矩
    float current;                // 当前电流
    
    // 温度
    float temperature_mos;         // MOS管温度
    float temperature_rotor;       // 转子温度
    
    // 时间相关
    uint32_t feed_cnt;            // 反馈计数
    float dt;                     // 时间间隔
} DM_Motor_Measure_s;
```

### 2. 修改DMMotorInstance结构体

- 添加了 `DM_Motor_Measure_s measure` 成员
- 重新组织了结构体成员，将测量数据统一放在 `measure` 中
- 移除了重复的成员变量（如原来的 `position`、`velocity` 等）

### 3. 更新数据处理函数

- 修改了 `Data_Process()` 函数，使用 `motor->measure` 来存储数据
- 添加了时间间隔计算和守护进程更新
- 改进了编码器数据处理逻辑

### 4. 新增获取函数

添加了以下新的获取函数：

```c
// 获取测量数据结构指针
DM_Motor_Measure_s* DMMotorGetMeasure(DMMotorInstance *motor);

// 获取电机电流
float DMMotorGetCurrent(DMMotorInstance *motor);

// 获取转子温度
float DMMotorGetRotorTemperature(DMMotorInstance *motor);

// 获取编码器值
uint16_t DMMotorGetEncoder(DMMotorInstance *motor);

// 获取总编码器值
int32_t DMMotorGetTotalEncoder(DMMotorInstance *motor);

// 获取总圈数
int32_t DMMotorGetTotalRound(DMMotorInstance *motor);
```

### 5. 更新现有获取函数

所有现有的获取函数（如 `DMMotorGetPosition()`、`DMMotorGetVelocity()` 等）现在都从 `measure` 结构体中获取数据。

## 使用方法

### 方法1: 使用传统获取函数
```c
float position = DMMotorGetPosition(motor);
float velocity = DMMotorGetVelocity(motor);
float torque = DMMotorGetTorque(motor);
```

### 方法2: 直接访问measure结构体（推荐）
```c
DM_Motor_Measure_s *measure = DMMotorGetMeasure(motor);
if (measure != NULL) {
    float position = measure->position;
    float velocity = measure->velocity;
    float torque = measure->torque;
    // ... 访问其他数据
}
```

### 方法3: 使用新的专用获取函数
```c
float current = DMMotorGetCurrent(motor);
float rotor_temp = DMMotorGetRotorTemperature(motor);
uint16_t encoder = DMMotorGetEncoder(motor);
```

## 优势

1. **统一的数据管理**: 所有测量数据都集中在 `measure` 结构体中
2. **更好的代码组织**: 与DJI电机模块保持一致的代码风格
3. **更丰富的数据访问**: 提供了更多获取电机数据的方法
4. **向后兼容**: 保留了所有原有的获取函数
5. **更灵活的使用方式**: 可以选择直接访问结构体或使用函数接口

## 注意事项

1. 所有原有的API都保持兼容，不会影响现有代码
2. 新的 `measure` 结构体提供了更丰富的数据访问方式
3. 建议使用 `DMMotorGetMeasure()` 函数获取结构体指针，然后直接访问成员
4. 在使用前请确保电机已正确初始化并在线

## 示例代码

详细的使用示例请参考 `example_usage.c` 文件。