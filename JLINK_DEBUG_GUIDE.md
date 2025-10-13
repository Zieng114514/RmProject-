# J-Link 调试指南

## 概述

本指南将帮助你在 macOS 上使用 J-Link 调试器调试 STM32F407IG 开发板。

## 硬件连接

### 1. J-Link 调试器连接
```
J-Link 调试器    →    STM32F407IG 开发板
VCC (3.3V)      →    3.3V
GND             →    GND
TMS             →    PA13 (SWDIO)
TCK             →    PA14 (SWCLK)
TDO             →    PB3  (SWO) [可选]
TDI             →    PA15 (TDI) [可选]
```

### 2. 开发板电源
- 确保开发板通过 USB 或外部电源供电
- 检查电源指示灯是否亮起

## 软件配置

### 1. 安装 J-Link 软件
J-Link 软件已安装在：`/Applications/SEGGER/JLink/`

### 2. VS Code 调试配置
已配置的调试选项：
- **J-Link Debug (STM32F407IG)**：完整调试会话（编译+烧录+调试）
- **J-Link Attach (STM32F407IG)**：附加到运行中的程序

### 3. 调试配置参数
```json
{
    "servertype": "jlink",
    "serverpath": "/Applications/SEGGER/JLink/JLinkGDBServerCLExe",
    "device": "STM32F407IG",
    "interface": "swd",
    "rtos": "FreeRTOS"
}
```

## 调试步骤

### 1. 硬件检查
```bash
# 检查 J-Link 是否被识别
ls /dev/cu.usbmodem*

# 检查 J-Link 状态
/Applications/SEGGER/JLink/JLinkExe -CommandFile /dev/stdin <<< "ShowEmuList"
```

### 2. 编译项目
```bash
# 清理并编译
make clean
make -j4

# 检查编译结果
ls -la build/basic_framework.elf
```

### 3. 启动调试
1. 在 VS Code 中按 `F5` 或点击调试按钮
2. 选择 "J-Link Debug (STM32F407IG)"
3. 等待连接建立

## 调试功能

### 1. 基本调试功能
- **断点设置**：在代码行左侧点击设置断点
- **单步执行**：F10 (Step Over), F11 (Step Into), Shift+F11 (Step Out)
- **继续执行**：F5 (Continue)
- **重启调试**：Ctrl+Shift+F5

### 2. 变量监控
- **监视窗口**：添加变量到监视列表
- **实时监控**：`liveWatch` 功能已启用
- **内存查看**：查看内存和寄存器状态

### 3. FreeRTOS 支持
- **任务列表**：查看所有 FreeRTOS 任务
- **任务切换**：在不同任务间切换调试
- **任务状态**：监控任务运行状态

## 常见问题解决

### 1. 连接失败
**问题**：`Could not connect to target`
**解决方案**：
```bash
# 检查硬件连接
# 1. 确认 SWD 线连接正确
# 2. 检查电源是否正常
# 3. 尝试降低 SWD 速度

# 在 launch.json 中添加：
"swdSpeed": 1000
```

### 2. 设备识别失败
**问题**：`Failed to get index for device name`
**解决方案**：
```bash
# 手动指定设备
/Applications/SEGGER/JLink/JLinkExe -CommandFile /dev/stdin <<< "
device STM32F407IG
si SWD
speed 4000
connect
"
```

### 3. 调试器权限问题
**问题**：权限被拒绝
**解决方案**：
```bash
# 添加用户到 dialout 组
sudo dseditgroup -o edit -a $(whoami) -t user _developer

# 或者使用 sudo 运行
sudo /Applications/SEGGER/JLink/JLinkGDBServerCLExe
```

### 4. 程序无法下载
**问题**：Flash 下载失败
**解决方案**：
```bash
# 检查 Flash 配置
# 在 launch.json 中添加：
"postLaunchCommands": [
    "monitor reset halt",
    "monitor flash device = STM32F407IG",
    "monitor flash download = 1"
]
```

## 高级调试技巧

### 1. 使用 J-Link Commander
```bash
# 启动 J-Link Commander
/Applications/SEGGER/JLink/JLinkExe

# 常用命令
device STM32F407IG    # 设置设备
si SWD               # 设置接口
speed 4000           # 设置速度
connect              # 连接
halt                 # 停止 CPU
go                   # 运行
```

### 2. 使用 RTT 输出
```bash
# 启动 RTT 客户端
/Applications/SEGGER/JLink/JLinkRTTClient

# 在代码中使用 RTT 输出
SEGGER_RTT_printf(0, "Debug message: %d\n", value);
```

### 3. 内存和寄存器查看
- **内存查看**：在调试面板中查看内存内容
- **寄存器查看**：查看 CPU 寄存器状态
- **SVD 文件**：使用 STM32F407.svd 查看外设寄存器

## 调试配置文件

### launch.json 配置说明
```json
{
    "name": "J-Link Debug (STM32F407IG)",
    "servertype": "jlink",                    // 使用 J-Link 服务器
    "serverpath": "/Applications/SEGGER/JLink/JLinkGDBServerCLExe",
    "device": "STM32F407IG",                  // 目标设备
    "interface": "swd",                       // 调试接口
    "rtos": "FreeRTOS",                       // RTOS 支持
    "showDevDebugOutput": "raw",              // 显示调试输出
    "liveWatch": { "enabled": true }          // 实时监控
}
```

## 性能优化

### 1. 调试速度优化
```json
{
    "swdSpeed": 1000,                         // 降低 SWD 速度
    "swoConfig": {
        "enabled": true,
        "cpuFrequency": 168000000,
        "swoFrequency": 2000000
    }
}
```

### 2. 内存使用优化
- 使用 `-Og` 优化级别进行调试
- 启用调试符号：`-g -gdwarf-2`
- 保留调试信息：`-fno-omit-frame-pointer`

## 故障排除

### 1. 调试器无法连接
1. 检查硬件连接
2. 确认电源状态
3. 尝试不同的 SWD 速度
4. 检查 J-Link 驱动

### 2. 程序运行异常
1. 检查时钟配置
2. 验证中断向量表
3. 检查内存映射
4. 使用硬件复位

### 3. 调试会话中断
1. 检查 USB 连接
2. 重新连接调试器
3. 重启调试会话

## 参考资源

- [J-Link 用户手册](https://www.segger.com/downloads/jlink/UM08001_JLink.pdf)
- [Cortex-Debug 扩展文档](https://github.com/Marus/cortex-debug)
- [STM32F407 参考手册](https://www.st.com/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

## 联系支持

如果遇到问题，请：
1. 检查本指南的故障排除部分
2. 查看 VS Code 调试控制台输出
3. 检查 J-Link 软件版本兼容性
4. 联系技术支持

