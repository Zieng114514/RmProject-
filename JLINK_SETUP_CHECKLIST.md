# J-Link 调试设置检查清单

## 🔧 硬件连接检查

### 1. J-Link 调试器连接
请按照以下方式连接 J-Link 调试器到 STM32F407IG 开发板：

```
J-Link 调试器引脚    →    STM32F407IG 开发板引脚
VCC (3.3V)          →    3.3V 电源
GND                 →    GND 地线
TMS (SWDIO)         →    PA13 (SWDIO)
TCK (SWCLK)         →    PA14 (SWCLK)
TDO (SWO)           →    PB3  (SWO) [可选，用于 SWO 输出]
TDI                 →    PA15 (TDI) [可选，JTAG 模式]
```

### 2. 开发板电源
- ✅ 确保开发板通过 USB 或外部电源供电
- ✅ 检查电源指示灯是否亮起
- ✅ 确认开发板正常工作

### 3. 连接检查
- ✅ SWD 线连接牢固
- ✅ 电源线连接正确
- ✅ 没有短路或接触不良

## 💻 软件配置检查

### 1. J-Link 软件状态
```bash
# 检查 J-Link 软件
ls -la /Applications/SEGGER/JLink/JLinkExe
# 应该显示可执行文件

# 检查版本
/Applications/SEGGER/JLink/JLinkExe -CommandFile /dev/stdin <<< "ShowEmuList"
```

### 2. VS Code 调试配置
已配置的调试选项：
- ✅ **J-Link Debug (STM32F407IG)**：完整调试会话
- ✅ **J-Link Attach (STM32F407IG)**：附加调试
- ✅ SVD 文件路径：`STM32F407.svd`
- ✅ 服务器路径：`/Applications/SEGGER/JLink/JLinkGDBServerCLExe`

### 3. 项目编译状态
```bash
# 确保项目已编译
make clean
make -j4

# 检查编译结果
ls -la build/basic_framework.elf
```

## 🔍 连接测试步骤

### 步骤 1：硬件连接测试
1. 将 J-Link 调试器连接到电脑 USB 端口
2. 按照上述引脚连接方式连接到开发板
3. 给开发板上电
4. 运行连接测试：
```bash
./test_jlink.sh
```

### 步骤 2：J-Link 手动测试
如果自动测试失败，请手动测试：

```bash
# 启动 J-Link Commander
/Applications/SEGGER/JLink/JLinkExe

# 在 J-Link Commander 中执行：
device STM32F407IG
si SWD
speed 1000
connect
```

### 步骤 3：VS Code 调试测试
1. 在 VS Code 中打开项目
2. 按 `F5` 或点击调试按钮
3. 选择 "J-Link Debug (STM32F407IG)"
4. 等待连接建立

## 🚨 常见问题解决

### 问题 1：设备未识别
**症状**：`未发现 USB 设备`
**解决方案**：
1. 检查 USB 连接
2. 尝试不同的 USB 端口
3. 检查 J-Link 驱动安装
4. 重启电脑

### 问题 2：连接超时
**症状**：`Could not connect to target`
**解决方案**：
1. 检查 SWD 线连接
2. 降低连接速度（在 launch.json 中设置 `"swdSpeed": 1000`）
3. 检查开发板电源
4. 尝试硬件复位

### 问题 3：权限问题
**症状**：权限被拒绝
**解决方案**：
```bash
# 添加用户权限
sudo dseditgroup -o edit -a $(whoami) -t user _developer

# 或者使用 sudo 运行
sudo /Applications/SEGGER/JLink/JLinkGDBServerCLExe
```

### 问题 4：设备不匹配
**症状**：`Failed to get index for device name`
**解决方案**：
1. 确认设备型号为 STM32F407IG
2. 检查 launch.json 中的 device 设置
3. 尝试手动指定设备

## 📋 调试前检查清单

在开始调试前，请确认：

- [ ] J-Link 调试器已连接到电脑
- [ ] 开发板已上电并正常工作
- [ ] SWD 线连接正确（TMS→PA13, TCK→PA14）
- [ ] 项目已成功编译
- [ ] VS Code 调试配置正确
- [ ] J-Link 软件正常工作
- [ ] 没有其他调试器占用接口

## 🎯 开始调试

当所有检查都通过后：

1. **在 VS Code 中设置断点**
   - 在需要调试的代码行左侧点击设置断点
   - 建议在 `main()` 函数开始处设置断点

2. **启动调试会话**
   - 按 `F5` 或点击调试按钮
   - 选择 "J-Link Debug (STM32F407IG)"
   - 等待连接建立

3. **调试操作**
   - `F5`：继续执行
   - `F10`：单步执行（Step Over）
   - `F11`：进入函数（Step Into）
   - `Shift+F11`：跳出函数（Step Out）
   - `Ctrl+Shift+F5`：重启调试

4. **监控变量**
   - 在监视窗口中添加变量
   - 使用实时监控功能
   - 查看内存和寄存器状态

## 📞 技术支持

如果遇到问题：

1. **检查硬件连接**
   - 重新连接所有线缆
   - 检查电源状态
   - 尝试不同的连接方式

2. **查看调试输出**
   - 检查 VS Code 调试控制台
   - 查看 J-Link 服务器输出
   - 检查错误信息

3. **联系技术支持**
   - 提供详细的错误信息
   - 说明硬件配置
   - 提供调试日志

## 🔗 相关资源

- [J-Link 用户手册](https://www.segger.com/downloads/jlink/UM08001_JLink.pdf)
- [Cortex-Debug 扩展文档](https://github.com/Marus/cortex-debug)
- [STM32F407 参考手册](https://www.st.com/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [项目调试指南](./JLINK_DEBUG_GUIDE.md)

