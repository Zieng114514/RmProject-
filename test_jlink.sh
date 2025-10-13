#!/bin/bash

# J-Link 连接测试脚本
# 用于验证 J-Link 调试器是否正常工作

echo "=== J-Link 连接测试 ==="
echo ""

# 检查 J-Link 软件
echo "1. 检查 J-Link 软件安装..."
if [ -f "/Applications/SEGGER/JLink/JLinkExe" ]; then
    echo "   ✓ J-Link 软件已安装"
    /Applications/SEGGER/JLink/JLinkExe -CommandFile /dev/stdin <<< "ShowEmuList" 2>/dev/null | head -5
else
    echo "   ✗ J-Link 软件未找到"
    exit 1
fi

echo ""

# 检查设备连接
echo "2. 检查设备连接..."
if ls /dev/cu.usbmodem* 2>/dev/null; then
    echo "   ✓ 发现 USB 设备"
else
    echo "   ⚠ 未发现 USB 设备，请检查连接"
fi

echo ""

# 测试 J-Link 连接
echo "3. 测试 J-Link 连接..."
echo "   正在尝试连接 STM32F407IG..."

# 创建临时命令文件
cat > /tmp/jlink_test.cmd << EOF
device STM32F407IG
si SWD
speed 1000
connect
halt
go
exit
EOF

# 运行 J-Link 测试
if /Applications/SEGGER/JLink/JLinkExe -CommandFile /tmp/jlink_test.cmd 2>/dev/null | grep -q "Connected"; then
    echo "   ✓ J-Link 连接成功"
else
    echo "   ✗ J-Link 连接失败"
    echo "   请检查："
    echo "   - 硬件连接是否正确"
    echo "   - 开发板是否上电"
    echo "   - SWD 线是否连接正确"
fi

# 清理临时文件
rm -f /tmp/jlink_test.cmd

echo ""
echo "=== 测试完成 ==="
echo ""
echo "如果连接成功，你现在可以："
echo "1. 在 VS Code 中按 F5 开始调试"
echo "2. 选择 'J-Link Debug (STM32F407IG)' 配置"
echo "3. 设置断点并开始调试"
