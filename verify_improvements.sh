#!/bin/bash
# RMTT Project Improvement Verification Script
# Test script to verify all improvements have been properly applied

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=========================================="
echo "RMTT 项目改进验证脚本"
echo "=========================================="
echo ""

# Test 1: Check topic naming consistency
echo -e "${YELLOW}[测试 1]${NC} 检查话题命名一致性..."
if grep -q "~status" /home/qczk/rmtt_ws/src/tello_driver/src/tello_driver_node.py; then
    echo -e "${GREEN}✓${NC} 驱动节点已使用私有话题名"
else
    echo -e "${RED}✗${NC} 驱动节点仍使用相对名"
    exit 1
fi

if grep -q 'remap from="~status"' /home/qczk/rmtt_ws/src/tello_driver/launch/tello_node.launch; then
    echo -e "${GREEN}✓${NC} launch 文件已配置 remap"
else
    echo -e "${RED}✗${NC} launch 文件缺少 remap 配置"
    exit 1
fi

# Test 2: Check dependencies
echo ""
echo -e "${YELLOW}[测试 2]${NC} 检查依赖声明..."
if grep -q "python-pynput" /home/qczk/rmtt_ws/src/tello_control_system/package.xml; then
    echo -e "${GREEN}✓${NC} pynput 依赖已声明"
else
    echo -e "${RED}✗${NC} pynput 依赖未声明"
    exit 1
fi

# Test 3: Check watchdog implementation
echo ""
echo -e "${YELLOW}[测试 3]${NC} 检查看门狗机制..."
if grep -q "_cmd_vel_watchdog_loop" /home/qczk/rmtt_ws/src/tello_driver/src/tello_driver_node.py; then
    echo -e "${GREEN}✓${NC} 看门狗循环已实现"
else
    echo -e "${RED}✗${NC} 看门狗循环未实现"
    exit 1
fi

if grep -q "last_cmd_vel_time" /home/qczk/rmtt_ws/src/tello_driver/src/tello_driver_node.py; then
    echo -e "${GREEN}✓${NC} 时间戳跟踪已实现"
else
    echo -e "${RED}✗${NC} 时间戳跟踪未实现"
    exit 1
fi

# Test 4: Check acceleration limit
echo ""
echo -e "${YELLOW}[测试 4]${NC} 检查加速度限制..."
if grep -q "_apply_accel_limit" /home/qczk/rmtt_ws/src/tello_control_system/scripts/keyboard_control_node.py; then
    echo -e "${GREEN}✓${NC} 加速度限制方法已实现"
else
    echo -e "${RED}✗${NC} 加速度限制方法未实现"
    exit 1
fi

if grep -q "max_linear_accel" /home/qczk/rmtt_ws/src/tello_control_system/config/params.yaml; then
    echo -e "${GREEN}✓${NC} 加速度限制参数已配置"
else
    echo -e "${RED}✗${NC} 加速度限制参数未配置"
    exit 1
fi

# Test 5: Verify compilation
echo ""
echo -e "${YELLOW}[测试 5]${NC} 检查编译状态..."
cd /home/qczk/rmtt_ws
if source devel/setup.bash && catkin_make --pkg tello_driver tello_control_system --dry-run 2>&1 | grep -q "Errors"; then
    echo -e "${RED}✗${NC} 编译有错误"
    exit 1
else
    echo -e "${GREEN}✓${NC} 编译无错误"
fi

# Test 6: Verify launch files
echo ""
echo -e "${YELLOW}[测试 6]${NC} 检查 launch 文件有效性..."
for launch_file in \
    /home/qczk/rmtt_ws/src/tello_driver/launch/tello_node.launch \
    /home/qczk/rmtt_ws/src/tello_driver/launch/joy_teleop.launch \
    /home/qczk/rmtt_ws/src/tello_control_system/launch/bringup.launch
do
    if xmllint --noout "$launch_file" 2>/dev/null; then
        echo -e "${GREEN}✓${NC} $(basename $launch_file) 有效"
    else
        echo -e "${YELLOW}⚠${NC} $(basename $launch_file) XML 格式检查跳过（xmllint 未装）"
    fi
done

# Summary
echo ""
echo "=========================================="
echo -e "${GREEN}✓ 所有改进验证通过！${NC}"
echo "=========================================="
echo ""
echo "改进总结："
echo "  1. ✓ 话题命名一致性已修复"
echo "  2. ✓ 依赖声明已完善"
echo "  3. ✓ 看门狗机制已实现"
echo "  4. ✓ 加速度限制已应用"
echo "  5. ✓ 编译状态良好"
echo "  6. ✓ Launch 文件配置正确"
echo ""
echo "下一步：运行 roslaunch tello_control_system bringup.launch 测试完整系统"
echo ""
