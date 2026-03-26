# RMTT Tello EDU 控制系统技术文档

## 目录
1. [系统概述](#系统概述)
2. [系统架构](#系统架构)
3. [核心功能](#核心功能)
4. [多输入仲裁机制](#多输入仲裁机制)
5. [启动操作指南](#启动操作指南)
6. [Agent接入指南](#agent接入指南)
7. [话题接口定义](#话题接口定义)
8. [安全机制](#安全机制)
9. [故障诊断](#故障诊断)

---

## 系统概述

### 项目背景
RMTT (ROS Tello Test) 是基于 DJI Tello EDU 无人机的 ROS 控制系统。该系统集成了多源控制输入，包括：
- **键盘输入**：本地实时控制
- **游戏手柄输入**：摇杆遥控
- **智能体代理**：自主导航和学习控制

### 系统定位
这是一个 **多输入仲裁控制系统**，通过优先级和超时机制确保不同控制源的安全协调，防止命令冲突。

### 主要特性
- ✅ **优先级仲裁**：键盘 > 游戏手柄 > 智能体
- ✅ **超时保护**：无效源自动剔除，发布零速度命令
- ✅ **驱动层看门狗**：0.5s 内无 `cmd_vel` 自动停止
- ✅ **加速度限制**：键盘控制的线性和角加速度限制
- ✅ **调试可观性**：实时发布活动输入源信息

---

## 系统架构

### 顶层数据流

```
┌─────────────────────────────────────────────────────────┐
│         Tello EDU Drone (192.168.10.1)                 │
└──────────────┬──────────────────────────────────────────┘
               │ UDP (cmd_vel)
               ▼
┌─────────────────────────────────────────────────────────┐
│         Tello Driver Node                              │
│   (tello_driver_node.py)                              │
│                                                        │
│   ├─ Listens: /tello/cmd_vel (30Hz)                  │
│   ├─ Publishes: /tello/status, /tello/image_raw    │
│   └─ Watchdog: 0.5s timeout → auto-zero             │
└──────────────┬──────────────────────────────────────────┘
               │
               ▲
               │ /tello/cmd_vel
               │
┌──────────────┴──────────────────────────────────────────┐
│         Tello Control MUX Node                         │
│   (tello_control_mux_node.py)                         │
│                                                        │
│   Priority Arbitration (30Hz):                        │
│   1. Check source freshness (age ≤ timeout)          │
│   2. Select highest priority valid source            │
│   3. Publish selected or zero Twist                  │
│                                                        │
│   ├─ Debug: /tello/mux/active_source                │
│   └─ Arbitration Log: [mux] source switched/timeout │
└──────────────┬──────────────────────────────────────────┘
    │          │          │
    │ priority │ priority │ priority
    │    3     │    2     │    1
    ▼          ▼          ▼
┌────────┐ ┌─────────┐ ┌──────────┐
│ KEYBD  │ │GAMEPAD  │ │  AGENT   │
│ CTRL   │ │ TELEOP  │ │ AUTOPILOT│
└────────┘ └─────────┘ └──────────┘
```

### 启动拓扑结构

```
tello_system.launch (统一启动入口)
├── tello_node.launch
│   └── tello_driver_node.py → /tello/cmd_vel (input)
│
├── control_mux.launch
│   └── tello_control_mux_node.py
│       ├─ Input: /tello/cmd_vel_keyboard
│       ├─ Input: /tello/cmd_vel_joy
│       ├─ Input: /tello/cmd_vel_agent
│       └─ Output: /tello/cmd_vel (to driver)
│
├── keyboard_control_node.py → /tello/cmd_vel_keyboard
│
├── [Optional] joy_teleop.launch (if enable_joy=true)
│   ├── joy_node → /joy
│   └── gamepad_marshall_node.py → /tello/cmd_vel_joy
│
└── tello_odom_node.py (可视化辅助)
```

---

## 核心功能

### 1. 驱动层功能 (tello_driver_node.py)

| 功能 | 说明 | 话题/参数 |
|------|------|---------|
| **飞行器连接** | UDP 连接到 Tello 飞行器 | IP: `192.168.10.1`, 端口: `8889` |
| **命令处理** | 处理速度/离散命令 | `/tello/cmd_vel` (Twist), 看门狗: 0.5s |
| **状态发布** | 电池、高度、姿态 | `/tello/status` (Int16) |
| **视频流** | H.264 原始或压缩 | `/tello/image_raw/compressed` |

### 2. 控制仲裁功能 (tello_control_mux_node.py)

| 功能 | 说明 | 参数 |
|------|------|------|
| **优先级管理** | 键盘(3) > 手柄(2) > 智能体(1) | `priority_*` 参数 |
| **超时检测** | 源年龄 > 超时 则剔除 | 键盘: 0.3s, 手柄: 0.5s, 智能体: 0.5s |
| **源选择** | 选择有效源中优先级最高的 | 平局: 选择最新消息 |
| **安全输出** | 无有效源时发布零速度 | Twist(linear.x=0, ...) |
| **调试输出** | 发布当前活动源名称 | `/tello/mux/active_source` (String) |

### 3. 键盘控制功能 (keyboard_control_node.py)

| 功能 | 说明 | 快捷键 |
|------|------|--------|
| **前后移动** | 线性速度控制 | W/S |
| **左右平移** | 侧向速度控制 | A/D |
| **垂直移动** | 升降速度控制 | ↑/↓ 或 Space/Shift |
| **旋转控制** | 角速度控制 | Q/E |
| **加速度限制** | 平滑加速，防止抖动 | max_linear_accel: 0.5 m/s², max_angular_accel: 2.0 rad/s² |
| **离散命令** | 一键执行预设动作 | T(takeoff), L(land), F(flip), R(reset) 等 |

### 4. 手柄控制功能 (gamepad_marshall_node.py)

| 功能 | 说明 | 手柄按键 |
|------|------|---------|
| **摇杆速度** | 双摇杆映射到速度 | 左摇杆(Twist.linear), 右摇杆(Twist.angular) |
| **按键命令** | 快捷命令 | A(takeoff), B(land), X(flip), Y(reset) 等 |
| **死区处理** | 摇杆死区 | deadzone: 0.2 |
| **MUX 路由** | 速度命令通过 MUX | `/tello/cmd_vel_joy` |

---

## 多输入仲裁机制

### 仲裁算法

```python
def arbitrate(now_time):
    # 第1步：检查源新鲜度 (age ≤ timeout)
    valid_sources = []
    for source in [keyboard, joy, agent]:
        age = (now_time - source.last_time).to_sec()
        if age <= source.timeout:
            valid_sources.append(source)
        else:
            log_warn(f"source timeout: {source.name}")
    
    # 第2步：从有效源中选择最高优先级
    if not valid_sources:
        return ZERO_TWIST  # 安全回退
    
    selected = max(valid_sources, 
                   key=lambda s: (s.priority, s.last_time))
    return selected.last_message
```

### 仲裁决策矩阵

| 情况 | 键盘 | 手柄 | 智能体 | 输出 | 说明 |
|------|------|------|--------|------|------|
| 仅键盘活跃 | ✓ | ✗ | ✗ | 键盘 | 用户直接控制 |
| 仅手柄活跃 | ✗ | ✓ | ✗ | 手柄 | 摇杆遥控 |
| 仅智能体活跃 | ✗ | ✗ | ✓ | 智能体 | 自主导航 |
| 键盘+手柄 | ✓ | ✓ | ✗ | 键盘 | 键盘优先 (p=3) |
| 键盘+智能体 | ✓ | ✗ | ✓ | 键盘 | 键盘优先 (p=3) |
| 手柄+智能体 | ✗ | ✓ | ✓ | 手柄 | 手柄优先 (p=2) |
| 全部活跃 | ✓ | ✓ | ✓ | 键盘 | 键盘最高优先级 |
| 全部超时 | ✗ | ✗ | ✗ | 零速度 | 安全停止 |

### 超时机制

```
时间轴 (以键盘为例):
├─ t=0s: 接收键盘消息 A, last_time=0s
├─ t=0.15s: 仲裁检查，age=0.15s ≤ 0.3s → 有效
├─ t=0.3s: 仲裁检查，age=0.3s ≤ 0.3s → 有效（临界）
├─ t=0.31s: 仲裁检查，age=0.31s > 0.3s → 超时！
│           日志: "[mux] source timeout: keyboard (age=0.31s, limit=0.3s)"
│           如果只有键盘：发布 ZERO_TWIST
└─ t=0.5s: 继续发布零速度，直到收到新消息
```

---

## 启动操作指南

### 前置条件

```bash
# 确保在工作空间根目录
cd /home/qczk/rmtt_ws

# 构建系统（首次或修改后）
source devel/setup.bash
catkin_make --pkg tello_control_system tello_driver

# 检查依赖
rosdep install --from-paths src --ignore-src -r -y
```

### 启动顺序（推荐）

#### **方式一：完整系统启动（推荐用于开发/测试）**

```bash
# 终端1：启动主系统（包含驱动、MUX、键盘）
cd /home/qczk/rmtt_ws
source devel/setup.bash
roslaunch tello_control_system tello_system.launch

# 输出示例：
# [INFO] [tello_driver_node]: Connected to Tello
# [INFO] [tello_control_mux_node]: [mux] started: output=/tello/cmd_vel, ...
# [INFO] [keyboard_control_node]: Keyboard control initialized
```

#### **方式二：启用游戏手柄**

```bash
# 终端1：启动系统（启用手柄）
roslaunch tello_control_system tello_system.launch enable_joy:=true joy_dev:=/dev/input/js0

# 自动包含：
# - Tello 驱动
# - Control MUX
# - 键盘节点
# - Joy 节点 + Gamepad Marshall
```

#### **方式三：仅键盘控制**

```bash
# 终端1：启动系统（禁用手柄）
roslaunch tello_control_system tello_system.launch enable_joy:=false
```

#### **方式四：传统分步启动（调试用）**

```bash
# 终端1：启动驱动
cd /home/qczk/rmtt_ws
source devel/setup.bash
roslaunch tello_driver tello_node.launch

# 终端2：启动 MUX
roslaunch tello_control_system control_mux.launch

# 终端3：启动键盘
rosrun tello_control_system keyboard_control_node.py

# 可选 - 终端4：启动手柄
roslaunch tello_driver joy_teleop.launch
```

### 启动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enable_joy` | `false` | 启用游戏手柄输入 |
| `joy_dev` | `/dev/input/gamepads/js_dft` | 手柄设备路径 |
| `tello_ip` | `192.168.10.1` | 飞行器 IP 地址 |
| `tello_cmd_server_port` | `8889` | 飞行器命令端口 |
| `local_vid_server_port` | `6038` | 本地视频服务端口 |

### 验证系统启动成功

```bash
# 检查节点是否运行
rosnode list
# 输出应包括：
# /tello_driver_node
# /tello_control_mux_node
# /keyboard_control_node
# /joy_node (if enable_joy=true)
# /gamepad_marshall

# 检查话题是否发布
rostopic list | grep tello
# 输出应包括：
# /tello/cmd_vel
# /tello/cmd_vel_keyboard
# /tello/cmd_vel_joy
# /tello/cmd_vel_agent
# /tello/mux/active_source
# /tello/status

# 实时查看活动输入源
rostopic echo /tello/mux/active_source
# 输出示例：
# data: "keyboard"  # 当按下 W/S/A/D 时
# data: "joy"       # 当移动手柄摇杆时
# data: "none"      # 所有输入超时时
```

---

## Agent接入指南

### 概述

智能体（Agent）是 RMTT 系统的第三个控制源，用于自主导航和学习控制。Agent 通过在 `/tello/cmd_vel_agent` 话题上发布 `geometry_msgs/Twist` 消息来与系统集成。

### Agent 的优先级和超时

```yaml
Agent 特性：
├─ 优先级：1 (最低)
│  └─ 被键盘(3)和手柄(2)中断
├─ 超时：0.5 秒
│  └─ 0.5s 内无消息 → 自动剔除，转向其他源
└─ 用途：
   ├─ 学习控制（RL/IL）
   ├─ 路径规划
   ├─ 自主避障
   └─ 试验性飞行行为
```

### 接入步骤

#### **步骤1：理解话题接口**

```python
import rospy
from geometry_msgs.msg import Twist

# Agent 必须发布到此话题
cmd_vel_pub = rospy.Publisher('/tello/cmd_vel_agent', Twist, queue_size=20)

# Twist 消息结构：
# geometry_msgs/Twist:
#   linear:
#     x: 前/后（m/s）， 范围：[-1.0, 1.0]
#     y: 左/右（m/s）， 范围：[-1.0, 1.0]
#     z: 上/下（m/s）， 范围：[-1.0, 1.0]
#   angular:
#     x: 前/后翻滚（rad/s）
#     y: 左/右俯仰（rad/s）
#     z: 垂直偏航（rad/s），范围：[-6.28, 6.28]
```

#### **步骤2：实现 Agent 节点**

**示例1：简单的 Agent (速度发布)**

```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

def simple_agent():
    rospy.init_node('simple_agent')
    pub = rospy.Publisher('/tello/cmd_vel_agent', Twist, queue_size=20)
    
    rate = rospy.Rate(30)  # 30Hz，与 MUX 同步
    
    rospy.loginfo("Simple Agent: publishing forward motion")
    
    while not rospy.is_shutdown():
        cmd = Twist()
        cmd.linear.x = 0.5  # 向前移动，0.5 m/s
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.z = 0.0
        
        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    simple_agent()
```

**示例2：智能体（轨迹跟踪）**

```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math

class TrajectoryAgent:
    def __init__(self):
        rospy.init_node('trajectory_agent')
        self.pub = rospy.Publisher('/tello/cmd_vel_agent', Twist, queue_size=20)
        self.start_time = time.time()
        self.rate = rospy.Rate(30)
    
    def compute_control(self):
        """计算控制命令（示例：圆形轨迹）"""
        t = time.time() - self.start_time
        
        # 圆形轨迹参数
        radius = 0.5
        period = 10.0
        
        # 极坐标到直角坐标
        x = radius * math.cos(2 * math.pi * t / period)
        z = radius * math.sin(2 * math.pi * t / period)
        
        # 速度微分
        vx = -radius * (2 * math.pi / period) * math.sin(2 * math.pi * t / period)
        vz = radius * (2 * math.pi / period) * math.cos(2 * math.pi * t / period)
        
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.z = vz
        return cmd
    
    def run(self):
        rospy.loginfo("Trajectory Agent: flying circle pattern")
        while not rospy.is_shutdown():
            cmd = self.compute_control()
            self.pub.publish(cmd)
            self.rate.sleep()

if __name__ == '__main__':
    agent = TrajectoryAgent()
    agent.run()
```

**示例3：学习型 Agent（神经网络控制）**

```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from tello_driver.msg import TelloStatus
import numpy as np

class LearningAgent:
    def __init__(self, model_path=None):
        rospy.init_node('learning_agent')
        
        # 发布速度命令到 Agent 输入
        self.pub_cmd = rospy.Publisher('/tello/cmd_vel_agent', Twist, queue_size=20)
        
        # 订阅飞行器状态
        self.sub_status = rospy.Subscriber('/tello/status', TelloStatus, self.cb_status)
        
        # 模型加载（示例）
        self.model = self.load_model(model_path)
        self.rate = rospy.Rate(30)
    
    def cb_status(self, msg):
        """接收飞行器状态"""
        self.current_state = {
            'battery': msg.battery,
            'height': msg.height,
        }
    
    def load_model(self, path):
        """加载训练好的策略模型"""
        # 示例：从文件或 PyTorch/TF 模型加载
        return None
    
    def infer_action(self, state):
        """推理：状态 → 动作"""
        # 模型前向传播
        action = self.model.predict(state)
        return action  # [vx, vy, vz, omega_z]
    
    def run(self):
        rospy.loginfo("Learning Agent: running policy inference")
        while not rospy.is_shutdown():
            if hasattr(self, 'current_state'):
                # 推理得到动作
                action = self.infer_action(self.current_state)
                
                # 转换为 Twist 消息
                cmd = Twist()
                cmd.linear.x = action[0]
                cmd.linear.y = action[1]
                cmd.linear.z = action[2]
                cmd.angular.z = action[3]
                
                self.pub_cmd.publish(cmd)
            
            self.rate.sleep()

if __name__ == '__main__':
    agent = LearningAgent()
    agent.run()
```

#### **步骤3：启动 Agent 并验证**

```bash
# 终端1：启动主系统
roslaunch tello_control_system tello_system.launch

# 终端2：启动你的 Agent 节点
python3 path/to/your_agent_node.py

# 或通过 rosrun
rosrun your_package your_agent_node.py

# 验证 Agent 是否在发布
rostopic echo /tello/cmd_vel_agent
# 输出应显示 Twist 消息

# 观察 MUX 是否切换到 Agent
rostopic echo /tello/mux/active_source
# 当手柄/键盘不活跃时，应输出: data: "agent"
```

### 与其他源的交互

#### **场景1：手动中断 Agent（键盘优先）**

```
时间线：
├─ t=0s: Agent 启动，开始发布速度
├─ [MUX 输出 Agent 速度]
├─ t=5s: 用户按下键盘 'W'
├─ [MUX 立即切换到键盘] ← Agent 优先级低
├─ [MUX 输出键盘速度，Agent 被中断]
└─ [用户放开 'W']
  └─ t=5.35s: Agent 重新激活（键盘超时 0.3s）
```

**日志输出示例：**

```
[INFO] [tello_control_mux_node]: [mux] source switched: agent → keyboard
[INFO] [keyboard_control_node]: Key pressed: W (forward)
[WARN] [tello_control_mux_node]: [mux] source timeout: keyboard (age=0.31s, limit=0.3s)
[INFO] [tello_control_mux_node]: [mux] source switched: keyboard → agent
```

#### **场景2：Agent 故障恢复**

```
时间线：
├─ t=0s: Agent 正常运行
├─ t=3s: Agent 崩溃/断连
├─ t=3.0-3.5s: MUX 继续发布 Agent 最后的消息
├─ t=3.51s: 超时！发布 ZERO_TWIST
│           日志: "[mux] source timeout: agent (age=0.51s, limit=0.5s)"
├─ [飞行器停止]
└─ t=4s: Agent 重启，恢复发布
  └─ [MUX 恢复输出 Agent 命令]
```

### Agent 开发最佳实践

| 实践 | 说明 |
|------|------|
| **发布频率 ≥ 30Hz** | 与 MUX 同步，确保超时前收到新消息 |
| **超时预留** | 发布频率应 ≥ 1/timeout = 2Hz，推荐 10Hz+ |
| **安全命令** | 启动时发布 ZERO_TWIST，避免突然加速 |
| **状态监听** | 订阅 `/tello/status` 获取实时反馈 |
| **故障隔离** | 崩溃时自动恢复（MUX 超时机制保护） |
| **调试模式** | 监听 `/tello/mux/active_source` 验证控制权 |

### Agent 接入检查清单

- [ ] Agent 节点创建完成，可以正常启动
- [ ] 发布话题正确：`/tello/cmd_vel_agent`
- [ ] 消息类型正确：`geometry_msgs/Twist`
- [ ] 发布频率 ≥ 10Hz（推荐 ≥ 30Hz）
- [ ] 初始命令为零速度（避免突然启动）
- [ ] 订阅 `/tello/status` 获取状态反馈
- [ ] 实现错误处理和自动恢复机制
- [ ] 测试与键盘/手柄的优先级切换
- [ ] 验证超时机制（停止发布 → MUX 发布零速度）
- [ ] 集成测试通过（无话题冲突、无命令循环）

---

## 话题接口定义

### 输入话题

#### `/tello/cmd_vel_keyboard`
- **类型**：`geometry_msgs/Twist`
- **发布者**：`keyboard_control_node.py`
- **频率**：30Hz
- **用途**：键盘输入速度命令

#### `/tello/cmd_vel_joy`
- **类型**：`geometry_msgs/Twist`
- **发布者**：`gamepad_marshall_node.py`
- **频率**：30Hz (来自 joy_node)
- **用途**：游戏手柄输入速度命令

#### `/tello/cmd_vel_agent`
- **类型**：`geometry_msgs/Twist`
- **发布者**：用户自定义智能体节点
- **频率**：推荐 ≥ 10Hz
- **用途**：自主控制系统输入

### 输出话题

#### `/tello/cmd_vel`
- **类型**：`geometry_msgs/Twist`
- **发布者**：`tello_control_mux_node.py`
- **频率**：30Hz
- **用途**：最终仲裁后的速度命令到驱动

#### `/tello/mux/active_source`
- **类型**：`std_msgs/String`
- **发布者**：`tello_control_mux_node.py`
- **频率**：1Hz (变化时立即发布)
- **用途**：调试用，显示当前活动输入源
- **值**：`"keyboard"` | `"joy"` | `"agent"` | `"none"`

### 离散命令话题

| 话题 | 消息类型 | 发布者 | 用途 |
|------|---------|--------|------|
| `/tello/takeoff` | `std_msgs/Empty` | 键盘(T), 手柄(A) | 起飞 |
| `/tello/land` | `std_msgs/Empty` | 键盘(L), 手柄(B) | 降落 |
| `/tello/reset` | `std_msgs/Empty` | 键盘(R), 手柄(Y) | 重置 |
| `/tello/flip` | `std_msgs/UInt8` | 键盘(F), 手柄(X) | 翻滚(0-4) |
| `/tello/throw_takeoff` | `std_msgs/Empty` | 手柄(LB) | 抛起 |
| `/tello/palm_land` | `std_msgs/Empty` | 手柄(RB) | 掌心降落 |
| `/tello/flattrim` | `std_msgs/Empty` | 键盘(E) | 水平校准 |
| `/tello/fast_mode` | `std_msgs/Empty` | 手柄(Start) | 快速模式 |

### 状态话题

#### `/tello/status`
- **类型**：`Int16` (或自定义 TelloStatus)
- **发布者**：`tello_driver_node.py`
- **频率**：~5-10Hz
- **用途**：飞行器状态（电池百分比、高度等）

#### `/tello/image_raw/compressed`
- **类型**：`sensor_msgs/CompressedImage`
- **发布者**：`tello_driver_node.py`
- **频率**：~30fps
- **用途**：压缩视频流

---

## 安全机制

### 分层安全设计

```
┌──────────────────────────────────────────────┐
│ 第1层：应用层（Agent/Keyboard/Gamepad）     │
│ └─ Agent 自检：崩溃恢复、目标监视          │
│ └─ Keyboard：加速度限制                      │
└──────────────────────────────────────────────┘
                    ▼
┌──────────────────────────────────────────────┐
│ 第2层：仲裁层（Control MUX）                │
│ └─ 优先级仲裁：防止低优先级控制干扰        │
│ └─ 超时检测：源失效 → 自动剔除              │
│ └─ 零速度回退：无有效源 → 停止             │
└──────────────────────────────────────────────┘
                    ▼
┌──────────────────────────────────────────────┐
│ 第3层：驱动层（Tello Driver）              │
│ └─ 看门狗：cmd_vel 0.5s 超时 → 停止       │
│ └─ UDP 连接管理：链路中断自动恢复          │
└──────────────────────────────────────────────┘
                    ▼
┌──────────────────────────────────────────────┐
│ 第4层：飞行器固件                            │
│ └─ 硬件限制：速度/角速度最大值              │
│ └─ 跌落保护：自动高度限制                    │
└──────────────────────────────────────────────┘
```

### 键盘加速度限制

```python
# 参数定义
max_linear_accel = 0.5  # m/s²
max_angular_accel = 2.0  # rad/s²

# 加速度限制算法（每 1/30s 一次）
dt = 1.0 / 30.0  # 0.033s
new_vel = clamp(target_vel, prev_vel, max_accel * dt)
```

**效果：** 防止键盘输入导致的突变加速

### MUX 超时保护

```
MUX 主循环（30Hz）：
├─ 接收 3 个输入源的最新消息
├─ 计算每个源的消息年龄
├─ 如果 age > timeout → 标记为 timeout，日志警告
├─ 从有效源中选择最高优先级
├─ 发布选中源或 ZERO_TWIST
└─ 发布活动源调试信息
```

**超时值：**
- 键盘：0.3s（用户响应延迟）
- 手柄：0.5s（网络/蓝牙延迟）
- Agent：0.5s（计算延迟）

### 驱动层看门狗

```c
// 伪代码
while(drone_running) {
    if (time_since_cmd_vel > 0.5s) {
        send_zero_velocity_to_drone();
        log_warning("cmd_vel timeout");
    }
}
```

**触发条件：**
- MUX 故障（停止发布）
- 网络中断
- 驱动节点崩溃

---

## 故障诊断

### 常见问题

#### **问题1：MUX 无输出（飞行器不移动）**

**诊断步骤：**

```bash
# 步骤1：检查 MUX 节点是否运行
rosnode list | grep mux
# 输出：/tello_control_mux_node ✓

# 步骤2：检查输入话题是否有消息
rostopic echo /tello/cmd_vel_keyboard
# 如果无输出 → 键盘节点可能未启动或崩溃

# 步骤3：检查 MUX 输出
rostopic echo /tello/cmd_vel
# 如果显示全零 Twist → MUX 未选中任何源

# 步骤4：查看活动源
rostopic echo /tello/mux/active_source
# 输出：data: "none" → 没有有效输入源
# 输出：data: "keyboard" → 有效的键盘输入

# 步骤5：检查 MUX 日志
rosnode info /tello_control_mux_node
# 或
rostopic echo /rosout | grep mux
```

**解决方案：**

| 原因 | 解决方案 |
|------|---------|
| 键盘节点未启动 | `rosrun tello_control_system keyboard_control_node.py` |
| MUX 未启动 | `roslaunch tello_control_system control_mux.launch` |
| 输入源超时 | 按下键盘快捷键（如 W/S/A/D）或移动手柄 |
| 驱动节点故障 | 重启驱动：`roslaunch tello_driver tello_node.launch` |

#### **问题2：键盘输入不响应**

**诊断步骤：**

```bash
# 检查键盘节点是否在列表中
rosnode list | grep keyboard

# 检查是否有消息发布
rostopic hz /tello/cmd_vel_keyboard

# 检查具体消息内容
rostopic echo /tello/cmd_vel_keyboard

# 检查焦点是否在终端窗口（某些 Linux 系统需要）
xdotool getactivewindow getwindowname
```

**解决方案：**

- 确保终端窗口有焦点
- 重启键盘节点：`rosrun tello_control_system keyboard_control_node.py`
- 检查 `/etc/X11/xorg.conf` 或键盘驱动

#### **问题3：手柄优先级低，无法中断 Agent**

**原因分析：**

```
当前优先级：keyboard(3) > joy(2) > agent(1)
如果 Agent 在持续发布：
├─ Agent 优先级为 1
├─ Joy 优先级为 2
├─ 因此 Joy 应该能中断 Agent ✓

排查问题：
```

**诊断步骤：**

```bash
# 步骤1：确认 Joy 节点在运行
rosnode list | grep joy

# 步骤2：检查 Joy 话题是否发布
rostopic hz /joy

# 步骤3：观察 MUX 源切换
rostopic echo /tello/mux/active_source
# 移动手柄 → 应该看到 agent → joy 切换

# 步骤4：检查发布频率
rostopic hz /tello/cmd_vel_joy
# 应该 ≥ 10Hz
```

**解决方案：**

| 原因 | 解决方案 |
|------|---------|
| Joy 节点未启动 | `roslaunch tello_control_system tello_system.launch enable_joy:=true` |
| Joy 频率太低 | 增加 Joy 节点的发布频率 |
| 优先级配置错误 | 检查 `control_mux.launch` 中的 `priority_*` 参数 |
| Joy 话题映射错误 | 验证 `joy_teleop.launch` 中的 `cmd_vel` remap 指向 `/tello/cmd_vel_joy` |

#### **问题4：Agent 被键盘中断后无法恢复**

**原因：** 键盘消息持续发布或超时设置不当

**诊断步骤：**

```bash
# 检查键盘超时是否过长
rostopic hz /tello/cmd_vel_keyboard
# 如果频率 < 10Hz，超时可能未触发

# 观察 MUX 日志
rostopic echo /rosout | grep "source timeout"
# 应该看到：[mux] source timeout: keyboard (age=0.31s, ...)

# 手动停止键盘
# 在键盘节点终端按 Ctrl+C
# 验证 Agent 是否恢复
rostopic echo /tello/mux/active_source
```

**解决方案：**

- 减小键盘超时值（`timeout_keyboard` 参数）
- 或手动杀死键盘节点：`rosnode kill /keyboard_control_node`
- 或在键盘节点中实现"释放"机制（停止发布而非发布零速度）

#### **问题5：多源冲突导致飞行不稳定**

**现象：** 飞行抖动、速度波动、方向不稳定

**原因分析：**

```
可能原因：
1. 多个源同时发布 → MUX 频繁切换
2. 消息频率不一致 → 时延变化
3. 优先级配置不当
4. 超时值设置不合理
```

**诊断步骤：**

```bash
# 步骤1：检查消息频率
rostopic hz /tello/cmd_vel_keyboard /tello/cmd_vel_joy /tello/cmd_vel_agent

# 步骤2：实时监控源切换
rostopic echo /tello/mux/active_source
# 观察是否频繁切换

# 步骤3：检查最终输出（应该平稳）
rostopic echo /tello/cmd_vel
# 观察速度值是否有跳跃

# 步骤4：查看 MUX 日志（源切换/超时）
rostopic echo /rosout | grep "mux"
```

**解决方案：**

| 问题 | 解决 |
|------|------|
| 频繁源切换 | 增加超时值（如 0.5s → 1.0s） |
| 频率不一致 | 统一所有源的发布频率为 30Hz |
| 优先级不清 | 明确制定优先级（推荐：user > teleop > auto） |
| 网络延迟 | 检查网络连接，或在本地机器上运行节点 |

### 调试工具

#### **实时监控面板**

```bash
#!/bin/bash
# save as: debug_mux.sh

while true; do
    clear
    echo "=== RMTT MUX System Status ==="
    echo ""
    echo "Running Nodes:"
    rosnode list | grep -E "tello|keyboard|joy"
    echo ""
    echo "Active Source:"
    rostopic echo -n 1 /tello/mux/active_source
    echo ""
    echo "Input Frequencies (Hz):"
    echo -n "  Keyboard: "; rostopic hz -w 3 /tello/cmd_vel_keyboard 2>/dev/null | tail -1
    echo -n "  Joy: "; rostopic hz -w 3 /tello/cmd_vel_joy 2>/dev/null | tail -1
    echo -n "  Agent: "; rostopic hz -w 3 /tello/cmd_vel_agent 2>/dev/null | tail -1
    echo ""
    echo "Output Frequency (Hz):"
    echo -n "  Cmd Vel: "; rostopic hz -w 3 /tello/cmd_vel 2>/dev/null | tail -1
    echo ""
    echo "Recent MUX Events:"
    rostopic echo /rosout | grep "mux" | tail -5
    echo ""
    sleep 2
done
```

**使用方法：**

```bash
chmod +x debug_mux.sh
./debug_mux.sh
```

#### **单输入测试**

```bash
# 测试1：仅键盘
roslaunch tello_control_system tello_system.launch enable_joy:=false
# 验证键盘控制有效

# 测试2：仅手柄
roslaunch tello_control_system tello_system.launch enable_joy:=true
# 验证手柄控制有效

# 测试3：仅 Agent（手动发布）
roslaunch tello_control_system tello_system.launch enable_joy:=false
# 在另一个终端：
rostopic pub -r 10 /tello/cmd_vel_agent geometry_msgs/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

---

## 附录

### 系统文件结构

```
/home/qczk/rmtt_ws/
├── src/
│   ├── tello_driver/
│   │   ├── launch/
│   │   │   ├── tello_node.launch          # 驱动启动
│   │   │   ├── joy_teleop.launch          # 手柄配置（已修改）
│   │   │   └── devel.launch               # 开发启动（已修改）
│   │   ├── scripts/
│   │   │   ├── tello_driver_node.py       # 核心驱动
│   │   │   ├── gamepad_marshall_node.py   # 手柄映射
│   │   │   └── ...
│   │   └── ...
│   │
│   ├── tello_control_system/
│   │   ├── launch/
│   │   │   ├── tello_system.launch        # ✨ 统一启动入口
│   │   │   ├── control_mux.launch         # ✨ MUX 启动配置
│   │   │   ├── bringup.launch             # ✨ 传统启动
│   │   │   └── ...
│   │   ├── scripts/
│   │   │   ├── tello_control_mux_node.py  # ✨ 核心MUX节点
│   │   │   ├── keyboard_control_node.py   # 键盘控制
│   │   │   ├── tello_odom_node.py         # 里程计辅助
│   │   │   └── ...
│   │   ├── package.xml                    # ✨ 依赖配置
│   │   ├── CMakeLists.txt                 # ✨ 构建配置
│   │   └── ...
│   └── ...
├── devel/
│   └── setup.bash                         # 环境设置
├── build/
│   └── ...
└── TECHNICAL_DOCUMENTATION.md             # ← 本文档
```

### 快速参考

#### 常用命令

```bash
# 源环境
source /home/qczk/rmtt_ws/devel/setup.bash

# 启动系统
roslaunch tello_control_system tello_system.launch [enable_joy:=true]

# 构建
catkin_make --pkg tello_control_system tello_driver

# 查看节点
rosnode list

# 查看话题
rostopic list

# 查看话题内容
rostopic echo /tello/mux/active_source

# 查看话题频率
rostopic hz /tello/cmd_vel

# 发布测试消息
rostopic pub /tello/cmd_vel_agent geometry_msgs/Twist "linear: {x: 0.5}"
```

#### 键盘快捷键

| 按键 | 功能 |
|------|------|
| W/S | 前/后 |
| A/D | 左/右 |
| ↑/↓ | 上/下 |
| Q/E | 逆时针/顺时针旋转 |
| T | 起飞 (Takeoff) |
| L | 降落 (Land) |
| R | 重置 (Reset) |
| F | 翻滚 (Flip) |
| Space | 应急停止 (或其他) |

#### 手柄按键（示例 Xbox 手柄）

| 按键 | 功能 |
|------|------|
| 左摇杆 | 前/后 + 左/右 |
| 右摇杆 | 旋转 |
| A | 起飞 |
| B | 降落 |
| X | 翻滚 |
| Y | 重置 |
| LB | 抛起 |
| RB | 掌心降落 |
| Start | 快速模式 |

---

## 更新日志

| 版本 | 日期 | 更改内容 |
|------|------|--------|
| 1.0 | 2026-03-26 | 初版，包含完整系统架构、启动指南、Agent接入文档、故障诊断 |

---

## 联系与支持

如有问题或建议，请联系系统维护者或提交 Issue。

---

**文档生成时间：** 2026-03-26  
**系统版本：** RMTT Control System v1.0  
**ROS 版本：** Noetic  
**Python 版本：** 3.8+

