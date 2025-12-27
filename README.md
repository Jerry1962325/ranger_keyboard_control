# Ranger Keyboard Control

ROS 2 键盘控制包，用于控制 Ranger Mini V3 机器人。

## 功能

- 键盘控制机器人运动
- 角度控制节点
- 键盘控制节点

## 环境要求

- ROS 2 (推荐 Jazzy 或更新版本)
- Python 3

## 依赖

- rclpy
- geometry_msgs

## 安装

```bash
# 克隆仓库到ROS工作空间的src目录下
cd ~/ros2_ws/src
git clone <repository-url>

# 编译工作空间
cd ~/ros2_ws
colcon build

# 设置环境变量
source install/setup.bash
```

## 使用

### 启动键盘控制节点

```bash
ros2 run ranger_keyboard_control keyboard_control
```

### 启动角度控制节点

```bash
ros2 run ranger_keyboard_control angle_control
```

## 节点说明

### keyboard_control_node
- 监听键盘输入
- 发布运动控制指令

### angle_control_node
- 处理角度相关的控制逻辑

## 话题

运行以下命令查看节点发布和订阅的话题：

```bash
ros2 node info /keyboard_control_node
ros2 node info /angle_control_node
```