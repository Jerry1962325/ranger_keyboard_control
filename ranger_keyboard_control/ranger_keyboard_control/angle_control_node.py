#!/usr/bin/env python3
"""
Ranger Mini V3 角度控制节点
可以指定任意角度进行斜向移动

控制方式:
- 数字键 0-9: 快速选择角度 (0°, 40°, 80°, ... 360°)
- 左右方向键: 微调角度 ±5°
- 上下方向键: 调整速度
- 空格: 开始/停止移动
- R: 原地旋转模式
- ESC/Q: 退出
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select
import math

# ANSI 颜色
class Colors:
    RESET = '\033[0m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    BOLD = '\033[1m'


class AngleControlNode(Node):
    def __init__(self):
        super().__init__('angle_control_node')
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 控制参数
        self.angle = 0.0          # 移动角度 (度)
        self.speed = 0.3          # 移动速度 (m/s)
        self.angular_speed = 0.5  # 旋转速度 (rad/s)
        self.is_moving = False    # 是否正在移动
        self.rotate_mode = False  # 原地旋转模式
        self.rotate_direction = 0 # 旋转方向: -1左, 0停, 1右
        
        # 速度限制
        self.max_speed = 1.0
        self.min_speed = 0.1
        self.speed_step = 0.1
        self.angle_step = 5.0
        
        # 终端设置
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        
    def get_key(self):
        """非阻塞读取键盘输入"""
        try:
            tty.setcbreak(self.fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)  # 50ms timeout
            if rlist:
                key = sys.stdin.read(1)
                # 处理方向键 (ESC序列)
                if key == '\x1b':
                    # 非阻塞读取后续字符
                    import fcntl
                    import os
                    fl = fcntl.fcntl(self.fd, fcntl.F_GETFL)
                    fcntl.fcntl(self.fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
                    try:
                        key += sys.stdin.read(2)
                    except:
                        pass
                    fcntl.fcntl(self.fd, fcntl.F_SETFL, fl)
                return key
            return None
        finally:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.old_settings)
    
    def normalize_angle(self, angle):
        """将角度归一化到 0-360"""
        while angle < 0:
            angle += 360
        while angle >= 360:
            angle -= 360
        return angle
    
    def draw_compass(self):
        """绘制方向罗盘"""
        # 清屏
        print('\033[2J\033[H', end='')
        
        print(f"{Colors.CYAN}{Colors.BOLD}")
        print("╔═══════════════════════════════════════════════════════════╗")
        print("║         Ranger Mini V3 角度控制 - 斜向移动                ║")
        print("╚═══════════════════════════════════════════════════════════╝")
        print(f"{Colors.RESET}")
        
        # 计算指针位置
        angle_rad = math.radians(self.angle)
        # 在终端中，y轴是反的，所以用负号
        # 0度=前，90度=右，180度=后，270度=左
        px = math.sin(angle_rad)
        py = -math.cos(angle_rad)
        
        # 罗盘大小
        radius = 4
        
        # 绘制罗盘
        compass = []
        for y in range(-radius-1, radius+2):
            row = ""
            for x in range(-radius*2-2, radius*2+3):
                # 归一化坐标
                nx = x / (radius * 2)
                ny = y / radius
                dist = math.sqrt(nx*nx + ny*ny)
                
                # 指针位置
                pointer_dist = math.sqrt((nx-px)**2 + (ny-py)**2)
                
                if pointer_dist < 0.3 and self.is_moving:
                    row += f"{Colors.RED}●{Colors.RESET}"
                elif pointer_dist < 0.3:
                    row += f"{Colors.GREEN}●{Colors.RESET}"
                elif abs(dist - 1.0) < 0.15:
                    # 圆周
                    if abs(y) < 0.5 and x > 0:
                        row += f"{Colors.YELLOW}E{Colors.RESET}"  # 东 (90°)
                    elif abs(y) < 0.5 and x < 0:
                        row += f"{Colors.YELLOW}W{Colors.RESET}"  # 西 (270°)
                    elif y < 0 and abs(x) < 1:
                        row += f"{Colors.YELLOW}N{Colors.RESET}"  # 北 (0°/前)
                    elif y > 0 and abs(x) < 1:
                        row += f"{Colors.YELLOW}S{Colors.RESET}"  # 南 (180°/后)
                    else:
                        row += f"{Colors.BLUE}○{Colors.RESET}"
                elif dist < 1.0:
                    # 内部 - 画线指向方向
                    line_dist = abs(nx * py - ny * px)  # 点到线的距离
                    if line_dist < 0.15 and (nx * px + ny * py) > 0:
                        row += f"{Colors.GREEN}·{Colors.RESET}"
                    else:
                        row += " "
                else:
                    row += " "
            compass.append(row)
        
        for line in compass:
            print(f"        {line}")
        
        # 状态显示
        print()
        status = f"{Colors.GREEN}● 移动中{Colors.RESET}" if self.is_moving else f"{Colors.YELLOW}○ 停止{Colors.RESET}"
        if self.rotate_mode:
            status = f"{Colors.MAGENTA}◐ 旋转模式{Colors.RESET}"
        
        print(f"  状态: {status}")
        print(f"  角度: {Colors.CYAN}{self.angle:>6.1f}°{Colors.RESET}  (0°=前, 90°=右, 180°=后, 270°=左)")
        print(f"  速度: {Colors.CYAN}{self.speed:>6.2f}{Colors.RESET} m/s")
        
        # 计算实际的 vx, vy
        if self.is_moving and not self.rotate_mode:
            vx = self.speed * math.cos(math.radians(self.angle))
            vy = -self.speed * math.sin(math.radians(self.angle))  # ROS中y轴左为正
            print(f"  Vx:   {Colors.CYAN}{vx:>6.2f}{Colors.RESET} m/s (前后)")
            print(f"  Vy:   {Colors.CYAN}{vy:>6.2f}{Colors.RESET} m/s (左右)")
        
        print()
        print(f"{Colors.WHITE}─────────────────── 控制说明 ───────────────────{Colors.RESET}")
        print(f"  {Colors.GREEN}数字 1-9, 0{Colors.RESET}: 快速选择角度")
        print(f"      1=0°(前) 2=40° 3=80° 4=120° 5=160°")
        print(f"      6=200° 7=240° 8=280° 9=320° 0=360°")
        print(f"  {Colors.GREEN}← →{Colors.RESET}: 微调角度 ±{self.angle_step}°")
        print(f"  {Colors.GREEN}↑ ↓{Colors.RESET}: 调整速度 ±{self.speed_step} m/s")
        print(f"  {Colors.GREEN}空格{Colors.RESET}: 开始/停止移动 (旋转模式下停止)")
        print(f"  {Colors.GREEN}R{Colors.RESET}: 原地旋转模式 (← → 控制方向)")
        print(f"  {Colors.GREEN}Q/ESC{Colors.RESET}: 退出")
        print()
    
    def publish_cmd(self):
        """发布控制指令"""
        twist = Twist()
        
        if self.rotate_mode:
            # 原地旋转模式
            twist.angular.z = self.angular_speed * self.rotate_direction
        elif self.is_moving:
            # 斜向移动
            # 0° = 前 (x正), 90° = 右 (y负), 180° = 后 (x负), 270° = 左 (y正)
            angle_rad = math.radians(self.angle)
            twist.linear.x = self.speed * math.cos(angle_rad)
            twist.linear.y = -self.speed * math.sin(angle_rad)  # ROS坐标系y轴左为正
        
        self.publisher.publish(twist)
    
    def run(self):
        """主循环"""
        try:
            self.draw_compass()
            
            while rclpy.ok():
                key = self.get_key()
                
                if key is None:
                    self.publish_cmd()
                    rclpy.spin_once(self, timeout_sec=0)
                    continue
                
                need_redraw = True
                
                # 调试：显示按键
                # print(f"Key: {repr(key)}")
                
                # 退出
                if key in ['q', 'Q', '\x03']:  # q, Ctrl+C
                    break
                
                # 空格 - 开始/停止
                elif key == ' ':
                    if not self.rotate_mode:
                        self.is_moving = not self.is_moving
                    else:
                        self.rotate_direction = 0
                
                # R - 旋转模式
                elif key in ['r', 'R']:
                    self.rotate_mode = not self.rotate_mode
                    self.is_moving = False
                    self.rotate_direction = 0
                
                # 数字键 - 快速选择角度
                elif key == '1':
                    self.angle = 0
                elif key == '2':
                    self.angle = 40
                elif key == '3':
                    self.angle = 80
                elif key == '4':
                    self.angle = 120
                elif key == '5':
                    self.angle = 160
                elif key == '6':
                    self.angle = 200
                elif key == '7':
                    self.angle = 240
                elif key == '8':
                    self.angle = 280
                elif key == '9':
                    self.angle = 320
                elif key == '0':
                    self.angle = 0  # 360° = 0°
                
                # 方向键 - 右箭头
                elif key == '\x1b[C':
                    if self.rotate_mode:
                        self.rotate_direction = -1  # 顺时针
                    else:
                        self.angle = self.normalize_angle(self.angle + self.angle_step)
                # 方向键 - 左箭头
                elif key == '\x1b[D':
                    if self.rotate_mode:
                        self.rotate_direction = 1  # 逆时针
                    else:
                        self.angle = self.normalize_angle(self.angle - self.angle_step)
                # 方向键 - 上箭头
                elif key == '\x1b[A':
                    self.speed = min(self.max_speed, self.speed + self.speed_step)
                # 方向键 - 下箭头
                elif key == '\x1b[B':
                    self.speed = max(self.min_speed, self.speed - self.speed_step)
                
                # A/D 键作为旋转控制 (推荐在旋转模式使用)
                elif key in ['a', 'A']:
                    if self.rotate_mode:
                        self.rotate_direction = 1  # 逆时针
                    else:
                        self.angle = self.normalize_angle(self.angle - self.angle_step)
                elif key in ['d', 'D']:
                    if self.rotate_mode:
                        self.rotate_direction = -1  # 顺时针
                    else:
                        self.angle = self.normalize_angle(self.angle + self.angle_step)
                # W/S 键作为加减速的备选
                elif key in ['w', 'W']:
                    self.speed = min(self.max_speed, self.speed + self.speed_step)
                elif key in ['s', 'S']:
                    self.speed = max(self.min_speed, self.speed - self.speed_step)
                
                else:
                    need_redraw = False
                
                if need_redraw:
                    self.draw_compass()
                
                self.publish_cmd()
                rclpy.spin_once(self, timeout_sec=0)
            
            # 没有按键时，旋转模式下停止旋转
            else:
                if self.rotate_mode:
                    self.rotate_direction = 0
                self.publish_cmd()
                rclpy.spin_once(self, timeout_sec=0)
                
        finally:
            # 停止机器人
            twist = Twist()
            self.publisher.publish(twist)
            # 恢复终端设置
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
            print(f"\n{Colors.YELLOW}已退出角度控制{Colors.RESET}")


def main(args=None):
    rclpy.init(args=args)
    node = AngleControlNode()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, node.old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
