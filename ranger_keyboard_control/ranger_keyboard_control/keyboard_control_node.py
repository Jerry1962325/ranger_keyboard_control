#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = """
Ranger Mini V3 Keyboard Control
---------------------------
Moving around:
   q    w    e
   a    s    d
        x

w/x : Forward/Backward (Linear X)
a/d : Left/Right (Linear Y - Crab Mode)
q/e : Spin Left/Right (Angular Z)
s   : Stop

t/g : Increase/Decrease max speeds
CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0),   # Forward
    'x': (-1, 0, 0),  # Backward
    'a': (0, 1, 0),   # Left (Crab)
    'd': (0, -1, 0),  # Right (Crab)
    'q': (0, 0, 1),   # Spin Left
    'e': (0, 0, -1),  # Spin Right
    's': (0, 0, 0),   # Stop
}

speedBindings = {
    't': (1.1, 1.1),
    'g': (0.9, 0.9),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    settings = saveTerminalSettings()
    rclpy.init()
    
    node = rclpy.create_node('ranger_keyboard_control')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    speed = 0.5 # Linear speed m/s
    turn = 1.0  # Angular speed rad/s
    
    x = 0.0
    y = 0.0
    th = 0.0
    status = 0

    try:
        print(msg)
        print(f"Current: speed {speed}, turn {turn}")
        
        while True:
            key = getKey(settings)
            
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                th = moveBindings[key][2]
                
                # If stopping, print message
                if x == 0 and y == 0 and th == 0:
                    print("Stop")
                else:
                    print(f"Command: x={x}, y={y}, th={th}")

            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(f"Current: speed {speed}, turn {turn}")
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
                continue

            else:
                x = 0.0
                y = 0.0
                th = 0.0
                if (key == '\x03'): # CTRL-C
                    break

            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        restoreTerminalSettings(settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
