#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys, select, termios, tty
from pynput import keyboard

cmd_vel = None
scan_data = None
is_key = False
speed_x = 0
speed_z = 0
twist = Twist()

last_key = 'stop'

def laser_callback(data):
    global scan_data
    scan_data = data

# Функция для получения команд с клавиатуры
def getKey():
    global last_key
    global is_key
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    last_key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return last_key

velocity = 0.26

def move_robot(): 
    global twist
    global speed_x, speed_z
    global last_key
    if last_key == 'w':
        speed_x = velocity
        speed_z = 0
    elif last_key == 's':
        speed_x = -velocity
        speed_z = 0
    elif last_key == 'd':
        speed_x = 0
        speed_z = -velocity
    elif last_key == 'a':
        speed_x = 0
        speed_z = velocity
    elif last_key == 'stop':
        speed_x = 0
        speed_z = 0
    elif last_key == 'space':
        speed_x = 0
        speed_z = 0

    twist.linear.x = speed_x
    twist.linear.z = speed_z

    cmd_vel.publish(twist)

def bypassing_obstacle():
    global scan_data, last_key
    if scan_data is None:
        return 's'

    left_scan = scan_data.ranges[:len(scan_data.ranges)//3]
    right_scan = scan_data.ranges[2*len(scan_data.ranges)//3:]
    front_scan = scan_data.ranges[len(scan_data.ranges)//3:2*len(scan_data.ranges)//3]

    if min(front_scan) < 0.5:
        if min(left_scan) > min(right_scan):
            last_key = 'a'  # Поворот влево
        else:
            last_key = 'd'  # Поворот вправо
    last_key = 'w'  # Вперед


def obstacle_detection(ranges):
    obstacle_thrashold = 0.5
    min_range = min(ranges)
    return min_range < obstacle_thrashold

def on_press(key):
    rospy.loginfo('space')

def Loop():
    rate = rospy.Rate(10)
    global last_key
    while True:
        
        getKey()
        if last_key == 'q':
            break
        move_robot()

        if obstacle_detection(scan_data.ranges) and last_key != 'stop':
            last_key = 'stop'
            bypassing_obstacle()
            rospy.loginfo('bypassing')

        rate.sleep()

# if __name__ == '__main__':
#     settings = termios.tcgetattr(sys.stdin)
#     rospy.init_node("node")
#     rospy.loginfo("Start node")
#     cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#     rospy.Subscriber('/scan', LaserScan, laser_callback)

#     listener = keyboard.Listener(
#         on_press=on_press
#     )
#     listener.start() 

#     Loop()

#     rospy.loginfo("Stop node")
#     rospy.sleep(1.0)
#     rospy.spin()

