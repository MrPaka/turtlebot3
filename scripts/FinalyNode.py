#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys, select, termios, tty
from pynput import keyboard

class RobotController:

    def __init__(self, cmd_vel) -> None:
        self.ranges = None
        self.cmd_vel = cmd_vel
        self.twist = Twist()

        self.target_speed_x = 0
        self.target_speed_z = 0

        self.speed_x = 0
        self.speed_z = 0

        self.obstacle_thrashold = 0.5

        self.is_stop = False

        self.cur_command = keyboard.Key.space

        self.set_value()

    def laser_callback(self, data):
        if (data != None):
            self.ranges = data.ranges
        else:
            rospy.loginfo('Error get data scan')

    def set_value(self):
        val1 = input('Enter target speed for X: ')
        if val1 == '':
            self.target_speed_x = 0.25
        else:
            self.target_speed_x = float(val1)
        
        val2 = input('Enter target speed for Z: ')
        if val2 == '':
            self.target_speed_z = 0.25
        else:

            self.target_speed_z = float(val2)
        
        val3 = input('Enter the threshold to the obstancle: ')
        if val3 == '':
            self.obstacle_thrashold = 0.5
        else:
            self.obstacle_thrashold = float(val3)

    def move_robot(self, key):
        self.cur_command = key
        if key == keyboard.Key.up:
            self.speed_x = self.target_speed_x
            self.speed_z = 0
        elif key == keyboard.Key.down:
            self.speed_x = -self.target_speed_x
            self.speed_z = 0
        elif key == keyboard.Key.left:
            self.speed_x = 0
            self.speed_z = self.target_speed_z
        elif key == keyboard.Key.right:
            self.speed_x = 0
            self.speed_z = -self.target_speed_z
        elif key == keyboard.Key.space:
            self.speed_x = 0
            self.speed_z = 0
        elif key == 'q':
            self.is_stop = True

        self.twist.linear.x = self.speed_x
        self.twist.angular.z = self.speed_z

        self.cmd_vel.publish(self.twist)


    def obstacle_detection(self):
        if (self.ranges != None):
            min_range = min(self.ranges)
            return min_range < self.obstacle_thrashold
        return False
    
    def bypassing_obstacle(self):

        left_scan = self.ranges[:len(self.ranges)//3]
        right_scan = self.ranges[2*len(self.ranges)//3:]
        front_scan = self.ranges[len(self.ranges)//3:2*len(self.ranges)//3]

        if min(front_scan) < 0.5:
            if min(left_scan) > min(right_scan):
                self.move_robot(keyboard.Key.left)  # Поворот влево
            else:
                self.move_robot(keyboard.Key.right) # Поворот вправо
        else:
            self.move_robot(keyboard.Key.down)  # Вперед

    def Loop(self):

        rate = rospy.Rate(100)

        while not self.is_stop:
            
            self.move_robot(self.cur_command)

            if self.obstacle_detection():
                self.bypassing_obstacle()

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('node')
    try:
        cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        robot = RobotController(cmd_vel)
    
        rospy.Subscriber('/scan', LaserScan, robot.laser_callback)

        keyboard.Listener(on_press=robot.move_robot).start()
        robot.Loop()

        rospy.loginfo('Start node')
    except Exception as ms:
        rospy.logerr(ms)
    finally:
        rospy.loginfo('Stop node')
        rospy.spin()
    
