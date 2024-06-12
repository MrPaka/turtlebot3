#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from pynput import keyboard
from RobotPredictor import *

class RobotController:

    def __init__(self) -> None:

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)    
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        keyboard.Listener(on_press=self.move_robot, 
                        on_release=self.stop_robot).start()
               
        self.robot_predictor = RobotPredictor()
        self.ranges = None
        self.twist = Twist()

        self.target_speed_x = 0
        self.target_speed_z = 0

        self.speed_x = 0
        self.speed_z = 0
        self.dt = 0.01
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
            self.target_speed_z = 1
        else:

            self.target_speed_z = float(val2)
        
        val3 = input('Enter the threshold to the obstancle: ')
        if val3 == '':
            self.obstacle_thrashold = 0.5
        else:
            self.obstacle_thrashold = float(val3)


    def stop_robot(self, key):
        if (key == keyboard.Key.up or key == keyboard.Key.down):
            self.speed_x = 0
        elif (key == keyboard.Key.left or key == keyboard.Key.right):
            self.speed_z = 0

    def move_robot(self, key):
        self.cur_command = key

        if (key == keyboard.Key.up):
            self.speed_x = self.speed_x + self.dt
        elif (key == keyboard.Key.down):
            self.speed_x = self.speed_x - self.dt
        elif (key == keyboard.Key.left):
            self.speed_z = self.speed_z + self.dt
        elif (key == keyboard.Key.right):
            self.speed_z = self.speed_z - self.dt
        elif (key == keyboard.Key.space):
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

        len_range = len(self.ranges)
        front_scan = self.ranges[-(len_range // 18):] + self.ranges[:(len_range // 18)]

        if (min(front_scan) < self.obstacle_thrashold and self.cur_command == keyboard.Key.up):
            self.move_robot(keyboard.Key.space)
            self.robot_predictor.Color = [1.0, 0.0, 0.0]
            rospy.logwarn('Движение в перед не возможно')
        else:
            self.robot_predictor.Color = [0.0, 1.0, 0.0]


    def Loop(self):

        rate = rospy.Rate(10)

        while not self.is_stop:
            if self.obstacle_detection():
                self.bypassing_obstacle()

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('node')
    try:

        robot = RobotController()
        robot.Loop()

        rospy.loginfo('Start node')
    except Exception as ms:
        rospy.logerr(ms)
    finally:
        rospy.loginfo('Stop node')
        rospy.spin()
    
