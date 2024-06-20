#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from pynput import keyboard
from std_msgs.msg import String, Int16, ColorRGBA, Float32, Float32MultiArray
import math

# from dynamic_reconfigure.server import Server
# from robot_controller.cfg import params

class RobotController:

    def __init__(self) -> None:
        # Server(params, self.get_param)
        self.color_predict = rospy.Publisher("/color_predict", ColorRGBA, queue_size=10)
        self.command_control = rospy.Publisher("/command_control", String, queue_size=10)
        self.direction = rospy.Publisher("/direction", Int16, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        rospy.Subscriber('/obstacle_thrashold', Float32, self.obs_callback)
        rospy.Subscriber('/coordinates_sector', Float32MultiArray, self.coordinates_sector_callback)
        self.cur_pos = [0, 0, 0]
        keyboard.Listener(on_press=self.move_robot, 
                        on_release=self.stop_robot).start()

        self.ranges = None
        self.obstacle_thrashold = 0.5
        self.command = ("stop", 0)
        self.cur_command = keyboard.Key.space
        self.isObstacle = False
        self.sector = 0

    # def get_param(self, config: params, level):
    #     self.obstacle_thrashold = config["obstacle_thrashold"]
    #     return config        

    def laser_callback(self, data):
        if (data != None):
            self.ranges = data.ranges
        else:
            rospy.loginfo("Error get data scan")

    def obs_callback(self, obs):
        self.obstacle_thrashold = abs(obs.data)

    def coordinates_sector_callback(self, data: Float32MultiArray):
        x = data.data[0]
        y = data.data[1]
        self.sector = int(math.degrees(math.atan2(y, x)))
        if self.sector < 0:
            self.sector += 360

    def stop_robot(self, key):

        if (key == keyboard.Key.up):
            self.command = ("braiking_straight", 1)
        elif (key == keyboard.Key.down):
            self.command = ("braiking_straight", -1)
        elif (key == keyboard.Key.left):
            self.command = ("braiking_rotation", 1)
        elif (key == keyboard.Key.right):
            self.command = ("braiking_rotation", -1)


    def move_robot(self, key):
        self.cur_command = key
        if (key == keyboard.Key.up):
            self.command = ("straight", 1)
        elif (key == keyboard.Key.down):
            self.command = ("straight", -1)
        elif (key == keyboard.Key.left):
            self.command = ("rotation", 1)
        elif (key == keyboard.Key.right):
            self.command = ("rotation", -1)
        elif (key == keyboard.Key.space):
            self.command = ("stop", 0)


    def obstacle_detection(self):
        if (self.ranges != None):
            min_range = min(self.ranges)
            return min_range < self.obstacle_thrashold
        self.color_predict.publish(ColorRGBA(0, 255 ,0, 1))
        return False
    
    def bypassing_obstacle(self):

        # len_range = len(self.ranges)
        # front_scan = self.ranges[-(len_range // 36):] + self.ranges[:(len_range // 36)]
        front_scan = []
        if (self.sector - 10) < 0:
            front_scan = self.ranges[(self.sector - 10):] + self.ranges[:(self.sector + 10)]
        elif (self.sector + 10) > 360:
            front_scan = self.ranges[-(360 - self.sector - 10):] + self.ranges[:(360 - self.sector + 10)]
        else:
            front_scan = self.ranges[(self.sector - 10):(self.sector + 10)]

        if (min(front_scan) < self.obstacle_thrashold):
            self.isObstacle = True
            self.move_robot(keyboard.Key.space)
            self.color_predict.publish(ColorRGBA(255, 0 ,0, 1))
            rospy.logwarn("Движение в перед не возможно")
        else:
            self.color_predict.publish(ColorRGBA(0, 255 ,0, 1))
            self.isObstacle = False


    def Loop(self):

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.obstacle_detection():
                self.bypassing_obstacle()

                
            self.command_control.publish(String(self.command[0]))
            self.direction.publish(Int16(self.command[1]))
            
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("robot_controller")
    try:

        robot = RobotController()
        robot.Loop()

        rospy.loginfo("Start node")
    except Exception as ms:
        rospy.logerr(ms)
    finally:
        rospy.loginfo("Stop node")
        rospy.spin()
    
