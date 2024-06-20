#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int16
import time

# from dynamic_reconfigure.server import Server
# from robot_controller.cfg import params

class SpeedController:

    def __init__(self) -> None:

        # Server(params, self.get_param)
        rospy.Subscriber('/command_control', String, self.command_callback)
        rospy.Subscriber('/direction', Int16, self.direction_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.velocity_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.direction = 0
        self.max_speed = 0.2
        self.twist = Twist()

    # def get_param(self, config: params, level):
    #     self.max_speed = config["max_speed"]
    #     self.acceleration = config["acceleration"]
    #     self.deceleration = config["deceleration"]
    #     return configs

    def velocity_callback(self, data):
        self.current_vel = data
        self.cur_time = time.time()

    def direction_callback(self, direction):
        self.direction = direction.data

    def command_callback(self, move):
        if (move.data == "straight"):
            self.move()
        elif (move.data == "rotation"):
            self.rotation()
        elif (move.data == "braiking_straight"):
            self.braiking_move()
        elif (move.data == "braiking_rotation"):
            self.braiking_rotation()
        elif (move.data == "stop"):
            self.stop()

    def stop(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0

    def move(self):
        self.twist.linear.x = self.direction * self.max_speed

    def rotation(self):
        self.twist.angular.z = self.direction * self.max_speed

    def braiking_move(self):
        self.twist.linear.x = 0

    def braiking_rotation(self):
        self.twist.angular.z = 0

    def Loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.twist)

            rate.sleep()

        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("seed_controller")
    try:

        robot = SpeedController()
        robot.Loop()

        rospy.loginfo("Start node speed_controller")
    except Exception as ms:
        rospy.logerr(ms)
    finally:
        rospy.loginfo("Stop node speed_controller")
        rospy.spin()