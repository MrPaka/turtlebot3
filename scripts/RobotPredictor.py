#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Float32, Float32MultiArray
import tf
import math
# from dynamic_reconfigure.server import Server
# from robot_controller.cfg import params

class RobotPredictor:
    
    def __init__(self) -> None:
        rospy.init_node('robot_predictor')
        # Server(params, self.get_param)
        rospy.Subscriber('/cmd_vel', Twist, self.velocity_callback)
        rospy.Subscriber('/color_predict', ColorRGBA, self.color_predict_callback)
        self.obstacle_thrashold = rospy.Publisher('/obstacle_thrashold', Float32, queue_size=10)
        self.sector = rospy.Publisher('/coordinates_sector', Float32MultiArray, queue_size=10)
        self.cur_pos = [0, 0, 0]
        self.cur_vel = None
        self.time_predict = 5
        self.color_predict = ColorRGBA(0, 255, 0, 1)
        self.marker_arr = rospy.Publisher('/marker_array', MarkerArray, queue_size=10)
        self.tf_listener = tf.TransformListener()

    # def get_param(self, config: params, level):
    #     self.time_predict = config["time_predict"]
    #     return config

    def velocity_callback(self, vel):
        self.cur_vel = vel
        
    def color_predict_callback(self, color):
        self.color_predict = color
            
    def predict(self):
        marker_array = MarkerArray()
        dt = 0.1
        num_steps= int(self.time_predict / dt)
        x, y, theta = self.cur_pos
        obs = self.cur_vel.linear.x * self.time_predict
        self.obstacle_thrashold.publish(Float32(obs))

        for i in range(num_steps):
            x += self.cur_vel.linear.x * dt * math.cos(theta)
            y += self.cur_vel.linear.x * dt * math.sin(theta)
            theta += self.cur_vel.angular.z * dt

            marker = Marker()

            marker.header.frame_id = 'base_link'
            marker.header.stamp = rospy.Time.now()
            marker.ns = 'predicted_path'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = self.color_predict.r
            marker.color.g = self.color_predict.g
            marker.color.b = self.color_predict.b

            marker_array.markers.append(marker)
        msg = Float32MultiArray()
        msg.data = [x, y]
        self.sector.publish(msg)
        self.marker_arr.publish(marker_array)
    
    def Loop(self):

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.predict()            
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("robot_predictor")
    try:

        robot = RobotPredictor()
        rospy.sleep(1)
        robot.Loop()

        rospy.loginfo("Start node robot_predictor")
    except Exception as ms:
        rospy.logerr(ms)
    finally:
        rospy.loginfo("Stop node robot_predictor")
        rospy.spin()