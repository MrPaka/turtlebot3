import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
import tf
import math

class RobotPredictor:
    Color = [0.0, 0.0, 0.0]
    
    def __init__(self) -> None:
       
        self.time_predict = 5
        color = [0.0, 0.0, 0.0]
        self.cur_pos = [0, 0, 0]
        self.cur_vel = None
        rospy.Subscriber('/cmd_vel', Twist, self.velocity_callback)
        # rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        # rospy.Subscriber('/camera/camera_info', CameraInfo, self.camera_info_callback)
        self.marker_arr = rospy.Publisher('/marker_array', MarkerArray, queue_size=10)
        self.tf_listener = tf.TransformListener()

    def image_callback(self, data):
        pass

    def camera_info_callback(self, data):
        pass
    

    def velocity_callback(self, vel):
        self.cur_vel = vel
        self.predict()

            
    def predict(self):
        marker_array = MarkerArray()

        dt = 0.1
        num_steps= int(self.time_predict / dt)
        x, y, theta = self.cur_pos

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
            # marker.color.r = RobotPredictor.Color[0]
            # marker.color.g = RobotPredictor.Color[1]
            # marker.color.b = RobotPredictor.Color[2]

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)

        self.marker_arr.publish(marker_array)


