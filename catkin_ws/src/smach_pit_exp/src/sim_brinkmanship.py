from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import numpy as np
import rospy
from std_msgs.msg import Float64, String
import time

class Brinkmanship:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.msg = Twist()
        return
    
    def generate_twist_msg(self, linear_vel, angular_vel):
        self.msg.linear.x = linear_vel[0]
        self.msg.linear.y = linear_vel[1]
        self.msg.linear.z = linear_vel[2]
        self.msg.angular.x = angular_vel[0]
        self.msg.angular.y = angular_vel[1]
        self.msg.angular.z = angular_vel[2]
        return

    def publish_twist_msg(self):
        self.cmd_vel_pub.publish(self.msg)
        return

if __name__ == '__main__':
    rospy.init_node('Brinkmanship')
    controller = Brinkmanship()
    linear_vel = [0, 0, 0]
    angular_vel = [0, 0, 0]
    while not rospy.is_shutdown():
        controller.generate_twist_msg(linear_vel, angular_vel)
        controller.publish_twist_msg()
        linear_vel[0] = linear_vel[0] + 0.1
        time.sleep(1)