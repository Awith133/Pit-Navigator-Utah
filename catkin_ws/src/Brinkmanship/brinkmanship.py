from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import numpy as np
import rospy
from std_msgs.msg import Float64, String
import time
import sys
from cloud_process import CloudSubscriber

class Brinkmanship:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
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
    translation = [0, 0.13, -0.18]
    rotation = [-0.3583641, 0, 0, 0.9335819]
    alert_helper = CloudSubscriber(translation, rotation)
    brink_controller = Brinkmanship()
    images = [(5,1),(5,2),(5,3),(5,4),(3,4),(5,5),(3,7),(5,7),(4,3),(5,8),(4,4),(4,5)]
    index = -1
    while not rospy.is_shutdown():
        if(rospy.get_param("/run_brinkmanship")):
            start_time_going = rospy.get_rostime().secs
            while not alert_helper.alert_bool:
                brink_controller.generate_twist_msg([0.05, 0, 0], [0, 0, 0])
                brink_controller.publish_twist_msg()
            end_time_going = rospy.get_rostime().secs
            # zero twist to stop
            brink_controller.generate_twist_msg([0, 0, 0], [0, 0, 0])
            brink_controller.publish_twist_msg()
            # camera functions
            if ('Simulation' in rospy.get_param("/robot_simulation_arg")):
                sys.path.insert(1,rospy.get_param("/system_name")+"/catkin_ws/src/smach_pit_exp/src")
                import smach_helper
                index+=1
                smach_helper.display_sim_images(images[index],rospy.get_param("/robot_simulation_arg")) #replace with pan tilt motions on robot #make it stop this one
            else:
                sys.path.insert(1,rospy.get_param("/robot_simulation_arg"))
                import main

            #return
            start_time_coming = rospy.get_rostime().secs
            while alert_helper.alert_bool or start_time_coming+(end_time_going-start_time_going)/2>rospy.get_rostime().secs:
                brink_controller.generate_twist_msg([-0.1, 0, 0], [0, 0, 0])
                brink_controller.publish_twist_msg()
            rospy.set_param("/run_brinkmanship",False)