#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String
import os
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from webots_ros.srv import set_int, set_float, camera_get_info
from sensor_msgs import point_cloud2

TIME_STEP = 32

SPEED_UNIT = 9.09
INCR = 0.01
WHEEL_BASE = 0.266
MAX_SPEED = .00001 #defualt 6.4 #meters/s? #RPM? #TODO find units

CONTROLLERCOUNT = 0
ROBOT_ROSNODE = ""

MOTOR_NAMES = {
    "left_back": "back_left_wheel",
    "right_back": "back_right_wheel",
    "left_front": "front_left_wheel",
    "right_front": "front_right_wheel"
}

MOTOR_VEL_CLIENT = {
    "left_back": None,
    "right_back": None,
    "left_front": None,
    "right_front": None
}

MOTOR_POS_CLIENT = {
    "left_back": None,
    "right_back": None,
    "left_front": None,
    "right_front": None
}

CAMERA_NAMES = {
    "left": "MultiSense_S21_left_camera",
    "right": "MultiSense_S21_right_camera",
    "meta": "MultiSense_S21_meta_camera",
    "depth": "MultiSense_S21_meta_range_finder",
    #adding backwards camera?
    "left_back": "MultiSense_S21_Backwards_left_camera",
    "right_back": "MultiSense_S21_Backwards_right_camera",
    "meta_back": "MultiSense_S21_Backwards_meta_camera",
    "depth_back": "MultiSense_S21_Backwards_meta_range_finder"
    }

CAMERA_INFOS = {
    "left": None,
    "right": None,
    "meta": None,
    "depth": None,
    #adding backwards camera?
    "left_back": None,
    "right_back": None,
    "meta_back": None,
    "depth_back": None
}

CAMERA_INFO_PUBLISHERS = {
    "left": None,
    "right": None,
    "meta": None,
    "depth": None,
    #adding backwards camera?
    "left_back": None,
    "right_back": None,
    "meta_back": None,
    "depth_back": None
}

PC_PUBLISHER = None

META_H = 544
META_W = 1024
META_F = 607.8678180488598
META_FOV = 2*math.atan(META_W/(2*META_F))

CV_BRIDGE = CvBridge()

SET_VEL_CONTROL = False

CURR_VELOCITY = {
    "left_back": 0,
    "right_back": 0,
    "left_front": 0,
    "right_front": 0
}

PREV_VELOCITY = {
    "left_back": None,
    "right_back": None,
    "left_front": None,
    "right_front": None
}

def set_velocity(all_wheel_velocities, speed=0):
    # rospy.loginfo("Setting all wheel velocities")
    global MOTOR_NAMES, MOTOR_POS_CLIENT, MOTOR_VEL_CLIENT, SET_VEL_CONTROL
    if(not SET_VEL_CONTROL):
        for motor in MOTOR_NAMES:
            if(MOTOR_POS_CLIENT[motor] is None):
                service_name = ROBOT_ROSNODE+"/"+MOTOR_NAMES[motor]+"/set_position"
                rospy.wait_for_service(service_name)
                set_position_client = rospy.ServiceProxy(service_name, set_float)
                set_position_client(float('inf'))
                MOTOR_POS_CLIENT[motor] = set_position_client
        SET_VEL_CONTROL = True
    for i, motor in enumerate(MOTOR_NAMES):
        if(MOTOR_VEL_CLIENT[motor] is None):
            service_name = ROBOT_ROSNODE+"/"+MOTOR_NAMES[motor]+"/set_velocity"
            rospy.wait_for_service(service_name)
            set_velocity_client = rospy.ServiceProxy(service_name, set_float)
            MOTOR_VEL_CLIENT[motor] = set_velocity_client
        if(PREV_VELOCITY[motor] is None or PREV_VELOCITY[motor] != all_wheel_velocities[motor]):
            set_velocity_client = MOTOR_VEL_CLIENT[motor]
            set_velocity_client(SPEED_UNIT*all_wheel_velocities[motor])
            PREV_VELOCITY[motor] = all_wheel_velocities[motor]
    
    
    # rospy.loginfo("Done setting up all wheel velocities")

def command_velocity_callback(data):
    robot_linear_velocity = data.linear.x
    robot_angular_velocity = data.angular.z

    left_velocity = robot_linear_velocity - 0.5*robot_angular_velocity*WHEEL_BASE
    right_velocity = robot_linear_velocity + 0.5*robot_angular_velocity*WHEEL_BASE
    velocity = {
        "left_back": left_velocity,
        "right_back": right_velocity,
        "left_front": left_velocity,
        "right_front": right_velocity
    }
    if(not velocity == PREV_VELOCITY):
        set_velocity(velocity, robot_linear_velocity)


def publish_camera_info(data, args):
    global CAMERA_NAMES, CAMERA_INFO_PUBLISHERS, ROBOT_ROSNODE
    camera_name = args
    if(CAMERA_INFO_PUBLISHERS[camera_name] is None):
        if(camera_name == "meta"):
            hack_camera_name = "depth"
        else:
            hack_camera_name = camera_name
        publisher = rospy.Publisher(ROBOT_ROSNODE+"/"+CAMERA_NAMES[hack_camera_name]+"/camera_info", CameraInfo, queue_size=10)
        CAMERA_INFO_PUBLISHERS[camera_name] = publisher
    publisher = CAMERA_INFO_PUBLISHERS[camera_name]
    camera_info = CAMERA_INFOS[camera_name]
    camera_info.header.stamp = data.header.stamp
    camera_info.header.frame_id = data.header.frame_id
    publisher.publish(camera_info)

def publish_pc(data):
    global PC_PUBLISHER, ROBOT_ROSNODE, CAMERA_NAMES
    if(PC_PUBLISHER is None):
        publisher = rospy.Publisher(ROBOT_ROSNODE+"/"+CAMERA_NAMES['depth']+"/point_cloud", PointCloud2, queue_size=10)
        PC_PUBLISHER = publisher 
    publisher = PC_PUBLISHER
    
    try:
        depth = CV_BRIDGE.imgmsg_to_cv2(data, "32FC1")
        depth = np.array(depth).flatten()
    except CvBridgeError as e:
        rospy.loginfo(e)
    max_depth = depth.max()

    fov_v = 2.0 * math.atan(math.tan( META_FOV * 0.5 ) * (float(META_H) / float(META_W)) )
    c_x = float(META_W/2.0)
    c_y = float(META_H/2.0)

    h_indices, w_indices = np.meshgrid(range(META_H), range(META_W), indexing='ij')
    h_indices = h_indices.flatten()
    w_indices = w_indices.flatten()

    x = 2 * depth * (w_indices - c_x) * math.tan(META_FOV/2)/META_W
    y = 2 * depth * (h_indices - c_y) * math.tan(fov_v/2)/ META_H
    z = depth

    # rospy.loginfo("Max and min y values of PC are: "+str(y.max()) + " " + str(y.min()))

    points = [[x_, y_, z_] for x_,y_,z_ in zip(x, y, z) if (not z_==max_depth)] 

    header = data.header
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]

    msg = point_cloud2.create_cloud(header, fields, points)
    publisher.publish(msg)

    # P3D.x= depth(x,y)*(x - c_x)/META_F # 2 * depth(x_d,y_d)* (x_d- cu) *math.tan(META_FOV/2)/ META_W
    # P3D.y= depth(x,y)*(y - c_y)/META_F # 2 * depth(x_d,y_d) * (y_d- cv) *math.tan(fov_v/2)/ META_H
    # P3D.x= 2 * depth(x_d,y_d)* (x_d- cu) *math.tan(META_FOV/2)/ META_W
    # P3D.y= 2 * depth(x_d,y_d) * (y_d- cv) *math.tan(fov_v/2)/ META_H
    # P3D.z = depth

def get_model_name(data):
    global ROBOT_ROSNODE
    ROBOT_ROSNODE = data.data
    rospy.loginfo(("Got the ROS node of the Robot as: " + ROBOT_ROSNODE))

def enable_sensor(sensor_name):
    global CAMERA_NAMES
    global ROBOT_ROSNODE
    try:
        # Publish camera images
        service_name = ROBOT_ROSNODE+"/"+CAMERA_NAMES[sensor_name]+"/enable"
        rospy.wait_for_service(service_name,10)
        enbale_client = rospy.ServiceProxy(service_name, set_int)
        rep1 = enbale_client(64)

        rospy.loginfo("Enabled camera: " + sensor_name)

        if(not sensor_name == "depth"):
            service_name = ROBOT_ROSNODE+"/"+CAMERA_NAMES[sensor_name]+"/get_info"
            rospy.wait_for_service(service_name)
            getinfo_client = rospy.ServiceProxy(service_name, camera_get_info)
            rep2 = getinfo_client(1)
            
            if(CAMERA_INFOS[sensor_name] is None):
                camera_info = CameraInfo()
            else:
                camera_info = CAMERA_INFOS[sensor_name]
            
            camera_info.width = rep2.width
            camera_info.height = rep2.height
            f = rep2.width/(2*math.tan(rep2.Fov/2))
            camera_info.K[0] = f
            camera_info.K[4] = f
            camera_info.K[2] = rep2.width/2
            camera_info.K[5] = rep2.height/2
            camera_info.K[8] = 1
            camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
            camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            camera_info.P = [f, 0.0, rep2.width/2, 0.0, 0.0, f, rep2.height/2, 0.0, 0.0, 0.0, 1.0, 0.0]
            CAMERA_INFOS[sensor_name] = camera_info
            rospy.loginfo(("Saved camera info for camera: " + sensor_name + " " + str(rep2.Fov)))

            topic_name = ROBOT_ROSNODE+"/"+CAMERA_NAMES[sensor_name]+"/image"
            rospy.Subscriber(topic_name, Image, publish_camera_info, (sensor_name))
        else:
            topic_name = ROBOT_ROSNODE+"/"+CAMERA_NAMES[sensor_name]+"/range_image"
            # rospy.Subscriber(topic_name, Image, publish_pc)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    rospy.init_node('pioneet3at_control', anonymous=True, log_level=rospy.INFO)
    rate = rospy.Rate(10) # 10hz
    rospy.loginfo(("Started Node with name initialized to: " + ROBOT_ROSNODE))
    # Subscribe to model_name to get the name of the node for the robot
    model_name_sub = rospy.Subscriber("/model_name", String, get_model_name) 
    
    counter = 0
    while(ROBOT_ROSNODE == ""):
        if(counter%200 == 0):
            counter = 1
        counter +=1
    
    rospy.loginfo(("Finally started Node with name initialized to: " + ROBOT_ROSNODE))
    needDepth = rospy.get_param("show_rock_distances", 0)
    #enable_sensor("meta")
    enable_sensor("left")
    enable_sensor("right")
    enable_sensor("left_back")
    enable_sensor("right_back")
    #enable_sensor("meta_back")
    if(needDepth):
        enable_sensor("depth")
    set_velocity(CURR_VELOCITY)
    
    rospy.Subscriber('cmd_vel', Twist, command_velocity_callback, queue_size=1)
    # Spin main ROS loop
    while not rospy.is_shutdown():
        # rospy.loginfo(("Started Node with name: " + ROBOT_ROSNODE))
        rate.sleep()
