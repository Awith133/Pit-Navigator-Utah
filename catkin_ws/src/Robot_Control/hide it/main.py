#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from arbotix_python.arbotix import ArbotiX
import panorama
import time
import sys
import time
from signal import signal, SIGINT
from sys import exit
import datetime

a = None
alert = False
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

def handler(signal_received, frame):
    print('SIGINT or CTRL-C detected. Exiting gracefully')
    bot_stop(0,0)
    bot_stop(0,0)
    sys.exit()

def bot_forward(speed, turn):
    x, y, z, th = 1, 0, 0, 0
    twist = Twist()
    twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
    pub.publish(twist)
    return True

def bot_stop(speed, turn):
    x, y, z, th = 0, 0, 0, 0
    twist = Twist()
    twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
    pub.publish(twist)

    return True

def change_speed(speed, increase = False):
    if increase:
        increment = 0.1
    else:
        increment = -0.1
        if speed + increment <=  0:
            return 0
    return speed + increment

def edge_alert_callback(msg):
    global alert, a
    '''
    # TODO:
    1. Check for reconstructed cloud and break upon reaching a safe point
    2. Should not call bot_forward until images are captured
    '''
    if alert == False:
        a = datetime.datetime.now()
    alert = True

def main():
    CONV_H = 0.37
    CONV_V = 0.3
    dyna_center = 512
    arby = ArbotiX("/dev/ttyUSB0", 115200)
    pan_rng, tilt_rng = panorama.get_camera_angles(dyna_center, CONV_V, CONV_H)

    rospy.init_node('brinkmanship_core')
    signal(SIGINT, handler)
    camera_pub = rospy.Publisher('image_capture_message', String, queue_size = 1)
    sub = rospy.Subscriber('edge_alert', String, edge_alert_callback)

    speed = 0.75
    turn = 1


    # bot_stop(0,0)
    # time.sleep(2)
    # bot_forward(speed, turn)
    # time.sleep(2)
    # bot_stop(0,0)
    panorama.set_camera_pose(arby, dyna_center, dyna_center - 51)
    bot_forward(speed, turn)
    flag = 0
    while not rospy.is_shutdown():
        if not alert:

            pass
        else:
            bot_stop(speed, turn)
            b = datetime.datetime.now()
            if flag == 0:
                print("Alert has been published")
                print("Time required for stopping {} microseconds".format((b-a).microseconds))
                print("Moving Pan Tilt Motors")
                for pan in pan_rng:
                    for tilt in tilt_rng:
                        panorama.set_camera_pose(arby, pan, tilt)
                        time.sleep(3)
                        camera_pub.publish(str(pan)+'_'+str(tilt))
                flag = 1
            # TODO: Execute camera motion and image capture

def panorama_method():
    CONV_H = 0.37
    CONV_V = 0.3
    dyna_center = 512
    arby = ArbotiX("/dev/ttyUSB0", 115200)
    pan_rng, tilt_rng = panorama.get_camera_angles(dyna_center, CONV_V, CONV_H)

    rospy.init_node('brinkmanship_core')
    signal(SIGINT, handler)
    camera_pub = rospy.Publisher('image_capture_message', String, queue_size = 1)

    speed = 0.75
    turn = 1

    panorama.set_camera_pose(arby, dyna_center, dyna_center - 51)
    print("Alert has been published")
    print("Moving Pan Tilt Motors")
    for pan in pan_rng:
        for tilt in tilt_rng:
            panorama.set_camera_pose(arby, pan, tilt)
            time.sleep(3)
            camera_pub.publish(str(pan)+'_'+str(tilt))
    flag = 1
    # TODO: Execute camera motion and image capture



if __name__ == '__main__':
    #main()
    panorama_method()