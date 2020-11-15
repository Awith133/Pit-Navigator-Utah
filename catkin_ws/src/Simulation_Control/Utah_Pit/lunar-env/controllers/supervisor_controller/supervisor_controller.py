#!/usr/bin/python
"""supervisor_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

import rospy
import tf
import numpy as np
import math

from controller import Supervisor
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float32
from math import sqrt, atan2, cos, sin
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import Temperature
from scipy.io import loadmat
from random import random

# from webots_control.srv import WhereToLook

last_odom = {'x': None,
            'z': None,
            'theta': None}

pose = {'x': None,
        'z': None,
        'theta': None}


total_distance = 1

a1 = 0.05
a2 = 15.0*np.pi/180.0
a3 = 0.05
a4 = 0.01

a1 = 0.01
a2 = 0.01
a3 = 0.01
a4 = 0.003




rospy.init_node('supervisor_controller', anonymous=True)
odometry_publisher = rospy.Publisher('odometry_ground_truth', Odometry, queue_size=100)
where_to_see_publisher = rospy.Publisher('where_to_see', Float32, queue_size=10)
noisy_odometry_publisher = rospy.Publisher('noisy_odometry', Odometry, queue_size=100)
odometry_error_publisher = rospy.Publisher('odom_error', Float64, queue_size=100)
battery_charge_publisher = rospy.Publisher('battery_charge', Temperature, queue_size=10)
sun_dir_publisher = rospy.Publisher('sun_direction', Quaternion, queue_size=10)
show_rock_distances = rospy.get_param('show_rock_distances', 0)
LUNAR_ENV_PATH = rospy.get_param("/lunar_env_path", "/home/hash/Documents/lunar-env")

def publish_where_to_see(illum_direction):
    illu_x_dir = illum_direction[0]
    illu_y_dir = illum_direction[1]
    illu_z_dir = illum_direction[2]

    ##TODO hard coded pit waypoints
    #where_to_see_x = 890*1.51 + 50*(illu_x_dir/np.sqrt(illu_x_dir*illu_x_dir + illu_z_dir*illu_z_dir))
    #where_to_see_z = 320*1.51 + 50*(illu_z_dir/np.sqrt(illu_x_dir*illu_x_dir + illu_z_dir*illu_z_dir))

    #yaw = -math.atan2(where_to_see_z-last_odom['z'], where_to_see_x-last_odom['x'])
    yaw = illu_z_dir
    where_to_see_publisher.publish(yaw)


def publish_odometry(position, orientation, velocity, angular_velocity):
    current_time = rospy.Time.now()
    broadcast_transform(position, orientation)

    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(Point(*position), Quaternion(*orientation))
    odom.twist.twist = Twist(Vector3(*velocity), Vector3(*angular_velocity))
    odom.child_frame_id = "base_link"
    odometry_publisher.publish(odom)

def broadcast_transform(position, orientation):
    current_time = rospy.Time.now()
    odom_broadcaster = tf.TransformBroadcaster()
    odom_broadcaster.sendTransform(
        position,
        orientation,
        current_time,
        "base_link",
        "odom"
    )

def publish_noisy_odometry(position, orientation):
    current_time = rospy.Time.now()
    broadcast_noisy_transform(position, orientation)

    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "noisy_odom"

    odom.pose.pose = Pose(Point(*position), Quaternion(*orientation))
    odom.child_frame_id = "base_link1"
    noisy_odometry_publisher.publish(odom)

def broadcast_noisy_transform(position, orientation):
    current_time = rospy.Time.now()
    odom_broadcaster = tf.TransformBroadcaster()
    odom_broadcaster.sendTransform(
        position,
        orientation,
        current_time,
        "base_link1",
        "noisy_odom"
    )


def calculate_noisy_odometry(position, orientation):
    global last_odom
    global pose
    global noisy_odom
    global total_distance
    global count
    rpy = tf.transformations.euler_from_matrix(orientation, 'rxyz')
    if (last_odom['x'] == None or last_odom['z'] == None or last_odom['theta'] == None):
        last_odom['x'] = position[0]
        last_odom['z'] = position[2]
        last_odom['theta'] =  rpy[1]

        pose['x'] = position[0]
        pose['z'] = position[2]
        pose['theta'] = rpy[1]
    else:
        total_distance += sqrt((position[0] - last_odom['x'])**2 + (position[2] - last_odom['z'])**2)
        # print("position {0} last_odom {1}".format(position, last_odom))
        dx = position[0] - last_odom['x']
        dz = position[2] - last_odom['z']

        # print("dx : {0}, dz: {1}".format(dx, dz))
        trans = sqrt(dx*dx + dz*dz)
        theta1 = last_odom['theta']
        theta2 = rpy[1]

        rot1 = atan2(dx, dz) - theta1
        rot2 = theta2-theta1-rot1

        sd_rot1 = a1*abs(rot1) + a2*trans
        sd_rot2 = a1*abs(rot2) + a2*trans
        sd_trans = a3*trans + a4*(abs(rot1) + abs(rot2))
        # print("sd_rot1 {0}  sd_rot2 {1} sd_trans {2}".format(sd_rot1, sd_rot2, sd_trans))
        trans +=  np.random.normal(0,sd_trans*sd_trans)
        rot1 += np.random.normal(0, sd_rot1*sd_rot1)
        rot2 += np.random.normal(0, sd_rot2*sd_rot2)
        # print("trans {0}  rot1 {1} rot2 {2}".format(trans, rot1, rot2))

        pose['x'] += trans*sin(theta1+rot1)
        pose['z'] += trans*cos(theta1+rot1)
        pose['theta'] = theta1 + rot1 + rot2

        last_odom['x'] = position[0]
        last_odom['z'] = position[2]
        last_odom['theta'] =  rpy[1]

        orientation = tf.transformations.euler_matrix(0, pose['theta'] ,0 , 'rxyz')
        orientation = np.matmul(orientation[0:3,0:3],np.array([[0,0,1],[0,1,0], [-1,0,0]])) # y 90
        orientation = np.matmul(orientation,np.array([[1,0,0],[0,0,1], [0,-1,0]])) # x -90
        orientation = np.hstack((np.vstack((orientation, np.zeros((1,3)))), np.array([[0,0,0,1]]).T))
        orientation = tf.transformations.quaternion_from_matrix(orientation)
        publish_noisy_odometry([pose['x'], 0.1073, pose['z']], orientation)

        # print("Pose: {0} position: {1}".format(pose, position))
        if pose['x'] or pose['y'] or pose['theta']:
            error = sqrt((pose['x'] - position[0])**2 + (pose['z'] - position[2])**2)
        else:
            error = 0
        if count % 5 == 0:
            odometry_error_publisher.publish(error/total_distance)
            count = 0

count = 0

# import time
TIME_STEP = 32

# create the Robot instance.
# robot = Robot()

supervisor = Supervisor()

root = supervisor.getRoot()

# self_robot = supervisor.getSelf()

children = root.getField("children")
n = children.getCount()
for i in range(n):
    name = children.getMFNode(i).getTypeName()
    if(name == "DirectionalLight"):
        light_node_index = i
    elif(name == "Pioneer3at"):
        pioneer_3_at_index = i

light_node = children.getMFNode(light_node_index)
direction_field = light_node.getField("direction")

robot_node = children.getMFNode(pioneer_3_at_index)

spice_data = loadmat(LUNAR_ENV_PATH+"/data/moon_rel_positions_44_25.mat")
dir_sunlight = spice_data['U_sun_point_enu']

rotation_matrix = np.array([[1,0,0],[0,0,1],[0,-1,0]])

dir_sunlight = np.matmul(rotation_matrix,dir_sunlight)
direction_ = (dir_sunlight[:,100]).tolist()

if(show_rock_distances):
    rock_pos = np.load(LUNAR_ENV_PATH+"/data/rock_info_demo1.npy")
    rock_dist_publisher = rospy.Publisher("min_rock_dist", Float32, queue_size=10)

num_loops = 1
time_count = 0
illumination_time_step = 0.001
factor = illumination_time_step*10000000000000/TIME_STEP #1000000 = sim second

#alex additions
total_battery_charge = 4000.0 #Watt*seconds #total battery capacity
battery_charge       = total_battery_charge
battery_use          = 26 #Watts Computer is 10-20 so 20 max all time + 5 for extra stuff
battery_solar        = 50 #Watts peak #battery_use*1.5
prev_second          = 0 #INITIALIZATION
prev_pos             = robot_node.getPosition()
prev_orien           = robot_node.getOrientation()
trigger_distance     = 0.1 #meters
trigger_orien        = 5*math.pi/180 #radians
motor_power          = 10 #Watts #200
num_motors           = 4.5 #number of motors
arc_multiplier       = motor_power/1.5*num_motors*4/4.5
point_multiplier     = motor_power*num_motors
straight_multiplier  = motor_power/2.25*num_motors*4/4.5
battery_use          = rospy.get_param('/battery/hotel_energy_out',25) #per second
total_battery_charge = rospy.get_param('/battery/capacity'        ,4000) #Wattseconds
point_multiplier     = rospy.get_param('/battery/turn_energy_out' ,25) #per second
arc_multiplier       = rospy.get_param('/battery/arc_energy_out'  ,10) #per second
straight_multiplier  = rospy.get_param('/battery/drive_energy_out', 5) #per second
battery_solar        = rospy.get_param('/battery/solar_peak'      ,50) #per second in sun
#review data on charge vs drive
total_sun_in = 0
total_drive_energy = 0
total_hotel_energy = 0
time_between_charges = 0
number_charges = 0
charging = False
#/alex

while(supervisor.step(TIME_STEP)!=-1):
    count = count + 1
    # if( num_loops % 125 == 0):
    # print("Changing light source now")

    #unused
    started_illumination = rospy.get_param("/start_illumination", 0)
    if not started_illumination:
        direction_ = (dir_sunlight[:,100]).tolist()
        direction_field.setSFVec3f(direction_)
    
    #unused
    if num_loops < dir_sunlight.shape[1] and started_illumination and not show_rock_distances:
        # print("Changing illumination")
        time_count += 1
        if(float(time_count)/factor > 1):
            print("Next illumination time step", time_count, num_loops, rospy.get_time())
            direction_ = (dir_sunlight[:,num_loops]).tolist()
            # direction_[1]=-0.5
            direction_field.setSFVec3f(direction_)
            num_loops += 1
            if(num_loops == dir_sunlight.shape[1]):
                print("Done with one day")
            time_count=0
    
    position    = robot_node.getPosition()
    position[1] = 0.1073
    orien       = robot_node.getOrientation()
    orientation = np.array(robot_node.getOrientation()).reshape(3,3)

    noisy_orientation = np.hstack((np.vstack((orientation, np.zeros((1,3)))), np.array([[0,0,0,1]]).T))
    noisy_position    = position
    calculate_noisy_odometry(noisy_position, noisy_orientation)

    orientation = np.matmul(orientation,np.array([[0,0,1],[0,1,0], [-1,0,0]])) # y 90
    orientation = np.matmul(orientation,np.array([[1,0,0],[0,0,1], [0,-1,0]])) # x -90
    orientation = np.hstack((np.vstack((orientation, np.zeros((1,3)))), np.array([[0,0,0,1]]).T))
    orientation = tf.transformations.quaternion_from_matrix(orientation)
    velocity = robot_node.getVelocity()
    publish_odometry(position, orientation, velocity[0:3], velocity[3:6])
    
    #alex additions
    
    #print("orientation",orientation[1]*180/math.pi)
    #print("direction_",direction_[1]*180/math.pi)
    #print("orientation[1]-direction_[1]",orientation[1]*180/math.pi-direction_[1]*180/math.pi)
    time = rospy.get_rostime().secs
    if not time == prev_second:
        #calc battery out
        battery_drive = 0
        poschange = np.sum(np.abs(np.array(prev_pos)-np.array(position)))     >trigger_distance
        orechange = np.sum(np.abs(np.array(prev_orien)[2]-np.array(orien)[2]))>trigger_orien
        #print(poschange,orechange)
        if poschange and not orechange:
            prev_pos = position
            battery_drive = straight_multiplier
        elif not poschange and orechange:
            prev_orien = orien
            battery_drive = point_multiplier
        elif poschange and orechange:
            prev_pos = position
            prev_orien = orien
            battery_drive = arc_multiplier
        
        #calc battery in
        orientation_euler = tf.transformations.euler_from_quaternion(orientation)
        sun_error = orientation_euler[1]-direction_[1]
        sun_error = abs(sun_error)
        if sun_error > math.pi/2.0:
            sun_error = math.pi-sun_error
        #print("sun_error",sun_error*180/math.pi)
        if   sun_error >= 0              and sun_error< 15*math.pi/180:
            battery_in = battery_solar*math.cos(7.5*math.pi/180) #99%
        elif sun_error >= 15*math.pi/180 and sun_error< 30*math.pi/180:
            battery_in = battery_solar*math.cos(22.5*math.pi/180) #92%
        elif sun_error >= 30*math.pi/180 and sun_error< 45*math.pi/180:
            battery_in = battery_solar*math.cos(37.5*math.pi/180) #79%
        elif sun_error >= 45*math.pi/180 and sun_error< 60*math.pi/180:
            battery_in = battery_solar*math.cos(52.5*math.pi/180) #60%
        elif sun_error >= 60*math.pi/180 and sun_error< 75*math.pi/180:
            battery_in = battery_solar*math.cos(67.5*math.pi/180) #38%
        elif sun_error >= 75*math.pi/180 and sun_error< 90*math.pi/180:
            battery_in = battery_solar*math.cos(82.5*math.pi/180) #13%

        #calc battery difference
        battery_charge_increment = battery_in-battery_use-battery_drive

        if (battery_charge+battery_charge_increment>total_battery_charge): battery_charge = total_battery_charge
        else: battery_charge += battery_charge_increment
        
        msg = Temperature()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.temperature = battery_charge/total_battery_charge*100
        msg.variance = 0
        #battery_charge_publisher.publish(msg)
        #rospy.set_param("/battery/charge",battery_charge/total_battery_charge)
        
        prev_second = time
        #print("published",battery_charge/total_battery_charge)

        total_sun_in += battery_in
        total_drive_energy += battery_drive 
        total_hotel_energy += battery_use
        time_between_charges += 1
        if (battery_charge <= total_battery_charge*.2 and not charging):
            number_charges += 1
            print('Charge number: ', number_charges)
            print('total_sun_in:         ',total_sun_in)
            print('total_drive_energy:   ',total_drive_energy)
            print('total_hotel_energy:   ',total_hotel_energy)
            print('time_between_charges: ',time_between_charges)
            charging = True
        elif (charging and battery_charge >= total_battery_charge*.95):
            total_sun_in = 0
            total_drive_energy = 0
            total_hotel_energy = 0
            time_between_charges = 0  
            charging = False
        
        
        #print("time",rospy.get_rostime().secs)
        
        rospy.set_param("/sun_orientation", {'x':direction_[0]-math.pi/2,'y':direction_[2],'z':direction_[1]})
        msg = Quaternion()
        msg.x, msg.y, msg.z, msg.w = tf.transformations.quaternion_from_euler(direction_[0]-math.pi/2,direction_[2],direction_[1])
        sun_dir_publisher.publish(msg)
        #/alex

        
        publish_where_to_see(direction_)



    # unused
    if(show_rock_distances):
        current_robot_position = np.array(robot_node.getPosition()).flatten()
        robot_rock_distances = np.sqrt(np.sum((rock_pos[:,:2] - current_robot_position[[0,2]])**2, axis=1)) - (rock_pos[:,2]/2 + 0.25)
        min_robot_rock_distance = robot_rock_distances.min()
        rock_dist_publisher.publish(min_robot_rock_distance)
