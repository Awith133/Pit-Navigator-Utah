#!/usr/bin/env python

#display image imports
#import cv2
import glob
#import matplotlib.pyplot as plt
#import matplotlib.image as mpimg
#from PIL import Image
import time
import subprocess

class BrinkStatus:
    def __init__(self):
       self.alert_flag = 'False'
       return
    
    def edge_alert_cb(self, msg):
        self.alert_flag = 'True'
        return


def display_real_images(userdata, file_locations):
    time.sleep(1)
    images_folder = file_locations['robot_simulation_env'] + '/lunar-env/images/'
    
    grep = images_folder + str(userdata.wp_around_pit[userdata.counter_wp_around_pit][5])+ '_' +str(userdata.wp_around_pit[userdata.counter_wp_around_pit][6])+ '_'+'2_1_1_0_0.png'
    #smalldst = images_folder+iteration[userdata.counter_wp_around_pit][0] +'_'+iteration[userdata.counter_wp_around_pit][1]+'/'
    #smallgrep = iteration[userdata.counter_wp_around_pit][0] + '_' +iteration[userdata.counter_wp_around_pit][1]+ '_'+ '2_1_1_*_*.png'
    #import os
    ##method 4
    #if not os.path.exists(smalldst):
    #os.mkdir(smalldst)
    
    list1 = glob.glob(grep)
    def compare(x, y):
        partsOfX = x.split('_')
        partsOfY = y.split('_')
        integerListx = [int(partsOfX[5]),int(partsOfX[6].split('.')[0])*100]
        integerListy = [int(partsOfY[5]),int(partsOfY[6].split('.')[0])*100]
        return integerListx[0] - integerListy[0]+ integerListy[1]- integerListx[1]
    list1.sort(cmp= compare)
    
    #i = 0
    for filename in list1:
        #make method 4
        #from shutil import copyfile
        #dst = smalldst+'{0}.png'.format(str(i).zfill(3))
        #i+=1
        #copyfile(filename, dst)

        #method#3
        viewer = subprocess.Popen(['eog', filename])
        time.sleep(2)
        viewer.terminate()
        viewer.kill()
        
        #method2
        #img=mpimg.imread(filename)
        #imgplot = plt.imshow(img)
        #plt.show(block=False)
        #plt.pause(1)
        #plt.close()

        #method1
        #image = cv2.imread(filename)
        #print('1')
        #cv2.imshow('image',image)
        #cv2.waitKey(1000)
        #cv2.destroyAllWindows()
        
    #make method 4
    #location = file_locations['project_file_location']+'/catkin_ws/src/smach_pit_exp/src/'
    #subprocess.call("{0}makeVantagegif.sh {1} {2}".format(location,smalldst,smallgrep),shell=True)
    #else:
        #method 4
        #viewer = subprocess.Popen(['eog', smalldst+smallgrep+'.gif'])
        #time.sleep(14)
        #viewer.terminate()
        #viewer.kill()
        




def show_model(model,file_locations):

    #make movie
    ##open paraview
    ##run paraview script
    ##make gif

    #show movie
    filename = file_locations['robot_simulation_env'] + '/lunar-env/'+ model[:-4]+'.gif'
    viewer = subprocess.Popen(['eog', filename])
    time.sleep(60)
    viewer.terminate()
    viewer.kill()



#get pose imports
from geometry_msgs.msg import PoseStamped, PolygonStamped 
def _get_pose(points):
    num_points = len(points)
    x_pose_arr = [points[i].x for i in range(0,num_points)]
    y_pose_arr = [points[i].y for i in range(0,num_points)]
    x_pose = sum(x_pose_arr)/num_points
    y_pose = sum(y_pose_arr)/num_points
    vec_x = points[0].x - points[8].x
    vec_y = points[0].y - points[8].y
    yaw = math.atan2(vec_y, vec_x) %6.28218
    if yaw > 3.14159:
        yaw = yaw - 6.28218
    return [x_pose, y_pose, yaw]

import tf
import math
def calculate_yaw(q,msg):
    yaw_goal = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])[2] %6.28218
    points = msg.polygon.points
    vec_x = points[0].x - points[8].x
    vec_y = points[0].y - points[8].y
    return math.atan2(vec_y, vec_x) %6.28218, yaw_goal




#read csv imports
import csv
def read_csv(filename,_map_resolution):
	wp = []
	resolution = _map_resolution
	with open(filename, 'rb') as f:
		reader = csv.reader(f, delimiter=',')
		for row in reader:
			tmp = []
			i = 0
			for elem in row:
				if(i == 0):
					tmp.append((int(elem)*resolution ))
				else:
					tmp.append(-1*int(elem)*resolution )
				i+=1
			wp.append(tmp)
	return wp
def read_csv_around_pit(filename, _map_resolution):
    wp = []
    map_resolution = _map_resolution
    time_resolution = 25
    time_offset = 0
    with open(filename, 'rb') as f:
        reader = csv.reader(f, delimiter=',')
        
        for row in reader:
            tmp = []
            i = 0
            for elem in row:
                if   (i==0):
                    tmp.append(   int(elem) * time_resolution + time_offset) #waypoint num
                elif (i==1):
                    tmp.append(   int(elem) * map_resolution ) #x
                elif (i==2):
                    tmp.append(-1*int(elem) * map_resolution ) #y
                elif (i==3 or i==5 or i==6):
                    tmp.append(   int(elem)) #vatage point y = 1, n = -1
                elif (i==4):
                    ang = (int(elem))
                    # if((ang-sunangle)<-45 and not (ang-sunangle)<-90):
                    #     ang = sunangle-45
                    # elif((ang-sunangle)<-90):
                    #     ang = sunangle-135
                    # elif((ang-sunangle)>45 and not (ang-sunangle)>90):
                    #     ang = sunangle+45
                    # elif ((ang-sunangle)>90):
                    #     ang = sunangle+135
                    tmp.append(  ang * 3.14159/180) #yaw degrees for waypoint
                i+=1
            wp.append(tmp)
    return wp





def timekeeper(state, start_time, end_time,file_locations):
    csvfile = open(file_locations['project_file_location']+'/catkin_ws/src/smach_pit_exp/src/timekeeping.csv','a')
    writer = csv.writer(csvfile)
    #print([state,end_time-start_time])
    writer.writerow([state,start_time,end_time])
    csvfile.close()

def time_estimations(file_locations , estimation=None):
    csvfile = open(file_locations['project_file_location']+'/catkin_ws/src/smach_pit_exp/src/timekeeping.csv','r')
    reader = csv.reader(csvfile)
    if ('Simulation' in file_locations['robot_simulation_env']):
        if ('Utah_Pit' in file_locations['robot_simulation_env']):
            average = [80, 78, 567, 132]
        if ('Utah_BIG' in file_locations['robot_simulation_env']):
            average = [80, 78, 567, 132]
        if ('Lacus_Mortis_Pit' in file_locations['robot_simulation_env']):
            average = [800, 780, 5670, 1320]
        if ('Pit_Edge_Test' in file_locations['robot_simulation_env']):
            average = [10, 78, 15, 25] 
    else:
        average = [80, 78, 567, 132]
    
    output = average[:]
    count = [0,0,0,0]
    for row in reader:
        diff = int(row[2])-int(row[1])
        if row[0] == 'Lander2Pit':
            if count[0] == 0:
                average[0] = diff
                count[0] = 1
                continue
            count[0] += 1
            average[0]= average[0] + (diff-average[0])/count[0]
        elif row[0] == 'navAROUNDPIT':
            if count[1] == 0:
                average[1] = diff
                count[1] = 1
                continue
            count[1] += 1
            average[1]= average[1] + (diff-average[1])/count[1]
        elif row[0] == 'Pit2Lander':
            if count[2] == 0:
                average[2] = diff
                count[2] = 1
                continue
            count[2] += 1
            average[2]= average[2] + (diff-average[2])/count[2]
        elif row[0] == 'Highway':
            if count[3] == 0:
                average[3] = diff
                count[3] = 1
                continue
            count[3] += 1
            average[3]= average[3] + (diff-average[3])/count[3]
        else:
            print('error in smach_helper: error in time estimation: unexpected state')
    csvfile.close()
    #print(average)
    if estimation == None:
        for i in range(4):
            output[i] = average[i]*.6+output[i]*.4
        return output
    else:
        for i in range(4):
            output[i] = average[i]*.7+estimation[i]*.3
        return output

def calcuate_risk(time_now,time_finish,num_unvisited_waypoints,file_locations):
    [pit2lander,navAroundPit,lander2pit,highway] = time_estimations(file_locations)
    safe_time = time_finish - time_now
    travel = pit2lander + lander2pit+highway*2
    if navAroundPit*num_unvisited_waypoints+travel <safe_time:
        trips = int((safe_time-navAroundPit*num_unvisited_waypoints)/travel)
        #print('safe',safe_time)
        #print('travel', travel)
        #print('pit nav', navAroundPit*num_unvisited_waypoints)
        print('possible shuttles:',trips)
        print('vantage points to visit: {0}'.format(int(num_unvisited_waypoints/trips)))
        return num_unvisited_waypoints/trips
    else:
        points = int((safe_time-travel)/(navAroundPit*num_unvisited_waypoints))
        return points

sunangle = 0.0
charging = False
min_charge = 1.0
waypoint_cords = {'x':0,'y':0,}
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import rospy
import actionlib

def charge_cb (msg,argc):
    global charging, pose, step_flag, min_charge
    if (not 'Utah' in rospy.get_param("robot_simulation_arg")):
        return
    battery_charge = msg.temperature/100.0
    userdata = argc[0]
    move_base_client = argc[1]
    #waypoint_pub = argc[1]
    if battery_charge <= min_charge:
        rospy.logwarn("MIN Battery charge: %f",battery_charge)
        min_charge=battery_charge
    
    if battery_charge%.1 <= 0.001 or battery_charge%.1 >= 0.999 and not battery_charge == 1.0:
        print('Battery charge:',battery_charge)
        
    if battery_charge <= .2:
        if battery_charge < 0: print('DEAD - NO BATTERY POWER')
        #pose = argc[2]
        msg = PoseStamped()
        msg.pose.position.x = pose['x']
        msg.pose.position.y = pose['y']
        if abs(pose['yaw']-sunangle)<=math.pi/2: q = quaternion_from_euler(0, 0, sunangle)
        else: q = quaternion_from_euler(0, 0, sunangle+math.pi)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        msg.header.frame_id = 'map'
        xdif = abs((waypoint_cords['x']-msg.pose.position.x)/msg.pose.position.x)
        ydif = abs((waypoint_cords['y']-msg.pose.position.y)/msg.pose.position.y)
        if xdif+ydif>.1:
            waypoint_cords['x'] = msg.pose.position.x
            waypoint_cords['y'] = msg.pose.position.y
            print('Turning to sun to charge at:', msg.pose.position.x , msg.pose.position.y )
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now() 
            goal.target_pose.pose = msg.pose
            rospy.loginfo("Sending goal pose "+str(current_goal)+" to Action Server")
            move_base_client.send_goal(goal, done_cb, active_cb, feedback_cb)
            charging = True

    if battery_charge >.99 and charging:
        charging = False
        step_flag = False
        msg = PoseStamped()
        msg.pose.position.x = userdata.current_wp_cords[0]
        msg.pose.position.y = userdata.current_wp_cords[1]
        q = quaternion_from_euler(0, 0, userdata.current_wp_cords[2])
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        msg.header.frame_id = 'map'
        print('Done charging returning to waypoint:', msg.pose.position.x , msg.pose.position.y)
        waypoint_cords['x'] =0
        waypoint_cords['y'] =0
        #waypoint_pub.publish(msg)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = msg.pose
        rospy.loginfo("Sending goal pose "+str(current_goal)+" to Action Server")
        move_base_client.send_goal(goal, done_cb, active_cb, feedback_cb)

current_goal = 0
def active_cb():
    global current_goal
    rospy.loginfo("Goal pose "+str(current_goal)+" is now being processed by the Action Server...")

pose ={	'x':0.0,
	    'y':0.0,	
	    'yaw':0.0,}
def feedback_cb(feedback):
    global pose, current_goal
    #To print current pose at each feedback:
    #rospy.loginfo("Feedback for goal "+str(current_goal)+": "+str(feedback))
    pose['x'] = feedback.base_position.pose.position.x
    pose['y'] = feedback.base_position.pose.position.y
    pose['yaw'] = euler_from_quaternion([feedback.base_position.pose.orientation.w,feedback.base_position.pose.orientation.x,feedback.base_position.pose.orientation.y,feedback.base_position.pose.orientation.z])[2]
    #rospy.loginfo("Feedback for goal pose "+str(current_goal)+" received")

step_flag = False
abort_flag = False
def done_cb(status, result):
    global step_flag,abort_flag, current_goal
    
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
    if status == 2:
        rospy.loginfo("Goal pose "+str(current_goal)+" received a cancel request after it started executing, completed execution!")

    if status == 3:
        rospy.loginfo("Goal pose "+str(current_goal)+" reached") 
        step_flag = True
        
    if status == 4:
        rospy.loginfo("Goal pose "+str(current_goal)+" was aborted by the Action Server")
        abort_flag = True
        #rospy.signal_shutdown("Goal pose "+str(current_goal)+" aborted, shutting down!")
        #return

    if status == 5:
        rospy.loginfo("Goal pose "+str(current_goal)+" has been rejected by the Action Server")
        rospy.signal_shutdown("Goal pose "+str(current_goal)+" rejected, shutting down!")
        return

    if status == 8:
        rospy.loginfo("Goal pose "+str(current_goal)+" received a cancel request before it started executing, successfully cancelled!")
    current_goal = current_goal + 1

def update_sun(msg):
    global sunangle
    sunangle = msg.data