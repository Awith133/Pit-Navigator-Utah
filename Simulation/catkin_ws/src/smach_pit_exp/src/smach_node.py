#!/usr/bin/env python

import csv
import rospy
import math
import smach
import pdb

import smach_ros
from smach import CBState
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, PolygonStamped
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion


from playsound import playsound

from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import Temperature
import tf

#display image imports
import smach_helper


GLOBAL_RADIUS = .45
GLOBAL_RADIUS2 = 1
YAW_THRESH = 0.16 #10 deg
TIME_OUT = 2200*1.3 #normal = 1.3 big = 1
file_locations = {
	'file_to_pit':rospy.get_param("file_to_pit"),
	'file_around_pit':rospy.get_param("file_around_pit"),
	'project_file_location':rospy.get_param("/system_name"),
}
halfway_point = len(smach_helper.read_csv_around_pit(file_locations['file_around_pit']))/2

waypoint_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1)
pit_edge_dist_pub = rospy.Publisher('/robot_at_edge_position', Odometry, queue_size = 1)

sub_where_to_see = rospy.Subscriber('where_to_see',Float32,smach_helper.update_sun)

current_goal = 0

listener = None
pose ={	'x':0.0,
	    'y':0.0,	
	    'yaw':0.0,}


#CLASS 1 from lander to pit
class Lander2Pit(smach.State):
	def __init__(self):
		smach.State.__init__(self,input_keys=['wp_2_pit','counter_wp_2_pit','illumination_start_time','current_wp_cords'],
						output_keys=['direction','alternative_point','current_wp_cords','counter_wp_2_pit','illumination_start_time'],
						outcomes=['reached_pit','mission_ongoing','failed'])
		self.success_flag = False
		self.start_time = 0
		self.end_time = 0

	def position_cb(self,msg,argc):
		if self.success_flag == True:
			return
		[pose['x'], pose['y'], pose['yaw']] = smach_helper._get_pose(msg.polygon.points)
		userdata = argc[0]                                          #userdata contains the input_key


		if(userdata.counter_wp_2_pit != -1):
			error = math.sqrt((pose['x'] - self.wp.pose.position.x)**2 + (pose['y'] - self.wp.pose.position.y)**2)
		else:
			error = 0
		#print("Lander2Pit: Pursing Waypoint {0}, Distance to waypoint: {1}".format(userdata.counter_wp_2_pit, error))
		if(error<GLOBAL_RADIUS or userdata.counter_wp_2_pit == -1):
			userdata.counter_wp_2_pit += 1
			if(userdata.counter_wp_2_pit >= len(userdata.wp_2_pit)):
				userdata.counter_wp_2_pit = -1
				self.success_flag = True
				return
			self.wp = self.global_wp_nav(userdata,waypoint_pub)

	def global_wp_nav(self,userdata,waypoint_pub):
			
		#publish points from csv and get x,y
		msg = PoseStamped()
		msg.pose.position.x = userdata.wp_2_pit[userdata.counter_wp_2_pit][0]#x
		msg.pose.position.y = userdata.wp_2_pit[userdata.counter_wp_2_pit][1]#y
		#print(msg.pose.position.y)
		yaw = 0
		msg.pose.orientation.w = 1
		msg.header.frame_id = 'map'
		print('Publishing wp', msg.pose.position.x , msg.pose.position.y)
		waypoint_pub.publish(msg)
		userdata.current_wp_cords = (msg.pose.position.x,msg.pose.position.y,yaw)
		return msg

	def execute(self,userdata):
		self.start_time = rospy.get_rostime().secs
		sub_odom = rospy.Subscriber("/move_base/local_costmap/footprint", PolygonStamped, self.position_cb, (userdata,self.success_flag))
		sub_battery = rospy.Subscriber("battery_charge", Temperature, smach_helper.charge_cb, (userdata,waypoint_pub,pose))
		rate = rospy.Rate(5)
		while not (self.success_flag):
			rate.sleep()
		sub_odom.unregister()
		sub_battery.unregister()
		if self.success_flag:
			#print("Reached waypoint. On to next waypoint: {0}".format(userdata.counter_wp_2_pit))
			print("------------------------------------------------------------------------------")
			self.success_flag = False
			userdata.illumination_start_time = rospy.get_rostime().secs
			rospy.set_param("/start_illumination", 0) #DONT START ILLUMINATION UNTIL ITS LONGER TODO
			userdata.direction = 'capture'
			self.end_time = rospy.get_rostime().secs
			smach_helper.timekeeper('Lander2Pit',self.start_time,self.end_time,file_locations)
			return 'reached_pit'
		return 'mission_ongoing'

#CLASS 2 around pit
class circum_wp_cb(smach.State):

	def __init__(self):
		smach.State.__init__(self,input_keys=['counter_wp_around_pit','current_wp_cords','wp_around_pit','illumination_start_time','towards_edge_time_start'],
					output_keys=['direction','alternative_point','current_wp_cords','counter_wp_around_pit','illumination_start_time','towards_edge_time_start','mission_flag'],
					outcomes=['mission_ongoing','save_data','failed'])
		self.success_flag = False 
		self.vantage_return = False
		self.start_time = 0
		self.end_time = 0
		self.count_visited = 0
		self.risk_safe = 9
		self.first_waypoint = True

	def position_cb(self,msg,argc):
		if self.success_flag == True:
			return
		# drive to next waypoint
		[pose['x'], pose['y'], pose['yaw']]= smach_helper._get_pose(msg.polygon.points)
		userdata = argc[0]                                          #userdata contains the input_keys


		if(userdata.counter_wp_around_pit != -1):
			error = math.sqrt((pose['x'] - self.wp.pose.position.x)**2 + (pose['y'] - self.wp.pose.position.y)**2)
		else:
			error = 0
		#print("navAroundPit: Pursing Waypoint {0}, Distance to waypoint: {1}".format(userdata.counter_wp_around_pit, error))
		
		if (error<GLOBAL_RADIUS or userdata.counter_wp_around_pit == -1 or self.first_waypoint): #have we made it to a waypoint?
		#end drive to next waypoint
	
			if(userdata.wp_around_pit[userdata.counter_wp_around_pit][3] == 1): #is this a vantage point? yes = 1
				complete = False
				#rotate to yaw
				q=self.wp.pose.orientation
				yaw_pose, yaw_goal = smach_helper.calculate_yaw(q,msg)
				
				
				
				if (abs(yaw_goal - yaw_pose) < YAW_THRESH or abs(yaw_goal+6.28218 - yaw_pose) < YAW_THRESH):   #am i looking in the right direction?
				#end rotation to yaw
	
					print('return real life pictures')
					#TODO make this an actionlib
					smach_helper.display_real_images(userdata,file_locations) #relpace with pan tilt motions on robot

					complete = True
				
				if (complete): #finished at waypoint - publish the next waypoint
					userdata.counter_wp_around_pit += 1
					self.vantage_return = True 
					self.count_visited += 1
					if(userdata.counter_wp_around_pit >= len(userdata.wp_around_pit)-1): #have we made it to the end of the waypoints?
						self.success_flag = True
						userdata.mission_flag = True
						return
					self.wp = self.global_wp_nav(userdata,waypoint_pub)
			
			
			else: #not a vantage point - publish the next waypoint
				if(self.vantage_return):
					self.end_time = rospy.get_rostime().secs
					smach_helper.timekeeper('navAROUNDPIT',self.start_time,self.end_time,file_locations)
					self.start_time = rospy.get_rostime().secs
					self.vantage_return = False
				if(userdata.counter_wp_around_pit >= len(userdata.wp_around_pit)-2): #have we made it to the end of the waypoints?
					print('MISSION SUCCESS')
					userdata.mission_flag = True
					self.success_flag = True
					self.count_visited = 0
					return
				if(self.count_visited >= self.risk_safe):
					self.success_flag = True
					self.count_visited = 0
					return
				userdata.counter_wp_around_pit += 1
				self.first_waypoint = False
				self.wp = self.global_wp_nav(userdata,waypoint_pub)

	def global_wp_nav(self,userdata, waypoint_pub): #publishes the next waypoint in the list and gives it to TEB algorithm to navigate to
		
		msg = PoseStamped()
		msg.pose.position.x = userdata.wp_around_pit[userdata.counter_wp_around_pit][1]#x
		msg.pose.position.y = userdata.wp_around_pit[userdata.counter_wp_around_pit][2]#y
		yaw = userdata.wp_around_pit[userdata.counter_wp_around_pit][4] #yaw

		q = quaternion_from_euler(0, 0, yaw)
		msg.pose.orientation.x = q[0]
		msg.pose.orientation.y = q[1]
		msg.pose.orientation.z = q[2]
		msg.pose.orientation.w = q[3]

		msg.header.frame_id = 'map'
		
		
		print('Publishing Waypoints', msg.pose.position.x , msg.pose.position.y)
		waypoint_pub.publish(msg)
		userdata.current_wp_cords = (msg.pose.position.x,msg.pose.position.y,yaw)
		self.success_flag = False
		return msg

	def execute(self,userdata):
		self.start_time = rospy.get_rostime().secs
		self.wp = self.global_wp_nav(userdata,waypoint_pub)
		num_unvisited =0
		for i in range(userdata.counter_wp_around_pit,len(userdata.wp_around_pit)):
			if userdata.wp_around_pit[i][3]==1:
				num_unvisited+=1
		#num_unvisited = math.ceil((halfway_point*2-userdata.counter_wp_around_pit)/3)
		self.risk_safe = smach_helper.calcuate_risk(self.start_time,TIME_OUT,num_unvisited,file_locations)
		print('Time left: {0} seconds'.format(TIME_OUT-rospy.get_rostime().secs))
		sub_odom = rospy.Subscriber("/move_base/local_costmap/footprint", PolygonStamped, self.position_cb, (userdata,self.success_flag))
		sub_battery = rospy.Subscriber("battery_charge", Temperature, smach_helper.charge_cb, (userdata,waypoint_pub,pose))
		rate = rospy.Rate(5)
		while not self.success_flag:
			rate.sleep()
		sub_odom.unregister()
		sub_battery.unregister()
		if self.success_flag:
			self.success_flag = False
			userdata.direction = 'save_data'
			userdata.alternative_point = userdata.counter_wp_around_pit
			return 'save_data'
		return 'mission_ongoing'

#CLASS 3 from pit to lander
class Pit2Lander(smach.State):

	def __init__(self):
		smach.State.__init__(self,input_keys=['wp_2_pit','counter_wp_2_pit','mission_flag','current_wp_cords'], 
						output_keys=['counter_wp_2_pit','current_wp_cords','illumination_start_time'],
						outcomes=['reached_lander','mission_ongoing','failed','MISSION_COMPLETE'])
		self.start_time = 0
		self.end_time = 0
		self.trip_number = 0
		self.success_flag = False

	def position_cb(self,msg,argc):
		if self.success_flag == True:
			return
		[pose['x'], pose['y'], pose['yaw']]= smach_helper._get_pose(msg.polygon.points)
		userdata = argc[0]                                          #userdata contains the input_keys


		if(userdata.counter_wp_2_pit != -1):
			error = math.sqrt((pose['x'] - self.wp.pose.position.x)**2 + (pose['y'] - self.wp.pose.position.y)**2)
		else:
			error = 0
		#print("Pit2Lander: Pursing Waypoint {0}, Distance to waypoint: {1}".format(userdata.counter_wp_2_pit, error))
		if(error<GLOBAL_RADIUS or userdata.counter_wp_2_pit == -1):
			userdata.counter_wp_2_pit += 1
			if(userdata.counter_wp_2_pit >= len(userdata.wp_2_pit)):
				userdata.counter_wp_2_pit = -1
				self.success_flag = True
				return
			self.wp = self.global_wp_nav(userdata,waypoint_pub)

	def global_wp_nav(self,userdata,waypoint_pub):
		
		
		
		

		msg = PoseStamped()
		num_waypoints = len(userdata.wp_2_pit)-1       #v reversed waypoint path
		msg.pose.position.x = userdata.wp_2_pit[num_waypoints - userdata.counter_wp_2_pit][0]#x
		msg.pose.position.y = userdata.wp_2_pit[num_waypoints - userdata.counter_wp_2_pit][1]#y
		yaw = 3.1415
		msg.pose.orientation.w = 0
		msg.pose.orientation.z = 1
		msg.header.frame_id = 'map'
		
		
		print('Publishing wp', msg.pose.position.x , msg.pose.position.y)
		waypoint_pub.publish(msg)
		userdata.current_wp_cords = (msg.pose.position.x,msg.pose.position.y,yaw)
		return msg		

	def execute(self,userdata):
		self.start_time = rospy.get_rostime().secs
		self.trip_number +=1
		sub_odom = rospy.Subscriber("/move_base/local_costmap/footprint", PolygonStamped, self.position_cb, (userdata,self.success_flag))
		sub_battery = rospy.Subscriber("battery_charge", Temperature, smach_helper.charge_cb, (userdata,waypoint_pub,pose))
		rate = rospy.Rate(5)
		while not (self.success_flag):
			rate.sleep()
		sub_odom.unregister()
		sub_battery.unregister()
		if self.success_flag:
			#print("Reached waypoint. On to next waypoint: {0}".format(userdata.counter_wp_2_pit))
			print("------------------------------------------------------------------------------")
			self.success_flag = False
			smach_helper.show_model('round_{0}.ply'.format(str(self.trip_number)),file_locations)

			self.end_time = rospy.get_rostime().secs
			smach_helper.timekeeper('Pit2Lander',self.start_time,self.end_time,file_locations)
			if userdata.mission_flag:
				return 'MISSION_COMPLETE'

			return 'reached_lander'
		return 'mission_ongoing'

#CLASS 4 highway around pit
class Highway(smach.State):

	def __init__(self):
		smach.State.__init__(self,input_keys=['wp_around_pit','counter_highway_wp','direction','alternative_point','current_wp_cords'],
						output_keys=['counter_highway_wp','illumination_start_time','current_wp_cords'],
						outcomes=['ready_to_nav','save_data','mission_ongoing','failed'])
		self.start_time = 0
		self.end_time = 0
		self.success_flag = False

	def position_cb(self,msg,argc):
		if self.success_flag == True:
			return
		[pose['x'], pose['y'], pose['yaw']]= smach_helper._get_pose(msg.polygon.points)
		userdata = argc[0]                                          #userdata contains the input_keys


		if(userdata.counter_highway_wp != -1):
			error = math.sqrt((pose['x'] - self.wp.pose.position.x)**2 + (pose['y'] - self.wp.pose.position.y)**2)
		else:
			error = 0
		#print("Highway: Pursing Waypoint {0}, Distance to waypoint: {1}".format(userdata.counter_highway_wp, error))
		if(error<GLOBAL_RADIUS or userdata.counter_highway_wp == -1):
			if(userdata.counter_highway_wp == userdata.alternative_point or 
			   userdata.alternative_point + userdata.counter_highway_wp == len(userdata.wp_around_pit)-1 or
			   userdata.alternative_point - userdata.counter_highway_wp == 0):
				userdata.counter_highway_wp = 0
				self.success_flag = True
				return
			
			userdata.counter_highway_wp += 1
			self.wp = self.global_wp_nav(userdata,waypoint_pub)

	def global_wp_nav(self,userdata,waypoint_pub):
		
		
		
		
		
		msg = PoseStamped()

		if userdata.direction == 'save_data' and userdata.alternative_point > halfway_point :
			num = userdata.alternative_point + userdata.counter_highway_wp
			maxn = len(userdata.wp_around_pit)-1
			if (userdata.wp_around_pit[min(num,maxn)][3] == 1):
				userdata.counter_highway_wp += 1
			msg.pose.position.x = userdata.wp_around_pit[userdata.alternative_point + userdata.counter_highway_wp][1]#x
			msg.pose.position.y = userdata.wp_around_pit[userdata.alternative_point + userdata.counter_highway_wp][2]#y

		elif userdata.direction == 'capture' and userdata.alternative_point > halfway_point:
			num_waypoints = len(userdata.wp_around_pit)-1
			if (userdata.wp_around_pit[num_waypoints - userdata.counter_highway_wp][3]              == 1):
				userdata.counter_highway_wp += 1
			msg.pose.position.x = userdata.wp_around_pit[num_waypoints - userdata.counter_highway_wp][1]#x
			msg.pose.position.y = userdata.wp_around_pit[num_waypoints - userdata.counter_highway_wp][2]#y

		elif userdata.direction == 'save_data' and userdata.alternative_point <= halfway_point:
			if (userdata.wp_around_pit[userdata.alternative_point - userdata.counter_highway_wp][3] == 1):
				userdata.counter_highway_wp += 1
			msg.pose.position.x = userdata.wp_around_pit[userdata.alternative_point - userdata.counter_highway_wp][1]#x
			msg.pose.position.y = userdata.wp_around_pit[userdata.alternative_point - userdata.counter_highway_wp][2]#y

		elif userdata.direction == 'capture' and userdata.alternative_point <= halfway_point:
			if (userdata.wp_around_pit[userdata.counter_highway_wp][3]                              == 1):
				userdata.counter_highway_wp += 1
			msg.pose.position.x = userdata.wp_around_pit[userdata.counter_highway_wp][1]#x
			msg.pose.position.y = userdata.wp_around_pit[userdata.counter_highway_wp][2]#y
		yaw = 3.14
		msg.pose.orientation.w = 0
		msg.pose.orientation.z = 1
		msg.header.frame_id = 'map'
		
		
		print('Publishing wp', msg.pose.position.x , msg.pose.position.y, current_goal )
		waypoint_pub.publish(msg)
		userdata.current_wp_cords = (msg.pose.position.x,msg.pose.position.y,yaw)
		return msg

	def execute(self,userdata):
		self.start_time = rospy.get_rostime().secs
		self.wp = self.global_wp_nav(userdata,waypoint_pub)
		sub_odom = rospy.Subscriber("/move_base/local_costmap/footprint", PolygonStamped, self.position_cb, (userdata,self.success_flag))
		sub_battery = rospy.Subscriber("battery_charge", Temperature, smach_helper.charge_cb, (userdata,waypoint_pub,pose))
		rate = rospy.Rate(5)
		while not (self.success_flag):
			rate.sleep()
		sub_odom.unregister()
		sub_battery.unregister()
		if self.success_flag:
			#print("Reached waypoint. On to next waypoint: {0}".format(userdata.counter_highway_wp))
			print("------------------------------------------------------------------------------")
			self.success_flag = False
			self.end_time = rospy.get_rostime().secs
			smach_helper.timekeeper('Highway',self.start_time,self.end_time,file_locations)
			if(userdata.direction == 'capture'):
				return 'ready_to_nav'
			elif(userdata.direction == 'save_data'):
				return 'save_data'
		return 'mission_ongoing'

def main():
	global listener
	rospy.init_node('smach_nodelet')
	listener = tf.TransformListener()
	
	#state machine initialize
	sm = smach.StateMachine(outcomes=['Mission_completed_succesfully','Mission_aborted'])
	sm.userdata.q                       = 5 # size of the array given # i dunno what this is and its not used ever TODO decide wether or not to keep this
	sm.userdata.counter_wp_2_pit        = -1
	sm.userdata.counter_wp_around_pit   = 0
	sm.userdata.wp_2_pit                = smach_helper.read_csv(file_locations['file_to_pit'])
	sm.userdata.current_wp              = sm.userdata.wp_2_pit[0]
	sm.userdata.wp_around_pit           = smach_helper.read_csv_around_pit(file_locations['file_around_pit'])
	sm.userdata.illumination_start_time = rospy.get_rostime().secs
	sm.userdata.towards_edge_time_start = rospy.get_rostime().secs
	sm.userdata.direction               = 'capture'
	sm.userdata.alternative_point       = 0
	sm.userdata.counter_highway_wp      = 0
	sm.userdata.mission_flag            = False
	sm.userdata.current_wp_cords        = (0,0,0)
	
	with sm:

		smach.StateMachine.add('Lander2Pit', Lander2Pit(),#BState(waypoint_cb),
			transitions = {'reached_pit':'Highway','mission_ongoing':'Lander2Pit' ,'failed':'Mission_aborted'})

		smach.StateMachine.add('navAROUNDPIT', circum_wp_cb(),#BState(waypoint_cb),
			transitions = {'mission_ongoing':'navAROUNDPIT', 'save_data':'Highway', 'failed':'Mission_aborted'})
		
		smach.StateMachine.add('Pit2Lander', Pit2Lander(),#BState(waypoint_cb), # reached lander is a transition point to an outside hook or is directly the data dump
			transitions = {'reached_lander':'Lander2Pit','mission_ongoing':'Pit2Lander' ,'failed':'Mission_aborted',
						   'MISSION_COMPLETE':'Mission_completed_succesfully'})

		smach.StateMachine.add('Highway', Highway(),#BState(waypoint_cb),
			transitions = {'ready_to_nav':'navAROUNDPIT','save_data':'Pit2Lander','mission_ongoing':'Highway' ,'failed':'Mission_aborted'})

	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	# Execute the state machine
	outcome = sm.execute()

	import subprocess
	viewer = subprocess.Popen(['totem', '/home/alex/Downloads/Europe_-_The Final Countdown(with).mp3'])
	import time
	time.sleep(60)
	viewer.terminate()
	viewer.kill()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()


if __name__== "__main__":
	main()
