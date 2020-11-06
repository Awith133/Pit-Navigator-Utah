#!/usr/bin/env python
import csv
import rospy
import math
import smach
import pdb

import smach_ros
from smach import CBState
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String
from geometry_msgs.msg import PoseStamped, PolygonStamped

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import Temperature
import tf
import time

#display image imports
import smach_helper
# from cloud_process import CloudSubscriber
from sim_brinkmanship import Brinkmanship
import sys

translation = [0, 0.13, -0.18]
rotation = [-0.3583641, 0, 0, 0.9335819]
# alert_helper = CloudSubscriber(translation, rotation)
brink_controller = Brinkmanship()

GLOBAL_RADIUS = .45
GLOBAL_RADIUS2 = 1
YAW_THRESH = 0.16 #10 deg
file_locations = {
	'file_to_pit':rospy.get_param("file_to_pit"),
	'file_around_pit':rospy.get_param("file_around_pit"),
	'project_file_location':rospy.get_param("/system_name"),
	'robot_simulation_env':rospy.get_param("/robot_simulation_arg"),}
if ('Simulation' in file_locations['robot_simulation_env']):
	sys.path.insert(1,file_locations['robot_simulation_env'][0:file_locations['robot_simulation_env'].rfind("/",0,-1)]+"/webots_control/src")
	from cloud_process import CloudSubscriber
	from sim_brinkmanship import Brinkmanship
	
	if ('Utah_Pit' in file_locations['robot_simulation_env']):
		TIME_OUT = 2200*1.3 #normal = 1.3 big = 1 
	if ('Utah_BIG' in file_locations['robot_simulation_env']):
		TIME_OUT = 2200*1 #normal = 1.3 big = 1 
	if ('Lacus_Mortis_Pit' in file_locations['robot_simulation_env']):
		TIME_OUT = 2200*10
	if ('Pit_Edge_Test' in file_locations['robot_simulation_env']):
		TIME_OUT = 4*160+160 #normal = 1.3 big = 1  
else:
	# sys.path.insert(1,file_locations['robot_simulation_env'][0:file_locations['robot_simulation_env'].rfind("/",0,-1)]+"/Brinkmanship")
	# from cloud_process import CloudSubscriber
	# from brinkmanship import Brinkmanship
	TIME_OUT = 80+3*160+240 + rospy.get_rostime().secs

# alert_helper = CloudSubscriber(translation, rotation)
# brink_controller = Brinkmanship()


map_resolution = rospy.get_param("/resolution")
halfway_point = len(smach_helper.read_csv_around_pit(file_locations['file_around_pit'],map_resolution))/2

waypoint_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1)
pit_edge_dist_pub = rospy.Publisher('/robot_at_edge_position', Odometry, queue_size = 1)

move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

sub_where_to_see = rospy.Subscriber('where_to_see',Float32,smach_helper.update_sun)

listener = None

# Brinkmanship Edge Alert Flag
alert_helper = smach_helper.BrinkStatus()

#Class 1 from lander to pit
class Lander2Pit(smach.State):
	def __init__(self):
		smach.State.__init__(self,input_keys=['wp_2_pit','counter_wp_2_pit','illumination_start_time','current_wp_cords'],
						output_keys=['direction','alternative_point','current_wp_cords','counter_wp_2_pit','illumination_start_time'],
						outcomes=['reached_pit','mission_ongoing','failed'])
		self.success_flag = False
		self.start_time = 0
		self.end_time = 0

	def nextwaypoint(self,userdata,num):
		userdata.counter_wp_2_pit += num
		if userdata.counter_wp_2_pit < len(userdata.wp_2_pit):
			time.sleep(1)
			self.global_wp_nav(userdata)
		else:
			userdata.counter_wp_2_pit -= 1
			self.success_flag = True

	def global_wp_nav(self,userdata):
		#publish points from csv and get x,y
		msg = PoseStamped()
		msg.pose.position.x = userdata.wp_2_pit[userdata.counter_wp_2_pit][0]#x
		msg.pose.position.y = userdata.wp_2_pit[userdata.counter_wp_2_pit][1]#y
		print(len(userdata.wp_2_pit[userdata.counter_wp_2_pit]))
		if(len(userdata.wp_2_pit[userdata.counter_wp_2_pit])== 3):
			yaw = userdata.wp_2_pit[userdata.counter_wp_2_pit][2]*3.14159/180.0
			q = quaternion_from_euler(0, 0, userdata.wp_2_pit[userdata.counter_wp_2_pit][2]*3.14159/180.0)
		else:
			if abs(smach_helper.pose['yaw']-smach_helper.sunangle)<=math.pi/2: 
				yaw = smach_helper.sunangle
				q = quaternion_from_euler(0, 0, smach_helper.sunangle)
			else: 
				yaw = smach_helper.sunangle+math.pi
				q = quaternion_from_euler(0, 0, smach_helper.sunangle+math.pi)
		msg.pose.orientation.x = q[0]
		msg.pose.orientation.y = q[1]
		msg.pose.orientation.z = q[2]
		msg.pose.orientation.w = q[3]
		msg.header.frame_id = 'map'
		print('Publishing wp', msg.pose.position.x , msg.pose.position.y, yaw)
		userdata.current_wp_cords = (msg.pose.position.x,msg.pose.position.y,smach_helper.sunangle)
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now() 
		goal.target_pose.pose = msg.pose
		rospy.loginfo("Sending goal pose "+str(smach_helper.current_goal)+" to Action Server")
		move_base_client.send_goal(goal, smach_helper.done_cb, smach_helper.active_cb, smach_helper.feedback_cb)
	
	def execute(self,userdata):
		self.start_time = rospy.get_rostime().secs
		sub_battery = rospy.Subscriber("battery_charge", Temperature, smach_helper.charge_cb, (userdata,move_base_client))
		self.nextwaypoint(userdata,1)
		rate = rospy.Rate(5)
		while not (self.success_flag):
			rate.sleep()
			if smach_helper.step_flag and not smach_helper.charging:
				smach_helper.step_flag = False
				while userdata.counter_wp_2_pit == 4:  ##TODO remove after testing
					rate.sleep()
				self.nextwaypoint(userdata,1)
			if smach_helper.abort_flag:
				smach_helper.abort_flag = False
				self.nextwaypoint(userdata,0)
		sub_battery.unregister()
		if self.success_flag:
			#print("Reached waypoint. On to next waypoint: {0}".format(userdata.counter_wp_2_pit))
			print("------------------------------------------------------------------------------")
			self.success_flag = False
			userdata.illumination_start_time = rospy.get_rostime().secs
			rospy.set_param("/start_illumination", 0) #DONT START ILLUMINATION UNTIL ITS LONGER TODO
			userdata.direction = 'capture'
			userdata.counter_wp_2_pit = 0
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
		self.risk_safe = 3
		self.first_waypoint = True

	def atThisWaypoint(self,userdata):
		#made it and ready to move on - anything to do here?
		if(self.vantage_return ):
			rospy.logwarn('vantage return')
			self.end_time = rospy.get_rostime().secs
			smach_helper.timekeeper('navAROUNDPIT',self.start_time,self.end_time,file_locations)
			self.start_time = rospy.get_rostime().secs
			self.vantage_return = False
			return
		if(userdata.wp_around_pit[userdata.counter_wp_around_pit][3] == 1 ): #is this a vantage point? yes = 1
			rospy.logwarn('return real life pictures')
			# Reached vantage point
			rospy.loginfo('Starting Brinkmanship Node')
			#go
			start_time_going = rospy.get_rostime().secs
			while alert_helper.alert_flag != 2:
				if alert_helper.is_published:
					if alert_helper.alert_flag == 0:
						brink_controller.generate_twist_msg([0.05, 0, 0], [0, 0, 0])
					else:
						brink_controller.generate_twist_msg([0.05, 0, 0], [0, 0, 0])
					brink_controller.publish_twist_msg()
					alert_helper.is_published = False
				else:
					pass
				#rospy.loginfo(alert_helper.alert_bool)
			end_time_going = rospy.get_rostime().secs
			# zero twist to stop
			brink_controller.generate_twist_msg([0, 0, 0], [0, 0, 0])
			brink_controller.publish_twist_msg()
			# camera functions
			smach_helper.display_real_images(userdata,file_locations) #relpace with pan tilt motions on robot #make it stop this one
			#return
			start_time_coming = rospy.get_rostime().secs
			while start_time_coming+(end_time_going-start_time_going)/2>rospy.get_rostime().secs:
				brink_controller.generate_twist_msg([-0.1, 0, 0], [0, 0, 0])
				brink_controller.publish_twist_msg()
			brink_controller.generate_twist_msg([0, 0, 0], [0, 0, 0])
			brink_controller.publish_twist_msg()
			self.vantage_return = True
			self.count_visited += 1
			return

	def nextwaypoint(self,userdata,num):
		#moving on to next waypoint
		userdata.counter_wp_around_pit += num
		if userdata.counter_wp_around_pit >= len(userdata.wp_around_pit)-1:
			print('MISSION SUCCESS')
			userdata.mission_flag = True
			self.count_visited = 0
			userdata.counter_wp_around_pit -= 1
			self.success_flag = True
		# elif self.count_visited >= self.risk_safe:
		# 	rospy.logerr("Risk too high - returning home")
		# 	self.count_visited = 0
		# 	userdata.counter_wp_around_pit -= 1
		# 	self.success_flag = True
		else:
			self.global_wp_nav(userdata)
			
	def global_wp_nav(self,userdata): #publishes the next waypoint in the list and gives it to local algorithm to navigate to
		rospy.sleep(rospy.Duration(5.0))
		msg = PoseStamped()
		msg.header.frame_id = 'map'
		#postion
		msg.pose.position.x = userdata.wp_around_pit[userdata.counter_wp_around_pit][1]#x
		msg.pose.position.y = userdata.wp_around_pit[userdata.counter_wp_around_pit][2]#y
		#orientation
		yaw = userdata.wp_around_pit[userdata.counter_wp_around_pit][4] #yaw
		q = quaternion_from_euler(0, 0, yaw)
		msg.pose.orientation.x = q[0]
		msg.pose.orientation.y = q[1]
		msg.pose.orientation.z = q[2]
		msg.pose.orientation.w = q[3]
		#update params
		print('Publishing Waypoints', msg.pose.position.x , msg.pose.position.y, userdata.counter_wp_around_pit)
		userdata.current_wp_cords = (msg.pose.position.x,msg.pose.position.y,yaw)
		#self.success_flag = False
		#send action
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now() 
		goal.target_pose.pose = msg.pose
		rospy.loginfo("Sending goal pose "+str(smach_helper.current_goal)+" to Action Server")
		move_base_client.send_goal(goal, smach_helper.done_cb, smach_helper.active_cb, smach_helper.feedback_cb)

	def execute(self,userdata):
		#initialize
		self.start_time = rospy.get_rostime().secs
		num_unvisited =0
		for i in range(userdata.counter_wp_around_pit,len(userdata.wp_around_pit)):
			if userdata.wp_around_pit[i][3]==1:
				num_unvisited+=1
		self.risk_safe = smach_helper.calcuate_risk(self.start_time,TIME_OUT,num_unvisited,file_locations)
		print('Time left: {0} seconds'.format(TIME_OUT-rospy.get_rostime().secs))
		sub_battery = rospy.Subscriber("battery_charge", Temperature, smach_helper.charge_cb, (userdata,move_base_client))
		rate = rospy.Rate(5)
		
		#run loop
		self.nextwaypoint(userdata,0)
		while not (self.success_flag):
			rate.sleep()
			if smach_helper.step_flag and not smach_helper.charging:
				smach_helper.step_flag = False
				self.atThisWaypoint(userdata)
				self.nextwaypoint(userdata,1)
			if smach_helper.abort_flag:
				smach_helper.abort_flag = False
				self.nextwaypoint(userdata,0)
		
		#close out and move on
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

	def nextwaypoint(self,userdata,num):
		userdata.counter_wp_2_pit += num
		if userdata.counter_wp_2_pit < len(userdata.wp_2_pit):
			self.global_wp_nav(userdata)
		else:
			userdata.counter_wp_2_pit -= 1
			self.success_flag = True

	def global_wp_nav(self,userdata):
		msg = PoseStamped()
		num_waypoints = len(userdata.wp_2_pit) -1      #v reversed waypoint path
		msg.pose.position.x = userdata.wp_2_pit[num_waypoints - userdata.counter_wp_2_pit][0]#x
		msg.pose.position.y = userdata.wp_2_pit[num_waypoints - userdata.counter_wp_2_pit][1]#y
		yaw = 3.1415
		yaw = smach_helper.sunangle+math.pi
		q = quaternion_from_euler(0, 0, smach_helper.sunangle+math.pi)
		msg.pose.orientation.x = q[0]
		msg.pose.orientation.y = q[1]
		msg.pose.orientation.z = q[2]
		msg.pose.orientation.w = q[3]
		msg.header.frame_id = 'map'

		print('Publishing wp', msg.pose.position.x , msg.pose.position.y)
		#waypoint_pub.publish(msg)
		userdata.current_wp_cords = (msg.pose.position.x,msg.pose.position.y,yaw)
		#return msg		
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now() 
		goal.target_pose.pose = msg.pose
		rospy.loginfo("Sending goal pose "+str(smach_helper.current_goal)+" to Action Server")
		move_base_client.send_goal(goal, smach_helper.done_cb, smach_helper.active_cb, smach_helper.feedback_cb)

	def execute(self,userdata):
		self.start_time = rospy.get_rostime().secs
		self.trip_number +=1
		sub_battery = rospy.Subscriber("battery_charge", Temperature, smach_helper.charge_cb, (userdata,move_base_client))
		self.nextwaypoint(userdata,0)
		rate = rospy.Rate(5)
		while not (self.success_flag):
			rate.sleep()
			if smach_helper.step_flag and not smach_helper.charging:
				smach_helper.step_flag = False
				self.nextwaypoint(userdata,1)
			if smach_helper.abort_flag:
				smach_helper.abort_flag = False
				self.nextwaypoint(userdata,0)
		sub_battery.unregister()
		if self.success_flag:
			#print("Reached waypoint. On to next waypoint: {0}".format(userdata.counter_wp_2_pit))
			print("------------------------------------------------------------------------------")
			self.success_flag = False
			smach_helper.show_model('round_{0}.ply'.format(str(self.trip_number)),file_locations)
			userdata.counter_wp_2_pit = -1
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

	def nextwaypoint(self,userdata,num):
		if(userdata.counter_highway_wp == userdata.alternative_point or 
		   userdata.alternative_point + userdata.counter_highway_wp == len(userdata.wp_around_pit)-1 or
		   userdata.alternative_point - userdata.counter_highway_wp == 0):
			userdata.counter_highway_wp = 0
			self.success_flag = True

		userdata.counter_highway_wp += num
		if userdata.counter_highway_wp < userdata.alternative_point:
			self.global_wp_nav(userdata)
		else:
			userdata.counter_highway_wp -= 1
			self.success_flag = True

	def global_wp_nav(self,userdata):
		msg = PoseStamped()

		#postion
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

		#orientation
		yaw = 3.14
		if abs(smach_helper.pose['yaw']-smach_helper.sunangle)<=math.pi/2: 
			yaw = smach_helper.sunangle
			q = quaternion_from_euler(0, 0, smach_helper.sunangle)
		else: 
			yaw = smach_helper.sunangle+math.pi
			q = quaternion_from_euler(0, 0, smach_helper.sunangle+math.pi)
		msg.pose.orientation.x = q[0]
		msg.pose.orientation.y = q[1]
		msg.pose.orientation.z = q[2]
		msg.pose.orientation.w = q[3]
		msg.header.frame_id = 'map'
		
		#update params
		print('Publishing wp', msg.pose.position.x , msg.pose.position.y, smach_helper.current_goal )
		userdata.current_wp_cords = (msg.pose.position.x,msg.pose.position.y,yaw)
		
		#send action
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now() 
		goal.target_pose.pose = msg.pose
		rospy.loginfo("Sending goal pose "+str(smach_helper.current_goal)+" to Action Server")
		move_base_client.send_goal(goal, smach_helper.done_cb, smach_helper.active_cb, smach_helper.feedback_cb)

	def execute(self,userdata):
		#initialize
		self.start_time = rospy.get_rostime().secs
		sub_battery = rospy.Subscriber("battery_charge", Temperature, smach_helper.charge_cb, (userdata,move_base_client))
		rate = rospy.Rate(5)
		
		#run loop
		self.nextwaypoint(userdata,0)
		while not (self.success_flag):
			rate.sleep()
			if smach_helper.step_flag and not smach_helper.charging:
				smach_helper.step_flag = False
				self.nextwaypoint(userdata,1)
			if smach_helper.abort_flag:
				smach_helper.abort_flag = False
				self.nextwaypoint(userdata,0)
		
		#close out and move on
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
	global listener, map_resolution
	rospy.init_node('smach_nodelet')
	listener = tf.TransformListener()
	#state machine initialize
	sm = smach.StateMachine(outcomes=['Mission_completed_succesfully','Mission_aborted'])
	sm.userdata.q                       = 5 # size of the array given # i dunno what this is and its not used ever TODO decide wether or not to keep this
	sm.userdata.counter_wp_2_pit        = -1
	sm.userdata.counter_wp_around_pit   = 0
	sm.userdata.wp_2_pit                = smach_helper.read_csv(file_locations['file_to_pit'], map_resolution)
	sm.userdata.current_wp              = sm.userdata.wp_2_pit[0]
	sm.userdata.wp_around_pit           = smach_helper.read_csv_around_pit(file_locations['file_around_pit'], map_resolution)
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
	rospy.loginfo("Waiting for move_base action server...")
	
	# while(True):
	# 	rospy.loginfo('logging info')
	# 	rospy.logwarn('return real life pictures')
	# 	rospy.loginfo(str(alert_helper.alert_bool))
	# 	# time.sleep(1)
	# 	rospy.sleep(1)
	brink_controller.generate_twist_msg([0.0, 0, 0], [0, 0, 0])
	brink_controller.publish_twist_msg()

	wait = move_base_client.wait_for_server(rospy.Duration(275.0))
	if not wait:
		rospy.logerr("Action server not available!")
		rospy.signal_shutdown("Action server not available!")
		return
	rospy.loginfo("Connected to move base server")
	rospy.loginfo("Starting goals achievements ...")

	# Execute the state machine
	outcome = sm.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()


if __name__== "__main__":
	main()
