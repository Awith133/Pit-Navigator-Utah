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
from waypoint_pit_planner.srv import waypoints
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion

from std_msgs.msg import Float32
import tf

#display image imports
import cv2
import glob


GLOBAL_RADIUS = 0.45
GLOBAL_RADIUS2 = 0.45
YAW_THRESH = 0.16 #10 deg
TIME_OUT = 200
time_resolution = 25
file_to_pit = rospy.get_param("file_to_pit")
FILE_TO_PIT = file_to_pit
file_around_pit = rospy.get_param("file_around_pit")
FILE_AROUND_PIT = file_around_pit

nav2pit_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1)
pit_edge_dist_pub = rospy.Publisher('/robot_at_edge_position', Odometry, queue_size = 1)
listener = None

def _get_pose(points):
	num_points = len(points)
	x_pose_arr = [points[i].x for i in range(0,num_points)]
	y_pose_arr = [points[i].y for i in range(0,num_points)]
	x_pose = sum(x_pose_arr)/num_points
	y_pose = sum(y_pose_arr)/num_points
	
	#print("Average current pose: {0}  {1}".format(x_pose, y_pose))
	return [x_pose, y_pose]


#CLASS 1 
class nav2PIT(smach.State):

	def __init__(self):
		smach.State.__init__(self,input_keys=['wp_2_pit','counter_wp_2_pit','illumination_start_time'],
						output_keys=['counter_wp_2_pit','illumination_start_time'],
						outcomes=['reached_pit','mission_ongoing','failed'])
		self.success_flag = False
		self.mission_flag = False

	def position_cb(self,msg,argc):
		if self.mission_flag:
			return
		[x_pose, y_pose] = _get_pose(msg.polygon.points)
		userdata = argc[0]
		if(userdata.counter_wp_2_pit != -1):
			#print(self.wp.pose.position.y)
			error = math.sqrt((x_pose - self.wp.pose.position.x)**2 + (y_pose - self.wp.pose.position.y)**2)
		else:
			error = 0
		print("nav2PIT: Pursing Waypoint {0}, Distance to waypoint: {1}".format(userdata.counter_wp_2_pit, error))
		if(error<GLOBAL_RADIUS or userdata.counter_wp_2_pit == -1):
			userdata.counter_wp_2_pit += 1
			if(userdata.counter_wp_2_pit >= len(userdata.wp_2_pit)):
				self.mission_flag = True
				return
			self.wp = self.global_wp_nav(userdata,nav2pit_pub)

	def global_wp_nav(self,userdata,nav2pit_pub):
		#publish points from csv and get x,y
		msg = PoseStamped()
		msg.pose.position.x = userdata.wp_2_pit[userdata.counter_wp_2_pit][0]#x
		msg.pose.position.y = userdata.wp_2_pit[userdata.counter_wp_2_pit][1]#y
		#print(msg.pose.position.y)
		
		msg.pose.orientation.w = 1
		msg.header.frame_id = 'map'
		print('Publishing wp', msg.pose.position.x , msg.pose.position.y )
		nav2pit_pub.publish(msg)
		return msg


	def execute(self,userdata):
		sub_odom = rospy.Subscriber("/move_base/local_costmap/footprint", PolygonStamped, self.position_cb, (userdata,self.success_flag))
		rate = rospy.Rate(5)
		while not (self.mission_flag):
			rate.sleep()
		sub_odom.unregister()
		if self.mission_flag:
			print("Reached waypoint. On to next waypoint: {0}".format(userdata.counter_wp_2_pit))
			print("------------------------------------------------------------------------------")
			self.success_flag = False
			self.mission_flag = False
			userdata.illumination_start_time = rospy.get_rostime().secs
			rospy.set_param("/start_illumination", 0) #DONT START ILLUMINATION UNTIL ITS LONGER TODO
			return 'reached_pit'
		return 'mission_ongoing'

#CLASS 2 around pit
class circum_wp_cb(smach.State):

	def __init__(self):
		smach.State.__init__(self,input_keys=['counter_wp_around_pit','wp_around_pit','illumination_start_time','towards_edge_time_start'],
					output_keys=['counter_wp_around_pit','illumination_start_time','towards_edge_time_start'],
					outcomes=['reached_vantage_zone','mission_ongoing','failed','MISSION_COMPLETE'])
		self.success_flag = False #reached a vantage zone
		self.mission_flag = False
		self.mission_start = True

#	def position_cb(self, msg, argc):
#		[x_pose, y_pose] = _get_pose(msg.polygon.points)
#		userdata = argc[0]
#		# counter is valid (not -1) and did not exaust the list of wp
#		if(userdata.counter_wp_around_pit != -1 or not self.mission_start):
#			error = math.sqrt((x_pose - self.wp.pose.position.x)**2 + (y_pose - self.wp.pose.position.y)**2)
#		else:
#			error = 0
#		# counter is valid (not -1)	#TODO UNDERSTAND THIS BETTER
#		if (userdata.counter_wp_around_pit>-1):
#			#when time is ahead
#			current_time = (rospy.get_rostime().secs - userdata.illumination_start_time)
#			while( current_time >= userdata.wp_around_pit[userdata.counter_wp_around_pit][0]): # will not work here
#				userdata.counter_wp_around_pit += 1
#			print("Circumnavigating Pit", "error", error, "time", (rospy.get_rostime().secs - userdata.illumination_start_time), "final_time", userdata.wp_around_pit[userdata.counter_wp_around_pit][0], 
#			"index",userdata.counter_wp_around_pit, "Success Flag", self.success_flag )
#		#error is less or counter is invalid or just returned from the third state
#		if(error<GLOBAL_RADIUS or userdata.counter_wp_around_pit == -1 or self.mission_start):
#			current_time = (rospy.get_rostime().secs - userdata.illumination_start_time)
#			#counter is invalid and you did not recently return from the third state
#			if (userdata.counter_wp_around_pit>-1 and not self.mission_start):
#				#you have reached the area inside the potential vantage zone
#				if (userdata.wp_around_pit[userdata.counter_wp_around_pit][3] == 1 ):
#					#you reached vantage zone in time ie before the final time elapsed
#					if not (current_time  >= userdata.wp_around_pit[userdata.counter_wp_around_pit][0]): #current time ahead of desired
#						self.success_flag = True
#						return
#				#when you reach the way point before the desired time
#				if((userdata.wp_around_pit[userdata.counter_wp_around_pit][0] - current_time )>0):
#					return
#			#update counter
#			userdata.counter_wp_around_pit += 1
#			#LIST OF WP EXHAUSTED
#			if(userdata.counter_wp_around_pit >= len(userdata.wp_around_pit)):
#				print('return to lander transition code here') #TODO
#				self.mission_flag = True
#				return
#			 
#			if userdata.wp_around_pit[userdata.counter_wp_around_pit][3] == -1:
#				print("Going in Hibernation mode :* index: value: ", userdata.counter_wp_around_pit,  userdata.wp_around_pit[userdata.counter_wp_around_pit][3] )
#			while(userdata.wp_around_pit[userdata.counter_wp_around_pit][3] == -1):
#				current_time = (rospy.get_rostime().secs - userdata.illumination_start_time)
#				if(userdata.wp_around_pit[userdata.counter_wp_around_pit][0] - current_time < 0): #desired time - current time
#					userdata.counter_wp_around_pit += 1
#			self.wp = self.global_wp_nav(nav2pit_pub,userdata)
	def position_cb(self,msg,argc):
		if self.mission_flag:
			return
		[x_pose, y_pose] = _get_pose(msg.polygon.points)
		userdata = argc[0]
		print(userdata.counter_wp_around_pit)
		if(userdata.counter_wp_around_pit != -1):
			#print(self.wp.pose.position.y)
			error = math.sqrt((x_pose - self.wp.pose.position.x)**2 + (y_pose - self.wp.pose.position.y)**2)
		else:
			error = 0
		print("navAroundPit: Pursing Waypoint {0}, Distance to waypoint: {1}".format(userdata.counter_wp_around_pit, error))
		if(error<GLOBAL_RADIUS or userdata.counter_wp_around_pit == -1):
			
			if(userdata.wp_around_pit[userdata.counter_wp_around_pit][3] == 1): #is this a vantage point == 1
				complete = False
				#rotate to waypoint TODO
				print('rotate to angle')
				q=self.wp.pose.orientation
				yaw_goal = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])[2] %6.28218
				points = msg.polygon.points
				vec_x = points[0].x - points[8].x
				vec_y = points[0].y - points[8].y
				yaw_pose = math.atan2(vec_y, vec_x) %6.28218
				print('goal, ',yaw_goal)
				print('pose, ',yaw_pose)
				print(abs(yaw_goal - yaw_pose))
				if (abs(yaw_goal - yaw_pose) < YAW_THRESH or abs(yaw_goal+6.28218 - yaw_pose) < YAW_THRESH):   #am i looking in the right direction?
					
					#display images TODO
					print('return real life pictures')
					images_folder = '/home/alex/pit-navigator-utah/Simulation/Pictures/UtahImagesMRSD/'
					iteration = {
						1:['2','1'],
						4:['2','2'],
						7:['2','3'],
						10:['2','4'],
						13:['3','1'],
						16:['3','2'],
						19:['3','3'],
						22:['3','4'],
						25:['3','5'],
						28:['3','6'],
						31:['3','7'],
						34:['4','1'],
						37:['4','2'],
						40:['4','3'],
						43:['4','4'],
						46:['4','5'],
						49:['4','6'],
						52:['4','7'],
						55:['4','8'],
						58:['5','1'],
						61:['5','2'],
						64:['5','3'],
					}
					grep = images_folder + iteration[userdata.counter_wp_around_pit][0] + '_' +iteration[userdata.counter_wp_around_pit][1]+ '_'+ '2_1_1_*_*.png'
					list1 = glob.glob(grep)
					def compare(x, y):
						partsOfX = x.split('_')
						partsOfY = y.split('_')
						integerListx = [int(partsOfX[5]),int(partsOfX[6].split('.')[0])*100]
						integerListy = [int(partsOfY[5]),int(partsOfY[6].split('.')[0])*100]
						return integerListx[0] - integerListy[0]+ integerListy[1]- integerListx[1]
					list1.sort(cmp= compare)
					for filename in list1:
						image = cv2.imread(filename)
						cv2.imshow('image',image)
						cv2.waitKey(1000)
						cv2.destroyAllWindows()
					complete = True
				if (complete):
					userdata.counter_wp_around_pit += 1
					if(userdata.counter_wp_around_pit >= len(userdata.wp_around_pit)):
						self.mission_flag = True
						return
					self.wp = self.global_wp_nav(nav2pit_pub,userdata)
			else:
				userdata.counter_wp_around_pit += 1
				if(userdata.counter_wp_around_pit >= len(userdata.wp_around_pit)):
					self.mission_flag = True
					return
				self.wp = self.global_wp_nav(nav2pit_pub,userdata)

	def global_wp_nav(self,nav2pit_pub, userdata):
		msg = PoseStamped()
		msg.pose.position.x = userdata.wp_around_pit[userdata.counter_wp_around_pit][1]#x
		msg.pose.position.y = userdata.wp_around_pit[userdata.counter_wp_around_pit][2]#y
		#print(msg.pose.position.y)
		#hardcoded pit center!!!! TODO fix
		#pit_centre = [55, 51]

		#vec_x = pit_centre[0] - msg.pose.position.x
		#vec_y = pit_centre[1] - msg.pose.position.y
		#yaw = math.atan2(vec_y, vec_x)
		yaw = userdata.wp_around_pit[userdata.counter_wp_around_pit][4] #yaw

		q = quaternion_from_euler(0, 0, yaw)
		msg.pose.orientation.x = q[0]
		msg.pose.orientation.y = q[1]
		msg.pose.orientation.z = q[2]
		msg.pose.orientation.w = q[3]

		msg.header.frame_id = 'map'
		print('Publishing Waypoints', msg.pose.position.x , msg.pose.position.y )
		nav2pit_pub.publish(msg)
		#print(userdata.wp_around_pit[userdata.counter_wp_around_pit])
		#if (self.mission_start):
		#	self.mission_start = False
		return msg

	def execute(self,userdata):
		#self.mission_start = True
		sub_odom = rospy.Subscriber("/move_base/local_costmap/footprint", PolygonStamped, self.position_cb, (userdata,self.success_flag))
		rate = rospy.Rate(10)
		while not (self.mission_flag or self.success_flag ):
			rate.sleep()
		sub_odom.unregister()
		if self.mission_flag:
			self.success_flag = False
			self.mission_flag = False
			self.mission_start = True
			return 'MISSION_COMPLETE'
		elif self.success_flag:
			self.success_flag = False
			self.mission_flag = False
			self.mission_start = True
			userdata.towards_edge_time_start = rospy.get_rostime().secs
			return 'reached_vantage_zone'
		return 'mission_ongoing'


#CLASS 3
class reach_edge_cb(smach.State):

	def __init__(self):
		smach.State.__init__(self,input_keys=['counter_wp_around_pit','wp_around_pit','illumination_start_time','towards_edge_time_start' ],
					output_keys=[ 'counter_wp_around_pit','illumination_start_time','towards_edge_time_start'],
					outcomes=['reached_edge','mission_ongoing','failed'])
		self.gen_first_flag = False
		self.mission_flag = False
		self.mission_failure = False
		self.wp = None		


	def position_cb(self,msg, argc):
		print("Mission Flag: ", self.mission_flag)
		userdata = argc[0]
		current_time = rospy.get_rostime().secs
		if(userdata.towards_edge_time_start - current_time > TIME_OUT):
			self.gen_first_flag = True
			self.mission_flag = False
			self.mission_failure = False
			print("I have been timed out")
			return
		[x_pose, y_pose] = _get_pose(msg.polygon.points)
		#--------------------------------------
		manual_override = rospy.get_param("manual_override")
		print("I am kinda safe: ", manual_override)
		if (manual_override):
			self.mission_flag = True
			
			msg = PoseStamped()
			msg.header.frame_id = 'map'
			msg.pose.position.x = x_pose#x
			msg.pose.position.y = -y_pose#y
			print(msg.pose.position.y)

			msg.pose.orientation.x = 0
			msg.pose.orientation.y = 0
			msg.pose.orientation.z = 0
			msg.pose.orientation.w = 1
			
			nav2pit_pub.publish(msg)
			print("staying at the same place")
			return
		if self.gen_first_flag:
			error = 0
			self.gen_first_flag = False
		else:
			error = (math.sqrt((x_pose - self.wp.pose.position.x)**2 + (y_pose - self.wp.pose.position.y)**2)) if self.wp is not None else 0
		
		print("nav2PIT: Pursing Waypoint {0}, Distance to waypoint: {1}".format(userdata.wp_around_pit[userdata.counter_wp_around_pit], error))
		print("Clear of the first set of ifs.")
		if (error<GLOBAL_RADIUS2):
			rospy.wait_for_service('gen_wp2pit')
			nav2pit_serv = rospy.ServiceProxy('gen_wp2pit', waypoints)
			try:
				#print('self.wp_gen.. 4')
				self.wp_gen = nav2pit_serv()
				self.mission_flag = self.wp_gen.mission_flag
				print("I am inside smach: {0}".format(self.mission_flag))
				if (self.mission_flag):
					(trans, rot) = listener.lookupTransform('map','base_link',rospy.Time())
					euler = tf.transformations.euler_from_quaternion(rot)
					while(abs(self.goal_yaw - euler[2])>YAW_THRESH):
						# print("Quaternion",rot)
						# print(euler)
						# print("ERROR in YAW: ",abs(self.goal_yaw - euler[2]) )
						(trans, rot) = listener.lookupTransform('map','base_link',rospy.Time())
						euler = tf.transformations.euler_from_quaternion(rot)
					msg_odom = Odometry()
					msg_odom.header.frame_id = 'map'
					msg_odom.pose.pose.position.x = x_pose#x
					msg_odom.pose.pose.position.y = -1*y_pose#y
					pit_edge_dist_pub.publish(msg_odom)

				self.mission_failure  = not self.wp_gen.wp_received
				if (self.wp_gen.wp_received and not self.gen_first_flag):# and not self.mission_flag):
						self.wp = self.global_wp_nav(nav2pit_pub,self.wp_gen)
						self.gen_first_flag = False
			except rospy.ServiceException as exc:
				self.gen_first_flag = True
				print("Service did not process request: " + str(exc))
		print("I am done now")

					

	def global_wp_nav(self,nav2pit_pub,wp_gen):
		#service call to ayush's function to get way point
		msg = PoseStamped()
		if (wp_gen.yaw == 100):
			light_dir_sev = rospy.wait_for_message('where_to_see', Float32)
			wp_gen.yaw = light_dir_sev.data
			self.goal_yaw = light_dir_sev.data
			print("Taking yaw from where to see", wp_gen.yaw)
		
		msg.pose.position.x = wp_gen.x#x
		msg.pose.position.y = wp_gen.y#y
		q = quaternion_from_euler(0, 0, wp_gen.yaw)
		msg.pose.orientation.x = q[0]
		msg.pose.orientation.y = q[1]
		msg.pose.orientation.z = q[2]
		msg.pose.orientation.w = q[3]
		print("Gen Points:", wp_gen.x, wp_gen.y,  wp_gen.yaw, q[0], q[1], q[2], q[3])
		msg.header.frame_id = 'map'
		nav2pit_pub.publish(msg)
		return msg


	def execute(self,userdata):
		############################REMOVE
		return 'reached_edge' #TODO remove
		# userdata.towards_edge_time_start = rospy.get_rostime().secs
		# sub_odom = rospy.Subscriber("/move_base/local_costmap/footprint", PolygonStamped, self.position_cb, (userdata,self.gen_first_flag))
		print("Starting wait for message")
		polygon = rospy.wait_for_message("/move_base/local_costmap/footprint", PolygonStamped)
		self.position_cb(polygon, (userdata, self.gen_first_flag))
		# rate = rospy.Rate(1)
		print("stage 3",self.mission_flag , self.mission_failure , (userdata.towards_edge_time_start - rospy.get_rostime().secs > TIME_OUT ))
		# while not (self.mission_flag or self.mission_failure or (userdata.towards_edge_time_start - rospy.get_rostime().secs > TIME_OUT) ):
		# 	print("I should sleep")
		# 	rate.sleep()

		current_time = rospy.get_rostime().secs
		# sub_odom.unregister()
		#--------------------------------------
		
		if self.mission_flag:
			rospy.set_param("manual_override", False)
			self.gen_first_flag = True
			self.mission_flag = False
			print("Mission flag reset")
			self.mission_failure = False
			return 'reached_edge'
		if self.mission_failure:
			self.gen_first_flag = True
			self.mission_flag = False
			self.mission_failure = False
			return 'failed'
		return 'mission_ongoing'


def read_csv(filename):
	wp = []
	offset = 2.5
	resolution = 0.1
	with open(filename, 'rb') as f:
		reader = csv.reader(f, delimiter=',')
		for row in reader:
			print(row)
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

def read_csv_with_time(filename):
	wp = []
	map_offset = 2.5
	map_resolution = 0.1
	
	time_offset = 0
	with open(filename, 'rb') as f:
		reader = csv.reader(f, delimiter=',')
		
		for row in reader:
			tmp = []
			i = 0
			for elem in row:
				if (i == 0):
					tmp.append(int(elem) * time_resolution + time_offset) #waypoint num
				elif (i==1):
					tmp.append((int(elem) * map_resolution )) #x
				elif (i == 2):
					tmp.append(-1*int(elem) * map_resolution ) #y
				elif( i==3):
					tmp.append(int(elem)) #TODO remove this back to int(elem) #vatage point y = 1, n = -1
				elif(i==4):
					tmp.append((int(elem)-90)*3.14159/180) #yaw degrees for waypoint
				i+=1
			wp.append(tmp)
	return wp


def main():
	global listener
	rospy.init_node('smach_nodelet')
	listener = tf.TransformListener()
	sm = smach.StateMachine(outcomes=['Mission_completed_succesfully','Mission_aborted'])
	sm.userdata.q = 5 # size of the array given

	sm.userdata.counter_wp_2_pit = -1
	sm.userdata.counter_wp_around_pit = -1
	sm.userdata.wp_2_pit = read_csv(FILE_TO_PIT)
	#print(sm.userdata.wp_2_pit)
	print("Length of CSV data is: "+str(len(sm.userdata.wp_2_pit)))
	sm.userdata.current_wp = sm.userdata.wp_2_pit[0]
	sm.userdata.wp_around_pit = read_csv_with_time(FILE_AROUND_PIT)
	sm.userdata.illumination_start_time = rospy.get_rostime().secs
	sm.userdata.towards_edge_time_start = rospy.get_rostime().secs

	with sm:

		smach.StateMachine.add('nav2PIT', nav2PIT(),#BState(nav2pit_cb),
								 transitions = {'reached_pit':'navAROUNDPIT','mission_ongoing':'nav2PIT' ,'failed':'Mission_aborted'})

		smach.StateMachine.add('navAROUNDPIT', circum_wp_cb(),#BState(nav2pit_cb),
						 transitions = {'reached_vantage_zone':'nav2EDGE', 'mission_ongoing':'navAROUNDPIT', 'failed':'Mission_aborted',
						 'MISSION_COMPLETE':'Mission_completed_succesfully'})
		
		smach.StateMachine.add('nav2EDGE', reach_edge_cb(), #BState(nav2pit_cb),
				 transitions = {'reached_edge':'navAROUNDPIT', 'mission_ongoing':'nav2EDGE','failed':'Mission_aborted'})



		# smach.StateMachine.add('nav2EDGE', reach_edge_cb(), #BState(nav2pit_cb),
		# 		 transitions = {'reached_edge':'Mission_aborted', 'mission_ongoing':'nav2EDGE','failed':'Mission_aborted'})

		# smach.StateMachine.add('navAROUNDPIT', circum_wp_cb(),#BState(nav2pit_cb),
		# 				 transitions = {'reached_vantage_zone':'Mission_aborted', 'mission_ongoing':'navAROUNDPIT', 'failed':'Mission_aborted',
		# 				 'MISSION_COMPLETE':'Mission_completed_succesfully'})


	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	# Execute the state machine
	outcome = sm.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()


if __name__== "__main__":
	main()
