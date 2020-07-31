#!/usr/bin/env python
import rospy
import numpy as np
from numpy import genfromtxt
import random
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import style
import pdb
from math import cos,sin,radians
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
from nav_msgs.msg import Odometry
from webots_ros.srv import save_image
import rosservice

#=========================================================================================================================================

fig,((ax1,ax2),(ax3,ax4)) = plt.subplots(2,2)
# fig, (ax1,ax2) = plt.subplots(1,2)
count = 0
global_least_euclidian_distances = []
global_pit_edges = []
local_least_euclidian_distances = []
g_number_of_waypoints_within_threshold = 0
global_waypoints_threshold_distance = 15	# distance in (m)
local_waypoints_threshold_distance = 1	# distance in (m)
g_local_waypoint_count = 0
# g_local_waypoints = np.array([[0,0]])	#Random default initialization
g_local_waypoints = None
GLOBAL_MAP_RESOLUTION = 5
GLOBAL_WAYPOINT_METRIC = 80	#80%
LOCAL_WAYPOINT_METRIC = 60	#60%
ROBOT_CENTER_TO_FRONT_DISTANCE = 0.25 #m

#==========================================================================================================================================

def local_waypoint_callback(msg):
	global g_local_waypoints
	global g_local_waypoint_count
	print("In localwaypoint callback : {0}".format(g_local_waypoints))
	if(g_local_waypoint_count==0):
		g_local_waypoints = np.array([[msg.pose.pose.position.x,msg.pose.pose.position.y]])
	else:
		g_local_waypoints = np.vstack((g_local_waypoints,np.array([[msg.pose.pose.position.x,msg.pose.pose.position.y]])))

# def pit_image_callback(msg):			#Use the msg.flag to see if you need to keep publishing old message or use the new one
# 	rospy.loginfo('Image received...')
# 	image = CvBridge().imgmsg_to_cv2(msg)
# 	ax2.imshow(image)
# 	ax2.axis('off')

def get_least_euclidian_distances(global_waypoints,pit_edges,distance_threshold):
	least_euclidian_distances = []
	number_of_waypoints_within_threshold = 0
	for i in range(global_waypoints.shape[0]):
		dist = np.asscalar(np.min(np.linalg.norm(global_waypoints[i,:] - pit_edges, axis=1)))
		least_euclidian_distances.append(dist)

		if(dist<distance_threshold):
			number_of_waypoints_within_threshold+=1

	return least_euclidian_distances,number_of_waypoints_within_threshold

def transform_global_waypoints(global_waypoints):
	return global_waypoints*GLOBAL_MAP_RESOLUTION + GLOBAL_MAP_RESOLUTION/2

def get_global_waypoints_data():
	path = str(rospy.get_param("/visualization_path"))
	# path = "/home/hash/catkin_ws/src/visualization"
	# path = rospy.get_param("path")
	pit_edges_file_name = path + "/data/pit_edges.csv"
	global_waypoints_file_name = path + "/data/global_waypoints.csv"
	global_waypoints = genfromtxt(global_waypoints_file_name, delimiter=',')		#TODO: Resolution transform these global waypoints
	global_waypoints = transform_global_waypoints(global_waypoints)
	pit_edges = genfromtxt(pit_edges_file_name, delimiter=',')
	pit_edges = pit_edges/2
	least_euclidian_distances,number_of_waypoints_within_threshold = get_least_euclidian_distances(global_waypoints,pit_edges,global_waypoints_threshold_distance)
	return least_euclidian_distances,pit_edges,number_of_waypoints_within_threshold

def local_waypoints_callback(msg):			#Use the msg.flag to see if this is the last point before the state machien transitions for a new state
	rospy.loginfo('New local waypoint received...')
	if(msg.flag):
		local_least_euclidian_distances.append(np.asscalar(np.min(np.linalg.norm(np.array([msg.x,msg.y]) - global_pit_edges, axis=1))))

def img_saver_client(file_path, quality=100):
    rospy.wait_for_service('/apnapioneer3at/MultiSense_S21_meta_camera/save_image')
    try:
        save_img = rospy.ServiceProxy('/apnapioneer3at/MultiSense_S21_meta_camera/save_image', save_image)
        resp1 = save_img(file_path, quality)
        print("Image saved")
        # return resp1.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def set_font_size(ax):
	for label in (ax.get_xticklabels() + ax.get_yticklabels()):
		label.set_fontname('Arial')
		label.set_fontsize(28)

def animate(frames):
	global ax1
	# global ax2
	# global ax3
	# global global_waypoint_distance_values
	# global global_waypoint_count
	global global_waypoints_threshold_distance
	global g_number_of_waypoints_within_threshold
	global count
	global global_least_euclidian_distances
	global global_pit_edges
	global g_local_waypoints
	global g_local_waypoint_count
	rospy.loginfo("In Animate \n")

	if(count==0):
		least_euclidian_distances,pit_edges,number_of_waypoints_within_threshold = get_global_waypoints_data()	
		global_least_euclidian_distances = least_euclidian_distances
		g_number_of_waypoints_within_threshold = number_of_waypoints_within_threshold
		global_pit_edges = pit_edges
		print("Distances calculated")
		count+=1
	else:
		least_euclidian_distances = global_least_euclidian_distances
		number_of_waypoints_within_threshold = g_number_of_waypoints_within_threshold

	ax1.clear()
	# average = sum(least_euclidian_distances) / len(least_euclidian_distances)
	x = np.arange(len(least_euclidian_distances))  # the label locations
	width = 0.35  # the width of the bars

	rects1 = ax1.bar(x - width/2, least_euclidian_distances, width, label='')

	# _, ymax = ax1.get_ylim()
	ax1.set_ylabel('Distance of global waypoints from pit edge (m)',fontsize=28)
	ax1.set_xlabel('Global Waypoint number',fontsize=28)
	set_font_size(ax1)
	ax1.set_title('Distances of global waypoints from pit edge',fontsize=28)
	ax1.hlines(y=global_waypoints_threshold_distance, xmin=x[0]-1, xmax=len(x), linestyle='--', color='r')
	ax1.text(int(0.5*(len(x)+x[0])), global_waypoints_threshold_distance*1.02, 'Threshold value',fontsize=20)
	# ax1.text(-20, global_waypoints_threshold_distance*1.02, 'Threshold distance: {:.2f}'.format(global_waypoints_threshold_distance),fontsize=20)

	# place a text box in upper left in axes coords
	metric = number_of_waypoints_within_threshold*100/len(least_euclidian_distances)
	textstr = '% of waypoints within threshold: {:.2f}'.format(metric)
	if(metric>GLOBAL_WAYPOINT_METRIC):
		props = dict(boxstyle='round', facecolor='lightgreen', alpha=0.5)
	else:
		props = dict(boxstyle='round', facecolor='darksalmon', alpha=0.5)

	ax1.text(0.02, 0.99, textstr, transform=ax1.transAxes, fontsize=22,
        verticalalignment='top', bbox=props, weight='bold')
	# ax1.set_xticks(x)
	# ax1.set_xticklabels(x)
	# ax2.get_xaxis().set_ticks([])
	ax1.legend()
	# autolabel(rects1,ax1)
	# fig.tight_layout()

	if(g_local_waypoints is not None and g_local_waypoint_count<g_local_waypoints.shape[0]):
		g_local_waypoint_count+=1
		plot_len = min(g_local_waypoint_count,g_local_waypoints.shape[0])
		ax3.clear()
		print("I am here ************************************")
		local_least_euclidian_distances,number_of_waypoints_within_threshold = get_least_euclidian_distances(g_local_waypoints[0:plot_len,:],global_pit_edges,local_waypoints_threshold_distance)
		# local_average = sum(local_least_euclidian_distances) / len(local_least_euclidian_distances)
		local_x = np.arange(len(local_least_euclidian_distances))
		local_least_euclidian_distances = [x - ROBOT_CENTER_TO_FRONT_DISTANCE for x in local_least_euclidian_distances]
		rects2 = ax3.bar(local_x - width/4, local_least_euclidian_distances, width, label='')

		ax3.set_ylabel('Distance of local waypoints from pit edge (m)',fontsize=28)
		ax3.set_xlabel('Local Waypoint number',fontsize=28)
		ax3.set_title('Distances of local waypoints from pit edge',fontsize=28)
		set_font_size(ax3)
		ax3.hlines(y=local_waypoints_threshold_distance, xmin=local_x[0]-1, xmax=len(local_x), linestyle='--', color='r')
		ax3.text(local_x[0]-1, local_waypoints_threshold_distance*1.02, 'Threshold value',fontsize=20)

		# place a text box in upper left in axes coords
		local_metric = number_of_waypoints_within_threshold*100/len(local_least_euclidian_distances)
		textstr = '% of waypoints within threshold: {:.2f}'.format(local_metric)
		if(local_metric>LOCAL_WAYPOINT_METRIC):
			props = dict(boxstyle='round', facecolor='lightgreen', alpha=0.5)
		else:
			props = dict(boxstyle='round', facecolor='darksalmon', alpha=0.5)

		ax3.text(0.02, 0.99, textstr, transform=ax3.transAxes, fontsize=22,verticalalignment='top', bbox=props, weight='bold')
		ax3.set_xticks(local_x)
		ax3.set_xticklabels(local_x)
		ax3.legend()
		autolabel(rects2,ax3)	
		stro = "/home/hash/Desktop/pit_images_captured/" + str(g_local_waypoint_count) + ".png"
		img_saver_client(stro)

	# if(count<5):
	# 	stro = "/home/hash/Desktop/pit_images_captured" + str(count) + ".png"
	# 	img_saver_client(stro)
	# 	count+=1


	# viz_path = "/home/hash/catkin_ws/src/visualization"
	viz_path = str(rospy.get_param("/visualization_path"))
	global_plan_file_path = viz_path + "/data/global_plan.png"
	image = cv2.imread(global_plan_file_path)
	ax4.imshow(image)
	ax4.axis('off')


	'''Plot Video'''
	# pdb.set_trace()

	# while i in range(inF.shape[0]):
	# 	im1 = ax1.imshow(grab_frame(i))
	# 	im1.set_data(grab_frame(i))

# def grab_frame(i):
# 	global inF
# 	frame = inF[i]
# 	return cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)

def autolabel(rects,ax):
    """Attach a text label above each bar in *rects*, displaying its height."""
    for rect in rects:
        height = rect.get_height()
        ax.annotate('{0:.2f}'.format(height),
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 3),  # 3 points vertical offset
                    textcoords="offset points",
                    ha='center', va='bottom',size=15)


if __name__ == '__main__':
	rospy.init_node('visualize', anonymous=True)
	# rospy.Subscriber("/global_waypoint_dist", Range, global_waypoint_dist_callback)
	# rospy.Subscriber("/apnapioneer3at/MultiSense_S21_meta_camera/image",Image,pit_image_callback,queue_size=1)
	# rospy.Subscriber("/robot_at_edge_position",Odometry,local_waypoint_callback)
	rate = rospy.Rate(5)
	rospy.loginfo("In Main \n")
	# ani = animation.FuncAnimation(fig,animate,frames = None,interval = 5)
	# movie = Movie_MP4(r"src/visualization/data/test_vid.mp4")
	# if raw_input("Press enter to play, anything else to exit") == '':
	# 	movie.play()
	while not rospy.is_shutdown():
		plt.show()
		rate.sleep()
