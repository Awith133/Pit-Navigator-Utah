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
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Float64
import cv2

#=========================================================================================================================================

fig, (ax1,ax2) = plt.subplots(1,2)
counter = 0
g_rock_dist = []
g_rock_dist_count = 0
g_rock_dist_counter = 0
ROCK_DIST_THRESHOLD = 0.5
ROCK_DIST_PUBLISH_FREQ = 25
ROCK_TEST_THRESHOLD = 80

g_odom_error_list = []
g_odom_error_count = 0
ODOM_ERROR_MEAN_THRESHOLD = 0.005
ODOM_TEST_THRESHOLD = 95

#==========================================================================================================================================

def odom_callback(msg):			#Use the msg.flag to see if you need to keep publishing old message or use the new one
	global g_odom_error_list
	g_odom_error_list.append(msg.data)

def rock_dist_callback(msg):			#Use the msg.flag to see if you need to keep publishing old message or use the new one
	global g_rock_dist_counter
	g_rock_dist_counter+=1
	if(g_rock_dist_counter%ROCK_DIST_PUBLISH_FREQ==0):
		g_rock_dist.append(msg.data)

def set_font_size(ax):
	for label in (ax.get_yticklabels()):
		label.set_fontname('Arial')
		label.set_fontsize(28)

def animate(frames):
	global ax1
	global ax2
	global g_rock_dist_count
	global g_odom_error_count
	global ROCK_TEST_THRESHOLD
	global ODOM_TEST_THRESHOLD
	rospy.loginfo("In Animate \n")

	if(g_rock_dist_count<len(g_rock_dist)):
		g_rock_dist_count +=1
		plot_len = min(g_rock_dist_count,len(g_rock_dist))
		ax2.clear()
		x = np.arange(plot_len)
		fail_count = len([i for i in g_rock_dist[0:plot_len] if i < ROCK_DIST_THRESHOLD]) 
		ax2.plot(x, g_rock_dist[0:plot_len])
		ax2.set_ylabel('Distance of rover from nearest rock (m)',fontsize=28)
		ax2.set_xlabel('Time index')
		ax2.set_title('Obstacles Avoidance Test',fontsize=28)
		set_font_size(ax2)

		# place a text box in upper left in axes coords
		metric = (g_rock_dist_count-fail_count)*100/g_rock_dist_count
		textstr = '% of times rover successfully avoids \n obstacles  upto threshold distance limit: {:.2f}'.format(metric)
		ax2.hlines(y=ROCK_DIST_THRESHOLD, xmin=x[0]-1, xmax=len(x), linestyle='--', color='r')
		ax2.text(x[0]-1, ROCK_DIST_THRESHOLD*1.02, 'Threshold value',fontsize=20)
		if(metric>ROCK_TEST_THRESHOLD):
			props = dict(boxstyle='round', facecolor='lightgreen', alpha=0.5)
		else:
			props = dict(boxstyle='round', facecolor='darksalmon', alpha=0.5)
		ax2.text(0.05, 0.95, textstr, transform=ax2.transAxes, fontsize=22,verticalalignment='top', bbox=props, weight='bold')
		ax2.set_xticks(x)
		ax2.set_xticklabels(x)
		ax2.get_xaxis().set_ticks([])
		ax2.legend()

	if(g_odom_error_count<len(g_odom_error_list)):
		g_odom_error_count +=1
		plot_len = min(g_odom_error_count,len(g_odom_error_list))
		ax1.clear()
		x = np.arange(plot_len)
		fail_count = len([i for i in g_odom_error_list[0:plot_len] if i > ODOM_ERROR_MEAN_THRESHOLD]) 
		ax1.plot(x, g_odom_error_list[0:plot_len])
		ax1.set_ylabel('Odometry drift relative to ground truth',fontsize=28)
		ax1.set_xlabel('Time index')
		ax1.set_title('Odometry Validation Test',fontsize=28)
		set_font_size(ax1)

		# place a text box in upper left in axes coords
		metric = (g_odom_error_count-fail_count)*100/g_odom_error_count
		textstr = '% of waypoints within threshold: {:.2f}'.format(metric)
		ax1.hlines(y=ODOM_ERROR_MEAN_THRESHOLD, xmin=x[0]-1, xmax=len(x), linestyle='--', color='r')
		ax1.text(x[0]-1, ODOM_ERROR_MEAN_THRESHOLD*1.02, 'Threshold value',fontsize=20)
		if(metric>ODOM_TEST_THRESHOLD):
			props = dict(boxstyle='round', facecolor='lightgreen', alpha=0.5)
		else:
			props = dict(boxstyle='round', facecolor='darksalmon', alpha=0.5)
		ax1.text(0.05, 0.95, textstr, transform=ax1.transAxes, fontsize=22,verticalalignment='top', bbox=props, weight='bold')
		ax1.set_xticks(x)
		ax1.set_xticklabels(x)
		ax1.get_xaxis().set_ticks([])
		ax1.legend()


if __name__ == '__main__':
	rospy.init_node('visualize_test1', anonymous=True)
	rospy.Subscriber("/odom_error",Float64,odom_callback)
	rospy.Subscriber("/min_rock_dist",Float32,rock_dist_callback)
	rate = rospy.Rate(10)
	rospy.loginfo("In Main \n")
	ani = animation.FuncAnimation(fig,animate,frames = None,interval = 10)
	count = 0
	while not rospy.is_shutdown():
		if(count%20 == 0):
			plt.show()
			count=0
		rate.sleep()
		count = count+1
