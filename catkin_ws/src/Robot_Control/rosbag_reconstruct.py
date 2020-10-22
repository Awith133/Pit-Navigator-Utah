#! /usr/bin/env python

import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt
import std_msgs
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
import rospy

# Uncomment plt.ion() when visualizing the depthmap using matplotlib
# plt.ion()

# matrix containing the camera parameters (passed to cv2.reprojectImageTo3D)
Q2 = np.float32([[1,0,0,-321.33],
                [0,1,0,-235.97],
                [0,0,0,380.160],
                [0,0,1/0.05,0]])

# parameters used for block matching in opencv
win_size = 3
min_disp = -1
max_disp = 31 #min_disp * 9
num_disp = max_disp - min_disp # Needs to be divisible by 16

# creating the block matching object
stereo = cv2.StereoSGBM_create(minDisparity= min_disp,
 numDisparities = num_disp,
 blockSize = 5,
 uniquenessRatio = 5,
 speckleWindowSize = 5,
 speckleRange = 5,
 disp12MaxDiff = 1,
 P1 = 8*3*win_size**2,
 P2 =32*3*win_size**2)



if __name__ == '__main__':

    # initialize a rosnode and create point cloud publisher
    rospy.init_node('Rosbag_Reconstruction_Node')
    cloud_pub = rospy.Publisher('cloud_from_images', PointCloud2, queue_size = 10)

    # loading the numpy arrays and converting them to required type
    im1_array = np.load('/home/pit_crew/pit_perception/catkin_ws/src/camera_functions/src/np_arrays/left_array.npy').astype(np.uint8)
    im2_array = np.load('/home/pit_crew/pit_perception/catkin_ws/src/camera_functions/src/np_arrays/right_array.npy').astype(np.uint8)

    # This part is used trim the numpy arrays
    # im1_array = np.load('/home/pit_crew/pit_perception/catkin_ws/src/camera_functions/src/np_arrays/bag8_im1.npy').astype(np.uint8)
    # im2_array = np.load('/home/pit_crew/pit_perception/catkin_ws/src/camera_functions/src/np_arrays/bag8_im2.npy').astype(np.uint8)
    # np.save('bag5_reduced_im1', im1_array[:,:,0:1000])
    # np.save('bag5_reduced_im2', im2_array[:,:,0:1000])

    i = 0
    total_points = 50000
    try:
        while i < im1_array.shape[2]:

            # taking one frame at a time
            im1 = im1_array[:,:,i]
            im2 = im2_array[:,:,i]

            # computing the depthmap
            print ("\nComputing the disparity  map...")
            # disparity_map = stereo.compute(im1, im2)
            #
            # # reconstructing from the depthmap
            # points_3D = cv2.reprojectImageTo3D(disparity_map, Q2)
            # mask_map = disparity_map > disparity_map.min()
            # output_points = points_3D[mask_map]
            #
            # # applying the distance threshold
            # output_points = output_points[output_points[:,2] < 0.06]
            # output_points = output_points[output_points[:,2] > 0]

            # filter based on average y co-ordinate
            # avg_y = sum(output_points[:,1])/(total_points)
            # output_points = output_points[output_points[:,1] < avg_y + 0.002]

            # condition for stopping the rover
            # if output_points.shape[0] < 3000:
            #     im1 = np.dstack((im1,im1,im1*2))


            # visualizing an image from the stereo pair
            cv2.imshow('IR Example2', im1)
            key = cv2.waitKey(30)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

            # generate and publish cloud message
            # header = std_msgs.msg.Header()
            # header.stamp = rospy.Time.now()
            # header.frame_id = 'base_link'
            # cloud = pcl2.create_cloud_xyz32(header, 10*output_points)
            # cloud_pub.publish(cloud)
            #

            # computing average depth of the points
            # avg_depth = sum(output_points[:,2])/(total_points)
            # print("[Average Depth]: ", avg_depth)

            # using matplotlib for visualizing the depthmap
            # plt.imshow(disparity_map, cmap = 'gray')
            # plt.show()
            # plt.pause(0.005)

            i += 1
    finally:
        pass
