#! /usr/bin/env python

# This program streams the stereo images from the realsense and publishes a reconstructed pointcloud

import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt
import std_msgs
from std_msgs import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
import rospy

# Uncomment plt.ion() while visualizing the depthmap using matplotlib
plt.ion()

# initializing the camera to stream the stereo images
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)
profile = pipeline.start(config)

# matrix containing the camera parameters
Q2 = np.float32([[1,0,0,-321.33],
                [0,1,0,-235.97],
                [0,0,0,380.160],
                [0,0,1/0.05,0]])

# constants used for block matching in opencv
win_size = 5
min_disp = -1
max_disp = 63 #min_disp * 9
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

    # initializing the rosnode and creating a cloud publisher
    rospy.init_node('Stereo_Reconstruction_Node')
    cloud_pub = rospy.Publisher('cloud_from_images', PointCloud2, queue_size = 10)
    alert_pub = rospy.Publisher('alert_message', String, queue_size = 10)

    try:
        while True:
            # poll the stereo pair from the camera
            frames = pipeline.wait_for_frames()
            nir_lf_frame = frames.get_infrared_frame(1)
            nir_rg_frame = frames.get_infrared_frame(2)

            if not nir_lf_frame or not nir_rg_frame:
                continue

            # convert the pair to numpy array
            im1 = np.asanyarray(nir_lf_frame.get_data())
            im2 = np.asanyarray(nir_rg_frame.get_data())

            #Compute disparity map
            print ("\nComputing the disparity  map...")
            disparity_map = stereo.compute(im1, im2)

            # reconstructing from the depthmap
            points_3D = cv2.reprojectImageTo3D(disparity_map, Q2)
            mask_map = disparity_map > disparity_map.min()
            output_points = points_3D[mask_map]

            # applying a distance threshold
            output_points = output_points[output_points[:,2] < 0.09]

            if output_points.shape[0] < 3000:
                im1 = np.dstack((im1,im1,im1*2))
                alert_pub.publish("Close to the edge")

            # visualizing one image from the stereo pair
            # cv2.imshow('IR Example2', disparity_map)
            # key = cv2.waitKey(1)
            # if key & 0xFF == ord('q') or key == 27:
            #     cv2.destroyAllWindows()
            #     break

            # generate and publish cloud message
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'camera_link'
            cloud = pcl2.create_cloud_xyz32(header, 10*output_points)
            cloud_pub.publish(cloud)

            # visualizing the depthmap using matplotlib
            plt.imshow(disparity_map, cmap = 'gray')
            plt.show()
            plt.pause(0.005)
    finally:
        pipeline.stop()
