# This code is used for converting the data from the rosbags to numpy arrays
# This code is not run with ROS
# The images from the rosbags are 3 channel. These are converted to grayscale before saving into the numpy np_arrays

import rosbag
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

bridge = CvBridge()

# loading the rosbag for conversion
bag = rosbag.Bag('/home/pit_crew/rosbags/bag_10.bag', 'r')
print(bag.get_type_and_topic_info().topics)

# initializing arrays
im1_array = np.zeros((480,640,2218)).astype('float')
im2_array = np.zeros((480,640,2218)).astype('float')

# converting image msg from topic /camera/infra1/image_rect_raw to numpy array
print("Converting infra1 image")
for idx, (topic, msg, t) in enumerate(bag.read_messages(topics=['/camera/infra1/image_rect_raw'])):
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    im1_array[:,:,idx] = gray
    cv2.imshow('IR Example2', gray)
    key = cv2.waitKey(3)
    if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
        break
print("Finished converting infra1 image")


# converting image msg from topic /camera/infra2/image_rect_raw to numpy array
print("Converting infra2 image")
for idx, (topic, msg, t) in enumerate(bag.read_messages(topics=['/camera/infra2/image_rect_raw'])):
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    im2_array[:,:,idx] = gray
    cv2.imshow('IR Example2', gray)
    key = cv2.waitKey(3)
    if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
        break
print("Finished converting infra2 image")

# saving the numpy arrays
print("Saving im1 array")
np.save('im1', im1_array)
print("Saving im2 array")
np.save('im2', im2_array)
print("Completed conversion")

bag.close()
