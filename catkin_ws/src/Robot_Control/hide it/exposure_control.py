#!/usr/bin/env python

# Program to control the camera exposure setting and saving a single image
# The image is saved in /home/pit_crew/test_files/test_images

import pyrealsense2 as rs
import numpy as np
import cv2
# import ipdb

def set_exposure(exposure):
    camera.set_option(rs.option.exposure,exposure)
    return

def get_exposure():
    exposure = camera.get_option(rs.option.exposure)
    return exposure

def capture_image(image_number):
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())
    image = cv2.putText(color_image, 'Exposure Setting: ' + str(camera.get_option(rs.option.exposure)), (50,50), cv2.FONT_HERSHEY_SIMPLEX , 1,(0, 0, 255), 1, cv2.LINE_AA, False)
    cv2.imwrite('/home/pit_crew/test_files/test_images/' + str(image_number) + '.jpg', color_image)
    return

if __name__ == '__main__':
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = config.resolve(pipeline) # does not start streaming

    profile = pipeline.start(config)
    camera = profile.get_device().first_color_sensor()
    camera.set_option(rs.option.enable_auto_exposure, 0)
    exposure = 5000
    for i in range(1,6):
        set_exposure(exposure//(i*2))
        capture_image(i)

    pipeline.stop()
