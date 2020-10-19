from exposure_control import *
from tag_spotter import vec_to_cmd
import imutils
import cv2
from arbotix_python.arbotix import ArbotiX

CONV_H = 0.37
CONV_V = 0.3
CAM_V_FOV = 45
CAM_H_FOV = 75
PT_SPEED = 8
centered = False
pic_taken = False

# TODO: Implement this fully
def get_panorama_reference():
    return 512, 512

# TODO: Implement this fully
def get_pan_tilt_ranges():
    pan_rng = [-75, 0, 75]
    tilt_rng = [0, -30, -60]
    return pan_rng, tilt_rng

def pan_tilt_to_pos(pan_tilt, pan=512, tilt=512):
    pan_pos = pan_tilt.getPosition(1)
    tilt_pos = pan_tilt.getPosition(2)

    while (abs(pan_pos - pan) > 5 or abs(tilt_pos - tilt) > 5):
        pan_move = min(PT_SPEED, abs(pan - pan_pos)) * (1 if pan >= pan_pos else -1)
        tilt_move = min(PT_SPEED, abs(tilt - tilt_pos)) * (1 if tilt >= tilt_pos else -1)

        pan_tilt.setPosition(1, pan_pos + pan_move)
        pan_tilt.setPosition(2, tilt_pos + tilt_move)

        pan_pos = pan_tilt.getPosition(1)
        tilt_pos = pan_tilt.getPosition(2)

    return

if __name__ == '__main__':
    arby = ArbotiX("/dev/ttyUSB0", 115200)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.resolve(pipeline)  # does not start streaming

    profile = pipeline.start(config)
    camera = profile.get_device().first_color_sensor()
    camera.set_option(rs.option.enable_auto_exposure, 0)
    set_exposure(camera, 25)

    # Initialize OpenCV image stitcher object
    stitcher = cv2.createStitcher() if imutils.is_cv3() else cv2.Stitcher_create()

    # pan_tilt_to_pos(arby)

    im_num = 1
    tilt_level = 1

    # Calculate reference point at top level of panorama
    (pan_ref, tilt_ref) = get_panorama_reference()

    # Calculate pan and tilt angles
    (pan_rng, tilt_rng) = get_pan_tilt_ranges()

    for tilt in tilt_rng:
        tilt_level_images = []
        for pan in pan_rng:
            # Move to next position
            pan_tilt_to_pos(arby, pan_ref + pan, tilt_ref + tilt)

            img_section = capture_image(pipeline, im_num, filepath='/home/justin/test_files/test_images/c_')
            print("Image captured")
            tilt_level_images.append(img_section)
            im_num += 1

        # Stitch together all images at a given pan level
        (status, stitched) = stitcher.stitch(tilt_level_images)

        if status == 0:
            # Write the output stitched image to disk
            cv2.imwrite('/home/justin/test_files/test_images/pano' + str(tilt_level) + '.jpg', stitched)
        else:
            print("[INFO] Image stitching failed ({})".format(status))
            break

        tilt_level += 1

    pipeline.stop()
