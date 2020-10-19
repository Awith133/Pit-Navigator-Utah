from exposure_control import *
import pyrealsense2 as rs
import apriltag
import cv2
from arbotix_python.arbotix import ArbotiX

CONV_H = 0.37
CONV_V = 0.3
centered = False
pic_taken = False


def find_tag(img, at_det, centered, pic_taken):
    img_center = np.array([img.shape[0] // 2, img.shape[1] // 2])
    move_vec = np.array([0, 0])

    # cv2.imshow('Gray', img)

    tags = at_det.detect(img)

    if len(tags) == 0:
        print('No tag in image')
        vec_str = str(int(move_vec[0])) + ', ' + str(int(move_vec[1]))
        return move_vec, centered, pic_taken, vec_str

    tag_pos = tags[0].center
    print(tag_pos)

    move_vec = np.array([img_center[1] - tag_pos[0], img_center[0] - tag_pos[1]])
    vec_str = str(int(move_vec[0])) + ', ' + str(int(move_vec[1]))

    vert = ''
    horz = ''

    if tag_pos[1] < img_center[0] - 25:
        vert = 'up'
    elif tag_pos[1] > img_center[0] + 25:
        vert = 'down'

    if tag_pos[0] < img_center[1] - 25:
        horz = 'left'
    elif tag_pos[0] > img_center[1] + 25:
        horz = 'right'

    if vert == '' and horz == '':
        print('Camera is centered on tag')
        centered = True
        return np.array([0, 0]), centered, pic_taken, vec_str
    elif vert == '' or horz == '':
        print('Camera must move ' + vert + horz + ' to center')
        centered = False
        pic_taken = False
    else:
        print('Camera must move ' + vert + ' and ' + horz + ' to center')
        centered = False
        pic_taken = False

    return move_vec, centered, pic_taken, vec_str


def vec_to_cmd(vec):
    conv = np.array([CONV_H, CONV_V])
    return np.array(conv * vec / 2).astype('int')
    

if __name__ == '__main__':
    arby = ArbotiX("/dev/ttyUSB0", 115200)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.resolve(pipeline)  # does not start streaming

    profile = pipeline.start(config)
    camera = profile.get_device().first_color_sensor()
    camera.set_option(rs.option.enable_auto_exposure, 0)
    set_exposure(camera, 200)

    options = apriltag.DetectorOptions(families='tag36h11')
    at_detector = apriltag.Detector(options=options)

    arby.setPosition(1, 512)
    arby.setPosition(2, 512)

    im_num = 1

    vector_str = ''

    while True:
        # cmd = raw_input('Enter \'q\' to quit. Enter any other key to take an image.')
        # if cmd == 'q':
            # break

        tag_img = capture_image(pipeline, im_num)

        gray_tag = cv2.cvtColor(tag_img, cv2.COLOR_RGB2GRAY)
        rect_image = cv2.rectangle(tag_img, (295,215), (345,265), (255,0,0), 2)

        move_vector, centered, pic_taken, vector_str = find_tag(gray_tag, at_detector, centered, pic_taken)

        cv2.putText(rect_image, vector_str, (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA, False)

        if centered and not pic_taken:
            print('Taking picture')
            cv2.imwrite('/home/justin/test_files/test_images/tag' + str(im_num) + '.jpg', rect_image)
            im_num += 1
            pic_taken = True

        cv2.imshow('img', rect_image)
        cv2.waitKey(3)

        print(move_vector)

        turret_cmd = vec_to_cmd(move_vector)

        pan_pos = arby.getPosition(1)
        tilt_pos = arby.getPosition(2)

        arby.setPosition(1, pan_pos + turret_cmd[0])
        arby.setPosition(2, tilt_pos + turret_cmd[1])

    pipeline.stop()
