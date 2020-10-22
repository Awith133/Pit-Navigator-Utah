#! /usr/bin/env python
from arbotix_python.arbotix import ArbotiX
import numpy as np
from std_msgs.msg import String

CONV_H = 0.37
CONV_V = 0.3
dyna_center = 512
arby = ArbotiX("/dev/ttyUSB0", 115200)


def get_camera_angles(dyna_center, CONV_V, CONV_H):
    pan_rng = np.array([int(dyna_center - (CONV_H * 320)), dyna_center, int(dyna_center + (CONV_H * 320))])
    tilt_rng = np.array([int(dyna_center - (CONV_V * 240)), dyna_center, int(dyna_center + (CONV_V * 240))])
    return pan_rng, tilt_rng



def move_camera(arby, pan_rng, tilt_rng):
    for pan in pan_rng:
        for tilt in tilt_rng:
            arby.setPosition(1, pan)
            arby.setPosition(2, tilt)

def set_camera_pose(arby, pan, tilt):
    arby.setPosition(1, pan)
    arby.setPosition(2, tilt)
    return pan, tilt

if __name__ == '__main__':
    print("yes")
    arby.setPosition(1, dyna_center)
    arby.setPosition(2, dyna_center - 51)
