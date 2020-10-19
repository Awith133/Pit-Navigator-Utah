from arbotix_python.arbotix import ArbotiX
import numpy as np

CONV_H = 0.37
CONV_V = 0.3

dyna_center = 512

pan_rng = np.array([int(dyna_center - (CONV_H * 320)), dyna_center, int(dyna_center + (CONV_H * 320))])
tilt_rng = np.array([int(dyna_center - (CONV_V * 240)), dyna_center, int(dyna_center + (CONV_V * 240))])

arby = ArbotiX("/dev/ttyUSB0", 115200)

arby.setPosition(1, dyna_center)
arby.setPosition(2, dyna_center)

raw_input('Press Enter to start.')

while True:
    for pan in pan_rng:
        for tilt in tilt_rng:
            arby.setPosition(1, pan)
            arby.setPosition(2, tilt)
            raw_input('Press Enter to move to next position.')

            print('Going to next position.')
    print('All positions reached. Restarting.')

print('Done! How did you get here?')
