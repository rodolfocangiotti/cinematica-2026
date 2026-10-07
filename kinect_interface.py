import os
import time

import numpy as np

from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
from pythonosc.udp_client import SimpleUDPClient

OSC_TARGET_ADDRESS = os.environ.get("OSC_TARGET_ADDRESS", "127.0.0.1")
OSC_TARGET_PORT = os.environ.get("OSC_TARGET_PORT", 9000)

CLUSTER_SIZE = "/cluster/size"
CLUSTER_ENERGY = "/cluster/energy"
PAD_1_TRIGGER = "/pad/1/enabled"
PAD_2_TRIGGER = "/pad/2/enabled"
PAD_3_TRIGGER = "/pad/3/enabled"
PAD_4_TRIGGER = "/pad/4/enabled"
PAD_5_TRIGGER = "/pad/5/enabled"
PAD_6_TRIGGER = "/pad/6/enabled"


def main():
    print("Starting Kinect interface...")

    kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Body)
    udp = SimpleUDPClient(OSC_TARGET_ADDRESS, OSC_TARGET_PORT)

    try:
        while True:
            # # Wait for new body frame...
            # while not kinect.has_new_body_frame():
            #     time.sleep(0.01)

            # body_frame = kinect.get_last_body_frame()
            # for body in body_frame.bodies:
            #     print(body)
            # floor_plane = body_frame.floor_clip_plane

            cluster_size = 10.0
            cluster_energy = 100.0
            is_pad_1_triggered = False
            is_pad_2_triggered = False
            is_pad_3_triggered = False
            is_pad_4_triggered = False
            is_pad_5_triggered = False
            is_pad_6_triggered = False

            udp.send_message(CLUSTER_SIZE, cluster_size)
            udp.send_message(CLUSTER_ENERGY, cluster_energy)
            udp.send_message(PAD_1_TRIGGER, int(is_pad_1_triggered))
            udp.send_message(PAD_2_TRIGGER, int(is_pad_2_triggered))
            udp.send_message(PAD_3_TRIGGER, int(is_pad_3_triggered))
            udp.send_message(PAD_4_TRIGGER, int(is_pad_4_triggered))
            udp.send_message(PAD_5_TRIGGER, int(is_pad_5_triggered))
            udp.send_message(PAD_6_TRIGGER, int(is_pad_6_triggered))

            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

    kinect.close()
    udp.close()


if __name__ == "__main__":
    main()
