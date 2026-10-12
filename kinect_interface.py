import ctypes
import os
import time
import traceback
from typing import Optional

import numpy as np

from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
from pythonosc.udp_client import SimpleUDPClient

from utilities import Point2D, Point3D

PI_SIXTH = np.pi / 6.0

KINECT_JOINT_POINT_TYPES = (
    PyKinectV2.JointType_SpineBase,
    PyKinectV2.JointType_SpineMid,
    PyKinectV2.JointType_Neck,
    PyKinectV2.JointType_Head,
    PyKinectV2.JointType_ShoulderLeft,
    PyKinectV2.JointType_ElbowLeft,
    PyKinectV2.JointType_WristLeft,
    PyKinectV2.JointType_HandLeft,
    PyKinectV2.JointType_ShoulderRight,
    PyKinectV2.JointType_ElbowRight,
    PyKinectV2.JointType_WristRight,
    PyKinectV2.JointType_HandRight,
    PyKinectV2.JointType_HipLeft,
    PyKinectV2.JointType_KneeLeft,
    PyKinectV2.JointType_AnkleLeft,
    PyKinectV2.JointType_FootLeft,
    PyKinectV2.JointType_HipRight,
    PyKinectV2.JointType_KneeRight,
    PyKinectV2.JointType_AnkleRight,
    PyKinectV2.JointType_FootRight,
    PyKinectV2.JointType_SpineShoulder,
    PyKinectV2.JointType_HandTipLeft,
    PyKinectV2.JointType_ThumbLeft,
    PyKinectV2.JointType_HandTipRight,
    PyKinectV2.JointType_ThumbRight,
)

PAD_EDGE_POINTS = {
    1: (
        Point2D(-1.5, 2.0),
        Point2D(-0.5, 3.0),
    ),
    2: (
        Point2D(-0.5, 2.0),
        Point2D(0.5, 3.0),
    ),
    3: (
        Point2D(0.5, 2.0),
        Point2D(1.5, 3.0),
    ),
    4: (
        Point2D(-1.5, 4.0),
        Point2D(-0.5, 5.0),
    ),
    5: (
        Point2D(-0.5, 3.0),
        Point2D(0.5, 4.0),
    ),
    6: (
        Point2D(0.5, 3.0),
        Point2D(1.5, 4.0),
    ),
}


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


def get_cluster_size(body_frame: PyKinectV2.KinectBodyFrameData) -> float:
    valid_positions = []
    for body in body_frame.bodies:
        if not body.is_tracked:
            # Skip current body...
            continue

        for type_id in KINECT_JOINT_POINT_TYPES:
            joint = body.joints[type_id]
            assert joint.JointType == type_id, "Consistency check failed."
            if joint.TrackingState != PyKinectV2.TrackingState_Tracked:
                continue  # Consider only tracked joints...
            valid_positions.append(joint.Position)

    if valid_positions:
        max_x = max(p.x for p in valid_positions)
        max_y = max(p.y for p in valid_positions)
        max_z = max(p.z for p in valid_positions)
        min_x = min(p.x for p in valid_positions)
        min_y = min(p.y for p in valid_positions)
        min_z = min(p.z for p in valid_positions)
        # Returns the volume of the sferoid, whose width, height and depth are calcultated...
        # ...from the difference between extreme points on a certain dimension.
        return (max_x - min_x) * (max_y - min_y) * (max_z - min_z) * PI_SIXTH
    return 0.0


def get_cluster_energy(
    body_frame: PyKinectV2.KinectBodyFrameData, cluster_history: dict
) -> float:
    valid_positions = {}
    for body in body_frame.bodies:
        if not body.is_tracked:
            # Skip current body...
            continue

        for type_id in KINECT_JOINT_POINT_TYPES:
            joint = body.joints[type_id]
            assert joint.JointType == type_id, "Consistency check failed."
            if joint.TrackingState != PyKinectV2.TrackingState_Tracked:
                continue  # Consider only tracked joints...
            position_id = (body.tracking_id, type_id)
            valid_positions[position_id] = joint.Position

    energy = 0.0
    for position_id, value in valid_positions.items():
        previous_value = cluster_history.get(position_id, value)
        difference = np.sqrt((value.x - previous_value.x) * (value.x - previous_value.x) + \
                             (value.y - previous_value.y) * (value.y - previous_value.y) + \
                             (value.z - previous_value.z) * (value.z - previous_value.z))
        energy += difference
        cluster_history[position_id] = value

    # Clear history for missing positions...
    for k in tuple(cluster_history.keys()):
        if k not in valid_positions:
            cluster_history.pop(k)
    return energy


def get_body_centroid(joints: ctypes.POINTER) -> Point3D:
    valid_positions = []
    for type_id in KINECT_JOINT_POINT_TYPES:
        joint = joints[type_id]

        assert joint.JointType == type_id, "Consistency check failed."
        if joint.TrackingState != PyKinectV2.TrackingState_Tracked:
            continue  # Consider only tracked joints...
        valid_positions.append(joint.Position)

    num_positions = len(valid_positions)
    x_average = sum(p.x for p in valid_positions) / num_positions
    y_average = sum(p.y for p in valid_positions) / num_positions
    z_average = sum(p.z for p in valid_positions) / num_positions
    return Point3D(x_average, y_average, z_average)


def find_pad(centroid: Point3D) -> Optional[int]:
    for pad_id, pad_edges in PAD_EDGE_POINTS.items():
        top_left, bottom_right = pad_edges
        if (
            top_left.x <= centroid.x <= bottom_right.x
            and top_left.y <= centroid.y <= bottom_right.y
        ):
            # Pad found...
            return pad_id
    # Pad not found, centroid is not inside any pad...
    return None


def main():
    print("Starting Kinect interface...")

    kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Body)
    udp = SimpleUDPClient(OSC_TARGET_ADDRESS, OSC_TARGET_PORT)

    # Initialize loop...
    cluster_size = 0.0
    cluster_energy = 0.0
    cluster_history = {}
    is_pad_triggered = {
        1: False,
        2: False,
        3: False,
        4: False,
        5: False,
        6: False,
    }

    try:
        while True:
            if kinect.has_new_body_frame():
                body_frame = kinect.get_last_body_frame()
                # Reset all pads...
                is_pad_triggered = {pad_id: False for pad_id in is_pad_triggered}

                for body in body_frame.bodies:
                    if not body.is_tracked:
                        # Skip current body...
                        continue
                    body_centroid = get_body_centroid(body.joints)
                    body_pad = find_pad(body_centroid)
                    if body_pad is not None:
                        is_pad_triggered[body_pad] = True

                cluster_size = get_cluster_size(body_frame)
                cluster_energy = get_cluster_energy(body_frame, cluster_history)

                # floor_plane = body_frame.floor_clip_plane

            # Send anyway the value, even if it is not updated...
            udp.send_message(CLUSTER_SIZE, cluster_size)
            udp.send_message(CLUSTER_ENERGY, cluster_energy)
            for pad_id, value in is_pad_triggered.items():
                udp.send_message(eval(f"PAD_{pad_id}_TRIGGER"), int(value))

            print(f"c. size: {cluster_size} - c. energy: {cluster_energy} - pads: {tuple(is_pad_triggered.values())}")

            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    except Exception:
        print(traceback.format_exc())

    print("Exiting...")
    kinect.close()
    udp.close()


if __name__ == "__main__":
    main()
