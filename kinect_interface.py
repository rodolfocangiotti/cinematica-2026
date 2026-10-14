import os
import time
import traceback

import numpy as np

from pykinect2 import PyKinectV2
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

DEBUG = os.environ.get("DEBUG", "false") in ("true", "True", "1")
print(f"DEBUG: {DEBUG}")

if DEBUG:
    EDGE_POINTS_PER_PAD_ID_MAP = {
        1: (
            Point2D(-1.0, 1.0),
            Point2D(0.0, 2.0),
        ),
        2: (
            Point2D(0.0, 1.0),
            Point2D(1.0, 2.0),
        ),
    }
else:
    EDGE_POINTS_PER_PAD_ID_MAP = {
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


OSC_TARGET_IP_ADDRESS = os.environ.get("OSC_TARGET_IP_ADDRESS", "127.0.0.1")
OSC_TARGET_PORT = os.environ.get("OSC_TARGET_PORT", 9000)

CLUSTER_SIZE = "/cluster/size"
CLUSTER_ENERGY = "/cluster/energy"
PAD_1_TRIGGER = "/pad/1/enabled"
PAD_2_TRIGGER = "/pad/2/enabled"
PAD_3_TRIGGER = "/pad/3/enabled"
PAD_4_TRIGGER = "/pad/4/enabled"
PAD_5_TRIGGER = "/pad/5/enabled"
PAD_6_TRIGGER = "/pad/6/enabled"
OSC_ADDRESS_PER_PAD_ID_MAP = {
    1: PAD_1_TRIGGER,
    2: PAD_2_TRIGGER,
    3: PAD_3_TRIGGER,
    4: PAD_4_TRIGGER,
    5: PAD_5_TRIGGER,
    6: PAD_6_TRIGGER,
}


def get_tracked_joint_positions(body_frame: PyKinectV2.KinectBodyFrameData) -> dict:
    joint_positions = {}
    for body in body_frame.bodies:
        if not body.is_tracked:
            # Skip current body...
            continue

        body_joints = joint_positions.setdefault(body.tracking_id, {})
        for type_id in KINECT_JOINT_POINT_TYPES:
            joint = body.joints[type_id]
            assert joint.JointType == type_id, "Joint type consistency check failed!"
            if joint.TrackingState != PyKinectV2.TrackingState_Tracked:
                continue  # Consider only tracked joints...
            # Position is the attribute of a join, which is pointed by the joints attribute...
            # body_joints[type_id] = joint.Position
            # Copy data in a Python object, to avoid any potential change of the C pointer...
            # ...content...
            body_joints[type_id] = Point3D(
                joint.Position.x, joint.Position.y, joint.Position.z
            )
    return joint_positions


def compute_cluster_size(joint_positions: dict) -> float:
    points = tuple(
        p
        for _, joint_positions in joint_positions.items()
        for _, p in joint_positions.items()
    )
    # If no joint positions found...
    if not points:
        return 0.0

    max_x = max(p.x for p in points)
    max_y = max(p.y for p in points)
    max_z = max(p.z for p in points)
    min_x = min(p.x for p in points)
    min_y = min(p.y for p in points)
    min_z = min(p.z for p in points)
    # Returns the volume of the sferoid, whose width, height and depth are calculated...
    # ...from the difference between extreme points on a certain dimension.
    return (max_x - min_x) * (max_y - min_y) * (max_z - min_z) * PI_SIXTH


def compute_cluster_energy(
    joint_positions: dict, joint_position_history: dict
) -> float:
    energy = 0.0
    for tracking_id, positions in joint_positions.items():
        for joint_type, value in positions.items():
            previous_value = joint_position_history.get(tracking_id, {}).get(
                joint_type, value
            )
            # Get euclidean distance between the two points...
            difference = np.sqrt(
                (value.x - previous_value.x) * (value.x - previous_value.x)
                + (value.y - previous_value.y) * (value.y - previous_value.y)
                + (value.z - previous_value.z) * (value.z - previous_value.z)
            )
            energy += difference
    return energy


def compute_body_centroids(joint_positions: dict) -> dict:
    body_centroids = {}
    for tracking_id, joint_positions in joint_positions.items():
        num_positions = len(joint_positions)
        x_average = sum(p.x for _, p in joint_positions.items()) / num_positions
        y_average = sum(p.y for _, p in joint_positions.items()) / num_positions
        z_average = sum(p.z for _, p in joint_positions.items()) / num_positions
        body_centroids[tracking_id] = Point3D(x_average, y_average, z_average)
    return body_centroids


def compute_pad_activations(body_centroids: dict) -> dict:
    pad_activations = {
        1: False,
        2: False,
        3: False,
        4: False,
        5: False,
        6: False,
    }
    for _, centroid in body_centroids.items():
        for pad_id, pad_edges in EDGE_POINTS_PER_PAD_ID_MAP.items():
            top_left, bottom_right = pad_edges
            # While in 2D points, x and y coordinates are respectively on the...
            # ...left/right plane and on close/far plane, the same planes in...
            # ...Kinect positions are represented respectively by x and z coordinates.
            if (
                top_left.x <= centroid.x <= bottom_right.x
                and top_left.y <= centroid.z <= bottom_right.y
            ):
                # A body centroid is inside this pad...
                pad_activations[pad_id] = True
    return pad_activations


def main():
    print("Starting Kinect interface...")

    kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Body)
    udp = SimpleUDPClient(OSC_TARGET_IP_ADDRESS, OSC_TARGET_PORT)

    # Initialize all the values to be shared via OSC...
    cluster_size = 0.0
    cluster_energy = 0.0
    pad_activations = {
        1: False,
        2: False,
        3: False,
        4: False,
        5: False,
        6: False,
    }

    previous_joints = {}
    try:
        while True:
            if kinect.has_new_body_frame():
                body_frame = kinect.get_last_body_frame()

                # TODO: investigate about floor_clip_plane attribute...
                # floor_plane = body_frame.floor_clip_plane
                tracked_joints = get_tracked_joint_positions(body_frame)
                cluster_size = compute_cluster_size(tracked_joints)
                cluster_energy = compute_cluster_energy(tracked_joints, previous_joints)
                body_centroids = compute_body_centroids(tracked_joints)
                pad_activations = compute_pad_activations(body_centroids)

                previous_joints = tracked_joints

            # Send anyway the value, even if it is not updated...
            udp.send_message(CLUSTER_SIZE, cluster_size)
            udp.send_message(CLUSTER_ENERGY, cluster_energy)
            for pad_id, value in pad_activations.items():
                osc_address = OSC_ADDRESS_PER_PAD_ID_MAP[pad_id]
                udp.send_message(osc_address, int(value))

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
