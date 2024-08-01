import numpy as np

from pathlib import Path
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header


def get_poses() -> list[tuple]:
    file_location = Path('/home/futama2/futama2_ws/src/futama2_robot/futama2_teleop/config/poses.txt')

    if not file_location.exists():
        raise FileNotFoundError(f'File {file_location} not found')

    with open(file_location, 'r', encoding='utf-8') as f:
        stream = f.read().strip().strip('[]')
        stream = stream.split('), (')
        poses = [tuple(map(float, i.strip('()').split(', '))) for i in stream]
        return poses


CAMERA_POSES = [
    (R.from_euler('XYZ', [0,   0, 0], degrees=True),            [0, 0, 0]),  # Front
    (R.from_euler('XYZ', [180, 0, 0], degrees=True),            [0, 0, 0]),   # 180°
    (R.from_euler('ZYX', [0,        90,  0], degrees=True),     [-0.02, -0.04]),   # Top
    (R.from_euler('ZYX', [0,       270,  180], degrees=True),   [-0.02, +0.04]),   # 180°
    (R.from_euler('ZYX', [0,       -90, 0], degrees=True),      [-0.02, +0.04]),  # Bottom
    (R.from_euler('ZYX', [0,      90,   180], degrees=True),    [-0.02, -0.04]),  # 180°
]

def tuple_to_pose(pose: tuple, current_time: float) -> PoseStamped:
    pose_stamped = PoseStamped()
    pose_stamped.header = Header(frame_id='base_link', stamp=current_time)
    pose_stamped.pose.position.x = pose[0]
    pose_stamped.pose.position.y = pose[1]
    pose_stamped.pose.position.z = pose[2]
    pose_stamped.pose.orientation.x = pose[3]
    pose_stamped.pose.orientation.y = pose[4]
    pose_stamped.pose.orientation.z = pose[5]
    pose_stamped.pose.orientation.w = pose[6]
    return pose_stamped

def alternative_poses(pose: PoseStamped, shifts: list[tuple]):
    pos, quat = pose.pose.position, pose.pose.orientation

    quat = R.from_quat(
            np.array([quat.x, quat.y,
                      quat.z, quat.w]))
    forward = R.as_matrix(quat).T[2]
    side = R.as_matrix(quat).T[0]

    for orient_shift, pos_shift in shifts:
        new_quat = quat * orient_shift
        new_quat = new_quat.as_quat()

        new_pos = np.array([pos.x, pos.y, pos.z]) + \
                  forward * pos_shift[1] + \
                  side * pos_shift[0]
        pose_tuple = new_pos.tolist() + new_quat.tolist()

        new_pose = tuple_to_pose(tuple(pose_tuple), pose.header.stamp)

        yield new_pose

    return new_pose
