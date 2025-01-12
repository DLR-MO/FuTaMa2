import numpy as np
from pathlib import Path
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header
from ament_index_python.packages import get_package_share_directory


def get_poses(package_name: str = 'futama2_teleop', file_name: str = 'poses.txt') -> list[tuple]:
    """
    Retrieves a list of poses from a configuration file.

    Args:
        package_name (str): Name of the ROS package where the poses file is located.
        file_name (str): Name of the file containing pose data.

    Returns:
        list[tuple]: List of poses as tuples of (x, y, z, qx, qy, qz, qw).

    Raises:
        FileNotFoundError: If the file or package is not found.
        ValueError: If the file contains invalid pose data.
    """
    try:
        package_share_directory = get_package_share_directory(package_name)
    except Exception as e:
        raise FileNotFoundError(
            f"Could not find package '{package_name}'. Ensure it is built and sourced."
        ) from e

    file_location = Path(package_share_directory) / 'config' / file_name

    if not file_location.exists():
        raise FileNotFoundError(f"File {file_location} not found. Ensure it exists in the package's config directory.")

    try:
        with open(file_location, 'r', encoding='utf-8') as f:
            stream = f.read().strip().strip('[]')
            stream = stream.split('), (')
            poses = [tuple(map(float, i.strip('()').split(', '))) for i in stream]
    except Exception as e:
        raise ValueError(f"Failed to parse pose data from {file_location}. Ensure the file format is correct.") from e

    return poses


CAMERA_POSES = [
    (R.from_euler('XYZ', [0,   0, 0], degrees=True), [0, 0, 0]),  # Front
    (R.from_euler('XYZ', [180, 0, 0], degrees=True), [0, 0, 0]),  # 180°
    (R.from_euler('ZYX', [0,  -90, 0], degrees=True), [-0.02, +0.04]),  # Bottom
    (R.from_euler('ZYX', [0,   90, 180], degrees=True), [-0.02, -0.04]),  # 180°
]


def tuple_to_pose(pose: tuple, current_time) -> PoseStamped:
    """
    Converts a tuple representing a pose into a PoseStamped message.

    Args:
        pose (tuple): The pose as (x, y, z, qx, qy, qz, qw).
        current_time: ROS2 clock timestamp.

    Returns:
        PoseStamped: ROS2 PoseStamped message.
    """
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


def alternative_poses(pose: PoseStamped, shifts: list[tuple]) -> list[PoseStamped]:
    """
    Generates alternative poses by applying orientation and position shifts.

    Args:
        pose (PoseStamped): The original pose.
        shifts (list[tuple]): List of tuples containing orientation shifts (Rotation) and position shifts (x, y).

    Yields:
        PoseStamped: Alternative poses generated from the input pose.
    """
    pos = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
    quat = R.from_quat([pose.pose.orientation.x, pose.pose.orientation.y,
                        pose.pose.orientation.z, pose.pose.orientation.w])

    forward = quat.as_matrix()[:, 2]  # Forward vector
    side = quat.as_matrix()[:, 0]    # Side vector

    for orient_shift, pos_shift in shifts:
        new_quat = (quat * orient_shift).as_quat()
        new_pos = pos + forward * pos_shift[1] + side * pos_shift[0]

        new_pose = PoseStamped()
        new_pose.header = pose.header
        new_pose.pose.position.x = new_pos[0]
        new_pose.pose.position.y = new_pos[1]
        new_pose.pose.position.z = new_pos[2]
        new_pose.pose.orientation.x = new_quat[0]
        new_pose.pose.orientation.y = new_quat[1]
        new_pose.pose.orientation.z = new_quat[2]
        new_pose.pose.orientation.w = new_quat[3]

        yield new_pose
