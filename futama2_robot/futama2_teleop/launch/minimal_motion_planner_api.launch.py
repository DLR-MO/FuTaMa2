from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (MoveItConfigsBuilder("robot", package_name="futama2_moveit_config")
                     .robot_description(file_path="config/robot.urdf.xacro")
                     .moveit_cpp(file_path=get_package_share_directory("futama2_teleop") + "/config/motion_planning.yaml")
    ).to_moveit_configs()

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"planning_pipelines": ["ompl", "pilz_industrial_motion_planner", "chomp"]},  # ✅ Ensure this is loaded!
        ],
    )

    minimal_motion_planner_node = Node(
        name="minimal_motion_planner",
        package="futama2_teleop",
        executable="minimal_motion_planner_api.py",
        output="both",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([
        move_group_node,
        minimal_motion_planner_node,
    ])
