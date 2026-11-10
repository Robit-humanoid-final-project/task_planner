from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    yaml_file_path = PathJoinSubstitution(
        [FindPackageShare("task_planner"), "config", "tp.yaml"]
    )

    task_planner_node = Node(
        package="task_planner",
        executable="task_planner",
        output="screen",
        emulate_tty=True,
        parameters=[yaml_file_path],
    )

    return LaunchDescription([task_planner_node])
