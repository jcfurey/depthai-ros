import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from depthai_ros_driver.launch_utils import (
    camera_launch_arguments,
    declare_camera_arguments,
)


def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration("name").perform(context)
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    rosbag_path = LaunchConfiguration("rosbag_path").perform(context)
    params_file = LaunchConfiguration("params_file")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, "launch", "driver.launch.py")
            ),
            launch_arguments={
                "name": name,
                "camera_model": LaunchConfiguration("camera_model"),
                "params_file": params_file,
                "use_rviz": LaunchConfiguration("use_rviz"),
                "rviz_config": LaunchConfiguration("rviz_config"),
                "rviz_fixed_frame": LaunchConfiguration("rviz_fixed_frame"),
                **camera_launch_arguments(),
            }.items(),
        ),
        ExecuteProcess(cmd=["ros2", "bag", "play", "-l", rosbag_path], output="screen"),
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("camera_model", default_value="OAK-D"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                depthai_prefix, "config", "stereo_from_rosbag.yaml"
            ),
        ),
        DeclareLaunchArgument("use_rviz", default_value="True"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(depthai_prefix, "config", "rviz", "rgbd.rviz"),
        ),
        DeclareLaunchArgument("rviz_fixed_frame", default_value=""),
        DeclareLaunchArgument(
            "rosbag_path",
            description="Path to a rosbag containing the configured stereo input topics.",
        ),
    ] + declare_camera_arguments()

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
