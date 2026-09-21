import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from depthai_ros_driver.launch_utils import (
    camera_launch_arguments,
    declare_camera_arguments,
)


def launch_setup(context, *args, **kwargs):
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    params_file = LaunchConfiguration("params_file")
    name = LaunchConfiguration("name").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    camera_root = "/" + "/".join(part for part in (namespace, name) if part)
    type = LaunchConfiguration("type").perform(context)
    size = LaunchConfiguration("size").perform(context)
    square = LaunchConfiguration("square").perform(context)
    mono_args = [
        "--size",
        size,
        "--square",
        square,
        "--camera_name",
        f"{camera_root.strip('/')}/rgb",
        "--no-service-check",
    ]
    stereo_args = [
        "--size",
        size,
        "--square",
        square,
        "--camera_name",
        "/rgb",
        "--approximate",
        "0.1",
        "--camera_name",
        camera_root.strip("/"),
        "left_camera",
        "left",
        "right_camera",
        "right",
        "--no-service-check",
    ]
    mono_remappings = [("/image", f"{camera_root}/rgb/image_raw")]
    stereo_remappings = [
        ("/left", f"{camera_root}/left/image_raw"),
        ("/right", f"{camera_root}/right/image_raw"),
    ]
    args = []
    remappings = []
    if type == "mono":
        args = mono_args
        remappings = mono_remappings
    else:
        args = stereo_args
        remappings = stereo_remappings

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, "launch", "driver.launch.py")
            ),
            launch_arguments={
                "name": name,
                "camera_model": LaunchConfiguration("camera_model"),
                "params_file": params_file,
                **camera_launch_arguments(),
            }.items(),
        ),
        Node(
            name="calibrator",
            namespace="",
            package="camera_calibration",
            executable="cameracalibrator",
            arguments=args,
            remappings=remappings,
        ),
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("camera_model", default_value="OAK-D"),
        DeclareLaunchArgument("size", default_value="8x6"),
        DeclareLaunchArgument("square", default_value="0.108"),
        DeclareLaunchArgument("type", default_value="mono"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(depthai_prefix, "config", "calibration.yaml"),
        ),
    ] + declare_camera_arguments()

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
