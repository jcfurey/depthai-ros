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
    name = LaunchConfiguration("name").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    camera_root = "/" + "/".join(part for part in (namespace, name) if part)
    parent_frame = LaunchConfiguration("parent_frame").perform(context)
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    params_file = LaunchConfiguration("params_file")
    parameters = [
        {
            "frame_id": parent_frame,
            "subscribe_rgb": True,
            "subscribe_depth": True,
            "subscribe_odom_info": False,
            "approx_sync": True,
            # RTAB-Map's parameters should be strings:
            'Mem/NotLinkedNodesKept':'false',
            "Rtabmap/DetectionRate": "1.0",
        }
    ]

    remappings = [
        ("rgb/image", f"{camera_root}/rgb/image_raw"),
        ("rgb/camera_info", f"{camera_root}/rgb/camera_info"),
        ("depth/image", f"{camera_root}/stereo/image_raw"),
        ("odom", f"{camera_root}/vio/odometry"),
    ]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, "launch", "driver.launch.py")
            ),
            launch_arguments={
                "name": name,
                "camera_model": LaunchConfiguration("camera_model"),
                "params_file": params_file,
                "parent_frame": parent_frame,
                **camera_launch_arguments(),
            }.items(),
        ),
        Node(
                    package="rtabmap_slam",
                    executable="rtabmap",
                    name="rtabmap",
                    parameters=parameters,
                    remappings=remappings,
        ),
        Node(
            package="rtabmap_viz",
            executable="rtabmap_viz",
            output="screen",
            parameters=parameters,
            remappings=remappings,
        ),
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("camera_model", default_value="OAK-D"),
        DeclareLaunchArgument("parent_frame", default_value="oak_parent_frame"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(depthai_prefix, "config", "rtabmap.yaml"),
        ),
    ] + declare_camera_arguments()

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
