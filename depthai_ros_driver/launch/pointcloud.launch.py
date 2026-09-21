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
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from depthai_ros_driver.launch_utils import (
    camera_launch_arguments,
    declare_camera_arguments,
)


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file")
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    name = LaunchConfiguration("name").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    camera_root = "/" + "/".join(part for part in (namespace, name) if part)
    container_name = "/" + "/".join(
        part for part in (namespace, f"{name}_container") if part
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, "launch", "driver.launch.py")
            ),
            launch_arguments={
                "name": name,
                "camera_model": LaunchConfiguration("camera_model"),
                "params_file": params_file,
                "parent_frame": LaunchConfiguration("parent_frame"),
                "cam_pos_x": LaunchConfiguration("cam_pos_x"),
                "cam_pos_y": LaunchConfiguration("cam_pos_y"),
                "cam_pos_z": LaunchConfiguration("cam_pos_z"),
                "cam_roll": LaunchConfiguration("cam_roll"),
                "cam_pitch": LaunchConfiguration("cam_pitch"),
                "cam_yaw": LaunchConfiguration("cam_yaw"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "rviz_config": LaunchConfiguration("rviz_config"),
                "rviz_fixed_frame": LaunchConfiguration("rviz_fixed_frame"),
                **camera_launch_arguments(),
            }.items(),
        ),
        LoadComposableNodes(
            target_container=container_name,
            composable_node_descriptions=[
                ComposableNode(
                    package="depth_image_proc",
                    plugin="depth_image_proc::PointCloudXyzNode",
                    name="point_cloud_xyz",
                    namespace=namespace,
                    remappings=[
                        ("image_rect", f"{camera_root}/stereo/image_raw"),
                        ("camera_info", f"{camera_root}/stereo/camera_info"),
                        ("points", f"{camera_root}/rgbd/points"),
                    ],
                ),
            ],
        ),
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("camera_model", default_value="OAK-D"),
        DeclareLaunchArgument("parent_frame", default_value="oak_parent_frame"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(depthai_prefix, "config", "pcl.yaml"),
        ),
        DeclareLaunchArgument("use_rviz", default_value="False"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(depthai_prefix, "config", "rviz", "rgbd.rviz"),
        ),
        DeclareLaunchArgument("rviz_fixed_frame", default_value=""),
    ] + declare_camera_arguments()

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
