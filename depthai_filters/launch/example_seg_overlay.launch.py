import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from depthai_ros_driver.launch_utils import (
    camera_component_names, camera_launch_arguments, declare_camera_arguments,
)


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file")
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    name = LaunchConfiguration('name').perform(context)
    container, topic_root, namespace = camera_component_names(context)
    
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, 'launch', 'example_segmentation.launch.py')),
            launch_arguments={**camera_launch_arguments(), "name": name,
                              "params_file": params_file}.items()),

        LoadComposableNodes(
            target_container=container,
            composable_node_descriptions=[
                    ComposableNode(
                        package="depthai_filters",
                        namespace=namespace,
                        plugin="depthai_filters::SegmentationOverlay",
                        remappings=[('rgb/preview/image_raw', topic_root+'/nn/passthrough/image_raw'),
                                    ('nn/image_raw', topic_root+'/nn/image_raw')]
                    ),
            ],
        ),

    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(depthai_prefix, 'config', 'segmentation.yaml')),
    ] + declare_camera_arguments()

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
