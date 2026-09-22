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
                os.path.join(depthai_prefix, 'launch', 'driver.launch.py')),
            launch_arguments={**camera_launch_arguments(), "name": name,
                              "params_file": params_file}.items()),

        LoadComposableNodes(
            target_container=container,
            composable_node_descriptions=[
                    ComposableNode(
                        package="depthai_filters",
                        namespace=namespace,
                        name="feature_overlay_rgb",
                        plugin="depthai_filters::FeatureTrackerOverlay",
                        remappings=[('rgb/preview/image_raw', topic_root+'/rgb/image_raw'),
                                    ('feature_tracker/tracked_features', topic_root+'/rgb_feature_tracker/tracked_features'),
                                    ('overlay', 'overlay_rgb')]
                    )
            ],
        ),

    ]


def generate_launch_description():
    depthai_filters_prefix = get_package_share_directory("depthai_filters")

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(depthai_filters_prefix, 'config', 'feature_tracker.yaml')),
    ] + declare_camera_arguments()

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
