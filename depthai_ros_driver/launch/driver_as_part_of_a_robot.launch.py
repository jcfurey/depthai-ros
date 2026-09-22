import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from depthai_ros_driver.launch_utils import (
    declare_device_arguments,
    image_transport_parameters,
)


def launch_setup(context, *args, **kwargs):
    log_level = "info"
    if context.environment.get("DEPTHAI_DEBUG") == "1":
        log_level = "debug"

    params_file = LaunchConfiguration("params_file")

    name = LaunchConfiguration("name").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    tf_prefix = tf_prefix.strip("/") or name
    use_intra_process = (
        LaunchConfiguration("use_intra_process").perform(context).lower() == "true"
    )

    parent_frame = LaunchConfiguration("parent_frame", default="oak_parent_frame")
    cam_pos_x = LaunchConfiguration("cam_pos_x", default="0.0")
    cam_pos_y = LaunchConfiguration("cam_pos_y", default="0.0")
    cam_pos_z = LaunchConfiguration("cam_pos_z", default="0.0")
    cam_roll = LaunchConfiguration("cam_roll", default="0.0")
    cam_pitch = LaunchConfiguration("cam_pitch", default="0.0")
    cam_yaw = LaunchConfiguration("cam_yaw", default="0.0")
    camera_model = LaunchConfiguration("camera_model", default="OAK-D")
    imu_from_descr = LaunchConfiguration("imu_from_descr", default="false")
    publish_tf_from_calibration = LaunchConfiguration(
        "publish_tf_from_calibration", default="true"
    )
    override_cam_model = LaunchConfiguration("override_cam_model", default="false")

    tf_params = {"driver": {"i_tf_prefix": tf_prefix, "i_autostart": LaunchConfiguration("autostart").perform(context).lower() == "true"}}
    connection_arguments = {
        "i_ip": LaunchConfiguration("device_ip").perform(context),
        "i_device_id": LaunchConfiguration("device_id").perform(context),
        "i_usb_port_id": LaunchConfiguration("usb_port_id").perform(context),
        "i_transport_profile": LaunchConfiguration("transport_profile").perform(context),
    }
    tf_params["driver"].update(
        {key: value for key, value in connection_arguments.items() if value}
    )
    ffmpeg_gop_size = int(
        LaunchConfiguration("image_transport_ffmpeg_gop_size").perform(context)
    )
    tf_params.update(image_transport_parameters(name, ffmpeg_gop_size))
    if publish_tf_from_calibration.perform(context) == "true":
        cam_model = ""
        if override_cam_model.perform(context) == "true":
            cam_model = camera_model.perform(context)
        tf_params["driver"].update(
            {
                "i_publish_tf_from_calibration": True,
                "i_tf_device_name": name,
                "i_tf_device_model": cam_model,
                "i_tf_base_frame": name,
                "i_tf_parent_frame": parent_frame.perform(context),
                "i_tf_cam_pos_x": cam_pos_x.perform(context),
                "i_tf_cam_pos_y": cam_pos_y.perform(context),
                "i_tf_cam_pos_z": cam_pos_z.perform(context),
                "i_tf_cam_roll": cam_roll.perform(context),
                "i_tf_cam_pitch": cam_pitch.perform(context),
                "i_tf_cam_yaw": cam_yaw.perform(context),
                "i_tf_imu_from_descr": imu_from_descr.perform(context),
            }
        )
    else:
        tf_params["driver"]["i_publish_tf_from_calibration"] = False

    return [
        ComposableNodeContainer(
            name=name + "_container",
            namespace=namespace,
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depthai_ros_driver",
                    plugin="depthai_ros_driver::Driver",
                    name=name,
                    namespace=namespace,
                    parameters=[params_file, tf_params],
                    extra_arguments=[
                        {"use_intra_process_comms": use_intra_process}
                    ],
                )
            ],
            arguments=[
                "--executor-type",
                "multi-threaded",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            output="both",
        ),
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="Prefix for camera-related TF frames. Defaults to the node name.",
        ),
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
            default_value=os.path.join(depthai_prefix, "config", "rgbd.yaml"),
        ),
        DeclareLaunchArgument("use_intra_process", default_value="true"),
        DeclareLaunchArgument(
            "image_transport_ffmpeg_gop_size", default_value="1"
        ),
        DeclareLaunchArgument(
            "publish_tf_from_calibration",
            default_value="false",
            description="Enables TF publishing from camera calibration file.",
        ),
        DeclareLaunchArgument(
            "imu_from_descr",
            default_value="false",
            description="Enables IMU publishing from URDF.",
        ),
        DeclareLaunchArgument(
            "override_cam_model",
            default_value="false",
            description="Overrides camera model from calibration file.",
        ),
    ] + declare_device_arguments()

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
