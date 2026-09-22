"""Shared launch arguments for selecting and tuning an OAK camera."""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def declare_device_arguments():
    return [
        DeclareLaunchArgument(
            "device_ip",
            default_value="",
            description="OAK PoE IP address. Empty uses normal DepthAI discovery.",
        ),
        DeclareLaunchArgument(
            "device_id",
            default_value="",
            description="OAK MXID. Empty uses normal DepthAI discovery.",
        ),
        DeclareLaunchArgument(
            "usb_port_id",
            default_value="",
            description="DepthAI USB port identifier. Empty uses normal discovery.",
        ),
        DeclareLaunchArgument(
            "transport_profile",
            default_value="",
            description="AUTO, RAW, or LOW_BANDWIDTH. Empty uses the parameter-file/default value.",
        ),
    ]


def device_launch_arguments():
    return {
        "device_ip": LaunchConfiguration("device_ip"),
        "device_id": LaunchConfiguration("device_id"),
        "usb_port_id": LaunchConfiguration("usb_port_id"),
        "transport_profile": LaunchConfiguration("transport_profile"),
    }


def declare_camera_arguments():
    """Arguments that every single-camera wrapper should expose."""
    return [
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Configure and activate automatically; false enables external lifecycle management.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="ROS namespace for the camera node and its topics.",
        ),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="TF frame prefix. Empty uses the resolved camera node name.",
        ),
        DeclareLaunchArgument(
            "use_intra_process",
            default_value="true",
            description="Use intra-process communication in the component container.",
        ),
        DeclareLaunchArgument(
            "image_transport_ffmpeg_gop_size",
            default_value="1",
            description="Keyframe interval for the optional host-side ffmpeg image transport.",
        ),
        *declare_device_arguments(),
    ]


def camera_launch_arguments():
    """Forward common single-camera arguments to ``driver.launch.py``."""
    return {
        "autostart": LaunchConfiguration("autostart"),
        "namespace": LaunchConfiguration("namespace"),
        "tf_prefix": LaunchConfiguration("tf_prefix"),
        "use_intra_process": LaunchConfiguration("use_intra_process"),
        "image_transport_ffmpeg_gop_size": LaunchConfiguration(
            "image_transport_ffmpeg_gop_size"
        ),
        **device_launch_arguments(),
    }


def image_transport_parameters(camera_name, ffmpeg_gop_size):
    """Build low-latency ffmpeg image-transport overrides for camera topics."""
    if ffmpeg_gop_size < 1:
        raise ValueError("image_transport_ffmpeg_gop_size must be at least 1")

    # image_transport derives its parameter namespace from the private topic,
    # which includes the node name but not its ROS namespace.
    parameter_root = camera_name.strip("/").replace("/", ".")
    stream_names = (
        "rgb",
        "color",
        "left",
        "right",
        "stereo",
        "depth",
        "infra1",
        "infra2",
        "tof",
        "thermal",
    )
    topic_names = ("image_raw", "image_rect", "image_rect_raw")
    return {
        f"{parameter_root}.{stream}.{topic}.ffmpeg.gop_size": ffmpeg_gop_size
        for stream in stream_names
        for topic in topic_names
    }


def camera_component_names(context):
    """Resolve an absolute container/topic root and a namespace for camera helpers."""
    name = LaunchConfiguration("name").perform(context).strip("/")
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    root = "/" + "/".join(part for part in (namespace, name) if part)
    return root + "_container", root, root
