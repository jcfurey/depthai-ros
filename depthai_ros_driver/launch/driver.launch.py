import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from depthai_ros_driver.launch_utils import (
    declare_camera_arguments,
    image_transport_parameters,
)


def is_launch_config_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() == "true"


def setup_launch_prefix(context, *args, **kwargs):
    use_gdb = LaunchConfiguration("use_gdb", default="false")
    use_valgrind = LaunchConfiguration("use_valgrind", default="false")
    valgrind_args = LaunchConfiguration(
        "valgrind_args",
        default="--leak-check=full --show-leak-kinds=all --track-origins=yes --verbose --log-file=memcheck.log",
    )
    use_perf = LaunchConfiguration("use_perf", default="false")

    launch_prefix = []

    if use_gdb.perform(context) == "true":
        launch_prefix.append("xterm -e gdb -ex run --args")
    if use_valgrind.perform(context) == "true":
        launch_prefix.append(f"valgrind {valgrind_args.perform(context)}")
    if use_perf.perform(context) == "true":
        launch_prefix.append(
            "perf record -g --call-graph dwarf --output=perf.out.node_name.data --"
        )

    return " ".join(launch_prefix)


def launch_setup(context, *args, **kwargs):
    log_level = "info"
    if context.environment.get("DEPTHAI_DEBUG") == "1":
        log_level = "debug"

    urdf_launch_dir = os.path.join(
        get_package_share_directory("depthai_descriptions"), "launch"
    )

    parent_frame = LaunchConfiguration(
        "parent_frame", default="oak_parent_frame"
    ).perform(context)
    cam_pos_x = LaunchConfiguration("cam_pos_x", default="0.0")
    cam_pos_y = LaunchConfiguration("cam_pos_y", default="0.0")
    cam_pos_z = LaunchConfiguration("cam_pos_z", default="0.0")
    cam_roll = LaunchConfiguration("cam_roll", default="0.0")
    cam_pitch = LaunchConfiguration("cam_pitch", default="0.0")
    cam_yaw = LaunchConfiguration("cam_yaw", default="0.0")
    use_composition = LaunchConfiguration("rsp_use_composition", default="true")
    imu_from_descr = LaunchConfiguration("imu_from_descr", default="false")
    publish_tf_from_calibration = LaunchConfiguration(
        "publish_tf_from_calibration", default="true"
    )
    override_cam_model = LaunchConfiguration("override_cam_model", default="false")
    params_file = ParameterFile(LaunchConfiguration("params_file"), allow_substs=True)
    camera_model = LaunchConfiguration("camera_model", default="OAK-D")
    rs_compat = LaunchConfiguration("rs_compat", default="false")
    pointcloud_enable = LaunchConfiguration("pointcloud.enable", default="false")
    namespace = LaunchConfiguration("namespace", default="").perform(context)
    name = LaunchConfiguration("name").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    use_intra_process = is_launch_config_true(context, "use_intra_process")

    # If RealSense compatibility is enabled, we need to override some parameters, topics and node names
    parameter_overrides = {}
    points_topic_name = f"{name}/rgbd/points"
    if rs_compat.perform(context) == "true":
        depth_profile = LaunchConfiguration("depth_module.depth_profile").perform(
            context
        )
        color_profile = LaunchConfiguration("rgb_camera.color_profile").perform(context)
        infra_profile = LaunchConfiguration("depth_module.infra_profile").perform(
            context
        )
        # split profile string (0,0,0 or 0x0x0 or 0X0X0) into with (int) height(int) and fps(double)
        # find delimiter
        delimiter = ","
        if "x" in depth_profile:
            delimiter = "x"
        elif "X" in depth_profile:
            delimiter = "X"
        depth_profile = depth_profile.split(delimiter)
        color_profile = color_profile.split(delimiter)
        infra_profile = infra_profile.split(delimiter)

        if name == "oak":
            name = "camera"
        points_topic_name = f"{name}/depth/color/points"
        if namespace == "":
            namespace = "camera"
        if parent_frame == "oak_parent_frame":
            parent_frame = f"{name}_link"
        parameter_overrides = {
            "driver": {
                "i_rs_compat": True,
            },
            "pipeline_gen": {
                "i_enable_sync": True,
            },
            "color": {
                "i_publish_topic": is_launch_config_true(context, "enable_color"),
                "i_synced": True,
                "i_width": int(color_profile[0]),
                "i_height": int(color_profile[1]),
                "i_fps": float(color_profile[2]),
            },
            "depth": {
                "i_publish_topic": is_launch_config_true(context, "enable_depth"),
                "i_synced": True,
                "i_subpixel": True,
                "i_width": int(depth_profile[0]),
                "i_height": int(depth_profile[1]),
                "i_fps": float(depth_profile[2]),
                "i_left_rect_publish_topic": is_launch_config_true(
                    context, "enable_infra1"
                ),
                "i_right_rect_publish_topic": is_launch_config_true(
                    context, "enable_infra2"
                ),
            },
            "infra1": {
                "i_width": int(infra_profile[0]),
                "i_height": int(infra_profile[1]),
                "i_fps": float(infra_profile[2]),
            },
            "infra2": {
                "i_width": int(infra_profile[0]),
                "i_height": int(infra_profile[1]),
                "i_fps": float(infra_profile[2]),
            },
        }
        if pointcloud_enable.perform(context) == "true":
            parameter_overrides["pipeline_gen"]["i_enable_rgbd"] = True

    tf_prefix = tf_prefix.strip("/") or name

    params = {"driver": {"i_tf_prefix": tf_prefix, "i_autostart": LaunchConfiguration("autostart").perform(context).lower() == "true"}}
    connection_arguments = {
        "i_ip": LaunchConfiguration("device_ip").perform(context),
        "i_device_id": LaunchConfiguration("device_id").perform(context),
        "i_usb_port_id": LaunchConfiguration("usb_port_id").perform(context),
        "i_transport_profile": LaunchConfiguration("transport_profile").perform(context),
    }
    params["driver"].update(
        {key: value for key, value in connection_arguments.items() if value}
    )
    if publish_tf_from_calibration.perform(context) == "true":
        cam_model = ""
        if override_cam_model.perform(context) == "true":
            cam_model = camera_model.perform(context)
        params["driver"].update(
            {
                "i_publish_tf_from_calibration": True,
                "i_tf_device_name": name,
                "i_tf_device_model": cam_model,
                "i_tf_base_frame": name,
                "i_tf_parent_frame": parent_frame,
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
        params["driver"]["i_publish_tf_from_calibration"] = False
    if pointcloud_enable.perform(context) == "true":
        params["pipeline_gen"] = {"i_enable_rgbd": True}

    launch_prefix = setup_launch_prefix(context)
    camera_root = "/" + "/".join(
        part.strip("/") for part in (namespace, name) if part.strip("/")
    )
    ffmpeg_gop_size = int(
        LaunchConfiguration("image_transport_ffmpeg_gop_size").perform(context)
    )
    params.update(image_transport_parameters(name, ffmpeg_gop_size))
    namespace_root = "/" + namespace.strip("/") if namespace.strip("/") else ""
    resolved_points_topic = (
        points_topic_name
        if points_topic_name.startswith("/")
        else f"{namespace_root}/{points_topic_name}"
    )
    rviz_fixed_frame = LaunchConfiguration("rviz_fixed_frame").perform(context)
    rviz_fixed_frame = rviz_fixed_frame.strip("/") or name
    rviz_remappings = [
        (f"/oak/{suffix}", f"{camera_root}/{suffix}")
        for suffix in (
            "rgb/image_raw",
            "rgb/camera_info",
            "stereo/image_raw",
            "stereo/camera_info",
            "imu/data",
            "vio/odometry",
            "nn/image_raw",
            "nn/detections",
            "nn/spatial_detections",
            "nn/passthrough/image_raw",
        )
    ]
    rviz_remappings.append(("/oak/rgbd/points", resolved_points_topic))
    rviz_remappings.append(("/spatial_bb", f"{camera_root}/spatial_bb"))

    return [
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d",
                LaunchConfiguration("rviz_config"),
                "-f",
                rviz_fixed_frame,
            ],
            remappings=rviz_remappings,
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(urdf_launch_dir, "urdf_launch.py")
            ),
            launch_arguments={
                "namespace": namespace,
                "rsp_name": name,
                "tf_prefix": tf_prefix,
                "camera_model": camera_model,
                "base_frame": name,
                "parent_frame": parent_frame,
                "cam_pos_x": cam_pos_x,
                "cam_pos_y": cam_pos_y,
                "cam_pos_z": cam_pos_z,
                "cam_roll": cam_roll,
                "cam_pitch": cam_pitch,
                "cam_yaw": cam_yaw,
                "use_composition": use_composition,
                "use_base_descr": publish_tf_from_calibration,
                "rs_compat": rs_compat,
            }.items(),
        ),
        ComposableNodeContainer(
            name=f"{name}_container",
            namespace=namespace,
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depthai_ros_driver",
                    plugin="depthai_ros_driver::Driver",
                    name=name,
                    namespace=namespace,
                    parameters=[
                        params_file,
                        params,
                        parameter_overrides,
                    ],
                    extra_arguments=[
                        {"use_intra_process_comms": use_intra_process},
                    ],
                    remappings=[(f"{name}/rgbd/points", points_topic_name)],
                )
            ],
            arguments=[
                "--executor-type",
                "multi-threaded",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            prefix=[launch_prefix],
            output="both",
        ),
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        *declare_camera_arguments(),
        DeclareLaunchArgument("parent_frame", default_value="oak_parent_frame"),
        DeclareLaunchArgument("camera_model", default_value="OAK-D"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(depthai_prefix, "config", "driver.yaml"),
        ),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument(
            "rviz_fixed_frame",
            default_value="",
            description="RViz fixed frame. Empty uses the camera base frame (name).",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(depthai_prefix, "config", "rviz", "rgbd.rviz"),
        ),
        DeclareLaunchArgument("rsp_use_composition", default_value="true"),
        DeclareLaunchArgument(
            "publish_tf_from_calibration",
            default_value="true",
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
        DeclareLaunchArgument("use_gdb", default_value="false"),
        DeclareLaunchArgument("use_valgrind", default_value="false"),
        DeclareLaunchArgument(
            "valgrind_args",
            default_value="--leak-check=full --show-leak-kinds=all --track-origins=yes --verbose --log-file=memcheck.log",
        ),
        DeclareLaunchArgument("use_perf", default_value="false"),
        DeclareLaunchArgument(
            "rs_compat",
            default_value="false",
            description="Enables compatibility with RealSense nodes.",
        ),
        DeclareLaunchArgument("pointcloud.enable", default_value="false"),
        DeclareLaunchArgument("enable_color", default_value="true"),
        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("enable_infra1", default_value="false"),
        DeclareLaunchArgument("enable_infra2", default_value="false"),
        DeclareLaunchArgument("depth_module.depth_profile", default_value="640,400,30"),
        DeclareLaunchArgument("rgb_camera.color_profile", default_value="640,400,30"),
        DeclareLaunchArgument("depth_module.infra_profile", default_value="640,400,30"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
