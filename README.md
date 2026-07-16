# Depthai ROS Repository
Hi and welcome to the main depthai-ros respository! Here you can find ROS related code for OAK cameras from Luxonis. Don't have one? You can get them [here!](https://shop.luxonis.com/)

You can find the newest documentation [here](https://docs.luxonis.com/software-v3/depthai/ros/)

### ROS 2 distro support

This branch builds from a single source tree on **humble**, **jazzy** and **lyrical**. The per-distro API differences are gated at build time:

| Difference | Gate |
|---|---|
| `cv_bridge/cv_bridge.hpp` vs `.h` (humble) | `__has_include` |
| `create_service` QoS overload (rclcpp >= 28, jazzy+) | `RCLCPP_VERSION_MAJOR` |
| `image_transport::create_camera_publisher` Node* overload deprecated (image_transport >= 6, lyrical) | `image_transport_VERSION` -> `DEPTHAI_ROS_IT_HAS_QOS_OVERLOAD` |
| `geometry_msgs/Pose2D` removed (lyrical) | interfaces use `vision_msgs/Point2D` |
| rosdep key `tar` resolves to `libtar-dev`, absent on lyrical's Ubuntu base | `rosdep install ... --skip-keys "tar"` |

Packages are suffixed `_v3` (e.g. `depthai_ros_driver_v3`) so they can be installed alongside the apt-released v2 driver on humble/jazzy.

### Telemetry
We collect some anonymized telemetry to understand how users use the library so we can improve it. This mainly includes which nodes are used most often, how long sessions last, and similar usage information. If you prefer, you can disable telemetry by setting the environment variable `DEPTHAI_TELEMETRY=0`.

