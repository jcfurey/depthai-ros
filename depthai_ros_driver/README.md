# DepthAI ROS driver

## Quick start

USB 3 cameras use raw image transport by default:

```bash
ros2 launch depthai_ros_driver driver.launch.py
```

PoE cameras can be selected directly. `AUTO` chooses device-side image encoding
for compatible PoE and USB2 image streams so camera traffic does not compete for
link bandwidth:

```bash
ros2 launch depthai_ros_driver driver.launch.py device_ip:=10.2.2.43
```

The same selection arguments work with the RGBD, point-cloud, VIO, calibration,
RTAB-Map, thermal, and short-range launch files:

- `device_ip`: PoE address
- `device_id`: MXID
- `usb_port_id`: DepthAI USB port identifier
- `transport_profile`: `AUTO`, `RAW`, or `LOW_BANDWIDTH`

Those single-camera launches also share `namespace`, `tf_prefix`, and
`use_intra_process`, so moving a camera into a robot namespace does not require
editing its YAML or wrapper launch file.

`LOW_BANDWIDTH` uses integer disparity. Select `RAW` when subpixel depth is more
important than constrained-link frame rate. A per-stream `i_low_bandwidth`
parameter in YAML overrides the global transport profile for that stream.
The optional host-side ffmpeg image transport defaults to a one-frame GOP for
low-latency viewing; tune `image_transport_ffmpeg_gop_size` when bandwidth is
more important than seek/recovery latency.

### RVC2 stereo low-bandwidth limitation

With DepthAI 3.10, the RVC2 video encoder used by OAK-D PoE models does not
accept the `StereoDepth` integer-disparity output (`ImgFrame` type 14/RAW8).
If `AUTO` or `LOW_BANDWIDTH` enables `stereo.i_low_bandwidth`, the device logs
the following warning repeatedly and `/oak/stereo/image_raw` remains silent:

```text
Arrived frame type (14) is not either NV12 or YUV400p (8-bit Gray)
```

Use raw transport for RGBD and point-cloud launches on affected RVC2 devices:

```bash
ros2 launch depthai_ros_driver rgbd_pcl.launch.py \
  device_ip:=10.2.2.50 transport_profile:=RAW
```

To retain compressed RGB while publishing raw 16-bit depth, override only the
stereo stream with `stereo.i_low_bandwidth:=false` in the parameter file. The
verified raw output is `640 x 400`, `16UC1`; point-cloud and IMU publication
continue normally.

## Configuration

All shipped single-camera YAML files use the `/**` ROS parameter selector, so
they work with custom node names, namespaces, launch files, and the executable:

```bash
ros2 run depthai_ros_driver driver --ros-args \
  --params-file /path/to/camera.yaml \
  -p driver.i_ip:=10.2.2.43
```

The executable's default node name is `oak`. Override it normally with
`-r __node:=front_oak` and use `driver.i_tf_prefix` when a different TF prefix
is needed.

## Common outputs

For the default `oak` node, the RGBD pipeline publishes:

- `/oak/rgb/image_raw` and `/oak/rgb/camera_info`
- `/oak/stereo/image_raw` and `/oak/stereo/camera_info`
- `/oak/imu/data`
- `/oak/rgbd/points` when `pointcloud.enable:=true`
- `/diagnostics` when diagnostics are enabled

Inspect the active graph rather than assuming optional streams are present:

```bash
ros2 topic list -t
ros2 topic hz /oak/rgb/image_raw
ros2 topic hz /oak/stereo/image_raw
ros2 topic hz /oak/imu/data
```

## OAK Thermal

Select an Ethernet OAK Thermal explicitly when other cameras share the network:

```bash
ros2 launch depthai_ros_driver oak_t.launch.py device_ip:=10.2.2.44
```

The native thermal outputs are 256 x 192 at a requested 25 FPS:

- `/oak/thermal/image_raw`: YUV422 display image
- `/oak/thermal/raw_data/image_raw`: `32FC1` temperature image in degrees Celsius
- matching `camera_info` topics below each image namespace

The launch also publishes `/oak/rgb/image_raw` and `/oak/imu/data` when those
sensors are present. On PoE, `AUTO` compresses RGB but deliberately leaves both
thermal products raw because the device video encoder does not accept their
YUV422 and FP16 source formats. Do not set `thermal.i_low_bandwidth` to true.

View the display image with the standard ROS viewer:

```bash
ros2 run rqt_image_view rqt_image_view /oak/thermal/image_raw
```

## Viewing

Launch RViz with the matching camera configuration:

```bash
ros2 launch depthai_ros_driver driver.launch.py use_rviz:=true
```

The launch file applies the resolved camera topics and TF prefix to RViz, so
custom `name`, `namespace`, and `tf_prefix` values do not require editing the
RViz file. Wrappers consistently expose `rviz_config` and `rviz_fixed_frame`;
VIO defaults the latter to `odom`.

## Runtime control and shutdown

The concise runtime services are:

```bash
ros2 service call /oak/stop std_srvs/srv/Trigger '{}'
ros2 service call /oak/start std_srvs/srv/Trigger '{}'
```

The legacy `/oak/stop_driver` and `/oak/start_driver` names remain available.
Ctrl-C performs the same orderly queue and pipeline teardown automatically; a
manual stop call is not required.
