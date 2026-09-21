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

`LOW_BANDWIDTH` encodes compatible image streams. RVC2 stereo depth, ToF depth,
and thermal outputs remain raw. On platforms that support encoded stereo, it
uses integer disparity; select `RAW` when subpixel depth is required.
A per-stream `i_low_bandwidth` parameter overrides the global transport profile
for that stream. Overrides set at runtime are also preserved across restarts.
Direct `i_publish_compressed` publication requires that stream's
`i_low_bandwidth=true`; incompatible settings are rejected at startup. With raw
device transport, use an `image_transport` compressed subscriber for host-side
encoding. Synchronized streams support device-side encoding. Lazy publishers
skip image conversion when neither image nor camera-info subscribers are present.
The optional host-side ffmpeg image transport defaults to a one-frame GOP for
low-latency viewing; tune `image_transport_ffmpeg_gop_size` when bandwidth is
more important than seek/recovery latency.

### RVC2 stereo low-bandwidth limitation

With DepthAI 3.10, the RVC2 video encoder used by OAK-D PoE models does not
accept the `StereoDepth` integer-disparity output (`ImgFrame` type 14/RAW8).
`AUTO` and `LOW_BANDWIDTH` therefore keep RVC2 stereo depth raw while encoding
compatible streams such as RGB. Explicitly setting `stereo.i_low_bandwidth`
(or `depth.i_low_bandwidth` in RealSense mode) to true on RVC2 is rejected at
startup with an explanatory error instead of silently dropping depth frames.
The shipped low-bandwidth configuration follows the same defaults.

Select `RAW` to keep all image streams raw:

```bash
ros2 launch depthai_ros_driver rgbd_pcl.launch.py \
  device_ip:=10.2.2.50 transport_profile:=RAW
```

The raw depth output retains its 16-bit format and subpixel configuration.

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

Set `rgbd.i_publish_topic=false` to suppress the point-cloud ROS output while
keeping the RGBD node available to the pipeline. Magnetometer messages use tesla;
`imu.i_mag_cov` is the variance in tesla squared. VIO and SLAM outputs retain
measurement timestamps so transport latency does not shift their time relative
to camera and IMU messages.
Separate magnetometer reports default to enabled only for `IMU_WITH_MAG` and
`IMU_WITH_MAG_SPLIT` messages; `imu.i_enable_mag` can override this. This avoids
throttling the BNO08x IMU stream with unused magnetic reports when publishing
plain `sensor_msgs/Imu`, while retaining rotation-vector fusion.

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

The launch file applies the resolved camera topics and camera base frame to RViz, so
custom `name`, `namespace`, and `tf_prefix` values do not require editing the
RViz file. Wrappers consistently expose `rviz_config` and `rviz_fixed_frame`;
VIO defaults the latter to `odom`. Otherwise, the fixed frame defaults to `name`,
even when a different `tf_prefix` is used for the sensor frames.

## Stereo input from rosbags

`stereo_from_rosbag.launch.py` is disabled until ROS image input queues are
implemented. It reports an error before starting a camera or bag playback.
Setting a sensor's `i_simulate_from_topic` directly is also rejected.

## Runtime control and shutdown

The concise runtime services are:

```bash
ros2 service call /oak/stop std_srvs/srv/Trigger '{}'
ros2 service call /oak/start std_srvs/srv/Trigger '{}'
```

The legacy `/oak/stop_driver` and `/oak/start_driver` names remain available.
Ctrl-C performs the same orderly queue and pipeline teardown automatically; a
manual stop call is not required.

Parameter changes during startup, stop, or restart are rejected with a retry
message. Save-pipeline and save-calibration requests wait for an active lifecycle
operation to finish, then report an error if the driver is stopped.
