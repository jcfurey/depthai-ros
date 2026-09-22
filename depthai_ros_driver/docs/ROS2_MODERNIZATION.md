# ROS 2 modernization and migration

This document describes the `cam-dev` modernization changes following the
2026-09-22 review. CI changes are deferred. Build and validation evidence is
recorded in [VALIDATION.md](VALIDATION.md); the branch's compatibility and quality
scope is in [QUALITY_DECLARATION.md](QUALITY_DECLARATION.md).

## Build and compatibility

Rebuild `depthai_bridge`, `depthai_ros_driver`, `depthai_filters`,
`depthai_examples`, and any downstream C++ consumers together. Converter and
publisher class layouts changed; mixing old binaries with new headers/libraries
is unsupported. ROS package versions and CMake project versions now agree at
3.4.0. This development branch is not an ABI-compatible binary patch release.

The validated target is ROS 2 Lyrical on Linux with the workspace DepthAI 3.10.0
SDK. Other distributions and devices require validation before deployment.
The SDK itself is unchanged. Use optimized builds for live camera work:

```bash
colcon build --packages-select depthai_ros_msgs depthai_descriptions \
  depthai_bridge depthai_ros_driver depthai_filters depthai_examples \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
colcon test --packages-select depthai_bridge depthai_ros_driver depthai_filters depthai_examples
colcon test-result --verbose
```

Build/install the DepthAI SDK first. The bridge, driver and filter packages
export their public targets and transitive dependencies. A consumer should only
need `find_package()` for the package whose exported target it links.

RViz is a declared runtime dependency of packages that launch it. RTAB-Map is an
optional integration: install `rtabmap_launch` separately when using its wrapper.
Core camera operation does not require RTAB-Map. The WLS and USB-camera overlay
example launchers currently fail explicitly because their complete input
pipelines are unsupported; they no longer return an empty successful launch.

## Managed camera operation

Launch arguments retain `autostart:=true` by default. To manage transitions:

```bash
ros2 launch depthai_ros_driver driver.launch.py \
  device_ip:=10.2.2.43 namespace:=robot name:=front \
  tf_prefix:=robot_front autostart:=false
ros2 lifecycle get /robot/front
ros2 lifecycle set /robot/front configure
ros2 lifecycle set /robot/front activate
ros2 lifecycle set /robot/front deactivate
ros2 lifecycle set /robot/front cleanup
ros2 lifecycle set /robot/front shutdown
```

The driver exposes the standard lifecycle services and transition events using
the canonical `rcl_lifecycle` state machine. It retains its `rclcpp::Node` base
for compatibility with pipeline plugins. Management services exist before any
hardware connection attempt, including after a failed configuration.

Configure acquires the device, builds the pipeline, queues, publishers and static
TF, but does not start camera streaming. Activate starts streaming. Deactivate
drains callbacks, stops the pipeline and releases the device. Reactivation
rebuilds the SDK pipeline, so it includes connection/startup latency. Cleanup and
shutdown also release resources. A configuration/activation exception runs error
cleanup and returns to unconfigured when cleanup succeeds.

The `~/start`, `~/stop`, `~/start_driver` and `~/stop_driver` compatibility services
remain. Start configures/activates as necessary; stop deactivates. Ctrl-C and
component unload stop producers before destroying converters or ROS publishers.
`driver.i_connection_timeout` bounds discovery retries (default 30 seconds);
individual SDK connection calls retain their own timeout. Autostart attempts
once; after a failure, correct the configuration and explicitly configure again.

## Parameters

Initialization parameters (`*.i_*`) and diagnostic thresholds can change while
inactive, and take effect when the pipeline is rebuilt. They are rejected while
active. `driver.i_autostart` is read-only after construction. Descriptors identify
initialization and runtime settings and retain their numeric ranges.

Runtime (`*.r_*`) parameter callbacks validate the entire proposed batch without
writing to hardware. Only committed ROS values enter the apply queue; a wall
timer applies them and coalesces repeated changes to the latest value. A rejected
atomic batch therefore produces no hardware writes. ROS parameter success means
that the desired values were accepted, not that every device write has already
completed. Hardware writes are not transactional. If applying committed values
fails, diagnostics report the failure and the driver deactivates; fix the desired
configuration and reactivate to rebuild it. Do not assume a partially applied
hardware batch was rolled back.

## Message contracts and time

* Disparity is a real `32FC1` image in pixels with matching payload, stride and
  endianness. Stereo baseline is metres. RAW16 subpixel disparity defaults to
  five fractional bits; direct bridge users can select three through five with
  `setSubpixelFractionalBits()` to match their SDK pipeline.
* Standard spatial `vision_msgs/Detection3DArray` positions and bounding-box
  centres are metres. The device's 2D box does not determine a metric 3D extent:
  zero box dimensions explicitly mean unknown. The SpatialBB component displays
  a small position marker for these detections, and a box only when real positive
  metric dimensions are supplied. Consumers requiring object volumes must supply
  an independent extent estimate.
* The companion `~/nn/spatial_detections_2d` topic carries the existing custom
  `depthai_ros_msgs/SpatialDetectionArray`: pixel boxes plus metric positions.
  The `nn` segment follows the actual neural-network node name. Migrate consumers
  that previously treated standard 3D boxes as pixel geometry to this topic.
* Untracked detection IDs are empty; class labels belong in hypotheses. Trackers
  supply object IDs. Detections carry their capture headers and valid pose
  orientations. Explicit normalized converter modes retain fractional boxes.
* Derived point clouds, markers and TF preserve input capture headers. Converters
  use the owning node's clock. With system time, hardware steady timestamps map
  to ROS capture time; large wall-clock changes reset that mapping.
* With active `use_sim_time`, live hardware has no inherent simulation-time
  mapping. The explicit policy stamps each hardware sample at the current
  `/clock` tick. Paused time remains paused, an uninitialized clock produces zero,
  and forward/backward clock jumps are followed. These stamps are simulation
  receipt ticks, not estimates of hardware capture age. When replaying already
  stamped ROS messages through filters, their original stamps remain intact.
  Feeding a ROS bag into the device stereo pipeline remains unsupported and is
  rejected explicitly.

## Transport, filters and namespaces

BridgePublisher uses reliable, volatile KeepLast(1) by default for live streams,
with standard QoS override policies enabled. Its lazy mode consults resolved
publisher endpoints and all image transports, including camera-info-only demand.
It supports local stop/destruction while the owning ROS context stays alive.
Converters captured by a publisher must outlive it. Do not call `stop()` from
inside that publisher's own converter/callback.

Filter inputs default to best-effort KeepLast(5), compatible with both reliable
and best-effort sources. Load-time parameters `input_qos.reliability`
(`best_effort` or `reliable`) and `input_qos.depth` (1–1000) select the subscription
profile. Synchronizers use that profile consistently for every input.

Features3D accepts `16UC1` millimetres and `32FC1` metres, both byte orders and
padded rows. Invalid/out-of-image depth becomes NaN rather than an out-of-bounds
read. Invalid intrinsics and malformed images are rejected. Overlays handle empty
hypotheses, individual box sizes and failed image conversion.

The WLS component accepts pixel disparity (`mono8` or `32FC1`) or `16UC1`
fixed-point disparity (`disparity_fractional_bits`, default 5). It requires
matching `left/image_raw`, `disparity/image_raw`, and calibrated right-camera
`disparity/camera_info` with nonzero projection `P[3] = -f * baseline`.
It publishes `32FC1` depth in metres on `wls_filtered`, with invalid depth as NaN.
Its example launcher remains unavailable until the camera pipeline exposes those
inputs consistently.

The thermal component runs headlessly. It subscribes to relative
`thermal/raw_data/image_raw`, publishes annotated `color`, and accepts
`sample_x`/`sample_y` for the sample point. View the output with RViz or
`rqt_image_view`; there is no OpenCV window inside the camera container.

Supported filter launchers forward camera name, namespace, device selection,
TF prefix, transport, intra-process and autostart settings. Filter outputs live
under `/<namespace>/<camera>/`; configuration selectors are independent of the
name `oak`. Choose distinct TF prefixes explicitly for multiple cameras.
Standalone examples accept `device_ip`, `device_id` or `usb_port_id` (one selector
at a time), and `tf_prefix`; their executor checks device state periodically and
stops the pipeline before releasing publisher/converter objects. Example startup
parameters are read-only after construction. The host image subscriber waits for
new input and forwards each received frame once. Encoder profiles, dimensions,
IMU mode and covariance values are validated before connecting. The spatial
example explicitly rejects its unimplemented IR switches; use the driver for IR
control.

## Diagnostics and performance

`/diagnostics` includes driver connection/lifecycle state, restart counters and
parameter-apply failures. Image stream diagnostics report observed FPS, sequence
gaps, last-frame age and p50/p95 **capture-to-conversion** time over at most 128
samples. These are measured with a steady clock, independent of `/clock`. They
exclude ROS delivery and RViz rendering; use subscriber receipt timestamps to
measure those costs. Sequence gaps mean gaps between observed frames, not a
precise count of transport packet loss. Lazy streams with no subscribers are
reported idle rather than stalled.

Threshold parameters (milliseconds unless stated otherwise):

| Parameter | Default |
| --- | ---: |
| `diagnostics.max_frame_age_ms` | 200 |
| `diagnostics.error_frame_age_ms` | 1000 |
| `diagnostics.stream_timeout_seconds` | 2 seconds |

System telemetry drains only available queue samples and reports stale samples;
it no longer blocks the executor waiting for telemetry. Automatic recovery is
opt-in with `driver.i_restart_on_diagnostics_error`; diagnostic device identity
is namespace-qualified so another camera cannot trigger a restart.

The existing `poe_rgbd_low_latency.yaml` remains the recommended measured starting
point for the attached RVC2 PoE camera: 640×400 RGB/depth at 15 FPS with point
clouds. Measure age, throughput and CPU under actual subscriber demand before
increasing resolution/rate. Lower queue depth cannot compensate for a pipeline
whose sustained source bandwidth exceeds its link or processing capacity.
