# Open issues backlog

Remaining findings from the 2026-09-25 review of `cam-dev`. Tier 1
correctness fixes landed in `0945cf2` (see the follow-up section of
[ROS2_MODERNIZATION.md](ROS2_MODERNIZATION.md)). Line numbers refer to
`0945cf2` and will drift. Items were found by code audit; those marked
**(live)** were also observed on the OAK-D-PRO-POE (RVC2, BNO08x IMU).

## Tier 2: ROS 2 convention deviations

Items marked **(rename)** change topic or frame names and break existing
remaps, RViz configs and URDF consumers. They need a maintainer decision:
adopt the standard names, or keep the current names and document them.

* **(rename) Rect and raw share `camera_info`.** `stereo.cpp` `setupRectQueue`
  publishes `~/left/image_rect` through `create_camera_publisher`, whose sibling
  is `~/left/camera_info`, the same topic as `left/image_raw`. With both enabled,
  two publishers alternate distortion/rectification on one topic. Compressed
  rect output also lands on `left/image_raw/compressed` (default
  `compressedTopicSuffix`). Fix: give rect streams their own namespace.
* **(rename) H.264/H.265 on the `compressed` topic name.** `img_pub.cpp` sends
  `FFMPEGPacket` to `.../image_raw/compressed`; `ffmpeg_image_transport`
  subscribes to `.../image_raw/ffmpeg`. The bridge's default encoding string is
  `libx264` (an encoder name); `FFMPEGPacket.encoding` expects a codec such as
  `h264`. `toRosFFMPEGPacket` uses `camWidth/camHeight`, which are -1 until
  `calibrationToCameraInfo` runs.
* **(rename) Base frame ignores `tf_prefix`.** `driver.launch.py` sets
  `i_tf_base_frame` / `base_frame` to `name`; two cameras with distinct
  prefixes still both publish `oak_parent_frame -> oak`. Same in
  `driver_as_part_of_a_robot.launch.py`. VIO's `i_child_frame_id` is hardcoded to
  `oak_parent_frame` (`vio_param_handler.cpp`). `disparity_publisher.cpp` and
  `feature_tracker_publisher.cpp` examples omit `tfPrefix` in `TFPublisher`.
* **Stereo `P[3]` on the left camera (live).** `calibrationToCameraInfo` puts
  `-fx * B` on the first (left) stereo camera and zero on the right. ROS
  convention is zero for the left and `-fx * B` for the right.
* **Distortion model.** `calibrationToCameraInfo` truncates the 14 DepthAI
  coefficients to 8 and hard-codes `rational_polynomial`; `generateCameraInfo`
  copies all 14 under the same label. Map `calibHandler.getDistortionModel()`
  and size `D` accordingly (fisheye and thin-prism calibrations are mislabelled).
* **IMU covariances default to 0 (live).** `imu_param_handler.cpp`
  `i_acc_cov`, `i_gyro_cov`, `i_rot_cov`, `i_mag_cov` default to 0.0, which
  EKFs treat as near-perfect data. Default to datasheet noise values.
* **Spatial/tracklet units.** Spatial and tracklet converters hard-code `/1000`
  and ignore the SDK `unit` field; track converters report `score = thresh`
  (constant) instead of the source detection confidence; `class_id` can be
  empty (fall back to `std::to_string(label)`).
* **Detection boxes in NN-input pixels.** `detection.hpp` uses
  `normalized=false`; boxes are scaled to the NN input size but stamped with the
  RGB optical frame beside `rgb/image_raw`. Remap through `ImgTransformation` or
  document that boxes are valid only against `passthrough`.
* **Deprecated APIs.** `CameraInfoManager(rclcpp::Node*)` (BridgePublisher);
  every instance registers the same default `set_camera_info` service
  (also `thermal.cpp`). `tf2_ros/*.h` includes (`TFPublisher.hpp` and others).
  `ImgDetectionConverter`/`SpatialDetectionConverter` dereference
  `transformation->getSize()` on an unchecked `std::optional`.
* **Packaging.** `depthai_ros_driver/package.xml` requires depthai >= 3.7.1 but
  uses 3.10 APIs; `ament_cmake_auto` is a `<depend>` (should be
  `buildtool_depend`); `${COMMON_DEPS}`/`${SENSOR_DEPS}` are undefined in
  CMake; `CMAKE_BUILD_SHARED_LIBS` is a no-op typo. Bridge declares unused
  `libboost-dev`, omits `geometry_msgs`, links `opencv_highgui` but not
  `opencv_imgcodecs`, and duplicates `ament_export_targets` with legacy exports.
  `depthai_examples` depends on launch/RViz packages it never uses.
* **Lifecycle on pipeline death.** A dead pipeline leaves the node ACTIVE with
  diagnostics ERROR (`publishStatus`); a lifecycle manager cannot see it.
  `i_restart_on_diagnostics_error` is one-shot with no backoff; a PoE camera
  slow to reboot ends in unconfigured.
* **Diagnostics.** Each stream and SysLogger owns its own
  `diagnostic_updater::Updater` (separate timers/arrays, "Node starting up" on
  every rebuild); driver status is hand-published. The driver subscribes to all
  of `/diagnostics` even when restart is disabled and takes `lifecycleMtx` per
  message. Consolidate into one driver-owned updater.

## Tier 3: performance

* **Image copies.** `ImageConverter::toRosMsg` pushes lvalues into the deque and
  `toRosMsgPtr` copies again via `make_shared<Image>(msg)`; same pattern in
  disparity, compressed, FFMPEG, detection, IMU and grid converters.
  `BridgePublisher` publishes by const reference, defeating intra-process
  zero-copy; use the `unique_ptr` overloads.
* **RGBD / SLAM clouds.** `rgbd.cpp` converts and copies ~4 MB clouds with no
  subscriber check and publishes by const reference; `slam.cpp` ground/obstacle
  clouds, Detection, IMU and VIO do the same `deq.front()` copy.
* **Lazy publishing missing** on rect streams (`setupRectQueue`) and NN
  passthrough configs (`detection.hpp`, `spatial_detection.hpp`).
* **Filters without intra-process.** `depthai_filters/launch/*` and
  `pointcloud.launch.py` omit `use_intra_process_comms`; overlay publishers copy
  on publish. `wls_filter.cpp` uses per-pixel `.at<>` loops.
* **Executor blocking.** `driver.launch.py` uses the single-threaded
  `component_container`; autostart discovery/connect (up to
  `i_connection_timeout`, max 300 s) runs on the executor under `lifecycleMtx`,
  and `get_state` also takes that mutex. `ros2 lifecycle get` and diagnostics
  stall during connection problems.
* **Polling.** A 20 ms wall timer (`applyPendingParameters`) takes the recursive
  lifecycle mutex forever, even when idle.
* **Queues.** Callback-only SDK queues still retain `maxSize` (default 8) stale
  frames. RGB output is forced to NV12 (`camera.cpp`), costing a host colour
  conversion plus cv_bridge copy per frame; expose the output type.
* **Duplicate work.** `img_pub.cpp` computes disparity-to-depth twice on the
  low-bandwidth path (second call outside the try/catch).
* **`PointCloudConverter::setDepthUnit`** mixes meanings (METER means mm input
  to m output; CENTIMETER/FOOT mean input unit). Values are correct for the
  driver's usage but the API is misleading; `rgbd_publisher.cpp` example never
  sets a unit and publishes millimetres.

## Tier 4: usability and broken examples

* `calibration.launch.py` cannot commit: `set_camera_info` clients are not
  remapped to the driver's services; bogus positional args and a duplicate
  `--camera_name`; `--no-service-check` hides the failure.
* `example_multicam.launch.py` runs a nonexistent `obj_pub.py`; its YAML uses a
  v2 model path, hard-codes three MXIDs and parents cameras to `map`.
* `imu_from_descr:=true` fails xacro on OAK-D-S2-POE, LR, SR and OAK4
  (`imu_offset_*` undefined in `base_macro.urdf.xacro`).
* Boolean launch args compared case-sensitively (`== "true"`); `rs_compat:=True`
  is applied to xacro but not the driver.
* Missing launch args: `log_level`, `use_sim_time`, `respawn`, existing
  container.
* Stale YAML keys silently ignored: `stereo.i_publish_synced_rect_pair`
  (`isaac_vslam.yaml`), `rgb.i_set_isp_scale` (`calibration.yaml`). Consider a
  test cross-checking config keys against declared parameters.
* `save_pipeline` ignores `ofstream` failure and returns success; neither save
  service returns the output path.
* No `~/get_device_info` service or device/pipeline summary in diagnostics.
* Pipeline/sensor-count mismatch only warns; plugin load failure omits the type
  name and valid choices. `depthPresetMap.at()` throws a bare
  `unordered_map::at` for bad presets; `base_pipeline.cpp` warns about the
  non-existent `camera.i_nn_type`.
* TF pose parameters are strings (YAML floats fail); `i_fps` type differs
  between nodes (int/float/double), so integer YAML values throw for some.
* Direct-IP fallback runs only when discovery finds no devices; a USB camera
  plus a routed PoE camera never tries the IP.
* `TFPublisher` discards the `robot_state_publisher` set-parameters future; a
  late-starting RSP never receives the URDF.
* Segmentation: transposed reshape, `now()` stamps, empty `camera_info`,
  full-resolution passthrough info, unchecked `layers[1]`, unused `ImageManip`.
* Dockerfile defaults to `kilted`, uses `sudo`, pipes an unpinned installer;
  `build.sh -r` help text is inverted.
* Dead code: unlisted `ImageMarker*.msg`, unused SpatialBB/Features3D/
  SegmentationOverlay members and parameters, `Detection::getInput()`.

## Unverified follow-ups

* **RVC2 firmware crash on stop.** On 2026-09-25 the device reported an
  `INTERNAL_ERROR_CORE` (SIPP) crash twice while stopping an unaligned-depth
  pipeline with rect streams at 1280x800. The session began right after a
  previous container was killed abruptly. It did not reproduce in nine further
  stop/start cycles with the same configuration. Crash dump:
  `~/.cache/depthai/crashdumps/0f78072d3117aeca0567b1e9a1fa2378128523db/`.
* **Neural depth alignment.** Unaligned `NeuralDepth` output keeps the align
  socket's frame; its actual reference camera was not confirmed.
