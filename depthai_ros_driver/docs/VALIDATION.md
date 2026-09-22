# Local modernization validation

Run on 2026-09-22 against ROS 2 Lyrical and workspace DepthAI 3.10.0 on Linux,
starting from `cam-dev` revision `be0dc9b`. CI was not changed.

## Software results

| Package | Test cases | CTest suites | Result |
| --- | ---: | ---: | --- |
| `depthai_bridge` | 62 | 14 | Pass |
| `depthai_ros_driver` | 48 | 6 | Pass |
| `depthai_filters` | 5 | 1 | Pass |
| `depthai_examples` | 5 | 1 | Pass |
| **Total** | **120** | **22** | **Pass** |

* Optimized builds of the bridge, driver, filters and all 14 configured examples
  succeeded. Message and description packages were rebuilt/installed as well.
* Bridge, driver, filter and example regression tests passed. Coverage includes
  disparity bytes/units/subpixel scaling; metric/fractional detections; covariance
  sizes; injected ROS clocks and exposure metadata; callback destruction and
  custom contexts; compressed-only and camera-info-only subscribers; effective
  QoS overrides; matching odometry/TF capture stamps; malformed images; real
  best-effort delivery; metric WLS/markers; and thermal processing without a
  display server.
* Driver tests exercise lifecycle transitions, errors/retry and start-service
  failure details, namespaced services,
  parameter validation/commit behavior, inactive-state recovery guards, stream
  age/stall diagnostics, and configuration/launch forwarding.
* Standalone consumers of `depthai_bridge`, `depthai_filters` and
  `depthai_ros_driver` each configured, linked and ran using only their direct
  package's `find_package()` and exported target. Package configurations and
  include/link paths came from the install space. This workspace uses symlink
  installation for some headers; it is not a fresh binary-only OS installation.
* An installed component-container probe loaded two inactive drivers in separate
  namespaces with intra-process communication enabled. `ros2 lifecycle get`
  recognized their state. Invalid activation was rejected; unavailable-device
  configuration failed and returned to unconfigured, then could be retried.
  Shutdown reached finalized. Both components unloaded while the container and
  ROS context stayed alive; reload/unload also succeeded.
* AddressSanitizer and UndefinedBehaviorSanitizer checks cover bridge lifetime,
  disparity and covariance bounds, and clock/exposure conversion. The SDK and
  ROS dependencies were not instrumented. Leak detection was disabled for these
  focused checks; they are not a whole-process leak or race certification.
* Python syntax, package XML, aligned 3.4.0 project/package versions, and
  `git diff --check` were checked.

## Measured idle-worker improvement

A Release-build microbenchmark compared the original `be0dc9b` BridgePublisher
header with the replacement, using one empty SDK queue and a running ROS context.
Each worker ran for two seconds. CPU time was measured with `std::clock()` and
wall time with a steady clock; percentages are relative to one CPU core.

| Worker | CPU seconds | Wall seconds | CPU, one core |
| --- | ---: | ---: | ---: |
| Original busy polling | 1.99961 | 2.00007 | 99.98% |
| Bounded wait | 0.002266 | 2.00007 | 0.11% |

This isolates idle worker overhead. It does not predict camera throughput or
RViz latency. The baseline required global ROS shutdown to finish destruction;
the new publisher also has regression tests for stopping while its context stays
alive.

## Reproduce the install-space export checks

After building and sourcing the ROS packages, run from the repository root:

```bash
for package in depthai_bridge depthai_filters depthai_ros_driver; do
  cmake -S depthai_ros_driver/test/install_consumer \
    -B "/tmp/consumer-$package" -DCONSUMER_PACKAGE="$package"
  cmake --build "/tmp/consumer-$package"
  "/tmp/consumer-$package/consumer"
done
```

Each configure deliberately finds just one direct package, so another consumer's
`find_package()` cannot conceal a missing transitive export. Use installed
prefixes in `CMAKE_PREFIX_PATH` if installing with CMake directly rather than
colcon.

## Live RVC2 validation

After software validation, the camera was reconnected on 2026-09-22. DepthAI
discovery found `10.2.2.43`, device ID `18443010D1BAE7F400`; the driver identified
an OAK-D-PRO-POE (RVC2, NG9097). Tests used isolated ROS domains and released the
device on completion.

* An installed component probe exercised configure, activate, deactivate,
  reactivate, cleanup and shutdown. Standard lifecycle CLI discovery worked,
  activation from unconfigured was rejected, and no frames were published while
  inactive. Runtime updates succeeded while active; initialization settings were
  rejected while active and accepted while inactive. Reactivation rebuilt the
  pipeline and resumed RGB, depth and point-cloud output. Unload/reload succeeded
  while the container and another inactive, namespaced driver remained alive.
* `test_driver.py` passed all 10 tests, including the shutdown check. This covers
  IMU/RGB/depth delivery, calibration overrides, parameter updates, pipeline and
  calibration saving, and stop/start. `test_image_publication.py` passed both
  tests: synchronized publication, compressed-only and camera-info-only demand,
  restart, rejection of an invalid transport combination with an actionable
  start-service error, recovery after correction, and process shutdown.
* A normal `driver.launch.py` run used namespace `/live`, camera name `front`,
  TF prefix `live_front`, autostart, intra-process communication and calibrated
  TF. RGB, aligned depth and cloud headers referenced an optical frame present
  in `/tf_static`. Camera and system telemetry diagnostics reported OK. Unloading
  the active driver took 0.119 seconds and stopped publication while the
  container and robot-state publisher remained alive. The container subsequently
  exited cleanly. The installed Lyrical Python launch stack still printed an
  ignored `rclpy.executors.Executor.__del__` / `InvalidHandle` exception after
  shutdown; the driver component had already been unloaded.
* Live telemetry exposed inconsistent CMX total-memory units. After correcting
  both diagnostic and text output to MiB, a focused run confirmed three samples
  with 2.5 MiB total and usage below that total, then shut down cleanly.

The default 30-second discovery retry budget was sufficient for PoE stop/start.
A probe using an explicit 5-second budget failed during reactivation; an SDK
connection attempt can itself take longer than that budget. Reactivation includes
device reconnection and pipeline construction, separately from steady-state
frame age.

### Subscriber frame age

The normal launch used `poe_rgbd_low_latency.yaml`: 640×400 RGB/depth, 15 FPS,
RGBD points enabled, AUTO transport, device diagnostics enabled and IR disabled.
A local Python subscriber received all three streams with sensor-data QoS.
After five seconds of warmup, it sampled for 20 seconds and measured receipt
time minus the ROS capture header stamp. RViz was not running.

| Stream | Samples | Received FPS | Median age (ms) | p95 age (ms) |
| --- | ---: | ---: | ---: | ---: |
| RGB | 299 | 14.95 | 61.7 | 104.2 |
| Depth | 300 | 15.00 | 60.6 | 103.6 |
| Point cloud | 290 | 14.50 | 69.2 | 114.8 |

A separate lifecycle probe measured median ages of 61.5/60.5/68.9 ms and p95
ages of 108.5/83.7/118.8 ms for RGB/depth/cloud respectively. These short samples
include ROS delivery and subscriber scheduling, but not RViz rendering. They
are neither a latency guarantee nor a before/after comparison against the
original driver. Driver capture-to-conversion diagnostics exclude those ROS
delivery costs and therefore report different values.

To rerun the repository's live tests after building and sourcing the packages,
with the camera free and reachable, run from the repository root:

```bash
export DEPTHAI_TEST_DEVICE_IP=10.2.2.43
export ROS_DOMAIN_ID=183
python3 -m launch_testing.launch_test depthai_ros_driver/test/test_driver.py
python3 -m launch_testing.launch_test depthai_ros_driver/test/test_image_publication.py
```

The optional live-test CMake timeout is 300 seconds to accommodate multiple PoE
reconnections. Helpers wait on lifecycle state, bound service calls, use
sensor-data subscriptions and deactivate before changing initialization settings.
Local run artifacts are in `/tmp/depthai-modernization-review/`: `live-driver.xml`,
`live-publication.xml`, `live-results.json`, `launch-live-results.json`,
`telemetry-live-results.json` and their corresponding logs. The lifecycle/launch
probes and exact launch YAML are saved there as well; this temporary directory
is not part of the repository.

### Remaining hardware boundary

Physical link-loss recovery, RViz motion-to-display latency, thermal/ToF/RVC4,
and simultaneous physical multi-camera operation remain hardware acceptance
items. Synthetic message and namespaced-component tests cover their shared ROS
interfaces, not those device-specific pipelines.
