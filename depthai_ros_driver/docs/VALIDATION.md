# Local modernization validation

Run on 2026-09-22 against ROS 2 Lyrical and workspace DepthAI 3.10.0 on Linux,
starting from `cam-dev` revision `be0dc9b`. CI was not changed.

## Software results

| Package | Test cases | CTest suites | Result |
| --- | ---: | ---: | --- |
| `depthai_bridge` | 62 | 14 | Pass |
| `depthai_ros_driver` | 47 | 6 | Pass |
| `depthai_filters` | 5 | 1 | Pass |
| `depthai_examples` | 5 | 1 | Pass |
| **Total** | **119** | **22** | **Pass** |

* Optimized builds of the bridge, driver, filters and all 14 configured examples
  succeeded. Message and description packages were rebuilt/installed as well.
* Bridge, driver, filter and example regression tests passed. Coverage includes
  disparity bytes/units/subpixel scaling; metric/fractional detections; covariance
  sizes; injected ROS clocks and exposure metadata; callback destruction and
  custom contexts; compressed-only and camera-info-only subscribers; effective
  QoS overrides; matching odometry/TF capture stamps; malformed images; real
  best-effort delivery; metric WLS/markers; and thermal processing without a
  display server.
* Driver tests exercise lifecycle transitions, errors/retry, namespaced services,
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

## Hardware boundary

The OAK-D was disconnected; the user confirmed this and requested completion of
software validation. SDK discovery returned no cameras, and `10.2.2.43` did not
respond. No new live camera, RViz latency, physical link-loss, or reconnect
measurement is claimed. The existing 640×400/15 FPS PoE preset was retained.

When hardware is available, run configure/activate/deactivate/reactivate with that
preset, verify no image output while inactive, compare subscriber p50/p95 frame
age and throughput under the same RGB/depth/cloud workload, and exercise device
loss/recovery. Thermal/ToF/RVC4 and simultaneous physical multi-camera operation
also remain hardware acceptance items. Synthetic message and namespaced-component
tests cover their shared ROS interfaces, not those device-specific pipelines.
