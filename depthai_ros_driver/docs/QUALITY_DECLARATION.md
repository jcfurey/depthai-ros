# Quality and compatibility scope

This declaration covers the ROS packages on the development `cam-dev` branch,
not the vendored DepthAI SDK. It follows the topics in
[REP-2004](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-2004.rst)
without claiming a numbered quality level. Establishing a release-level claim
requires wider platform testing, coverage evidence and a maintained release
process; CI work is explicitly deferred for this change.

* **Version and compatibility:** ROS manifests and CMake use 3.4.0. This branch
  includes public C++ layout and corrected message-semantics changes. Rebuild
  consumers together and follow [the migration guide](ROS2_MODERNIZATION.md).
  Compatibility guarantees of an upstream binary release do not apply to this
  unreleased development snapshot.
* **Public interfaces:** installed bridge converter/publisher headers, driver
  pipeline-plugin interfaces, exported filter component headers, ROS messages,
  parameters, services, topics and documented launch arguments. Internal SDK
  objects and implementation details are not additional ROS API guarantees.
  Existing start/stop aliases and constructor entry points are retained where
  possible; corrected units and invalid-input rejection are intentional changes.
* **Target platform:** ROS 2 Lyrical, Linux, the Lyrical C++ toolchain and the workspace DepthAI 3.10.0
  SDK. Local hardware validation uses an RVC2 OAK-D-PRO-POE. Thermal, ToF, RVC4,
  multiple simultaneous physical cameras and other ROS distributions need their
  own acceptance runs; passing synthetic message tests does not certify them.
* **Verification:** local unit/component tests cover message units, malformed
  buffers, callback lifetime, clock policy, lifecycle services, parameter commit
  behavior, QoS, headless operation and launch configuration. Install-space
  consumers check exported interfaces. See [validation evidence](VALIDATION.md)
  for executed tests and measured performance. No coverage percentage or formal
  thread-safety certification is claimed.
* **Dependencies:** direct build/export/runtime/test dependencies are declared
  in package manifests. RTAB-Map remains an optional integration. No isolated
  operating-system image was provisioned solely from rosdep for this review.
* **Change review:** contract changes should update the migration guide and
  relevant regression tests. Release acceptance should rerun package tests and
  the affected hardware scenarios, review diagnostics and compare frame-age
  distributions against the same camera/subscriber workload.
* **Security and robustness:** image layouts and externally supplied geometry
  are validated before indexing. Xacro runs with literal argument vectors,
  bounded output/time and exit-status checks rather than shell evaluation.
  Device/network failures are surfaced through lifecycle results and diagnostics.
  This is not a comprehensive security assessment of ROS middleware or the SDK.
