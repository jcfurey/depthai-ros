"""Live regression coverage; enabled with TEST_DEPTHAI_ROS_DRIVER."""

import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_srvs.srv import Trigger


@pytest.mark.rostest
def generate_test_description():
    container = ComposableNodeContainer(
        name="publication_test_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="depthai_ros_driver",
                plugin="depthai_ros_driver::Driver",
                name="oak",
                parameters=[{
                    "driver.i_enable_ir": False,
                    "driver.i_ip": os.environ.get("DEPTHAI_TEST_DEVICE_IP", ""),
                    "driver.i_publish_tf_from_calibration": False,
                    "pipeline_gen.i_pipeline_type": "RGBD",
                    "pipeline_gen.i_nn_type": "none",
                    "rgb.i_low_bandwidth": True,
                    "rgb.i_synced": True,
                    "stereo.i_synced": True,
                    "stereo.i_depth_preset": "DEFAULT",
                    "stereo.i_low_bandwidth": False,
                }],
            ),
        ],
        output="both",
    )
    return launch.LaunchDescription([
        container, launch_testing.actions.ReadyToTest(),
    ])


class TestImagePublication(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node("publication_test_probe")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()

    def wait(self, predicate, timeout=90):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.1)
        self.assertTrue(predicate(), "Timed out waiting for driver response/data")

    def call(self, srv_type, suffix, request):
        client = self.node.create_client(srv_type, "/oak/" + suffix)
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=90))
            future = client.call_async(request)
            self.wait(future.done)
            return future.result()
        finally:
            self.node.destroy_client(client)

    def trigger(self, name):
        result = self.call(Trigger, name, Trigger.Request())
        self.assertTrue(result.success, result.message)

    def configure(self, **params):
        request = SetParametersAtomically.Request(parameters=[
            Parameter(name, value=value).to_parameter_msg()
            for name, value in params.items()
        ])
        result = self.call(SetParametersAtomically, "set_parameters_atomically", request)
        self.assertTrue(result.result.successful, result.result.reason)

    def receive(self, specs):
        messages = {suffix: [] for _, suffix in specs}
        subscriptions = [
            self.node.create_subscription(
                kind, "/oak/" + suffix,
                lambda msg, key=suffix: messages[key].append(msg),
                qos_profile_sensor_data,
            )
            for kind, suffix in specs
        ]
        try:
            self.wait(lambda: all(len(values) >= 5 for values in messages.values()))
            return messages
        finally:
            for subscription in subscriptions:
                self.node.destroy_subscription(subscription)

    def test_streaming_restart_and_invalid_configuration(self):
        images = [(Image, "rgb/image_raw"), (Image, "stereo/image_raw")]
        self.receive(images)
        self.trigger("stop")
        self.trigger("start")
        self.receive(images)

        self.trigger("stop")
        self.configure(**{"rgb.i_publish_compressed": True})
        self.trigger("start")
        # Info-only subscribers must continue to receive calibration updates.
        self.receive([(CameraInfo, "rgb/camera_info")])
        self.receive([(CompressedImage, "rgb/image_raw/compressed")])

        self.trigger("stop")
        self.configure(**{"rgb.i_low_bandwidth": False})
        result = self.call(Trigger, "start", Trigger.Request())
        self.assertFalse(result.success)
        self.assertIn("i_publish_compressed requires", result.message)
        self.configure(**{"rgb.i_low_bandwidth": True})
        self.trigger("start")
        self.receive([(CompressedImage, "rgb/image_raw/compressed")])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
