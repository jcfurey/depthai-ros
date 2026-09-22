"""Bounded helpers for optional live-camera integration tests."""

import time

import rclpy
from lifecycle_msgs.srv import GetState
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger


class TestHelper:
    def __init__(self, node: Node, rsMode=False) -> None:
        self.node = node
        prefix = "/camera/camera" if rsMode else "/oak"
        self.stopSrv = node.create_client(Trigger, f"{prefix}/stop_driver")
        self.startSrv = node.create_client(Trigger, f"{prefix}/start_driver")
        self.stateSrv = node.create_client(GetState, f"{prefix}/get_state")
        self.setParamSrv = node.create_client(
            SetParametersAtomically, f"{prefix}/set_parameters_atomically"
        )
        self.getParamSrv = node.create_client(GetParameters, f"{prefix}/get_parameters")

    def _call(self, client, request, timeout=90):
        deadline = time.monotonic() + timeout
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f"Service unavailable: {client.srv_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=max(0.0, deadline - time.monotonic())
        )
        if not future.done():
            client.remove_pending_request(future)
            raise TimeoutError(f"Service timed out: {client.srv_name}")
        return future.result()

    def waitForDriverActive(self, timeout=90):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            result = self._call(
                self.stateSrv, GetState.Request(), deadline - time.monotonic()
            )
            if result.current_state.id == 3:
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return False

    def setParameters(self, params: list[Parameter], restart: bool = True) -> bool:
        # Initialization settings must be changed while the driver is inactive.
        if restart and not self._call(self.stopSrv, Trigger.Request()).success:
            return False
        result = self._call(
            self.setParamSrv, SetParametersAtomically.Request(parameters=params)
        ).result
        if not result.successful:
            self.node.get_logger().error(result.reason)
            return False
        return not restart or self._call(self.startSrv, Trigger.Request()).success

    def getParameter(self, param: str) -> ParameterValue:
        return self._call(
            self.getParamSrv, GetParameters.Request(names=[param])
        ).values[0]

    def restartDriver(self) -> bool:
        return (
            self._call(self.stopSrv, Trigger.Request()).success
            and self._call(self.startSrv, Trigger.Request()).success
        )

    def testIncomingMessages(
        self, msg_type, topic, callback=None, min_messages=30, timeout=10
    ):
        messages_received = 0

        def receive(message):
            nonlocal messages_received
            if callback is not None:
                callback(message)
            messages_received += 1

        sub = self.node.create_subscription(
            msg_type, topic, receive, qos_profile_sensor_data
        )
        try:
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if messages_received >= min_messages:
                    return True
            return False
        finally:
            self.node.destroy_subscription(sub)

    def testTriggerService(self, service_name, timeout=90) -> bool:
        client = self.node.create_client(Trigger, service_name)
        try:
            return self._call(client, Trigger.Request(), timeout).success
        finally:
            self.node.destroy_client(client)
