# Copyright (c) 2012-2026 Murilo Marques Marinho
#
#    This file is part of sas_common.
#
#    sas_common is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_common is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_common.  If not, see <https://www.gnu.org/licenses/>.
"""Tests for the ObjectClient, ObjectClientManager and SimulatorClient bindings.

The C++ peers (ObjectServer, SimulatorServer) are exercised from the other
side with plain rclpy counterparts: pose topics (geometry_msgs/PoseStamped)
and Trigger services (std_srvs/srv/Trigger).
"""

import time

import numpy as np
import pytest
import rclpy
from dqrobotics import DQ, vec8
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import ok, try_shutdown

from sas_common import (
    ObjectClient,
    ObjectClientManager,
    rclcpp_init,
    rclcpp_Node,
    rclcpp_shutdown,
    rclcpp_spin_some,
)

TIMEOUT = 15.0


def pose_dq(x, y, z):
    """Build the unit dual quaternion encoding the pure translation (x, y, z)."""
    return DQ([1.0, 0.0, 0.0, 0.0, 0.0, 0.5 * x, 0.5 * y, 0.5 * z])


@pytest.fixture(scope="module")
def rclpy_context():
    """Initialize rclpy once; rclcpp nodes join this process-wide context."""
    if not ok():
        rclpy.init()
    try:
        rclcpp_init()
        cpp_initialized = True
    except RuntimeError:
        cpp_initialized = False
    yield
    if cpp_initialized:
        rclcpp_shutdown()
    try_shutdown()


def _pump(cpp_node, py_node):
    rclcpp_spin_some(cpp_node)
    try:
        rclpy.spin_once(py_node, timeout_sec=0.0)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


def _wait_until(predicate, cpp_node, py_node, timeout=TIMEOUT):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        _pump(cpp_node, py_node)
        if predicate():
            return True
        time.sleep(0.005)
    return predicate()


class PoseRecorder(Node):
    """rclpy counterpart that records poses received on <prefix>/set/pose."""

    def __init__(self, prefix):
        super().__init__(node_name="pose_recorder")
        self.received = []
        self.pub = self.create_publisher(PoseStamped, f"{prefix}/get/pose", 1)
        self.create_subscription(PoseStamped, f"{prefix}/set/pose", self._cb, 1)

    def _cb(self, msg):
        self.received.append(msg)


def test_object_client_send_pose(rclpy_context):
    cpp_node = rclcpp_Node("sas_common_cpp_node")
    recorder = PoseRecorder("test_object_send")
    client = ObjectClient(cpp_node, "test_object_send")

    assert client.get_topic_prefix() == "test_object_send"

    def delivered():
        client.send_pose(pose_dq(1.0, 2.0, 3.0))
        return len(recorder.received) > 0

    assert _wait_until(delivered, cpp_node, recorder)

    msg = recorder.received[-1]
    assert np.allclose(
        [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
        [1.0, 2.0, 3.0],
        atol=1e-9,
    )
    assert abs(msg.pose.orientation.w - 1.0) < 1e-9


def test_object_client_get_pose(rclpy_context):
    cpp_node = rclcpp_Node("sas_common_cpp_node_get")
    recorder = PoseRecorder("test_object_get")
    client = ObjectClient(cpp_node, "test_object_get")

    msg = PoseStamped()
    msg.pose.position.x = 4.0
    msg.pose.position.y = 5.0
    msg.pose.position.z = 6.0
    msg.pose.orientation.w = 1.0

    def published():
        recorder.pub.publish(msg)
        try:
            client.get_pose()
        except RuntimeError:
            return False
        return True

    assert _wait_until(published, cpp_node, recorder)
    assert client.is_enabled()
    assert np.allclose(list(vec8(client.get_pose())),
                       list(vec8(pose_dq(4.0, 5.0, 6.0))),
                       atol=1e-9)


def test_object_client_get_pose_raises_when_uninitialized(rclpy_context):
    cpp_node = rclcpp_Node("sas_common_cpp_node_raise")
    client = ObjectClient(cpp_node, "test_object_uninit")
    with pytest.raises(RuntimeError):
        client.get_pose()


def test_object_client_manager(rclpy_context):
    cpp_node = rclcpp_Node("sas_common_cpp_node_mgr")
    mgr = ObjectClientManager(cpp_node)
    assert mgr.size() == 0

    mgr.add_client("arm")
    mgr.add_client("hand")
    assert mgr.size() == 2
    assert mgr.has_client("arm")
    assert not mgr.has_client("gripper")
    assert set(mgr.get_client_names()) == {"arm", "hand"}

    arm = mgr.get_client("arm")
    assert arm.get_topic_prefix() == "arm"

    assert mgr.remove_client("hand") is True
    assert mgr.remove_client("hand") is False
    assert mgr.size() == 1


