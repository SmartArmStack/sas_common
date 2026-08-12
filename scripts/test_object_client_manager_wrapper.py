#!/usr/bin/python3
"""
# Copyright (c) 2026 Murilo Marques Marinho
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
#
# #######################################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# #######################################################################################
"""
"""Test script for the ObjectClientManager Python bindings.

This script demonstrates usage of the ObjectClientManager exposed via
the sas_common Python package. It initializes rclcpp, creates a node,
instantiates an ObjectClientManager, and exercises add/remove/get/list operations.

Run with: python3 test_object_client_manager_wrapper.py
"""
from dqrobotics import *
from sas_common import rclcpp_init, rclcpp_Node, rclcpp_spin_some, rclcpp_shutdown, ObjectClientManager

def main():
    """Run a basic smoke-test for the ObjectClientManager Python bindings."""
    try:
        rclcpp_init()
        node = rclcpp_Node("sas_common_object_client_manager_test_node")

        mgr = ObjectClientManager(node)
        print("Initial size:", mgr.size())

        mgr.add_client("arm")
        mgr.add_client("hand")
        print("Size after adding arm and hand:", mgr.size())

        print("Has arm:", mgr.has_client("arm"))
        print("Has hand:", mgr.has_client("hand"))
        print("Has gripper:", mgr.has_client("gripper"))
        print("Client names:", mgr.get_client_names())

        arm = mgr.get_client("arm")
        print("Arm enabled:", arm.is_enabled())
        print("Arm topic prefix:", arm.get_topic_prefix())

        x = DQ([1])
        arm.send_pose(x)

        removed = mgr.remove_client("hand")
        print("Removed hand:", removed)
        print("Size after removal:", mgr.size())
        print("Client names after removal:", mgr.get_client_names())

        rclcpp_spin_some(node)
        rclcpp_shutdown()
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print("Unhandled exception:", e)

if __name__ == '__main__':
    main()
