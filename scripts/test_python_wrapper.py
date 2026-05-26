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
from dqrobotics import *
from sas_common import rclcpp_init, rclcpp_Node, rclcpp_spin_some, rclcpp_shutdown, ObjectClient, SimulatorClient

def main():
    try:
        rclcpp_init()
        node = rclcpp_Node("sas_common_test_python_wrapper_node")
        oc = ObjectClient(node, "test_topic")
        print(oc.is_enabled())
        x = DQ([1])
        oc.send_pose(x)
        sc = SimulatorClient(node, "test_topic")
        print(sc.is_enabled())
        rclcpp_spin_some(node)
        rclcpp_shutdown()
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print("Unhandled excepts", e)

if __name__ == '__main__':
    main()