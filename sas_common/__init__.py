"""
# Copyright (c) 2012-2026 Murilo Marques Marinho
#
#    This file is part of sas_robot_driver.
#
#    sas_robot_driver is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################

Package entry for the sas_common Python bindings.

This module re-exports the primary Python bindings provided by the
compiled extension module :mod:`sas_common._sas_common` for convenience.

Exports:
- rclcpp_init: Initialize rclcpp
- rclcpp_Node: Node wrapper exposed from the C++ extension
- rclcpp_spin_some: Spin the node briefly
- rclcpp_shutdown: Shutdown rclcpp
- ObjectClient, SimulatorClient: C++ client wrappers for object/simulator control
- ObjectClientManager: Manager for multiple ObjectClient instances

"""

from sas_common._sas_common import rclcpp_init, rclcpp_Node, rclcpp_spin_some, rclcpp_shutdown, ObjectClient, SimulatorClient, ObjectClientManager
