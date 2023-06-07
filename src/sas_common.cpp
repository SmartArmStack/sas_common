/*
# Copyright (c) 2016-2023 Murilo Marques Marinho
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
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################*/
#include <sas_common/sas_common.hpp>

namespace sas
{

/**
 * @brief display_signal_handler_none_bug_info Add this before the "return 0" for any node that does not have
 * a sigint handler attached. This is just to prevent future users to get confused by the rclcpp error.
 * @param node an active node, otherwise nothing will be printed.
 */
void display_signal_handler_none_bug_info(std::shared_ptr<rclcpp::Node> &node)
{
    if(node)
    {
        //The [rclcpp] error is a known issue in Humble, possibly already fixed in rolling.
        //https://github.com/ros2/rclcpp/pull/2019
        RCLCPP_WARN_STREAM_ONCE(node->get_logger(), " ");
        RCLCPP_WARN_STREAM_ONCE(node->get_logger(), " ");
        RCLCPP_WARN_STREAM_ONCE(node->get_logger(), "::Ignore the [rclcpp] error below, it is a bug in ROS 2 Humble.");
    }
}

}
