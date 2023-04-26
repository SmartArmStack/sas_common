#pragma once
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

#include <rclcpp/rclcpp.hpp>

namespace sas
{

template<class T>
/**
 * @brief get_ros_parameter a wrapper of Node->get_parameter to throw an exception of the parameter was not found.
 * @param node the relevant rclcpp::Node.
 * @param parameter_name a std::string with the parameter name.
 * @param t the reference for the variable that will store the parameter.
 * @param is_local_name if true, obtains the parameter relative to the node name, otherwise obtains the value from the absolute
 * namespace.
 */
void get_ros_parameter(std::shared_ptr<rclcpp::Node>& node, const std::string& parameter_name, T& t)
{
    if(!node->has_parameter(parameter_name))
        node->declare_parameter<T>(parameter_name);
    if(!node->get_parameter(parameter_name,t))
    {
        throw std::runtime_error("::Error loading " + parameter_name);
    }
}

void display_signal_handler_none_bug_info(std::shared_ptr<rclcpp::Node>& node);

// [2023/04/26] Murilo M. Marinho
// In ideal conditions, everything defined in the common namespace is supposed to allow us to have a static rclcpp::Node,
// that is automatically managed with a corresponding spin(). (similar to the one I made for ROS 1)
// However, because what I believe to be this bug here: https://github.com/ros2/rclcpp/issues/1766
// we get a segfault on shutdown. Instead of further wasting my time on this, I'm wrapping the least possible amount
// of rclcpp code in pybind11 so that my own bindings work, but that is not ideal given the additional complications in the Python code.
// Commented out since it is not used right now, but left here as a reminder to myself that I already tried to make this work.
/*
namespace common
{
static std::thread ros2_spin_thread;
static std::mutex node_mutex;
std::shared_ptr<rclcpp::Node> get_static_node();

struct SingletonNodeManager
{
    std::shared_ptr<rclcpp::Node> node_;

    SingletonNodeManager(const std::shared_ptr<rclcpp::Node>& node);

    ~SingletonNodeManager();
};

static std::unique_ptr<SingletonNodeManager> node_manager;
}
*/

}
