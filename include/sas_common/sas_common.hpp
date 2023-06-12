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

#include <type_traits>
#include <rclcpp/rclcpp.hpp>

namespace sas
{

template<typename T>
/**
 * @brief get_ros_parameter a wrapper of Node->get_parameter to throw a descriptive exception if the parameter was not found.
 * @param node[in] the relevant rclcpp::Node.
 * @param parameter_name[in] a std::string with the parameter name.
 * @param t[out] the reference for the variable that will store the parameter.
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

template<typename T>
/**
 * @brief get_type_name A slightly prettier version of "typeid(T).name()" for the types
 * currently in need in get_ros_parameter.
 * @return A string representation of the input if defined, returns an empty string otherwise.
 */
std::string get_type_name(const std::vector<T>&)
{
    if constexpr(std::is_same_v<T,std::string>)
    {
        return "std::vector<std::string>";
    }
    if constexpr(std::is_same_v<T,int64_t>)
    {
        return "std::vector<int64_t>";
    }
    if constexpr(std::is_same_v<T,double>)
    {
        return "std::vector<double>";
    }
    if constexpr(std::is_same_v<T,bool>)
    {
        return "std::vector<bool>";
    }
    return "";
}

template<typename T>
/**
 * @brief get_ros_parameter a wrapper of Node->get_parameter to throw an exception of the parameter was not found.
 * This is a specialization for std::vector<T>, to consider empty sequences, because they are not handled in ROS2
 *   See:
 *   1. https://answers.ros.org/question/396556/what-is-best-practice-for-parameters-which-are-empty-lists-in-ros2/
 *   2. https://docs.ros2.org/foxy/api/rclpy/api/parameters.html
 *   3. https://design.ros2.org/articles/ros_parameters.html
 *
 * @param node[in] the relevant rclcpp::Node.
 * @param parameter_name[in] a std::string with the parameter name.
 * @param v[out] the reference for the std::vector<T> that will store the parameter.
 *
 */
void get_ros_parameter(std::shared_ptr<rclcpp::Node>& node, const std::string& parameter_name, std::vector<T>& v)
{
    if(!node->has_parameter(parameter_name))
    {
        try
        {
            node->declare_parameter<std::vector<T>>(parameter_name);
        }
        catch (const rclcpp::exceptions::InvalidParameterTypeException& e)
        {
            RCLCPP_INFO_STREAM(
                        node->get_logger(),
                        "sas::get_ros_parameter: List "
                        << parameter_name
                        << " does not have expected type, a.k.a "
                        << get_type_name(v)
                        << ", in the parameter server, checking for empty list...");
            node->declare_parameter<std::vector<std::string>>(parameter_name);
        }
    }

    if(node->get_parameter(parameter_name).get_type() == rclcpp::PARAMETER_STRING_ARRAY)
    {
        auto v_checker = node->get_parameter(parameter_name).as_string_array();
        if(v_checker.size() == 1 && v_checker.at(0) == "EMPTY_LIST")
        {
            RCLCPP_INFO_STREAM(
                        node->get_logger(),
                        "sas::get_ros_parameter: List "
                        << parameter_name
                        << " was loaded as an EMPTY list of type "
                        << get_type_name(v)
                        << ".");
            v = std::vector<T>();
            return;
        }
        else
        {
            if constexpr (not std::is_same_v<T,std::string>)
            {
                throw std::runtime_error("::Was this list, "
                                         + parameter_name
                                         + ", supposed to be empty? \n "
                                           "It should be a list such as list:[\"EMPTY_LIST\"].");
            }
        }
    }
    if(!node->get_parameter(parameter_name,v))
    {
        throw std::runtime_error("::Unable to get parameter " + parameter_name);
    }
}

void display_signal_handler_none_bug_info(std::shared_ptr<rclcpp::Node>& node);

}
