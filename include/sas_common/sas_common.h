/*
# Copyright (c) 2022 Murilo Marques Marinho
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
#   Author: Murilo M. Marinho, email: murilo@g.ecc.u-tokyo.ac.jp
#
# ################################################################*/
#pragma once

#include <ros/ros.h>

namespace sas
{

template<class T>
/**
 * @brief get_ros_param a template version of ros::NodeHandle::getParam(), that has built-in throw.
 * @param nh a ros::NodeHandle.
 * @param name the name of the parameter.
 * @param t a reference to the value to be obtained, will be written by this function.
 * @param is_local_name true if the name is local (name will be prepended with ros::this_node::getName(), false if no prefix is needed.
 * @exception std::runtime_error if the parameter could not be loaded.
 */
void get_ros_param(ros::NodeHandle& nh, const std::string& name, T& t, const bool& is_local_name=true)
{
    const std::string prefix = is_local_name?"":ros::this_node::getName();
    if(!nh.getParam(prefix+name,t))
    {
        throw std::runtime_error(prefix + "::Error loading " + name);
    }
}

namespace common
{
static std::unique_ptr<ros::NodeHandle,std::function<void(ros::NodeHandle*)>> node_handle; //Used for the Python interfaces to make a Singleton node_handle
static std::unique_ptr<ros::AsyncSpinner> async_spinner;
static std::mutex node_handle_mutex;
ros::NodeHandle& get_static_node_handle();
}

}
