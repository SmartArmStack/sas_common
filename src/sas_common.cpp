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
#include <sas_common/sas_common.h>
#include <mutex>

namespace sas
{
namespace common
{

/**
 * @brief get_static_node_handle a version of roscpp_initializer without using boost.
 * @return a reference to the global NodeHandle. Should only be used for the Python wrappers of the sas Classes.
 * Inspired on: https://github.com/ros-planning/moveit/blob/master/moveit_ros/planning_interface/py_bindings_tools/src/roscpp_initializer.cpp
 */
ros::NodeHandle& get_static_node_handle()
{
    std::lock_guard<std::mutex> lock(node_handle_mutex);
    if(!ros::isInitialized())
    {
        //ROS_INFO_STREAM("sas_common::Initializing roscpp.");
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc,argv,"sas_common_nodehandle_dummy", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    }
    if(not node_handle)
    {
        //ROS_INFO_STREAM("sas_common::Initializing ros::NodeHandle.");
        node_handle = {new ros::NodeHandle(),
                       [](ros::NodeHandle* nh)
                       {
                           //std::cout << "sas_common::Shutdown roscpp."<< std::endl;
                           if (ros::isInitialized() && !ros::isShuttingDown())
                           {
                               async_spinner->stop();
                               ros::shutdown();
                           }
                           if(nh != nullptr)
                           {
                               delete nh;
                           }
                       }
                      };
        async_spinner.reset(new ros::AsyncSpinner(1));
        async_spinner->start();
    }
    return (*node_handle.get());
}
}
}
