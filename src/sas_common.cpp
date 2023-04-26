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
        RCLCPP_WARN_STREAM_ONCE(node->get_logger(), "::Ignore the [rclcpp] error below, it is a bug in ROS 2 Humble.");
    }
}

namespace common
{

/**
 * @brief ros2_spin_thread_function The function for the ros2_spin_thread thread.
 * @param node the initialized node in this namespace.
 */
/*
void ros2_spin_thread_function(const std::shared_ptr<rclcpp::Node>& node)
{
    rclcpp::spin(node);
}
*/

/**
 * @brief get_static_node of rclcpp. The intended use are for the Python-wrapped classes of sas,
 * so it shouldn't be used for anything running in rclcpp directly.
 * @return a reference to the global rclcpp Node.
 */
/*
std::shared_ptr<rclcpp::Node> get_static_node()
{
    std::lock_guard<std::mutex> lock(node_mutex);

    if(!rclcpp::ok())
    {
        int argc = 0;
        char** argv = nullptr;
        //std::cout << "Initializing rclcpp" << std::endl;
        //rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
        rclcpp::init(argc,argv);
        //std::cout << "rclcpp initialized." << std::endl;
    }
    if(!node_manager)
    {
        //std::cout << "Initializing node." << std::endl;
        auto microsecondsUTC = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        //node = std::make_shared<rclcpp::Node>(std::string("anonymous_node__")+std::to_string(microsecondsUTC));

        auto node = std::make_shared<rclcpp::Node>(std::string("anonymous_node__")+std::to_string(microsecondsUTC));
        node_manager = std::make_unique<SingletonNodeManager>(node);

        //std::cout << "Node initialized with name " << node->get_name() << " ." << std::endl;
        //std::cout << "Creating spin thread." << std::endl;
        ros2_spin_thread = std::thread(ros2_spin_thread_function,node_manager->node_);
        //std::cout << "Created spin thread" << std::endl;
    }
    return node_manager->node_;
}

SingletonNodeManager::SingletonNodeManager(const std::shared_ptr<rclcpp::Node> &node):
    node_(node)
{

}


SingletonNodeManager::~SingletonNodeManager()
{
    //std::cout << "sas_common::Shutting down [rclcpp]."<< std::endl;
    if (rclcpp::ok())
    {
        //std::cout << "rclcpp::ok() [rclcpp]."<< std::endl;
        rclcpp::shutdown();
        //std::cout << "rclcpp::shutdown() [rclcpp]."<< std::endl;
        if(ros2_spin_thread.joinable())
            ros2_spin_thread.join();
        //std::cout << "rclcpp::spin() [rclcpp]."<< std::endl;
    }
    //std::cout << "sas_common::Shut down [rclcpp] and spin thread joined."<< std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
}
*/
}
}
