#pragma once
/*
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
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################
# Contributors:
#   ---
*/

#include <atomic>

#include<dqrobotics/DQ.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sas_core/sas_object.hpp>

using namespace rclcpp;
using namespace DQ_robotics;

namespace sas
{

class ObjectServer: private sas::Object
{
private:
    std::shared_ptr<Node> node_;

    std::atomic_bool enabled_;
    std::string topic_prefix_;

    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_pose_;
    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_;
    DQ get_target_pose_;

    void _callback_target_pose(const geometry_msgs::msg::PoseStamped& msg);
public:
    ObjectServer() = delete;
    ObjectServer(const ObjectServer&) = delete;

    ObjectServer(const std::shared_ptr<Node> &node,
                 const std::string topic_prefix="GET_FROM_NODE");

    void send_pose(const DQ& pose);
    DQ get_target_pose() const;

    bool is_enabled() const;
    std::string get_topic_prefix() const;
};

}