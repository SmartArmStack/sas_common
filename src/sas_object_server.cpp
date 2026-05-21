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
#
#   -
#
*/

#include <sas_common/sas_object_server.hpp>
#include <sas_conversions/sas_conversions.hpp>
using std::placeholders::_1;

namespace sas
{

void ObjectServer::_callback_target_pose(const geometry_msgs::msg::PoseStamped& msg)
{
    target_pose_ = geometry_msgs_pose_stamped_to_dq(msg);
}


ObjectServer::ObjectServer(const std::shared_ptr<Node> &node,
                           const std::string topic_prefix):
    sas::Object("sas::ObjectServer"),
    node_(node),
    topic_prefix_(topic_prefix == "GET_FROM_NODE"? node->get_name() : topic_prefix),
    pose_(0)
{
    RCLCPP_INFO_STREAM(node_->get_logger(),"::Initializing ObjectServer with prefix " + topic_prefix_);

    publisher_pose_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic_prefix + "/get/pose",1);

    subscriber_pose_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
                topic_prefix + "/set/pose", 1, std::bind(&ObjectServer::_callback_pose, this, _1)
                );
}

void ObjectServer::send_pose(const DQ& pose)
{
    publisher_pose_->publish(dq_to_geometry_msgs_pose_stamped(pose));
}

DQ ObjectServer::get_target_pose() const
{
    if(is_enabled())
        return target_pose_;
    else
        throw std::runtime_error(topic_prefix_ + "::ObjectServer::get_target_pose()::trying to get pose but uninitialized.");
}

bool ObjectServer::is_enabled() const
{
    return is_unit(target_pose_);
}

std::string ObjectServer::get_topic_prefix() const
{
    return topic_prefix_;
}


}