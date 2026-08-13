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

#include <sas_common/sas_object_client.hpp>
#include <sas_conversions/sas_conversions.hpp>
using std::placeholders::_1;

namespace sas
{

void ObjectClient::_callback_pose(const geometry_msgs::msg::PoseStamped& msg)
{
    pose_ = geometry_msgs_pose_stamped_to_dq(msg);
}


ObjectClient::ObjectClient(const std::shared_ptr<Node> &node,
                           const std::string topic_prefix):
    sas::Object("sas::ObjectClient"),
    node_(node),
    topic_prefix_(topic_prefix == "GET_FROM_NODE"? node->get_name() : topic_prefix),
    pose_(0)
{
    RCLCPP_INFO_STREAM(node_->get_logger(),"::Initializing ObjectClient with prefix " + topic_prefix_);

    publisher_pose_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic_prefix + "/set/pose",1);

    subscriber_pose_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
                topic_prefix + "/get/pose", 1, std::bind(&ObjectClient::_callback_pose, this, _1)
                );
}

void ObjectClient::send_pose(const DQ& pose) const
{
    publisher_pose_->publish(dq_to_geometry_msgs_pose_stamped(pose));
}

DQ ObjectClient::get_pose() const
{
    if(is_enabled())
        return pose_;
    else
        throw std::runtime_error(topic_prefix_ + "::ObjectClient::get_pose()::trying to get pose but uninitialized.");
}

bool ObjectClient::is_enabled() const
{
    return is_unit(pose_);
}

std::string ObjectClient::get_topic_prefix() const
{
    return topic_prefix_;
}


}