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

#include <dqrobotics/DQ.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sas_core/sas_object.hpp>

using namespace rclcpp;
using namespace DQ_robotics;

namespace sas
{

/**
 * @brief Client wrapper for object pose.
 *
 * The ObjectClient connects to a ROS node to subscribe and publish
 * geometry_msgs::msg::PoseStamped messages representing an object's pose.
 * Internally the pose is stored using DQ_robotics::DQ. The class exposes
 * simple methods to send and retrieve the pose as well as to query
 * configuration such as the topic prefix and enabled state.
 */
class ObjectClient: private sas::Object
{
private:
    std::shared_ptr<Node> node_;

    std::atomic_bool enabled_;
    std::string topic_prefix_;

    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_pose_;
    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_;
    DQ pose_;

    void _callback_pose(const geometry_msgs::msg::PoseStamped& msg);
public:
    ObjectClient() = delete;
    ObjectClient(const ObjectClient&) = delete;

    /**
     * @brief Construct a new ObjectClient
     *
     * @param node Shared pointer to the rclcpp::Node used for ROS communications.
     * @param topic_prefix Topic prefix used for publisher/subscriber names. Defaults to "GET_FROM_NODE".
     */
    ObjectClient(const std::shared_ptr<Node> &node,
                 const std::string topic_prefix="GET_FROM_NODE");

    /**
     * @brief Publish the provided pose to the configured topic.
     *
     * This will publish the given DQ pose using the internal publisher.
     *
     * @param pose The pose to send (DQ representation).
     */
    void send_pose(const DQ& pose) const;

    /**
     * @brief Get the last received pose.
     *
     * @return DQ The last pose received via the subscription.
     * @throws std::runtime_error if no pose has been received (client uninitialized).
     */
    DQ get_pose() const;

    /**
     * @brief Query whether the client is enabled.
     *
     * @return true if enabled, false otherwise.
     */
    bool is_enabled() const;

    /**
     * @brief Get the configured topic prefix.
     *
     * @return std::string The topic prefix used by this client.
     */
    std::string get_topic_prefix() const;
};

}