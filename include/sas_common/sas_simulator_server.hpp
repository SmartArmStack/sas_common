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
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sas_core/sas_object.hpp>

using namespace rclcpp;

namespace sas
{

/**
 * @brief Server exposing simulator control services.
 *
 * SimulatorServer provides ROS service servers for starting and stopping a
 * simulator (std_srvs::srv::Trigger). Callers can register callbacks that
 * will be executed when the corresponding service is invoked. The class also
 * exposes query methods for enabled state and topic/service prefix.
 */
class SimulatorServer: private sas::Object
{
private:
    std::shared_ptr<Node> node_;

    std::atomic_bool enabled_;
    std::string topic_prefix_;

    std::shared_ptr<Service<std_srvs::srv::Trigger>> service_server_start_simulation_;
    std::shared_ptr<Service<std_srvs::srv::Trigger>> service_server_stop_simulation_;

    std::function<void()> start_simulation_callback_;
    void start_simulation_callback_ros_(
          const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response>);

    std::function<void()> stop_simulation_callback_;
    void stop_simulation_callback_ros_(
          const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response>);

public:
    SimulatorServer() = delete;
    SimulatorServer(const SimulatorServer&) = delete;

    /**
     * @brief Construct a new SimulatorServer
     *
     * @param node Shared pointer to the rclcpp::Node used for ROS communications.
     * @param topic_prefix Topic/service prefix used to build service names. Defaults to "GET_FROM_NODE".
     */
    SimulatorServer(const std::shared_ptr<Node> &node,
                    const std::string topic_prefix="GET_FROM_NODE");

    /**
     * @brief Register a callback executed when the start service is called.
     *
     * @param cb Function invoked on start service requests.
     */
    void set_start_simulation_callback(const std::function<void()>&);

    /**
     * @brief Register a callback executed when the stop service is called.
     *
     * @param cb Function invoked on stop service requests.
     */
    void set_stop_simulation_callback(const std::function<void()>&);

    /**
     * @brief Query whether the server is enabled.
     *
     * @return true if enabled, false otherwise.
     */
    bool is_enabled() const;

    /**
     * @brief Get the configured topic/service prefix.
     *
     * @return std::string The topic/service prefix used by this server.
     */
    std::string get_topic_prefix() const;
};

}