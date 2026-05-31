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

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sas_core/sas_object.hpp>

using namespace rclcpp;

namespace sas
{

/**
 * @brief Client for controlling a simulator via ROS services.
 *
 * The SimulatorClient provides a lightweight interface to call start/stop
 * simulation services (std_srvs::srv::Trigger) and to query configuration
 * such as whether the client is enabled and the topic/service prefix.
 */
class SimulatorClient: private sas::Object
{
private:
    std::shared_ptr<Node> node_;

    std::atomic_bool enabled_;
    std::string topic_prefix_;

    std::shared_ptr<Client<std_srvs::srv::Trigger>> service_client_start_simulation_;
    std::shared_ptr<Client<std_srvs::srv::Trigger>> service_client_stop_simulation_;

public:
    SimulatorClient() = delete;
    SimulatorClient(const SimulatorClient&) = delete;

    /**
     * @brief Construct a new SimulatorClient
     *
     * @param node Shared pointer to the rclcpp::Node used for ROS communications.
     * @param topic_prefix Topic/service prefix used to build service names. Defaults to "GET_FROM_NODE".
     */
    SimulatorClient(const std::shared_ptr<Node> &node,
                    const std::string topic_prefix="GET_FROM_NODE");

    /**
     * @brief Call the start simulation service.
     *
     * Invokes the configured start service and returns true if the call
     * succeeded and the service responded positively.
     *
     * @return true on successful start, false otherwise.
     */
    bool start_simulation();

    /**
     * @brief Call the stop simulation service.
     *
     * Invokes the configured stop service and returns true if the call
     * succeeded and the service responded positively.
     *
     * @return true on successful stop, false otherwise.
     */
    bool stop_simulation();

    /**
     * @brief Query whether the client is enabled.
     *
     * @return true if enabled, false otherwise.
     */
    bool is_enabled() const;

    /**
     * @brief Get the configured topic/service prefix.
     *
     * @return std::string The topic/service prefix used by this client.
     */
    std::string get_topic_prefix() const;
};

}