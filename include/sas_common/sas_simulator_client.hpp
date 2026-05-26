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

    SimulatorClient(const std::shared_ptr<Node> &node,
                    const std::string topic_prefix="GET_FROM_NODE");

    bool start_simulation();
    bool stop_simulation();

    bool is_enabled() const;
    std::string get_topic_prefix() const;
};

}