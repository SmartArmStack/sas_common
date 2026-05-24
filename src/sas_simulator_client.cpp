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

#include <sas_common/sas_simulator_client.hpp>
#include <rclcpp/executor.hpp>

namespace sas
{

SimulatorClient::SimulatorClient(const std::shared_ptr<Node> &node,
                                 const std::string topic_prefix):
    sas::Object("sas::SimulatorClient"),
    node_(node),
    topic_prefix_(topic_prefix == "GET_FROM_NODE"? node->get_name() : topic_prefix)
{
    RCLCPP_INFO_STREAM(node_->get_logger(),"::Initializing SimulatorClient with prefix " + topic_prefix_);

    service_client_start_simulation_ = node_->create_client<std_srvs::srv::Trigger>("start_simulation");
    service_client_stop_simulation_ = node_->create_client<std_srvs::srv::Trigger>("stop_simulation");
}

bool SimulatorClient::start_simulation()
{
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = service_client_start_simulation_->async_send_request(request);
    return (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS);
}

bool SimulatorClient::stop_simulation()
{
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = service_client_stop_simulation_->async_send_request(request);
    return (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS);
}

bool SimulatorClient::is_enabled() const
{
    //TODO add check to see if clients are connected
    return true;
}

std::string SimulatorClient::get_topic_prefix() const
{
    return topic_prefix_;
}


}