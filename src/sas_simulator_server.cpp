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

#include <sas_common/sas_simulator_server.hpp>
#include <rclcpp/executor.hpp>

using namespace std::placeholders;

namespace sas
{

SimulatorServer::SimulatorServer(const std::shared_ptr<Node> &node,
                                 const std::string topic_prefix):
    sas::Object("sas::SimulatorServer"),
    node_(node),
    topic_prefix_(topic_prefix == "GET_FROM_NODE"? node->get_name() : topic_prefix)
{
    RCLCPP_INFO_STREAM(node_->get_logger(),"::Initializing SimulatorServer with prefix " + topic_prefix_);

    service_server_start_simulation_ = node_->create_service<std_srvs::srv::Trigger>(
        topic_prefix_ + "/start_simulation",
        std::bind(&SimulatorServer::start_simulation_callback_ros_,
        this,
        _1,
        _2)
        );

    service_server_stop_simulation_ = node_->create_service<std_srvs::srv::Trigger>(
        topic_prefix_ + "/stop_simulation",
        std::bind(&SimulatorServer::stop_simulation_callback_ros_,
        this,
        _1,
        _2)
        );

}

void SimulatorServer::start_simulation_callback_ros_(
          const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
    if(!is_enabled())
        return;
    start_simulation_callback_();
    resp->success = true;
}

void SimulatorServer::stop_simulation_callback_ros_(
          const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
    if(!is_enabled())
        return;
    stop_simulation_callback_();
    resp->success = true;
}

void SimulatorServer::set_start_simulation_callback(const std::function<void()>& start_simulation_callback)
{
    start_simulation_callback_ = start_simulation_callback;
}
void SimulatorServer::set_stop_simulation_callback(const std::function<void()>& stop_simulation_callback)
{
    stop_simulation_callback_ = stop_simulation_callback;
}

bool SimulatorServer::is_enabled() const
{
    return !(start_simulation_callback_ == nullptr ||
             stop_simulation_callback_ == nullptr);
}

std::string SimulatorServer::get_topic_prefix() const
{
    return topic_prefix_;
}


}