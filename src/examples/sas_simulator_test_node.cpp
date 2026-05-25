/*
# Copyright (c) 2026 Murilo Marques Marinodeo
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
#   Author: Murilo M. Marinho, email: murilomarinodeo@ieee.org
#
# ################################################################*/
#include <exception>
#include <rclcpp/rclcpp.hpp>
#include <sas_core/sas_clock.hpp>
#include <sas_common/sas_simulator_client.hpp>
#include <sas_common/sas_simulator_server.hpp>


#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}


int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error("::Error setting the signal int handler.");
    }

    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("sas_simulator_test_node");
    auto clock = sas::Clock(0.001);

    try
    {
        auto simulator_client = sas::SimulatorClient(node);
        auto simulator_server = sas::SimulatorServer(node);
        auto f1 = [](){};
        auto f2 = [](){};
        simulator_server.set_start_simulation_callback(f1);
        simulator_server.set_stop_simulation_callback(f2);

        clock.init();
        while(!(simulator_client.is_enabled() &&
                simulator_server.is_enabled()))
        {
            clock.update_and_sleep();
        }

    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(),"::Exception::" << e.what());
    }


    return 0;
}
