/*
# Copyright (c) 2016-2022 Murilo Marques Marinodeo
#
#    This file is part of sas_common.
#
#    sas_robot_driver is free software: you can redistribute it and/or modify
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
#include <sas_common/sas_common.hpp>

#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}

template<typename T>
void print_vector(const std::string& parameter_name,
                  const std::vector<T>& v)
{
    std::cout << parameter_name << ":";
    if(v.size()==0)
    {
        std::cout << "EMPTY_VECTOR" << std::endl;
    }
    else
    {
        std::cout << "[";
        for(const auto& a : v)
        {
            std::cout << a << " ";
        }
        std::cout << "]" << std::endl;
    }
}

int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error("::Error setting the signal int handler.");
    }

    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_ros_composer_node");

    try
    {
        std::vector<std::string> empty_string_vector;
        sas::get_ros_parameter(node,"empty_string_vector",empty_string_vector);
        print_vector("empty_string_vector",empty_string_vector);

        std::vector<int64_t> empty_integer_vector;
        sas::get_ros_parameter(node,"empty_integer_vector",empty_integer_vector);
        print_vector("empty_integer_vector",empty_integer_vector);

        std::vector<double> empty_double_vector;
        sas::get_ros_parameter(node,"empty_double_vector",empty_double_vector);
        print_vector("empty_double_vector",empty_double_vector);

        std::vector<bool> empty_bool_vector;
        sas::get_ros_parameter(node,"empty_bool_vector",empty_bool_vector);
        print_vector("empty_bool_vector",empty_bool_vector);

        std::vector<std::string> string_vector;
        sas::get_ros_parameter(node,"string_vector",string_vector);
        print_vector("string_vector",string_vector);

        std::vector<int64_t> integer_vector;
        sas::get_ros_parameter(node,"integer_vector",integer_vector);
        print_vector("integer_vector",integer_vector);

        std::vector<double> double_vector;
        sas::get_ros_parameter(node,"double_vector",double_vector);
        print_vector("double_vector",double_vector);

        std::vector<bool> bool_vector;
        sas::get_ros_parameter(node,"bool_vector",bool_vector);
        print_vector("bool_vector",bool_vector);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(),"Do not run this example directly, use the launch file 'sas_common_ros2_parameter_test_launch.py'.");
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(),"::Exception::" << e.what());
    }


    sas::display_signal_handler_none_bug_info(node);
    return 0;
}
