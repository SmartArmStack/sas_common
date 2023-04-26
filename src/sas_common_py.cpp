/*
# Copyright (c) 2016-2023 Murilo Marques Marinho
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
# ################################################################*/
#include <pybind11/pybind11.h>
#include <rclcpp/rclcpp.hpp>

namespace py = pybind11;

PYBIND11_MODULE(_sas_common, m) {

    m.def("rclcpp_init", [](){rclcpp::init(0,nullptr);});
    m.def("rclcpp_shutdown", [](){rclcpp::shutdown();});
    m.def("rclcpp_spin_some", [](const rclcpp::Node::SharedPtr& node){rclcpp::spin_some(node);});

    py::class_<rclcpp::Node,std::shared_ptr<rclcpp::Node>>(m, "rclcpp_Node")
            .def(py::init<const std::string&>());

}
