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
#include <pybind11/stl.h>
#include <rclcpp/rclcpp.hpp>

#include <sas_common/sas_object_client.hpp>
#include <sas_common/sas_object_client_manager.hpp>
#include <sas_common/sas_simulator_client.hpp>

namespace py = pybind11;
using OC = sas::ObjectClient;
using OCM = sas::ObjectClientManager;
using SC = sas::SimulatorClient;

PYBIND11_MODULE(_sas_common, m) {

    m.def("rclcpp_init", [](){rclcpp::init(0,nullptr,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);});
    m.def("rclcpp_shutdown", [](){rclcpp::shutdown();});
    m.def("rclcpp_spin_some", [](const rclcpp::Node::SharedPtr& node){rclcpp::spin_some(node);});

    py::class_<rclcpp::Node,std::shared_ptr<rclcpp::Node>>(m, "rclcpp_Node")
            .def(py::init<const std::string&>());

    py::class_<OC>(m, "ObjectClient")
            .def(py::init<const std::shared_ptr<rclcpp::Node>&, const std::string&>(),
                 py::arg("node"),
                 py::arg("topic_prefix") = "GET_FROM_NODE",
                 "Construct an ObjectClient bound to the provided node and topic prefix.")
            .def("send_pose",&OC::send_pose,
                 "Publish the provided pose (DQ) to the configured topic.")
            .def("get_pose",&OC::get_pose,
                 "Return the last received pose (DQ). Raises RuntimeError if no pose has been received and the client is uninitialized.")
            .def("is_enabled",&OC::is_enabled,"Returns true if the ObjectClient is enabled.")
            .def("get_topic_prefix",&OC::get_topic_prefix,
                 "Return the topic prefix configured for this client.");

     py::class_<SC>(m, "SimulatorClient")
            .def(py::init<const std::shared_ptr<rclcpp::Node>&, const std::string&>(),
                 py::arg("node"),
                 py::arg("topic_prefix") = "GET_FROM_NODE",
                 "Construct a SimulatorClient bound to the provided node and topic/service prefix.")
            .def("start_simulation",&SC::start_simulation,
                 "Call the start_simulation service; returns True on success.")
            .def("stop_simulation",&SC::stop_simulation,
                 "Call the stop_simulation service; returns True on success.")
            .def("is_enabled",&SC::is_enabled,"Returns true if the SimulatorClient is enabled.")
            .def("get_topic_prefix",&SC::get_topic_prefix,
                 "Return the topic/service prefix configured for this client.");

    py::class_<OCM>(m, "ObjectClientManager")
            .def(py::init<const std::shared_ptr<rclcpp::Node>&>(),
                 py::arg("node"),
                 "Construct an ObjectClientManager that manages multiple ObjectClient instances.")
            .def("add_client", &OCM::add_client,
                 py::arg("name"),
                 "Add (or replace) an ObjectClient identified by name.")
            .def("remove_client", &OCM::remove_client,
                 py::arg("name"),
                 "Remove an ObjectClient by name. Returns True if found and removed.")
            .def("get_client", [](OCM& self, const std::string& name) -> OC& {
                 return self.get_client(name);
             },
                 py::arg("name"),
                 py::return_value_policy::reference_internal,
                 "Return a reference to the managed ObjectClient named `name`.")
            .def("has_client", &OCM::has_client,
                 py::arg("name"),
                 "Return True if a client with the given name exists.")
            .def("get_client_names", &OCM::get_client_names,
                 "Return a list of all managed client names.")
            .def("size", &OCM::size,
                 "Return the number of managed clients.");
}
