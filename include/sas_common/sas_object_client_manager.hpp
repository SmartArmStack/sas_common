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

#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sas_common/sas_object_client.hpp>

namespace sas
{

/**
 * @brief Manager for multiple ObjectClient instances.
 *
 * The ObjectClientManager maintains a named collection of ObjectClient
 * objects. Each client is identified by a unique string name, which also
 * serves as its topic prefix. Clients are created lazily from a shared
 * rclcpp::Node.
 */
class ObjectClientManager
{
private:
    std::shared_ptr<rclcpp::Node> node_;
    std::unordered_map<std::string, std::unique_ptr<ObjectClient>> clients_;

public:
    ObjectClientManager() = delete;
    ObjectClientManager(const ObjectClientManager&) = delete;
    ObjectClientManager& operator=(const ObjectClientManager&) = delete;

    /**
     * @brief Construct a new ObjectClientManager
     *
     * @param node Shared pointer to the rclcpp::Node used for all managed clients.
     */
    explicit ObjectClientManager(const std::shared_ptr<rclcpp::Node>& node);

    /**
     * @brief Add (or replace) an ObjectClient identified by a name.
     *
     * The name is used as the topic prefix for the client. If a client
     * with the same name already exists, it will be replaced.
     *
     * @param name Unique identifier and topic prefix for the client.
     */
    void add_client(const std::string& name);

    /**
     * @brief Remove an ObjectClient by name.
     *
     * @param name Name of the client to remove.
     * @return true if the client was found and removed, false otherwise.
     */
    bool remove_client(const std::string& name);

    /**
     * @brief Get a reference to an existing ObjectClient.
     *
     * @param name Name of the client.
     * @return ObjectClient& Reference to the managed client.
     * @throws std::runtime_error if no client with the given name exists.
     */
    ObjectClient& get_client(const std::string& name);

    /**
     * @brief Get a const reference to an existing ObjectClient.
     *
     * @param name Name of the client.
     * @return const ObjectClient& Reference to the managed client.
     * @throws std::runtime_error if no client with the given name exists.
     */
    const ObjectClient& get_client(const std::string& name) const;

    /**
     * @brief Check if a client with the given name exists.
     *
     * @param name Name to check.
     * @return true if a client with the name is managed, false otherwise.
     */
    bool has_client(const std::string& name) const;

    /**
     * @brief Get a list of all managed client names.
     *
     * @return std::vector<std::string> Vector of client names.
     */
    std::vector<std::string> get_client_names() const;

    /**
     * @brief Get the number of managed clients.
     *
     * @return size_t Number of clients currently managed.
     */
    size_t size() const;
};

}
