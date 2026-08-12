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

#include <sas_common/sas_object_client_manager.hpp>

namespace sas
{

ObjectClientManager::ObjectClientManager(const std::shared_ptr<rclcpp::Node>& node)
    : node_(node)
{
}

void ObjectClientManager::add_client(const std::string& name)
{
    clients_[name] = std::make_unique<ObjectClient>(node_, name);
}

bool ObjectClientManager::remove_client(const std::string& name)
{
    return clients_.erase(name) > 0;
}

ObjectClient& ObjectClientManager::get_client(const std::string& name)
{
    auto it = clients_.find(name);
    if (it == clients_.end())
    {
        throw std::runtime_error("ObjectClientManager::get_client: No client named \"" + name + "\"");
    }
    return *(it->second);
}

const ObjectClient& ObjectClientManager::get_client(const std::string& name) const
{
    auto it = clients_.find(name);
    if (it == clients_.end())
    {
        throw std::runtime_error("ObjectClientManager::get_client: No client named \"" + name + "\"");
    }
    return *(it->second);
}

bool ObjectClientManager::has_client(const std::string& name) const
{
    return clients_.find(name) != clients_.end();
}

std::vector<std::string> ObjectClientManager::get_client_names() const
{
    std::vector<std::string> names;
    names.reserve(clients_.size());
    for (const auto& pair : clients_)
    {
        names.push_back(pair.first);
    }
    return names;
}

size_t ObjectClientManager::size() const
{
    return clients_.size();
}

bool ObjectClientManager::are_all_clients_enabled() const
{
    for (const auto& pair : clients_)
    {
        if (!pair.second->is_enabled())
        {
            return false;
        }
    }
    return true;
}

}
