// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/memory_strategy.hpp"
#include <memory>

using rclcpp::memory_strategy::MemoryStrategy;

rclcpp::SubscriptionBase::SharedPtr
MemoryStrategy::get_subscription_by_handle(
  std::shared_ptr<const rcl_subscription_t> subscriber_handle,
  const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  for (const auto & pair : weak_groups_to_nodes) {
    auto group = pair.first.lock();
    if (!group) {
      continue;
    }
    for (auto & weak_subscription : group->get_subscription_ptrs()) {
      auto subscription = weak_subscription.lock();
      if (subscription) {
        if (subscription->get_subscription_handle() == subscriber_handle) {
          return subscription;
        }
        if (subscription->get_intra_process_subscription_handle() == subscriber_handle) {
          return subscription;
        }
      }
    }
  }
  return nullptr;
}

rclcpp::ServiceBase::SharedPtr
MemoryStrategy::get_service_by_handle(
  std::shared_ptr<const rcl_service_t> service_handle,
  const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  for (const auto & pair : weak_groups_to_nodes) {
    auto group = pair.first.lock();
    if (!group) {
      continue;
    }
    for (auto & weak_service : group->get_service_ptrs()) {
      auto service = weak_service.lock();
      if (service && service->get_service_handle() == service_handle) {
        return service;
      }
    }
  }
  return nullptr;
}

rclcpp::ClientBase::SharedPtr
MemoryStrategy::get_client_by_handle(
  std::shared_ptr<const rcl_client_t> client_handle,
  const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  for (const auto & pair : weak_groups_to_nodes) {
    auto group = pair.first.lock();
    if (!group) {
      continue;
    }
    for (auto & weak_client : group->get_client_ptrs()) {
      auto client = weak_client.lock();
      if (client && client->get_client_handle() == client_handle) {
        return client;
      }
    }
  }
  return nullptr;
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
MemoryStrategy::get_node_by_group(
  rclcpp::callback_group::CallbackGroup::SharedPtr group,
  const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  if (!group) {
    return nullptr;
  }

  rclcpp::callback_group::CallbackGroup::WeakPtr weak_group_ptr(group);
  const auto finder = weak_groups_to_nodes.find(weak_group_ptr);
  if (finder != weak_groups_to_nodes.end()) {
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr = finder->second.lock();
    return node_ptr;
  }
  return nullptr;
}

rclcpp::callback_group::CallbackGroup::SharedPtr
MemoryStrategy::get_group_by_subscription(
  rclcpp::SubscriptionBase::SharedPtr subscription,
  const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  for (const auto & pair : weak_groups_to_nodes) {
    auto group = pair.first.lock();
    if (!group) {
      continue;
    }
    for (auto & weak_sub : group->get_subscription_ptrs()) {
      auto sub = weak_sub.lock();
      if (sub == subscription) {
        return group;
      }
    }
  }
  return nullptr;
}

rclcpp::callback_group::CallbackGroup::SharedPtr
MemoryStrategy::get_group_by_service(
  rclcpp::ServiceBase::SharedPtr service,
  const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  for (const auto & pair : weak_groups_to_nodes) {
    auto group = pair.first.lock();
    if (!group) {
      continue;
    }
    for (auto & weak_serv : group->get_service_ptrs()) {
      auto serv = weak_serv.lock();
      if (serv && serv == service) {
        return group;
      }
    }
  }
  return nullptr;
}

rclcpp::callback_group::CallbackGroup::SharedPtr
MemoryStrategy::get_group_by_client(
  rclcpp::ClientBase::SharedPtr client,
  const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  for (const auto & pair : weak_groups_to_nodes) {
    auto group = pair.first.lock();
    if (!group) {
      continue;
    }
    for (auto & weak_client : group->get_client_ptrs()) {
      auto cli = weak_client.lock();
      if (cli && cli == client) {
        return group;
      }
    }
  }
  return nullptr;
}
