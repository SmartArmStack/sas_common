// Copyright (c) 2012-2026 Murilo Marques Marinho
//
//    This file is part of sas_common.
//
//    sas_common is free software: you can redistribute it and/or modify
//    it under the terms of the GNU Lesser General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    sas_common is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU Lesser General Public License for more details.
//
//    You should have received a copy of the GNU Lesser General Public License
//    along with sas_common.  If not, see <https://www.gnu.org/licenses/>.
/**
 * @file test_object_simulator_servers.cpp
 * @brief gtests for the ObjectServer and SimulatorServer implementations,
 * exercised against their ObjectClient/SimulatorClient peers on a shared node.
 */
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_object_client.hpp>
#include <sas_common/sas_object_server.hpp>
#include <sas_common/sas_simulator_client.hpp>
#include <sas_common/sas_simulator_server.hpp>

namespace
{
constexpr int MAX_SPINS = 400;
constexpr auto SPIN_PERIOD = std::chrono::milliseconds(5);

void spin_until(rclcpp::Node::SharedPtr node, const std::function<bool()> &predicate)
{
  for (int i = 0; i < MAX_SPINS && !predicate(); ++i)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(SPIN_PERIOD);
  }
  rclcpp::spin_some(node);
}

DQ pure_translation_dq(double x, double y, double z)
{
  return DQ(1.0, 0.0, 0.0, 0.0, 0.0, 0.5*x, 0.5*y, 0.5*z);
}

void expect_dq_approx(const DQ &lhs, const DQ &rhs)
{
  for (unsigned i = 0; i < 8; ++i)
  {
    EXPECT_NEAR(lhs.vec8()(i), rhs.vec8()(i), 1e-9);
  }
}
}  // namespace

TEST(ObjectServerTest, RoundTripWithClient)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("gtest_object_node");
  sas::ObjectServer server(node, "gtest_object");
  sas::ObjectClient client(node, "gtest_object");

  EXPECT_EQ(server.get_topic_prefix(), "gtest_object");
  EXPECT_THROW(server.get_target_pose(), std::runtime_error);

  client.send_pose(pure_translation_dq(1.0, 2.0, 3.0));
  bool target_received = false;
  spin_until(node, [&]()
  {
    try
    {
      target_received = true;
      server.get_target_pose();
    }
    catch (const std::runtime_error &)
    {
      target_received = false;
    }
    return target_received;
  });
  ASSERT_TRUE(target_received);
  EXPECT_TRUE(server.is_enabled());
  expect_dq_approx(server.get_target_pose(), pure_translation_dq(1.0, 2.0, 3.0));

  server.send_pose(pure_translation_dq(4.0, 5.0, 6.0));
  bool pose_received = false;
  spin_until(node, [&]()
  {
    try
    {
      pose_received = true;
      client.get_pose();
    }
    catch (const std::runtime_error &)
    {
      pose_received = false;
    }
    return pose_received;
  });
  ASSERT_TRUE(pose_received);
  EXPECT_TRUE(client.is_enabled());
  expect_dq_approx(client.get_pose(), pure_translation_dq(4.0, 5.0, 6.0));

  rclcpp::shutdown();
}

TEST(SimulatorServerTest, CallbacksFireThroughClient)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("gtest_sim_node");
  sas::SimulatorServer server(node, "gtest_sim");
  sas::SimulatorClient client(node, "gtest_sim");

  EXPECT_EQ(server.get_topic_prefix(), "gtest_sim");

  std::atomic_int starts{0};
  std::atomic_int stops{0};
  server.set_start_simulation_callback([&]()
  {
    starts++;
  });
  server.set_stop_simulation_callback([&]()
  {
    stops++;
  });
  EXPECT_TRUE(server.is_enabled());

  bool start_ok = false;
  spin_until(node, [&]()
  {
    start_ok = client.start_simulation();
    return start_ok;
  });
  EXPECT_TRUE(start_ok);
  spin_until(node, [&]()
  {
    return starts.load() >= 1;
  });
  EXPECT_GE(starts.load(), 1);

  bool stop_ok = false;
  spin_until(node, [&]()
  {
    stop_ok = client.stop_simulation();
    return stop_ok;
  });
  EXPECT_TRUE(stop_ok);
  spin_until(node, [&]()
  {
    return stops.load() >= 1;
  });
  EXPECT_GE(stops.load(), 1);

  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
