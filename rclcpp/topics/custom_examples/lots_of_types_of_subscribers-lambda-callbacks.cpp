// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/char.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/byte.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      [this](std_msgs::msg::String::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      });
    this->create_subscription<std_msgs::msg::Char>(
      "topic",
      10,
      [this](std_msgs::msg::Char::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: Char");
      });
    this->create_subscription<std_msgs::msg::Int64>(
      "topic",
      10,
      [this](std_msgs::msg::Int64::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: Int64");
      });
    this->create_subscription<std_msgs::msg::Int32>(
      "topic",
      10,
      [this](std_msgs::msg::Int32::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: Int32");
      });
    this->create_subscription<std_msgs::msg::UInt32>(
      "topic",
      10,
      [this](std_msgs::msg::UInt32::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: UInt32");
      });
    this->create_subscription<std_msgs::msg::Bool>(
      "topic",
      10,
      [this](std_msgs::msg::Bool::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: Bool");
      });
    this->create_subscription<std_msgs::msg::UInt8>(
      "topic",
      10,
      [this](std_msgs::msg::UInt8::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: UInt8");
      });
    this->create_subscription<std_msgs::msg::ColorRGBA>(
      "topic",
      10,
      [this](std_msgs::msg::ColorRGBA::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: ColorRGBA");
      });
    this->create_subscription<std_msgs::msg::Float64>(
      "topic",
      10,
      [this](std_msgs::msg::Float64::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: Float64");
      });
    this->create_subscription<std_msgs::msg::Byte>(
      "topic",
      10,
      [this](std_msgs::msg::Byte::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: Byte");
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
