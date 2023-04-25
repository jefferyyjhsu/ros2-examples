cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_custom_examples)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(minimal_subscriber minimal_subscriber.cpp)
ament_target_dependencies(minimal_subscriber rclcpp std_msgs)

add_executable(minimal_subscriber-without-create_subsciption minimal_subscriber-without-create_subsciption.cpp)
ament_target_dependencies(minimal_subscriber-without-create_subsciption rclcpp std_msgs)

add_executable(minimal_subscriber-without-create_subsciption_and_withtout_subsciber_ptr minimal_subscriber-without-create_subsciption_and_withtout_subsciber_ptr.cpp)
ament_target_dependencies(minimal_subscriber-without-create_subsciption_and_withtout_subsciber_ptr rclcpp std_msgs)

add_executable(minimal_subscriber-without-std_msg_include minimal_subscriber-without-std_msg_include.cpp)
ament_target_dependencies(minimal_subscriber-without-std_msg_include rclcpp)

add_executable(lots_of_subscribers lots_of_subscribers.cpp)
ament_target_dependencies(lots_of_subscribers rclcpp std_msgs)

add_executable(lots_of_subscribers-direct-include lots_of_subscribers-direct-include.cpp)
ament_target_dependencies(lots_of_subscribers-direct-include rclcpp std_msgs)

add_executable(lots_of_subscribers-lambda-callbacks lots_of_subscribers-lambda-callbacks.cpp)
ament_target_dependencies(lots_of_subscribers-lambda-callbacks rclcpp std_msgs)

add_executable(lots_of_types_of_subscribers-lambda-callbacks lots_of_types_of_subscribers-lambda-callbacks.cpp)
ament_target_dependencies(lots_of_types_of_subscribers-lambda-callbacks rclcpp std_msgs)

install(TARGETS
  minimal_subscriber
  minimal_subscriber-without-create_subsciption
  minimal_subscriber-without-create_subsciption_and_withtout_subsciber_ptr
  minimal_subscriber-without-std_msg_include
  lots_of_subscribers
  lots_of_subscribers-direct-include
  lots_of_subscribers-lambda-callbacks
  lots_of_types_of_subscribers-lambda-callbacks
  DESTINATION lib/${PROJECT_NAME})


ament_package()
