// Copyright 2022-2023 iRobot Corporation. All Rights Reserved.

#include <cstdlib>
#include <string>
#include <string_view>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <bloxx/logging/logging.h>
#include <resource-profiling/segment-profiler.hpp>

bool log_msg_va(const char *fmt, va_list vl)
{
    vfprintf(stdout, fmt, vl);
    fprintf(stdout, "\n");
    fflush(stdout);
    return true;
}

static std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> create_simple_nodes(int num_nodes, bool no_params)
{
    std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> nodes;

    {
        resource_profiling::SegmentProfiler profiler("rclcpp-init");
        auto log_on_exit = profiler.log_usage_on_exit();
        rclcpp::init(0, nullptr);
    }

    for (int i = 0; i < num_nodes; i++) {
        std::string node_name = "node_" + std::to_string(i);
        resource_profiling::SegmentProfiler profiler(node_name);
        auto log_on_exit = profiler.log_usage_on_exit();

        auto node_options = rclcpp::NodeOptions();
        if (no_params) {
            node_options.start_parameter_services(false);
            node_options.start_parameter_event_publisher(false);
        }

        auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, node_options);
        nodes.push_back(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return nodes;
}

int main(int argc, char* argv[])
{
    ERSP_LOG_DEBUG("Start!");

    int num_nodes = 5;
    if (argc > 1) {
        num_nodes = atoi(argv[1]);
    }

    using namespace std::literals;
    bool no_params = std::count(argv, argv + argc, "--no-params"sv);

    std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> nodes;

    {
        resource_profiling::SegmentProfiler profiler("everything");
        auto log_on_exit = profiler.log_usage_on_exit();
        nodes = create_simple_nodes(num_nodes, no_params);

        ERSP_LOG_DEBUG("Waiting!");
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    nodes.clear();

    rclcpp::shutdown();

    ERSP_LOG_DEBUG("Done!");

    return 0;
}
