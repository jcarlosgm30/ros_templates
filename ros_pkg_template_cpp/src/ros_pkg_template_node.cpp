// Copyright 2021-2025
//
// @author: Jose Carlos Garcia
// @email: jcarlos3094@gmail.com
//

#include <memory>
#include "ros_pkg_template/ros_pkg_template.hpp"
#include "rclcpp/rclcpp.hpp"
#include <lifecycle_msgs/msg/state.hpp>

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    bool node_started = false;
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto ros_pkg_template_node = std::make_shared<RosPkgTemplate>(options);
    exec.add_node(ros_pkg_template_node->get_node_base_interface());
    auto node_configure_state = ros_pkg_template_node->configure();
    if (node_configure_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        auto node_activate_state = ros_pkg_template_node->activate();
        if (node_activate_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            node_started = true;
        }
    }
    if (node_started) {
        exec.spin();
    }
   rclcpp::shutdown();
   return 0;
}