// Copyright 2021-2025
//
// @author: Jose Carlos Garcia
// @email: jcarlos3094@gmail.com
//

#ifndef ROS_PKG_TEMPLATE_NODE_HPP_
#define ROS_PKG_TEMPLATE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "ros_pkg_template/visibility.h"
#include "ros_pkg_template/algorithm.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/set_bool.hpp"

using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

/*!
 * Main class for the node to handle the ROS2 interfacing.
 */
class RosPkgTemplate : public rclcpp_lifecycle::LifecycleNode
{
public:
  /*!
   * Constructor.
   * @param nodeOptions the ROS2 node options.
   */
    ROS_PKG_TEMPLATE_PUBLIC RosPkgTemplate(const rclcpp::NodeOptions& options);
  
  /*!
   * On configure lifecycle state.
   * @param State Previous state.
   */
    LNI::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /*!
   * On activate lifecycle state.
   * @param State Previous lifecycle state.
   */
    LNI::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /*!
   * On deactivate lifecycle state.
   * @param State Previous lifecycle state.
   */
    LNI::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  
  /*!
   * On cleanup lifecycle state.
   * @param State Previous lifecycle state.
   */
    LNI::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  
  /*!
   * On shutdown lifecycle state.
   * @param State Previous lifecycle state.
   */
    LNI::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  
  /*!
   * On error lifecycle state.
   * @param State Previous lifecycle state.
   */
    LNI::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

private:
    //! Parameters
    rclcpp::Parameter param;
 
    //! Algorithm computation object
    Algorithm algorithm_;
 
    //! Interfaces definition
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
    
  /*!
   * Dynamic reconfiguration callback.
   * @param parameters Vector of node parameters.
   * @return Accept or not the change of parameters.
   */
    rcl_interfaces::msg::SetParametersResult dyn_reconf_callback(const std::vector<rclcpp::Parameter> & parameters);

  /*!
   * Subscriptor to float data.
   * @param msg the received message.
   */
    void callback(const std_msgs::msg::Float64::ConstSharedPtr msg);

  /*!
   * Request service method.
   * @param resp the response to the request message.
   * @return The response to the request message.
   */
    std_srvs::srv::SetBool::Response::ConstSharedPtr request_service(std_srvs::srv::SetBool::Response::ConstSharedPtr resp);
};

#endif  // ROS_PKG_TEMPLATE_NODE_HPP_
