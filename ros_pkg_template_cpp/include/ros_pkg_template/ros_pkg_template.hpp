// Copyright 2021-2025
//
// @author: Jose Carlos Garcia
// @email: jcarlos3094@gmail.com
//

#ifndef ROS_PKG_TEMPLATE_NODE_HPP_
#define ROS_PKG_TEMPLATE_NODE_HPP_

#include <ros/ros.h>
#include "nodelet/nodelet.h"
#include <dynamic_reconfigure/server.h>
#include <ros_pkg_template_cpp/DynConfConfig.h>
#include "std_msgs/Float64.h"
#include "std_srvs/SetBool.h"
#include "ros_pkg_template/algorithm.hpp"

namespace ros_pkg_template
{
/*!
 * Main class for the node to handle the ROS2 interfacing.
 */
class RosPkgTemplate : public nodelet::Nodelet
{
public:
  /*!
    * Constructor.
    * @param nodeOptions the ROS2 node options.
    */
    RosPkgTemplate();

  /*!
    * Destructor.
    */
  virtual ~RosPkgTemplate();
  
private:
  //! ROS node handles
  ros::NodeHandle nodeHandle_;
  ros::NodeHandle privateNodeHandle_;

  //! Parameter
  bool param;
  ros_pkg_template_cpp::DynConfConfig currentConfig;

  //! Algorithm computation object
  Algorithm algorithm_;

  //! Interfaces definition
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  
  //! Dynamic reconfigure interfaces definition
  boost::recursive_mutex dynReconfServerMutex;  // To avoid Dynamic Reconfigure Server warning
  boost::shared_ptr<dynamic_reconfigure::Server<ros_pkg_template_cpp::DynConfConfig>> dynReconfServer_;

  /*!
   * Nodelet initialization virtual function
   */
  virtual void onInit();

 /*!
  * Callback from dynamic reconfigure server.
  * @param config New configuration.
  * @param level Level, which is result of ORing together all of level values
  */
  void dyn_reconf_callback(ros_pkg_template_cpp::DynConfConfig &config, uint32_t level);
  
  /*!
  * Subscriptor to float data.
  * @param msg the received message.
  */
  void callback(const std_msgs::Float64ConstPtr &msg);
};
} // namespace ros_pkg_template

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ros_pkg_template::RosPkgTemplate, nodelet::Nodelet);

#endif  // ROS_PKG_TEMPLATE_NODE_HPP_
