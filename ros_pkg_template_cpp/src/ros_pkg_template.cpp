// Copyright 2021-2025
//
// @author: Jose Carlos Garcia
// @email: jcarlos3094@gmail.com
//
#include <chrono>
#include "ros_pkg_template/ros_pkg_template.hpp"

RosPkgTemplate::RosPkgTemplate(ros::NodeHandle& nodeHandle)
: nodeHandle_(nodeHandle)
{
    // Dynamic reconfigure parameters
    dynReconfServer_ =  boost::make_shared<dynamic_reconfigure::Server<ros_pkg_template_cpp::DynConfConfig>>(dynReconfServerMutex);
    dynamic_reconfigure::Server<ros_pkg_template_cpp::DynConfConfig>::CallbackType fDynReconfCb;
    fDynReconfCb = boost::bind(&RosPkgTemplate::dyn_reconf_callback, this, _1, _2);
    dynReconfServer_->setCallback(fDynReconfCb);

    // Update parameters
    ros_pkg_template_cpp::DynConfConfig currentConfig;
    dynReconfServer_->getConfigDefault(currentConfig);
    dynReconfServerMutex.lock();
    dynReconfServer_->updateConfig(currentConfig);
    dynReconfServerMutex.unlock();

    // Interfaces 
    subscriber_ = nodeHandle_.subscribe("input_float_topic",
                                        10,
                                        &RosPkgTemplate::callback, this);
    publisher_ = nodeHandle_.advertise<std_msgs::Float64>("output_float_topic",
                                                          10);
    ROS_INFO("Successfully launched node.");
}

RosPkgTemplate::~RosPkgTemplate()
{
}

void RosPkgTemplate::dyn_reconf_callback(ros_pkg_template_cpp::DynConfConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
            config.int_param, config.double_param, 
            config.str_param.c_str(), 
            config.bool_param?"True":"False", 
            config.size);
    bool config_accepted = true;
    if (config_accepted)
    {
        dynReconfServer_->getConfigDefault(currentConfig);
        dynReconfServerMutex.lock();
        dynReconfServer_->updateConfig(currentConfig);
        dynReconfServerMutex.unlock();
    }
}

void RosPkgTemplate::callback(const std_msgs::Float64ConstPtr &msg)
{
    ROS_INFO("Receiving preprocessed float data");
    try {
        algorithm_.addData(msg->data);
        if (algorithm_.getDataSize() > 100)
        {
            algorithm_.accReset();
            ROS_INFO("Accumulator reseting");
        }
        else
        {
            std_msgs::Float64 resultMsg;
            resultMsg.data = algorithm_.getAverage();
            publisher_.publish(resultMsg);
            ROS_INFO("Publishing average");
        }
    }
    catch(std::exception & e)
    {
        ROS_ERROR_STREAM(e.what());
    }
    
}