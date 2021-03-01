// Copyright 2021-2025
//
// @author: Jose Carlos Garcia
// @email: jcarlos3094@gmail.com
//
#include <chrono>
#include "ros_pkg_template/ros_pkg_template.hpp"


using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using namespace std::placeholders;
using namespace std::chrono_literals;

RosPkgTemplate::RosPkgTemplate(const rclcpp::NodeOptions& options)
: rclcpp_lifecycle::LifecycleNode("ros_pkg_template", options)
{
    RCLCPP_INFO(this->get_logger(), "Obtaining node '%s' parameters ", this->get_name());
    //! Declare parameters
    this->declare_parameter<bool>("param", true);
    //! Get parameters
    this->param = this->get_parameter("param");
}

LNI::CallbackReturn RosPkgTemplate::on_configure(const rclcpp_lifecycle::State &state) {
    (void)state;
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
    try {
        // Define parameters callback
        this->set_on_parameters_set_callback(std::bind(&RosPkgTemplate::dyn_reconf_callback, this, _1));

        // Subscribers
        subscriber_ = this->create_subscription<std_msgs::msg::Float64>("input_float_topic",
                                                                   10,
                                                                   std::bind(&RosPkgTemplate::callback, this, _1));
        // Publishers
        publisher_ = create_publisher<std_msgs::msg::Float64>("output_float_topic", 10);

        // Client
        client_ = create_client<std_srvs::srv::SetBool>("get_obstacle_service");

    }
    catch(std::exception & e)
    {
        RCLCPP_ERROR(this->get_logger(), e.what());
        return LNI::CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(this->get_logger(), "Node '%s' configured", this->get_name());

    return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn RosPkgTemplate::on_activate(const rclcpp_lifecycle::State &state) {
    (void)state;
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
    try {
        publisher_->on_activate();
    }
    catch(std::exception & e)
    {
        RCLCPP_ERROR(this->get_logger(), e.what());
        return LNI::CallbackReturn::FAILURE;
    }
    return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn RosPkgTemplate::on_deactivate(const rclcpp_lifecycle::State &state) {
    (void)state;
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
    try {
        publisher_->on_deactivate();
    }
    catch(std::exception & e)
    {
        RCLCPP_ERROR(this->get_logger(), e.what());
        return LNI::CallbackReturn::FAILURE;
    }
    return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn RosPkgTemplate::on_cleanup(const rclcpp_lifecycle::State &state) {
    (void)state;
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup() is called.");
    try {
        this->param = this->get_parameter("param");
        publisher_.reset();
    }
    catch(std::exception & e)
    {
        RCLCPP_ERROR(this->get_logger(), e.what());
        return LNI::CallbackReturn::FAILURE;
    }
    return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn RosPkgTemplate::on_error(const rclcpp_lifecycle::State &state) {
    (void)state;
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_error is called from state %s.", state.label().c_str());

    try {
        this->param = this->get_parameter("param");
        publisher_.reset();
    }
    catch(std::exception & e)
    {
        RCLCPP_ERROR(this->get_logger(), e.what());
        return LNI::CallbackReturn::FAILURE;
    }
    return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn RosPkgTemplate::on_shutdown(const rclcpp_lifecycle::State &state) {
    (void)state;
    RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called from state %s.", state.label().c_str());
    try {
        publisher_.reset();
    }
    catch(std::exception & e)
    {
        RCLCPP_ERROR(this->get_logger(), e.what());
        return LNI::CallbackReturn::FAILURE;
    }
    return LNI::CallbackReturn::SUCCESS;
}

void RosPkgTemplate::callback(const std_msgs::msg::Float64::ConstSharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Receiving preprocessed float data");
    if (publisher_->is_activated())
    {
        try {
            algorithm_.addData(msg->data);
            if (algorithm_.getDataSize() > 100)
            {
                algorithm_.accReset();
                RCLCPP_INFO(this->get_logger(), "Accumulator reseting");
            }
            else
            {
                auto resultMsg = std_msgs::msg::Float64 ();
                resultMsg.data = algorithm_.getAverage();
                publisher_->publish(resultMsg);
                RCLCPP_INFO(this->get_logger(), "Publishing average");
            }
        }
        catch(std::exception & e)
        {
            RCLCPP_ERROR(this->get_logger(), e.what());
        }
    }
    
}


rcl_interfaces::msg::SetParametersResult RosPkgTemplate::dyn_reconf_callback(const std::vector<rclcpp::Parameter> & parameters)
{
    RCLCPP_INFO(this->get_logger(), "Parameter change request");
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = "The reason it could not be allowed";
    for (const auto & parameter : parameters) {
        try {
            if (parameter.get_name() == "param" && parameter.get_value<bool>() != this->param.get_value<bool>()) {
                result.successful = true;
                result.reason = "Parameter change accepted";
                RCLCPP_INFO(this->get_logger(), "Parameter change accepted", parameter.get_name().c_str());
                this->param = parameter;
                RCLCPP_INFO(this->get_logger(), "Parameter '%s' changed", parameter.get_name().c_str());
            }
            }
            catch(std::exception & e)
            {
                RCLCPP_WARN(this->get_logger(), e.what());
                result.reason = e.what();
                RCLCPP_INFO(this->get_logger(), "Parameter change canceled", parameter.get_name().c_str());
            }
        }
    return result;
}

std_srvs::srv::SetBool::Response::ConstSharedPtr
RosPkgTemplate::request_service(std_srvs::srv::SetBool::Response::ConstSharedPtr resp) {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    RCLCPP_INFO(this->get_logger(), "Sending request");
    auto result = client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Image classification request has been response");
    return resp;
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(RosPkgTemplate)
