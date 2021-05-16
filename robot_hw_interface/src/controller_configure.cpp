#include <chrono>
#include <memory>
#include <string>
#include <controller_manager_msgs/srv/load_start_controller.hpp>
#include <controller_manager_msgs/srv/load_configure_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_hw_interface/controller_configure.h>

using namespace std::chrono_literals;

namespace controller_configure
{
ControllerConfigure::ControllerConfigure(const std::string & node_name)
{
    nh_ = std::make_shared<rclcpp::Node>(node_name);

    load_start_controller_cli_ = nh_->create_client<controller_manager_msgs::srv::LoadStartController>
        ("/controller_manager/load_and_start_controller");

    load_configure_controller_cli_ = nh_->create_client<controller_manager_msgs::srv::LoadConfigureController>
        ("/controller_manager/load_and_configure_controller");
}

ControllerConfigure::~ControllerConfigure(){}

void ControllerConfigure::load_start_controller(const std::string & controller_name)
{
    auto request = std::make_shared<controller_manager_msgs::srv::LoadStartController::Request>();
    request->name = controller_name;
    while (!load_start_controller_cli_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("load_start_controller"), 
                "Interrupted while waiting for the service. Exiting.");
        }

        RCLCPP_INFO(rclcpp::get_logger("load_start_controller"), 
            "service [/controller_manager/load_and_start_controller] not available, waiting again...");
    }

    auto result = load_start_controller_cli_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(nh_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("load_start_controller"), 
            "load and start %s successfully.", controller_name.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("load_start_controller"), 
            "Failed to load and start %s.", controller_name.c_str());
    }
}

void ControllerConfigure::load_configure_controller(const std::string & controller_name)
{
    auto request = std::make_shared<controller_manager_msgs::srv::LoadConfigureController::Request>();
    request->name = controller_name;
    while (!load_configure_controller_cli_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("load_configure_controller"), 
                "Interrupted while waiting for the service. Exiting.");
        }

        RCLCPP_INFO(rclcpp::get_logger("load_configure_controller"), 
            "service [/controller_manager/load_and_configure_controller] not available, waiting again...");
    }

    auto result = load_configure_controller_cli_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(nh_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("load_configure_controller"), 
            "load and configure %s successfully.", controller_name.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("load_configure_controller"), 
            "Failed to load and start %s.", controller_name.c_str());
    }
}

}
