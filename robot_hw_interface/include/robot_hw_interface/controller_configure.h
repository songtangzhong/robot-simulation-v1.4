#ifndef CONTROLLER_CONFIGURE_H_
#define CONTROLLER_CONFIGURE_H_

#include <chrono>
#include <memory>
#include <string>
#include <controller_manager_msgs/srv/load_start_controller.hpp>
#include <controller_manager_msgs/srv/load_configure_controller.hpp>
#include <rclcpp/rclcpp.hpp>

namespace controller_configure
{
class ControllerConfigure
{
public:
    ControllerConfigure(const std::string & node_name);
    ~ControllerConfigure();

    void load_start_controller(const std::string & controller_name);

    void load_configure_controller(const std::string & controller_name);

private:
    std::shared_ptr<rclcpp::Node> nh_;

    rclcpp::Client<controller_manager_msgs::srv::LoadStartController>::SharedPtr 
        load_start_controller_cli_;
    rclcpp::Client<controller_manager_msgs::srv::LoadConfigureController>::SharedPtr 
        load_configure_controller_cli_;
};

}

#endif
