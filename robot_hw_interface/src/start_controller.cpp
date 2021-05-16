#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <robot_hw_interface/controller_configure.h>
#include <robot_info/robot_macro.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<controller_configure::ControllerConfigure> manager = 
    std::make_shared<controller_configure::ControllerConfigure>("start_controller");

  rclcpp::WallRate wait_ready(1);
  unsigned int count = 5;
  for (unsigned int j=0; j<=count; j++)
  {
    RCLCPP_INFO(rclcpp::get_logger("start_controller"), 
      "Waitting for the operating environment to be ready... %d seconds.", count-j);
    wait_ready.sleep();
  }

  manager->load_start_controller(JOINT_STATE_CONTROLLER);
  manager->load_start_controller(POSITION_CONTROLLERS);
  manager->load_configure_controller(VELOCITY_CONTROLLERS);
  manager->load_configure_controller(EFFORT_CONTROLLERS);

  rclcpp::shutdown();
  return 0;
}
