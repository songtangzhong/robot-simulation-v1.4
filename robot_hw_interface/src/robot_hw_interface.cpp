#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_hw_interface/robot_hw_interface.h>

namespace robot_hw
{
hardware_interface::return_type RobotHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }

  ///////////////////////////////////////////////////////////////////////////////////////////
  arm_shm_id_ = shm_common::create_shm(robot_->arm_->shm_key_, &arm_shm_);
  if (arm_shm_id_ == SHM_STATE_NO)
  {
      RCLCPP_ERROR(rclcpp::get_logger("RobotHardware"), 
          "Create arm shared memory failed.");
  }

  arm_sem_id_ = sem_common::create_semaphore(robot_->arm_->sem_key_);
  if (arm_sem_id_ == SEM_STATE_NO)
  {
      RCLCPP_ERROR(rclcpp::get_logger("RobotHardware"), 
          "Create arm semaphore failed.");
  }

#ifdef USE_GRIPPER
  ///////////////////////////////////////////////////////////////////////////////////////////
  gripper_shm_id_ = shm_common::create_shm(robot_->gripper_->shm_key_, &gripper_shm_);
  if (gripper_shm_id_ == SHM_STATE_NO)
  {
      RCLCPP_ERROR(rclcpp::get_logger("RobotHardware"), 
          "Create gripper shared memory failed.");
  }

  gripper_sem_id_ = sem_common::create_semaphore(robot_->gripper_->sem_key_);
  if (gripper_sem_id_ == SEM_STATE_NO)
  {
      RCLCPP_ERROR(rclcpp::get_logger("RobotHardware"), 
          "Create gripper semaphore failed.");
  }
#endif

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
RobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (unsigned int j = 0; j < robot_->arm_->dof_; j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        robot_->arm_->joint_names_[j], hardware_interface::HW_IF_POSITION, &robot_->arm_->cur_positions_[j]));
  }
  for (unsigned int j = 0; j < robot_->arm_->dof_; j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        robot_->arm_->joint_names_[j], hardware_interface::HW_IF_VELOCITY, &robot_->arm_->cur_velocities_[j]));
  }
  for (unsigned int j = 0; j < robot_->arm_->dof_; j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        robot_->arm_->joint_names_[j], hardware_interface::HW_IF_EFFORT, &robot_->arm_->cur_efforts_[j]));
  }

#ifdef USE_GRIPPER
  //////////////////////////////////////////////////////////////////////////////////////////////
  for (unsigned int j = 0; j < robot_->gripper_->dof_; j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        robot_->gripper_->joint_names_[j], hardware_interface::HW_IF_POSITION, &robot_->gripper_->cur_positions_[j]));
  }
  for (unsigned int j = 0; j < robot_->gripper_->dof_; j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        robot_->gripper_->joint_names_[j], hardware_interface::HW_IF_VELOCITY, &robot_->gripper_->cur_velocities_[j]));
  }
  for (unsigned int j = 0; j < robot_->gripper_->dof_; j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        robot_->gripper_->joint_names_[j], hardware_interface::HW_IF_EFFORT, &robot_->gripper_->cur_efforts_[j]));
  }
#endif

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (unsigned int j = 0; j < robot_->arm_->dof_; j++) 
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        robot_->arm_->joint_names_[j], hardware_interface::HW_IF_POSITION, &robot_->arm_->cmd_positions_[j]));
  }
  for (unsigned int j = 0; j < robot_->arm_->dof_; j++) 
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        robot_->arm_->joint_names_[j], hardware_interface::HW_IF_VELOCITY, &robot_->arm_->cmd_velocities_[j]));
  }
  for (unsigned int j = 0; j < robot_->arm_->dof_; j++) 
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        robot_->arm_->joint_names_[j], hardware_interface::HW_IF_EFFORT, &robot_->arm_->cmd_efforts_[j]));
  }

  return command_interfaces;
}


hardware_interface::return_type RobotHardware::start()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RobotHardware"),
    "Starting ...please wait...");

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("RobotHardware"),
    "System Sucessfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHardware::stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RobotHardware"),
    "Stopping ...please wait...");

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("RobotHardware"),
    "System sucessfully stopped!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHardware::read()
{
  sem_common::semaphore_p(arm_sem_id_);
  for (unsigned int j = 0; j < robot_->arm_->dof_; j++) 
  {
    robot_->arm_->cur_positions_[j] = arm_shm_->cur_positions_[j];
    robot_->arm_->cur_velocities_[j] = arm_shm_->cur_velocities_[j];
    robot_->arm_->cur_efforts_[j] = arm_shm_->cur_efforts_[j];
  }

#ifdef USE_GRIPPER
  sem_common::semaphore_p(gripper_sem_id_);
  for (unsigned int j = 0; j < robot_->gripper_->dof_; j++) 
  {
    robot_->gripper_->cur_positions_[j] = gripper_shm_->cur_positions_[j];
    robot_->gripper_->cur_velocities_[j] = gripper_shm_->cur_velocities_[j];
    robot_->gripper_->cur_efforts_[j] = gripper_shm_->cur_efforts_[j];
  }
  sem_common::semaphore_v(gripper_sem_id_);
#endif

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHardware::write()
{
  for (unsigned int j = 0; j < robot_->arm_->dof_; j++) 
  {
    if ((robot_->arm_->control_modes_[j]=arm_shm_->control_modes_[j]) & robot_->position_mode_) 
    {
      arm_shm_->cmd_positions_[j] = robot_->arm_->cmd_positions_[j];
    }
    else if ((robot_->arm_->control_modes_[j]=arm_shm_->control_modes_[j]) & robot_->velocity_mode_) 
    {
      arm_shm_->cmd_velocities_[j] = robot_->arm_->cmd_velocities_[j];
    }
    else if ((robot_->arm_->control_modes_[j]=arm_shm_->control_modes_[j]) & robot_->effort_mode_) 
    {
      arm_shm_->cmd_efforts_[j] = robot_->arm_->cmd_efforts_[j];
    }
  }
  sem_common::semaphore_v(arm_sem_id_);
  
  return hardware_interface::return_type::OK;
}

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  robot_hw::RobotHardware,
  hardware_interface::SystemInterface
)
