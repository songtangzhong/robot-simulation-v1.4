#include <robot_info/robot_info.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<robot_info::Robot> robot = 
        std::make_shared<robot_info::Robot>();

    std::cout << "robot info:" << std::endl;
    std::cout << "position_mode: " << robot->position_mode_ << std::endl;
    std::cout << "velocity_mode: " << robot->velocity_mode_ << std::endl;
    std::cout << "effort_mode: " << robot->effort_mode_ << std::endl;
    std::cout << "state_shm_key: " << robot->state_shm_key_ << std::endl;
    std::cout << "state_sem_key: " << robot->state_sem_key_ << std::endl;

    std::cout << "arm info:" << std::endl;
    std::cout << "shm_key: " << robot->arm_->shm_key_ << std::endl;
    std::cout << "sem_key: " << robot->arm_->sem_key_ << std::endl;
    std::cout << "dof: " << robot->arm_->dof_ << std::endl;
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "control_modes[" << j << "]: " << robot->arm_->control_modes_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "cur_positions[" << j << "]: " << robot->arm_->cur_positions_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "cur_velocities[" << j << "]: " << robot->arm_->cur_velocities_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "cur_efforts[" << j << "]: " << robot->arm_->cur_efforts_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "cmd_positions[" << j << "]: " << robot->arm_->cmd_positions_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "cmd_velocities[" << j << "]: " << robot->arm_->cmd_velocities_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "cmd_efforts[" << j << "]: " << robot->arm_->cmd_efforts_[j] << std::endl;
    }

#ifdef USE_GRIPPER
    std::cout << "gripper info:" << std::endl;
    std::cout << "shm_key: " << robot->gripper_->shm_key_ << std::endl;
    std::cout << "sem_key: " << robot->gripper_->sem_key_ << std::endl;
    std::cout << "dof: " << robot->gripper_->dof_ << std::endl;
    for (unsigned int j=0; j< robot->gripper_->dof_; j++)
    {
        std::cout << "control_modes[" << j << "]: " << robot->gripper_->control_modes_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->gripper_->dof_; j++)
    {
        std::cout << "cur_positions[" << j << "]: " << robot->gripper_->cur_positions_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->gripper_->dof_; j++)
    {
        std::cout << "cur_velocities[" << j << "]: " << robot->gripper_->cur_velocities_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->gripper_->dof_; j++)
    {
        std::cout << "cur_efforts[" << j << "]: " << robot->gripper_->cur_efforts_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->gripper_->dof_; j++)
    {
        std::cout << "cmd_positions[" << j << "]: " << robot->gripper_->cmd_positions_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->gripper_->dof_; j++)
    {
        std::cout << "cmd_velocities[" << j << "]: " << robot->gripper_->cmd_velocities_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->gripper_->dof_; j++)
    {
        std::cout << "cmd_efforts[" << j << "]: " << robot->gripper_->cmd_efforts_[j] << std::endl;
    }
#endif

    rclcpp::shutdown();
    return 0;
}
