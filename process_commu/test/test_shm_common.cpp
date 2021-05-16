#include <process_commu/shm_common.h>
#include <robot_info/robot_info.h>
#include <process_commu/robot_state_shm.h>
#include <process_commu/arm_shm.h>

#ifdef USE_GRIPPER
#include <process_commu/gripper_shm.h>
#endif

#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<robot_info::Robot> robot = 
        std::make_shared<robot_info::Robot>();
    
    robot_state_shm::RobotState *robot_state_shm;
    int robot_state_shm_id;

    arm_shm::Arm *arm_shm;
    int arm_shm_id;

#ifdef USE_GRIPPER
    gripper_shm::Gripper *gripper_shm;
    int gripper_shm_id;
#endif

    /////////////////////////////////////////////////////////////////////////////////////
    robot_state_shm_id = shm_common::create_shm(robot->state_shm_key_, &robot_state_shm);
    if (robot_state_shm_id != SHM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("test"), 
            "Create robot state shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test"), 
            "Create robot state shared memory failed.");
        return 0;
    }

    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        robot_state_shm->cur_arm_positions_[j] = j;
        robot_state_shm->cur_arm_velocities_[j] = j;
        robot_state_shm->cur_arm_efforts_[j] = j;
    }

#ifdef USE_GRIPPER
    for (unsigned int j=0; j< robot->gripper_->dof_; j++)
    {
        robot_state_shm->cur_gripper_positions_[j] = j;
        robot_state_shm->cur_gripper_velocities_[j] = j;
        robot_state_shm->cur_gripper_efforts_[j] = j;

    }
#endif

    std::this_thread::sleep_for(std::chrono::seconds(5));

    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "robot_state_shm->cur_arm_positions_[" << j << "]: " 
            << robot_state_shm->cur_arm_positions_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "robot_state_shm->cur_arm_velocities_[" << j << "]: " 
            << robot_state_shm->cur_arm_velocities_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "robot_state_shm->cur_arm_efforts_[" << j << "]: " 
            << robot_state_shm->cur_arm_efforts_[j] << std::endl;
    }

#ifdef USE_GRIPPER
    for (unsigned int j=0; j< robot->gripper_->dof_; j++)
    {
        std::cout << "robot_state_shm->cur_gripper_positions_[" << j << "]: " 
            << robot_state_shm->cur_gripper_positions_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->gripper_->dof_; j++)
    {
        std::cout << "robot_state_shm->cur_gripper_velocities_[" << j << "]: " 
            << robot_state_shm->cur_gripper_velocities_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->gripper_->dof_; j++)
    {
        std::cout << "robot_state_shm->cur_gripper_efforts_[" << j << "]: " 
            << robot_state_shm->cur_gripper_efforts_[j] << std::endl;
    }
#endif

    if (shm_common::release_shm(robot_state_shm_id, &robot_state_shm) == SHM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("test"), 
            "Release robot state shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test"), 
            "Release robot state shared memory failed.");
        return 0;
    }

    /////////////////////////////////////////////////////////////////////////////////////
    arm_shm_id = shm_common::create_shm(robot->arm_->shm_key_, &arm_shm);
    if (arm_shm_id != SHM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("test"), 
            "Create arm shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test"), 
            "Create arm shared memory failed.");
        return 0;
    }

    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        arm_shm->control_modes_[j] = j;
        
        arm_shm->cur_positions_[j] = j;
        arm_shm->cur_velocities_[j] = j;
        arm_shm->cur_efforts_[j] = j;

        arm_shm->cmd_positions_[j] = j;
        arm_shm->cmd_velocities_[j] = j;
        arm_shm->cmd_efforts_[j] = j;
    }

    std::this_thread::sleep_for(std::chrono::seconds(5));

    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "arm_shm->control_modes_[" << j << "]: " 
            << arm_shm->control_modes_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "arm_shm->cur_positions_[" << j << "]: " 
            << arm_shm->cur_positions_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "arm_shm->cur_velocities_[" << j << "]: " 
            << arm_shm->cur_velocities_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "arm_shm->cur_efforts_[" << j << "]: " 
            << arm_shm->cur_efforts_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "arm_shm->cmd_positions_[" << j << "]: " 
            << arm_shm->cmd_positions_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "arm_shm->cmd_velocities_[" << j << "]: " 
            << arm_shm->cmd_velocities_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "arm_shm->cmd_efforts_[" << j << "]: " 
            << arm_shm->cmd_efforts_[j] << std::endl;
    }

    if (shm_common::release_shm(arm_shm_id, &arm_shm) == SHM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("test"), 
            "Release arm shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test"), 
            "Release arm shared memory failed.");
        return 0;
    }

#ifdef USE_GRIPPER
    /////////////////////////////////////////////////////////////////////////////////////
    gripper_shm_id = shm_common::create_shm(robot->gripper_->shm_key_, &gripper_shm);
    if (gripper_shm_id != SHM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("test"), 
            "Create gripper shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test"), 
            "Create gripper shared memory failed.");
        return 0;
    }

    for (unsigned int j=0; j< robot->gripper_->dof_; j++)
    {
        gripper_shm->control_modes_[j] = j;

        gripper_shm->cur_positions_[j] = j;
        gripper_shm->cur_velocities_[j] = j;
        gripper_shm->cur_efforts_[j] = j;

        gripper_shm->cmd_positions_[j] = j;
        gripper_shm->cmd_velocities_[j] = j;
        gripper_shm->cmd_efforts_[j] = j;
    }

    std::this_thread::sleep_for(std::chrono::seconds(5));

    for (unsigned int j=0; j< robot->gripper_->dof_; j++)
    {
        std::cout << "gripper_shm->control_modes_[" << j << "]: " 
            << gripper_shm->control_modes_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->gripper_->dof_; j++)
    {
        std::cout << "gripper_shm->cur_positions_[" << j << "]: " 
            << gripper_shm->cur_positions_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->gripper_->dof_; j++)
    {
        std::cout << "gripper_shm->cur_velocities_[" << j << "]: " 
            << gripper_shm->cur_velocities_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->gripper_->dof_; j++)
    {
        std::cout << "gripper_shm->cur_efforts_[" << j << "]: " 
            << gripper_shm->cur_efforts_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->gripper_->dof_; j++)
    {
        std::cout << "gripper_shm->cmd_positions_[" << j << "]: " 
            << gripper_shm->cmd_positions_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->gripper_->dof_; j++)
    {
        std::cout << "gripper_shm->cmd_velocities_[" << j << "]: " 
            << gripper_shm->cmd_velocities_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->gripper_->dof_; j++)
    {
        std::cout << "gripper_shm->cmd_efforts_[" << j << "]: " 
            << gripper_shm->cmd_efforts_[j] << std::endl;
    }

    if (shm_common::release_shm(gripper_shm_id, &gripper_shm) == SHM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("test"), 
            "Release gripper shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test"), 
            "Release gripper shared memory failed.");
        return 0;
    }
#endif
    
    rclcpp::shutdown();
    return 0;
}
