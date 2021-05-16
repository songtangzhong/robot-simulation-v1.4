#include <process_commu/shm_common.h>
#include <process_commu/sem_common.h>
#include <process_commu/arm_shm.h>

#ifdef USE_GRIPPER
#include <process_commu/gripper_shm.h>
#endif

#include <robot_info/robot_info.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<robot_info::Robot> robot = 
        std::make_shared<robot_info::Robot>();

    arm_shm::Arm *arm_shm;
    int arm_shm_id;

    arm_shm_id = shm_common::create_shm(robot->arm_->shm_key_, &arm_shm);
    if (arm_shm_id != SHM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_write"), 
            "Create arm shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_write"), 
            "Create arm shared memory failed.");
        return 0;
    }

    int arm_sem_id;
    arm_sem_id = sem_common::create_semaphore(robot->arm_->sem_key_);
    if (arm_sem_id != SEM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_write"), 
            "Create arm semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_write"), 
            "Create arm semaphore failed.");
        return 0;
    }

    rclcpp::WallRate loop_rate(1000);
    unsigned int i = 0;
    while (rclcpp::ok())
    {
        sem_common::semaphore_p(arm_sem_id);
        for (unsigned int j=0; j< robot->arm_->dof_; j++)
        {
            arm_shm->control_modes_[j] = robot->position_mode_;

            arm_shm->cmd_positions_[j] = arm_shm->cur_positions_[j]+0.001;
            arm_shm->cmd_velocities_[j] = 0;
            arm_shm->cmd_efforts_[j] = 0;
        }
        sem_common::semaphore_v(arm_sem_id);
        loop_rate.sleep();
        if (i++ == 1000)
        {
            RCLCPP_INFO(rclcpp::get_logger("test_write"), 
                "Execute arm control successfully.");
            break;
        }
    }

#ifdef USE_GRIPPER
    ///////////////////////////////////////////////////////////////////////////////////////////////
    gripper_shm::Gripper *gripper_shm;
    int gripper_shm_id;

    gripper_shm_id = shm_common::create_shm(robot->gripper_->shm_key_, &gripper_shm);
    if (gripper_shm_id != SHM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_write"), 
            "Create gripper shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_write"), 
            "Create gripper shared memory failed.");
        return 0;
    }

    int gripper_sem_id;
    gripper_sem_id = sem_common::create_semaphore(robot->gripper_->sem_key_);
    if (gripper_sem_id != SEM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_write"), 
            "Create gripper semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_write"), 
            "Create gripper semaphore failed.");
        return 0;
    }

    i = 0;
    while (rclcpp::ok())
    {
        sem_common::semaphore_p(gripper_sem_id);
        for (unsigned int j=0; j< robot->gripper_->dof_; j++)
        {
            gripper_shm->control_modes_[j] = robot->position_mode_;

            gripper_shm->cmd_positions_[j] = gripper_shm->cur_positions_[j]+0.001;
            gripper_shm->cmd_velocities_[j] = 0;
            gripper_shm->cmd_efforts_[j] = 0;
        }
        sem_common::semaphore_v(gripper_sem_id);
        loop_rate.sleep();
        if (i++ == 400)
        {
            RCLCPP_INFO(rclcpp::get_logger("test_write"), 
                "Execute gripper control successfully.");
            break;
        }
    }
#endif

    rclcpp::shutdown();
    return 0;
}
