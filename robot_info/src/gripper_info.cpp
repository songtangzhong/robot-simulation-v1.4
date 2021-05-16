#include <robot_info/gripper_info.h>
#include <robot_info/robot_macro.h>
#include <sys/sem.h>

namespace gripper_info
{
Gripper::Gripper()
{
    dof_ = GRIPPER_DOF;

    joint_names_.resize(dof_);

    control_modes_.resize(dof_);
    
    cur_positions_.resize(dof_);
    cur_velocities_.resize(dof_);
    cur_efforts_.resize(dof_);

    cmd_positions_.resize(dof_);
    cmd_velocities_.resize(dof_);
    cmd_efforts_.resize(dof_);

    joint_names_ = {"panda_finger_joint1","panda_finger_joint2"};

    for (unsigned int j=0; j<dof_; j++)
    {
        control_modes_[j] = ROBOT_POSITION_MODE; // default: position mode

        cur_positions_[j] = cmd_positions_[j] = 0;
        cur_velocities_[j] = cmd_velocities_[j] = 0;
        cur_efforts_[j] = cmd_efforts_[j] = 0;
    }

    shm_key_ = ftok(GRIPPER_SHM_FILE, 1);
    if (shm_key_ == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("shared memory"),
            "Generate key value of gripper shared memory failed.");
    }

    sem_key_ = ftok(GRIPPER_SEM_FILE, 1);
    if (sem_key_ == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("semaphore"),
            "Generate key value of gripper semaphore failed.");
    }
}

Gripper::~Gripper(){}

}
