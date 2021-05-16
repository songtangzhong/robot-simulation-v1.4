#include <process_commu/shm_common.h>
#include <process_commu/arm_shm.h>

#ifdef USE_GRIPPER
#include <process_commu/gripper_shm.h>
#endif

#include <process_commu/robot_state_shm.h>
#include <rclcpp/rclcpp.hpp>

namespace shm_common
{
template <class T>
int create_shm(key_t key, T ** shm_ptr)
{
    void *shm_ln_ptr = NULL;

    T *shm_ptr_;

    int shm_id;

    shm_id = shmget(key, sizeof(T), IPC_CREAT | 0666);
    if (shm_id == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("shared memory"), 
            "Create shared memory failed.");
		return SHM_STATE_NO;
    }

    shm_ln_ptr = shmat(shm_id, 0, 0);
    if (shm_ln_ptr == (void*)-1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("shared memory"), 
            "Create link address failed.");
		return SHM_STATE_NO;
    }

    *shm_ptr = shm_ptr_ = (T*)shm_ln_ptr;

	return shm_id;
}

template <class T>
int release_shm(int shm_id, T ** shm_ptr)
{
    if (shmdt(*shm_ptr) == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("shared memory"), 
            "Seperate shared memory failed.");
		return SHM_STATE_NO;
    }

    if (shmctl(shm_id, IPC_RMID, 0) == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("shared memory"), 
            "Release shared memory failed.");
		return SHM_STATE_NO;
    }

	return SHM_STATE_OK;
}

template int create_shm<arm_shm::Arm>(key_t key, arm_shm::Arm ** shm_ptr);
template int release_shm<arm_shm::Arm>(int shm_id, arm_shm::Arm ** shm_ptr);

#ifdef USE_GRIPPER
template int create_shm<gripper_shm::Gripper>(key_t key, gripper_shm::Gripper ** shm_ptr);
template int release_shm<gripper_shm::Gripper>(int shm_id, gripper_shm::Gripper ** shm_ptr);
#endif

template int create_shm<robot_state_shm::RobotState>(key_t key, robot_state_shm::RobotState ** shm_ptr);
template int release_shm<robot_state_shm::RobotState>(int shm_id, robot_state_shm::RobotState ** shm_ptr);

}
