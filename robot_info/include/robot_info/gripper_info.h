#ifndef GRIPPER_INFO_H_
#define GRIPPER_INFO_H_

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>

namespace gripper_info
{
class Gripper
{
public:
    Gripper();
    ~Gripper();

    unsigned int dof_;

    std::vector<std::string> joint_names_;

    std::vector<unsigned int> control_modes_;

    std::vector<double> cur_positions_;
    std::vector<double> cur_velocities_;
    std::vector<double> cur_efforts_;

    std::vector<double> cmd_positions_;
    std::vector<double> cmd_velocities_;
    std::vector<double> cmd_efforts_;

    key_t shm_key_;
    key_t sem_key_;
};

}

#endif