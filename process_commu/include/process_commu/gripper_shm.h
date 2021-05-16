#ifndef GRIPPER_SHM_H_
#define GRIPPER_SHM_H_

#include <robot_info/robot_macro.h>

namespace gripper_shm
{
typedef struct
{
    unsigned int control_modes_[GRIPPER_DOF];

    double cur_positions_[GRIPPER_DOF];
    double cur_velocities_[GRIPPER_DOF];
    double cur_efforts_[GRIPPER_DOF];

    double cmd_positions_[GRIPPER_DOF];
    double cmd_velocities_[GRIPPER_DOF];
    double cmd_efforts_[GRIPPER_DOF];
} Gripper;

}

#endif