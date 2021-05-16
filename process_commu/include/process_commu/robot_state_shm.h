#ifndef ROBOT_STATE_SHM_H_
#define ROBOT_STATE_SHM_H_

#include <robot_info/robot_macro.h>

namespace robot_state_shm
{
typedef struct
{
    double cur_arm_positions_[ARM_DOF];
    double cur_arm_velocities_[ARM_DOF];
    double cur_arm_efforts_[ARM_DOF];

#ifdef USE_GRIPPER
    double cur_gripper_positions_[GRIPPER_DOF];
    double cur_gripper_velocities_[GRIPPER_DOF];
    double cur_gripper_efforts_[GRIPPER_DOF];
#endif
} RobotState;

}

#endif
