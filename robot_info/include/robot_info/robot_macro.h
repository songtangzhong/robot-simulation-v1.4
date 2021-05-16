#ifndef ROBOT_MACRO_H_
#define ROBOT_MACRO_H_

#define ARM_DOF 7
#define ARM_SHM_FILE "/usr/local/robot_files/robot_arm_shm" 
#define ARM_SEM_FILE "/usr/local/robot_files/robot_arm_sem"

#ifdef USE_GRIPPER
#define GRIPPER_DOF 2
#define GRIPPER_SHM_FILE "/usr/local/robot_files/robot_gripper_shm"
#define GRIPPER_SEM_FILE "/usr/local/robot_files/robot_gripper_sem"
#endif

#define ROBOT_STATE_SHM_FILE "/usr/local/robot_files/robot_state_shm"
#define ROBOT_STATE_SEM_FILE "/usr/local/robot_files/robot_state_sem"

#define ROBOT_POSITION_MODE 1<<0
#define ROBOT_VELOCITY_MODE 1<<1
#define ROBOT_EFFORT_MODE   1<<2

#endif