#ifndef ROBOT_INFO_H_
#define ROBOT_INFO_H_

#include <robot_info/arm_info.h>

#ifdef USE_GRIPPER
#include <robot_info/gripper_info.h>
#endif

#include <robot_info/robot_macro.h>

namespace robot_info
{
class Robot
{
public:
    Robot();
    ~Robot();

    std::shared_ptr<arm_info::Arm> arm_ = 
        std::make_shared<arm_info::Arm>();

#ifdef USE_GRIPPER
    std::shared_ptr<gripper_info::Gripper> gripper_ = 
        std::make_shared<gripper_info::Gripper>();
#endif

    const unsigned int position_mode_ = ROBOT_POSITION_MODE;
    const unsigned int velocity_mode_ = ROBOT_VELOCITY_MODE;
    const unsigned int effort_mode_ = ROBOT_EFFORT_MODE;

    key_t state_shm_key_;
    key_t state_sem_key_;
};

}

#endif