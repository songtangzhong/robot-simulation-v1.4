#ifndef GAZEBO_CONTROL_PLUGIN_H_
#define GAZEBO_CONTROL_PLUGIN_H_

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Joint.hh>

#include <robot_info/robot_info.h>
#include <process_commu/arm_shm.h>

#ifdef USE_GRIPPER
#include <process_commu/gripper_shm.h>
#endif

#include <process_commu/robot_state_shm.h>
#include <process_commu/shm_common.h>
#include <process_commu/sem_common.h>

namespace gazebo_control_plugin
{
class ControlPlugin : public gazebo::ModelPlugin
{
public:
    ControlPlugin();
    ~ControlPlugin();

    void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

    void Update();

private:
    gazebo::physics::ModelPtr parent_model_;

    std::vector<gazebo::physics::JointPtr> arm_joints_;

#ifdef USE_GRIPPER
    std::vector<gazebo::physics::JointPtr> gripper_joints_;
#endif

    gazebo::event::ConnectionPtr update_connection_;

    std::shared_ptr<robot_info::Robot> robot_ = 
        std::make_shared<robot_info::Robot>();

    arm_shm::Arm *arm_shm_;
    int arm_shm_id_;
    int arm_sem_id_;

#ifdef USE_GRIPPER
    gripper_shm::Gripper *gripper_shm_;
    int gripper_shm_id_;
    int gripper_sem_id_;
#endif

    robot_state_shm::RobotState *robot_state_shm_;
    int robot_state_shm_id_;
    int robot_state_sem_id_;
}; 

}

#endif
