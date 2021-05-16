#include <gazebo_control_plugin/gazebo_control_plugin.h>
#include <rclcpp/rclcpp.hpp>

namespace gazebo_control_plugin
{
ControlPlugin::ControlPlugin()
{
    RCLCPP_INFO(rclcpp::get_logger("gazebo"), 
        "Start Robot Simulation ...");

    ///////////////////////////////////////////////////////////////////////////////////////////
    arm_shm_id_ = shm_common::create_shm(robot_->arm_->shm_key_, &arm_shm_);
    if (arm_shm_id_ != SHM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), 
            "Create arm shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), 
            "Create arm shared memory failed.");
    }

    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        arm_shm_->control_modes_[j] = robot_->arm_->control_modes_[j];

        arm_shm_->cur_positions_[j] = robot_->arm_->cur_positions_[j];
        arm_shm_->cur_velocities_[j] = robot_->arm_->cur_velocities_[j];
        arm_shm_->cur_efforts_[j] = robot_->arm_->cur_efforts_[j];

        arm_shm_->cmd_positions_[j] = robot_->arm_->cmd_positions_[j];
        arm_shm_->cmd_velocities_[j] = robot_->arm_->cmd_velocities_[j];
        arm_shm_->cmd_efforts_[j] = robot_->arm_->cmd_efforts_[j];
    }

    arm_sem_id_ = sem_common::create_semaphore(robot_->arm_->sem_key_);
    if (arm_sem_id_ != SEM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), 
            "Create arm semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), 
            "Create arm semaphore failed.");
    }

#ifdef USE_GRIPPER
    ///////////////////////////////////////////////////////////////////////////////////////////
    gripper_shm_id_ = shm_common::create_shm(robot_->gripper_->shm_key_, &gripper_shm_);
    if (gripper_shm_id_ != SHM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), 
            "Create gripper shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), 
            "Create gripper shared memory failed.");
    }

    for (unsigned int j=0; j< robot_->gripper_->dof_; j++)
    {
        gripper_shm_->control_modes_[j] = robot_->gripper_->control_modes_[j];

        gripper_shm_->cur_positions_[j] = robot_->gripper_->cur_positions_[j];
        gripper_shm_->cur_velocities_[j] = robot_->gripper_->cur_velocities_[j];
        gripper_shm_->cur_efforts_[j] = robot_->gripper_->cur_efforts_[j];

        gripper_shm_->cmd_positions_[j] = robot_->gripper_->cmd_positions_[j];
        gripper_shm_->cmd_velocities_[j] = robot_->gripper_->cmd_velocities_[j];
        gripper_shm_->cmd_efforts_[j] = robot_->gripper_->cmd_efforts_[j];
    }

    gripper_sem_id_ = sem_common::create_semaphore(robot_->gripper_->sem_key_);
    if (gripper_sem_id_ != SEM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), 
            "Create gripper semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), 
            "Create gripper semaphore failed.");
    }
#endif

    ///////////////////////////////////////////////////////////////////////////////////////////
    robot_state_shm_id_ = shm_common::create_shm(robot_->state_shm_key_, &robot_state_shm_);
    if (robot_state_shm_id_ != SHM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), 
            "Create robot state shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), 
            "Create robot state shared memory failed.");
    }

    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        robot_state_shm_->cur_arm_positions_[j] = robot_->arm_->cur_positions_[j];
        robot_state_shm_->cur_arm_velocities_[j] = robot_->arm_->cur_velocities_[j];
        robot_state_shm_->cur_arm_efforts_[j] = robot_->arm_->cur_efforts_[j];
    }
#ifdef USE_GRIPPER
    for (unsigned int j=0; j< robot_->gripper_->dof_; j++)
    {
        robot_state_shm_->cur_gripper_positions_[j] = robot_->gripper_->cur_positions_[j];
        robot_state_shm_->cur_gripper_velocities_[j] = robot_->gripper_->cur_velocities_[j];
        robot_state_shm_->cur_gripper_efforts_[j] = robot_->gripper_->cur_efforts_[j];
    }
#endif
    robot_state_sem_id_ = sem_common::create_semaphore(robot_->state_sem_key_);
    if (robot_state_sem_id_ != SEM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), 
            "Create robot state semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), 
            "Create robot state semaphore failed.");
    }
}

ControlPlugin::~ControlPlugin()
{ 
    ////////////////////////////////////////////////////////////////////////////////////
    if (shm_common::release_shm(arm_shm_id_, &arm_shm_) == SHM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), 
            "Release arm shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), 
            "Release arm shared memory failed.");
    }

    if (sem_common::delete_semaphore(arm_sem_id_) == SEM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), 
            "Delete arm semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), 
            "Delete arm semaphore failed.");
    }

#ifdef USE_GRIPPER
    ////////////////////////////////////////////////////////////////////////////////////
    if (shm_common::release_shm(gripper_shm_id_, &gripper_shm_) == SHM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), 
            "Release gripper shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), 
            "Release gripper shared memory failed.");
    }

    if (sem_common::delete_semaphore(gripper_sem_id_) == SEM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), 
            "Delete gripper semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), 
            "Delete gripper semaphore failed.");
    }
#endif

    ////////////////////////////////////////////////////////////////////////////////////
    if (shm_common::release_shm(robot_state_shm_id_, &robot_state_shm_) == SHM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), 
            "Release robot state shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), 
            "Release robot state shared memory failed.");
    }

    if (sem_common::delete_semaphore(robot_state_sem_id_) == SEM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), 
            "Delete robot state semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), 
            "Delete robot state semaphore failed.");
    }

    RCLCPP_INFO(rclcpp::get_logger("gazebo"), 
        "Simulation has been finished.");
}

void ControlPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  RCLCPP_INFO(rclcpp::get_logger("gazebo"), 
    "Load control plugin ...");

  parent_model_ = parent;

  double sim_rate = parent_model_->GetWorld()->Physics()->GetRealTimeUpdateRate();
  RCLCPP_INFO(rclcpp::get_logger("gazebo"), 
    "Simulation rate: %.2f Hz.", sim_rate);

  for (unsigned int j=0; j< robot_->arm_->dof_; j++)
  {
     gazebo::physics::JointPtr arm_joint = parent_model_->GetJoint(robot_->arm_->joint_names_[j]);
     arm_joints_.push_back(arm_joint);
  }

#ifdef USE_GRIPPER
  for (unsigned int j=0; j< robot_->gripper_->dof_; j++)
  {
     gazebo::physics::JointPtr gripper_joint = parent_model_->GetJoint(robot_->gripper_->joint_names_[j]);
     gripper_joints_.push_back(gripper_joint);
  }
#endif

  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&ControlPlugin::Update, this));

  RCLCPP_INFO(rclcpp::get_logger("gazebo"), 
    "Load control plugin successfully.");
}

void ControlPlugin::Update()
{
  /* Note: 
     We must do writing operation, and then do reading operation (for gazebo) in this function.
     Also, we must do reading operation, and then do writing operation (for ROS2) in ROS2 controller manager.
  */

  sem_common::semaphore_p(arm_sem_id_);
  for (unsigned int j=0; j< robot_->arm_->dof_; j++)
  {
    if ((robot_->arm_->control_modes_[j]=arm_shm_->control_modes_[j]) & robot_->position_mode_)
    {
      arm_joints_[j]->SetPosition(0, robot_->arm_->cmd_positions_[j]=arm_shm_->cmd_positions_[j]);
    }
    else if ((robot_->arm_->control_modes_[j]=arm_shm_->control_modes_[j]) & robot_->velocity_mode_)
    {
      arm_joints_[j]->SetVelocity(0, robot_->arm_->cmd_velocities_[j]=arm_shm_->cmd_velocities_[j]);
    }
    else if ((robot_->arm_->control_modes_[j]=arm_shm_->control_modes_[j]) & robot_->effort_mode_)
    {
      arm_joints_[j]->SetForce(0, robot_->arm_->cmd_efforts_[j]=arm_shm_->cmd_efforts_[j]);
    }
    
    arm_shm_->cur_positions_[j] = robot_->arm_->cur_positions_[j] = arm_joints_[j]->Position(0);
    arm_shm_->cur_velocities_[j] = robot_->arm_->cur_velocities_[j] = arm_joints_[j]->GetVelocity(0);
    arm_shm_->cur_efforts_[j] = robot_->arm_->cur_efforts_[j] = arm_joints_[j]->GetForce(0u);
  }
  sem_common::semaphore_v(arm_sem_id_);

#ifdef USE_GRIPPER
  sem_common::semaphore_p(gripper_sem_id_);
  for (unsigned int j=0; j< robot_->gripper_->dof_; j++)
  {
    if ((robot_->gripper_->control_modes_[j]=gripper_shm_->control_modes_[j]) & robot_->position_mode_)
    {
      gripper_joints_[j]->SetPosition(0, robot_->gripper_->cmd_positions_[j]=gripper_shm_->cmd_positions_[j]);
    }
    else if ((robot_->gripper_->control_modes_[j]=gripper_shm_->control_modes_[j]) & robot_->velocity_mode_)
    {
      gripper_joints_[j]->SetVelocity(0, robot_->gripper_->cmd_velocities_[j]=gripper_shm_->cmd_velocities_[j]);
    }
    else if ((robot_->gripper_->control_modes_[j]=gripper_shm_->control_modes_[j]) & robot_->effort_mode_)
    {
      gripper_joints_[j]->SetForce(0, robot_->gripper_->cmd_efforts_[j]=gripper_shm_->cmd_efforts_[j]);
    }
    
    gripper_shm_->cur_positions_[j] = robot_->gripper_->cur_positions_[j] = gripper_joints_[j]->Position(0);
    gripper_shm_->cur_velocities_[j] = robot_->gripper_->cur_velocities_[j] = gripper_joints_[j]->GetVelocity(0);
    gripper_shm_->cur_efforts_[j] = robot_->gripper_->cur_efforts_[j] = gripper_joints_[j]->GetForce(0u);
  }
  sem_common::semaphore_v(gripper_sem_id_);
#endif
}

GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)

}
