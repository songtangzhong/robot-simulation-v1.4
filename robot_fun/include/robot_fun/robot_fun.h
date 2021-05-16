#ifndef ROBOT_FUN_H_
#define ROBOT_FUN_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <robot_info/robot_info.h>
#include <process_commu/robot_state_shm.h>
#include <process_commu/arm_shm.h>
#include <process_commu/shm_common.h>
#include <process_commu/sem_common.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

#ifdef USE_GRIPPER
#include <process_commu/gripper_shm.h>
#endif

namespace robot_fun
{
class Robot : public rclcpp::Node
{
public:
    Robot(const std::string & node_name);
    ~Robot();

    void get_arm_joint_positions(std::vector<double> & positions);
    void get_arm_joint_velocities(std::vector<double> & velocities);
    void get_arm_joint_efforts(std::vector<double> & efforts);

    int set_arm_joint_positions(std::vector<double> & positions);
    int set_arm_joint_velocities(std::vector<double> & velocities);
    int set_arm_joint_efforts(std::vector<double> & efforts);

    std::string get_arm_control_mode(void);

    int arm_switch_controller(const std::string & start_controller);

#ifdef USE_GRIPPER
    void get_gripper_joint_positions(std::vector<double> & positions);
    void get_gripper_joint_velocities(std::vector<double> & velocities);
    void get_gripper_joint_efforts(std::vector<double> & efforts);

    int set_gripper_joint_positions(std::vector<double> & positions);
    int set_gripper_joint_velocities(std::vector<double> & velocities);
    int set_gripper_joint_efforts(std::vector<double> & efforts);

    std::string get_gripper_control_mode(void);
#endif

private:
    void callback_robot_state_sub_(const sensor_msgs::msg::JointState::SharedPtr msg);

    std::shared_ptr<rclcpp::Node> nh_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robot_state_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_positions_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_velocities_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_efforts_pub_;

    std::shared_ptr<robot_info::Robot> robot_ = 
        std::make_shared<robot_info::Robot>();

    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr 
        switch_controller_cli_;

    robot_state_shm::RobotState *robot_state_shm_;
    int robot_state_shm_id_;
    int robot_state_sem_id_;

    arm_shm::Arm *arm_shm_;
    int arm_shm_id_;
    int arm_sem_id_;

#ifdef USE_GRIPPER
    gripper_shm::Gripper *gripper_shm_;
    int gripper_shm_id_;
    int gripper_sem_id_;
#endif
};

}

#endif