#include <rclcpp/rclcpp.hpp>
#include <robot_fun/robot_fun.h>
#include <robot_info/robot_macro.h>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<robot_fun::Robot> robot = 
        std::make_shared<robot_fun::Robot>("test_robot_fun");

    rclcpp::WallRate wait_ready(1);
    unsigned int count = 5;
    for (unsigned int j=0; j<=count; j++)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_robot_fun"), 
            "Waitting for the operating environment to be ready... %d seconds.", count-j);
        wait_ready.sleep();
    }

    std::vector<double> cur_arm_positions_1;
    std::vector<double> cur_arm_positions_2;
    cur_arm_positions_1.resize(ARM_DOF);
    cur_arm_positions_2.resize(ARM_DOF);

#ifdef USE_GRIPPER
    std::vector<double> cur_gripper_positions;
    cur_gripper_positions.resize(GRIPPER_DOF);
#endif

    while (rclcpp::ok())
    {
        robot->get_arm_joint_positions(cur_arm_positions_1);
        for (unsigned int j=0; j< ARM_DOF; j++)
        {
            cur_arm_positions_1[j] = cur_arm_positions_1[j]+0.5;
            cur_arm_positions_2[j] = cur_arm_positions_1[j]+0.5;
        }

        robot->set_arm_joint_positions(cur_arm_positions_1);
        std::this_thread::sleep_for(std::chrono::seconds(5));
        robot->set_arm_joint_positions(cur_arm_positions_2);

#ifdef USE_GRIPPER
        robot->get_gripper_joint_positions(cur_gripper_positions);
        for (unsigned int j=0; j<GRIPPER_DOF; j++)
        {
            cur_gripper_positions[j] += 0.03;
        }
        robot->set_gripper_joint_positions(cur_gripper_positions);
#endif

        break;
    }

    rclcpp::shutdown();
    return 0;
}
