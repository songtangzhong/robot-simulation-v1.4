# robot-simulation-v1.4
The project is based on ros2-foxy and gazebo-11. This is the basic version of the project, which contains all basic interfaces and operating functions to operate a robot.

## Usage:
1. Clone the project.
```
cd ~
mkdir -p robot-simulation-v1.4/src
cd robot-simulation-v1.4/src
git clone https://github.com/songtangzhong/robot-simulation-v1.4.git
cd robot-simulation-v1.4
mv ./* ../
cd ..
rm -rf robot-simulation-v1.4
```

2. Create following files in you /usr/local folder.
```
sudo mkdir /usr/local/robot_files
sudo touch /usr/local/robot_files/robot_arm_shm
sudo touch /usr/local/robot_files/robot_arm_sem
sudo touch /usr/local/robot_files/robot_gripper_shm
sudo touch /usr/local/robot_files/robot_gripper_sem
sudo touch /usr/local/robot_files/robot_state_shm
sudo touch /usr/local/robot_files/robot_state_sem
```

3. Build the workspace.
```
cd ~/robot-simulation-v1.4
colcon build --ament-cmake-args -DGRIPPER=true
```
    If you don't use gripper, run the following command.
```
colcon build --ament-cmake-args -DGRIPPER=false
```
    Then modify robot_hw_interface/launch/robot_hw_interface.launch.py, set "gripper" to "false".

4. Add following commands to you ~/.bashrc.
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/robot-simulation-v1.4/src/robot_sim_world
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/robot-simulation-v1.4/install/robot_sim_world/lib
source ~/.bashrc
```

5. Run the project (in different terminals), don't forget to source you ros2 environment firstly.
```
cd ~/robot-simulation-v1.4
source install/setup.bash
gazebo src/robot_sim_world/world/panda.world
ros2 launch robot_hw_interface robot_hw_interface.launch.py
```

6. Test if all the controllers have been loaded, started and configured successfully.
```
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

7. Test robot state callback function.
```
ros2 run robot_fun test_trigger
```

8. Test some interface functions.
```
ros2 run robot_fun test_set_joint_positions
```

9. Test switch controllers.
```
ros2 run robot_fun test_switch_to_target_controller effort_controllers
ros2 run robot_fun test_switch_to_target_controller velocity_controllers
```

10. Test get arm control mode.
```
ros2 run robot_fun test_get_arm_control_mode
```

11. Detailed illustration of the project will be added in the future.