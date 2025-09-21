# Quickstart: Run Gazebo simulation and mecabridge_hardware HAL

1. Source ROS 2 and the workspace:

   source /opt/ros/humble/setup.bash
   source /workspaces/my_steel-robot_ws/install/setup.bash  # after build

2. Launch Gazebo with the mecanum robot and controllers (sim mode is the default):

   ros2 launch robot mecabridge_gazebo.launch.py use_sim_time:=true

   Optional arguments:
   - `world:=obstacles.world` (file is resolved relative to `robot/worlds`)
   - `x_pose`, `y_pose`, `z_pose` to set the initial spawn pose

3. Drive the robot by publishing to `/cmd_vel`, e.g.:

   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/mecanum_controller/cmd_vel_unstamped

4. The launch file already brings up `mecabridge_controllers_launch.py` with `mode:=sim`. To switch to hardware mode later, call:

   ros2 service call /mecabridge/switch_mode mecabridge_hardware/SwitchMode "mode: 'hw'"

5. Re-enable actuators when on real hardware:

   ros2 service call /mecabridge/enable std_srvs/SetBool "{data: true}"

Notes:
- Switching modes performs a safe-stop before reconfiguring backends.
- If hardware is unavailable in `hw` mode, the node will publish diagnostics and optionally fall back to sim.
