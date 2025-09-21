# Quickstart: Run Gazebo simulation and mecabridge_hardware HAL

1. Source ROS2 and workspace:

   source /opt/ros/humble/setup.bash
   source /workspaces/my_steel-robot_ws/install/setup.bash  # after build

2. Launch Gazebo with robot model (sim):

   ros2 launch robot_gazebo mecanum_world.launch.py

3. Start `mecabridge_hardware` in simulation mode (mode=sim):

   ros2 run mecabridge_hardware mecabridge_node --ros-args -p mode:=sim

4. Manual control (Xbox):

   ros2 run teleop_twist_joy teleop_node

5. Switch to hardware mode (on remote PC with hardware connected):

   ros2 service call /mecabridge/switch_mode mecabridge_hardware/SwitchMode "mode: 'hw'"

Notes:
- Switching modes performs a safe-stop before reconfiguring backends.
- If hardware is unavailable in `hw` mode, the node will publish diagnostics and optionally fall back to sim.
# Quickstart

## Simulation
1. Launch Gazebo with:
   ```bash
   ros2 launch robot mecanum_sim.launch.py
   ```
2. In a new terminal, start teleop:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
   ```
3. Observe robot motion in Gazebo.

## Hardware
1. Connect Raspberry Pi Pico via USB to Raspberry Pi 4B.
2. Launch hardware stack:
   ```bash
   ros2 launch robot mecanum_hardware.launch.py
   ```
3. Enable actuators:
   ```bash
   ros2 service call /mecabridge_hardware/enable std_srvs/SetBool "{data: true}"
   ```
4. Drive robot with teleop or web dashboard.

## Switching Modes
- Use launch arguments to select simulation or hardware mode.
- Only one actuation path (sim or hardware) should be active at a time.
