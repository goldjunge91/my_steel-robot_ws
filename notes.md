colcon build --symlink-install --merge-install --event-handlers console_direct+
cd /home/pi/workspace/ros2_dev_ws/my_steel-robot_ws
bash scripts/start_robot.sh 
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
ros2 run foxglove_bridge foxglove_bridge --port 8765

 ros2 control list_hardware_interfaces
 ros2 control list_hardware_interfaces 2>&1 | head -30
 ros2 node list