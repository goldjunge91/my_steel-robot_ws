#!/usr/bin/env python3

"""
Remote PC joystick/teleop launch file
Run this on your laptop/desktop to control the robot
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation time",
        choices=["True", "False"],
    )
    
    # Try to find the joystick config, fall back to inline params if not found
    try:
        joy_params_file = PathJoinSubstitution(
            [FindPackageShare("robot"), "config", "joystick.yaml"]
        )
        joy_params = [joy_params_file, {"use_sim_time": use_sim_time}]
    except:
        # Fallback inline parameters
        joy_params = [
            {
                "use_sim_time": use_sim_time,
                "device_id": 0,
                "deadzone": 0.1,
                "autorepeat_rate": 20.0,
            }
        ]
    
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=joy_params,
        output="screen",
    )
    
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "enable_button": 4,  # LB button (Xbox controller)
                "enable_turbo_button": 5,  # RB button
                "axis_linear.x": 1,  # Left stick vertical
                "axis_linear.y": 0,  # Left stick horizontal  
                "axis_angular.yaw": 2,  # Right stick horizontal
                "scale_linear": 0.5,
                "scale_linear_turbo": 1.0,
                "scale_angular": 1.0,
                "scale_angular_turbo": 1.5,
            }
        ],
        remappings=[
            ("/cmd_vel", "/diff_drive_controller/cmd_vel_unstamped"),
        ],
        output="screen",
    )
    
    return LaunchDescription(
        [
            declare_use_sim_time_arg,
            joy_node,
            teleop_node,
        ]
    )
