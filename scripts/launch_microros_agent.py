#!/usr/bin/env python3

"""
Custom micro-ROS Agent launcher for my_steel robot
Handles automatic device detection and fallback options
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def detect_pico_device():
    """Detect Raspberry Pi Pico micro-ROS device"""
    common_devices = [
        '/dev/ttyACM0',  # Most common for Pico
        '/dev/ttyUSB0',  # FTDI adapter
  
    ]
    
    for device in common_devices:
        if os.path.exists(device):
            return device
    
    return '/dev/ttyACM0'  # Default fallback


def launch_setup(context, *args, **kwargs):
    """Setup function to configure micro-ROS agent based on parameters"""
    
    # Get launch arguments
    transport = LaunchConfiguration('transport').perform(context)
    dev = LaunchConfiguration('dev').perform(context)
    baud = LaunchConfiguration('baud').perform(context)
    
    # Auto-detect device if not specified
    if dev == 'auto':
        dev = detect_pico_device()
        print(f"Auto-detected micro-ROS device: {dev}")
    
    # Configure arguments based on transport
    if transport == 'serial':
        arguments = ['serial', '--dev', dev, '-b', baud, '-v']
    elif transport == 'udp4':
        port = LaunchConfiguration('port').perform(context)
        arguments = ['udp4', '-p', port, '-v']
    else:
        raise ValueError(f"Unsupported transport: {transport}")
    
    # Create micro-ROS agent node
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=arguments,
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )
    
    return [micro_ros_agent]


def generate_launch_description():
    """Generate launch description for micro-ROS agent"""
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'transport',
            default_value='serial',
            description='Transport type: serial | udp4'
        ),
        DeclareLaunchArgument(
            'dev',
            default_value='auto',
            description='Serial device (use "auto" for auto-detection)'
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='115200',
            description='Serial baudrate'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='8888',
            description='UDP port (when transport=udp4)'
        ),
        
        # Setup function
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    generate_launch_description()