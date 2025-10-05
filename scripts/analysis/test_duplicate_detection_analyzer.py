#!/usr/bin/env python3
"""
Unit tests for Duplicate Detection Analyzer

Tests cover:
- Duplicate detection with known duplicate scenarios
- URDF/robot description overlap detection
- Hardware interface similarity analysis
- Launch file functionality comparison

Requirements addressed: 1.3
"""

import unittest
import tempfile
import shutil
import json
from pathlib import Path
import sys

# Add analysis directory to path
sys.path.insert(0, str(Path(__file__).parent))

from duplicate_detection_analyzer import DuplicateDetectionAnalyzer, DuplicateMatch

class TestDuplicateDetectionAnalyzer(unittest.TestCase):
    """Test cases for DuplicateDetectionAnalyzer class."""
    
    def setUp(self):
        """Set up test environment with temporary workspace."""
        self.test_workspace = tempfile.mkdtemp()
        self.workspace_path = Path(self.test_workspace)
        self.src_path = self.workspace_path / "src"
        self.src_path.mkdir()
        
        self.analyzer = DuplicateDetectionAnalyzer(str(self.workspace_path))
    
    def tearDown(self):
        """Clean up test environment."""
        shutil.rmtree(self.test_workspace)
    
    def create_mock_package_with_urdf(self, name: str, robot_name: str = "test_robot", 
                                    has_mecanum: bool = False, has_lidar: bool = False):
        """Create a mock package with URDF content."""
        package_path = self.src_path / name
        package_path.mkdir()
        
        # Create package.xml
        (package_path / "package.xml").write_text(f"""<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <version>1.0.0</version>
  <description>Test package {name}</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        # Create URDF directory and file
        urdf_dir = package_path / "urdf"
        urdf_dir.mkdir()
        
        urdf_content = f"""<?xml version="1.0"?>
<robot name="{robot_name}">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </visual>
  </link>
"""
        
        if has_mecanum:
            urdf_content += """
  <link name="front_left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="front_left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="front_left_wheel"/>
    <origin xyz="0.15 0.15 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  
  <!-- Mecanum drive configuration -->
  <gazebo>
    <plugin name="mecanum_drive_controller" filename="libgazebo_ros_mecanum_drive.so">
      <front_left_joint>front_left_wheel_joint</front_left_joint>
    </plugin>
  </gazebo>
"""
        
        if has_lidar:
            urdf_content += """
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.07"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
  
  <gazebo reference="lidar_link">
    <sensor type="ray" name="lidar">
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
      </ray>
    </sensor>
  </gazebo>
"""
        
        urdf_content += "</robot>"
        
        (urdf_dir / f"{robot_name}.urdf").write_text(urdf_content)
        
        return package_path
    
    def create_mock_hardware_interface_package(self, name: str, interface_type: str = "system"):
        """Create a mock package with hardware interface implementation."""
        package_path = self.src_path / name
        package_path.mkdir()
        
        # Create package.xml
        (package_path / "package.xml").write_text(f"""<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <version>1.0.0</version>
  <description>Hardware interface package {name}</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>hardware_interface</depend>
  <depend>rclcpp</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        # Create source directory
        src_dir = package_path / "src"
        src_dir.mkdir()
        
        # Create hardware interface implementation
        if interface_type == "system":
            cpp_content = f"""#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {name}
{{

class {name.title()}SystemInterface : public hardware_interface::SystemInterface
{{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {{
    RCLCPP_INFO(rclcpp::get_logger("{name}"), "Initializing hardware interface");
    return hardware_interface::CallbackReturn::SUCCESS;
  }}

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override
  {{
    RCLCPP_INFO(rclcpp::get_logger("{name}"), "Configuring hardware interface");
    return hardware_interface::CallbackReturn::SUCCESS;
  }}

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override
  {{
    // Read hardware state
    return hardware_interface::return_type::OK;
  }}

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override
  {{
    // Write commands to hardware
    return hardware_interface::return_type::OK;
  }}
}};

}} // namespace {name}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS({name}::{name.title()}SystemInterface, hardware_interface::SystemInterface)
"""
        else:
            cpp_content = f"""#include <rclcpp/rclcpp.hpp>

namespace {name}
{{

class {name.title()}Node : public rclcpp::Node
{{
public:
  {name.title()}Node() : Node("{name}_node")
  {{
    RCLCPP_INFO(this->get_logger(), "Hardware interface node started");
  }}
}};

}} // namespace {name}
"""
        
        (src_dir / f"{name}_interface.cpp").write_text(cpp_content)
        
        # Create CMakeLists.txt
        cmake_content = f"""cmake_minimum_required(VERSION 3.8)
project({name})

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_library({name}_interface SHARED src/{name}_interface.cpp)
ament_target_dependencies({name}_interface hardware_interface rclcpp)

install(TARGETS {name}_interface
  DESTINATION lib)

ament_package()
"""
        (package_path / "CMakeLists.txt").write_text(cmake_content)
        
        return package_path
    
    def create_mock_controller_package(self, name: str, controller_type: str = "config"):
        """Create a mock package with controller functionality."""
        package_path = self.src_path / name
        package_path.mkdir()
        
        # Create package.xml
        (package_path / "package.xml").write_text(f"""<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <version>1.0.0</version>
  <description>Controller package {name}</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        if controller_type == "config":
            # Create controller configuration
            config_dir = package_path / "config"
            config_dir.mkdir()
            
            config_content = """controller_manager:
  ros__parameters:
    update_rate: 100
    
    mecanum_drive_controller:
      type: mecanum_drive_controller/MecanumDriveController
      
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

mecanum_drive_controller:
  ros__parameters:
    front_left_wheel_name: "front_left_wheel_joint"
    front_right_wheel_name: "front_right_wheel_joint"
    rear_left_wheel_name: "rear_left_wheel_joint"
    rear_right_wheel_name: "rear_right_wheel_joint"
    
    wheel_separation_x: 0.3
    wheel_separation_y: 0.3
    wheel_radius: 0.05
"""
            (config_dir / "controllers.yaml").write_text(config_content)
            
        elif controller_type == "implementation":
            # Create controller implementation
            src_dir = package_path / "src"
            src_dir.mkdir()
            
            cpp_content = f"""#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {name}
{{

class CustomController : public controller_interface::ControllerInterface
{{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {{
    controller_interface::InterfaceConfiguration config;
    return config;
  }}

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {{
    controller_interface::InterfaceConfiguration config;
    return config;
  }}

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override
  {{
    return controller_interface::return_type::OK;
  }}
}};

}} // namespace {name}
"""
            (src_dir / "custom_controller.cpp").write_text(cpp_content)
        
        return package_path
    
    def test_find_packages(self):
        """Test package discovery functionality."""
        # Create test packages
        self.create_mock_package_with_urdf("robot_pkg1", "robot1")
        self.create_mock_package_with_urdf("robot_pkg2", "robot2")
        
        packages = self.analyzer.find_packages()
        
        self.assertEqual(len(packages), 2)
        package_names = [p.name for p in packages]
        self.assertIn("robot_pkg1", package_names)
        self.assertIn("robot_pkg2", package_names)
    
    def test_extract_functionality_keywords(self):
        """Test functionality keyword extraction."""
        # Create package with hardware interface
        hw_pkg = self.create_mock_hardware_interface_package("robot_hardware", "system")
        
        keywords = self.analyzer.extract_functionality_keywords(hw_pkg)
        
        self.assertIn("hardware_interface", keywords)
        self.assertIn("robot", keywords)
        self.assertIn("hardware", keywords)
    
    def test_analyze_urdf_content(self):
        """Test URDF content analysis."""
        # Create package with URDF
        pkg_path = self.create_mock_package_with_urdf("test_robot_pkg", "my_robot", 
                                                    has_mecanum=True, has_lidar=True)
        
        urdf_info = self.analyzer.analyze_urdf_content(pkg_path)
        
        self.assertTrue(any("robot:my_robot" in info for info in urdf_info))
        self.assertTrue(any("drive:mecanum" in info for info in urdf_info))
        self.assertTrue(any("sensor:lidar" in info for info in urdf_info))
    
    def test_analyze_hardware_interfaces(self):
        """Test hardware interface analysis."""
        # Create package with SystemInterface
        pkg_path = self.create_mock_hardware_interface_package("test_hw", "system")
        
        interfaces = self.analyzer.analyze_hardware_interfaces(pkg_path)
        
        self.assertIn("ros2_control_interface", interfaces)
        self.assertIn("system_interface", interfaces)
    
    def test_create_package_fingerprint(self):
        """Test package fingerprint creation."""
        # Create package with various content types
        pkg_path = self.create_mock_package_with_urdf("test_pkg", "test_robot")
        
        # Add launch file
        launch_dir = pkg_path / "launch"
        launch_dir.mkdir()
        launch_content = """from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher')
    ])
"""
        (launch_dir / "robot.launch.py").write_text(launch_content)
        
        fingerprint = self.analyzer.create_package_fingerprint(pkg_path)
        
        self.assertEqual(fingerprint.name, "test_pkg")
        self.assertGreater(len(fingerprint.urdf_files), 0)
        self.assertGreater(len(fingerprint.launch_files), 0)
        self.assertIn("robot_description", fingerprint.functionality_keywords)
    
    def test_analyze_urdf_logic_overlap_same_robot(self):
        """Test URDF logic overlap detection for same robot definition."""
        # Create two packages with same robot name
        pkg1_path = self.create_mock_package_with_urdf("robot_pkg", "my_steel_robot")
        pkg2_path = self.create_mock_package_with_urdf("robot_description", "my_steel_robot")
        
        overlap = self.analyzer.analyze_urdf_logic_overlap(pkg1_path, pkg2_path)
        
        self.assertTrue(overlap['has_overlap'])
        self.assertGreater(len(overlap['evidence']), 0)
    
    def test_analyze_urdf_logic_overlap_different_robots(self):
        """Test URDF logic overlap detection for different robots."""
        # Create two packages with different robot names
        pkg1_path = self.create_mock_package_with_urdf("robot_pkg", "robot_a")
        pkg2_path = self.create_mock_package_with_urdf("other_pkg", "robot_b")
        
        overlap = self.analyzer.analyze_urdf_logic_overlap(pkg1_path, pkg2_path)
        
        self.assertFalse(overlap['has_overlap'])
    
    def test_analyze_hardware_interface_logic_both_implement(self):
        """Test hardware interface logic overlap when both packages implement interfaces."""
        # Create two packages that both implement SystemInterface
        pkg1_path = self.create_mock_hardware_interface_package("robot_hardware", "system")
        pkg2_path = self.create_mock_hardware_interface_package("mecabridge_hardware", "system")
        
        overlap = self.analyzer.analyze_hardware_interface_logic(pkg1_path, pkg2_path)
        
        self.assertTrue(overlap['has_overlap'])
        self.assertGreater(len(overlap['evidence']), 0)
    
    def test_analyze_hardware_interface_logic_one_implements(self):
        """Test hardware interface logic when only one package implements interface."""
        # Create one package with SystemInterface, one without
        pkg1_path = self.create_mock_hardware_interface_package("robot_hardware", "system")
        pkg2_path = self.create_mock_hardware_interface_package("robot_utils", "node")
        
        overlap = self.analyzer.analyze_hardware_interface_logic(pkg1_path, pkg2_path)
        
        self.assertFalse(overlap['has_overlap'])
    
    def test_analyze_controller_logic_both_config(self):
        """Test controller logic overlap when both packages only configure controllers."""
        # Create two packages that both configure controllers
        pkg1_path = self.create_mock_controller_package("robot_controller", "config")
        pkg2_path = self.create_mock_controller_package("robot_controllers", "config")
        
        overlap = self.analyzer.analyze_controller_logic(pkg1_path, pkg2_path)
        
        self.assertTrue(overlap['has_overlap'])
        self.assertGreater(len(overlap['evidence']), 0)
    
    def test_analyze_controller_logic_implementation_vs_config(self):
        """Test controller logic when one implements and one configures (complementary)."""
        # Create one package with implementation, one with configuration
        pkg1_path = self.create_mock_controller_package("custom_controller", "implementation")
        pkg2_path = self.create_mock_controller_package("robot_controllers", "config")
        
        overlap = self.analyzer.analyze_controller_logic(pkg1_path, pkg2_path)
        
        # Should not be considered duplicate (complementary functionality)
        self.assertFalse(overlap['has_overlap'])
    
    def test_calculate_similarity_score_high_overlap(self):
        """Test similarity score calculation for packages with high overlap."""
        # Create two packages with URDF overlap (robot description packages)
        pkg1_path = self.create_mock_package_with_urdf("robot", "my_robot")
        pkg2_path = self.create_mock_package_with_urdf("robot_description", "my_robot")
        
        fp1 = self.analyzer.create_package_fingerprint(pkg1_path)
        fp2 = self.analyzer.create_package_fingerprint(pkg2_path)
        
        score, evidence = self.analyzer.calculate_similarity_score(fp1, fp2)
        
        self.assertGreaterEqual(score, 0.3)  # Should meet threshold for duplicates
        self.assertGreater(len(evidence), 0)
    
    def test_calculate_similarity_score_low_overlap(self):
        """Test similarity score calculation for packages with low overlap."""
        # Create two packages with different functionality
        pkg1_path = self.create_mock_package_with_urdf("robot_description", "robot")
        pkg2_path = self.create_mock_controller_package("robot_vision", "config")
        
        fp1 = self.analyzer.create_package_fingerprint(pkg1_path)
        fp2 = self.analyzer.create_package_fingerprint(pkg2_path)
        
        score, evidence = self.analyzer.calculate_similarity_score(fp1, fp2)
        
        self.assertLess(score, 0.3)  # Should not meet threshold for duplicates
    
    def test_detect_duplicates_known_scenario(self):
        """Test duplicate detection with known duplicate scenario."""
        # Create the known duplicate: robot vs robot_description
        self.create_mock_package_with_urdf("robot", "my_steel_robot")
        self.create_mock_package_with_urdf("robot_description", "my_steel_robot")
        
        duplicates = self.analyzer.detect_duplicates(similarity_threshold=0.3)
        
        # Should detect the duplicate
        self.assertGreater(len(duplicates), 0)
        
        # Check that robot and robot_description are identified as duplicates
        duplicate_pairs = [(d.package1, d.package2) for d in duplicates]
        self.assertTrue(
            ("robot", "robot_description") in duplicate_pairs or 
            ("robot_description", "robot") in duplicate_pairs
        )
    
    def test_detect_duplicates_no_duplicates(self):
        """Test duplicate detection when no duplicates exist."""
        # Create packages with different functionality
        self.create_mock_package_with_urdf("robot_description", "robot")
        self.create_mock_controller_package("robot_controllers", "config")
        self.create_mock_hardware_interface_package("robot_vision", "node")
        
        duplicates = self.analyzer.detect_duplicates(similarity_threshold=0.3)
        
        # Should not detect any duplicates
        self.assertEqual(len(duplicates), 0)
    
    def test_detect_duplicates_hardware_interfaces(self):
        """Test duplicate detection for hardware interface packages."""
        # Create two packages that both implement hardware interfaces
        self.create_mock_hardware_interface_package("robot_hardware", "system")
        self.create_mock_hardware_interface_package("mecabridge_hardware", "system")
        
        duplicates = self.analyzer.detect_duplicates(similarity_threshold=0.3)
        
        # Should detect hardware interface duplication
        self.assertGreater(len(duplicates), 0)
        
        # Check that hardware packages are identified as duplicates
        duplicate_pairs = [(d.package1, d.package2) for d in duplicates]
        self.assertTrue(
            ("robot_hardware", "mecabridge_hardware") in duplicate_pairs or 
            ("mecabridge_hardware", "robot_hardware") in duplicate_pairs
        )
    
    def test_generate_report(self):
        """Test duplicate detection report generation."""
        # Create test packages with known duplicates
        self.create_mock_package_with_urdf("robot", "test_robot")
        self.create_mock_package_with_urdf("robot_description", "test_robot")
        
        report = self.analyzer.generate_report("test_duplicate_report.json")
        
        # Check report structure
        self.assertIn('summary', report)
        self.assertIn('duplicate_matches', report)
        self.assertIn('package_fingerprints', report)
        self.assertIn('recommendations', report)
        
        # Check that report file was created
        report_file = self.workspace_path / "test_duplicate_report.json"
        self.assertTrue(report_file.exists())
        
        # Verify report content
        with open(report_file) as f:
            saved_report = json.load(f)
        
        self.assertGreaterEqual(saved_report['summary']['total_packages_analyzed'], 2)
    
    def test_generate_consolidation_recommendations(self):
        """Test consolidation recommendations generation."""
        # Create duplicate matches
        match1 = DuplicateMatch(
            package1="robot",
            package2="robot_description",
            similarity_score=0.8,
            duplicate_type="URDF/description overlap",
            evidence=["Both define robot URDF"],
            recommendation="Merge into single robot_description package"
        )
        
        self.analyzer.duplicate_matches = [match1]
        
        recommendations = self.analyzer.generate_consolidation_recommendations()
        
        self.assertIn('robot_vs_robot_description', recommendations)
        self.assertIn('robot_description_consolidation', recommendations)
        self.assertIn('hardware_interface_consolidation', recommendations)
        self.assertIn('controller_consolidation', recommendations)


class TestDuplicateDetectionIntegration(unittest.TestCase):
    """Integration tests for duplicate detection with complex scenarios."""
    
    def setUp(self):
        """Set up test environment."""
        self.test_workspace = tempfile.mkdtemp()
        self.workspace_path = Path(self.test_workspace)
        self.src_path = self.workspace_path / "src"
        self.src_path.mkdir()
        
        self.analyzer = DuplicateDetectionAnalyzer(str(self.workspace_path))
    
    def tearDown(self):
        """Clean up test environment."""
        shutil.rmtree(self.test_workspace)
    
    def test_complex_duplicate_scenario(self):
        """Test detection of multiple types of duplicates in one workspace."""
        # Create robot description duplicates
        robot_pkg = self.src_path / "robot"
        robot_pkg.mkdir()
        (robot_pkg / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>robot</name>
  <version>1.0.0</version>
  <description>Robot package</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        urdf_dir = robot_pkg / "urdf"
        urdf_dir.mkdir()
        (urdf_dir / "robot.urdf").write_text("""<?xml version="1.0"?>
<robot name="my_steel_robot">
  <link name="base_link"/>
</robot>""")
        
        robot_desc_pkg = self.src_path / "robot_description"
        robot_desc_pkg.mkdir()
        (robot_desc_pkg / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>robot_description</name>
  <version>1.0.0</version>
  <description>Robot description package</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        urdf_dir2 = robot_desc_pkg / "urdf"
        urdf_dir2.mkdir()
        (urdf_dir2 / "my_steel_robot.urdf").write_text("""<?xml version="1.0"?>
<robot name="my_steel_robot">
  <link name="base_link"/>
</robot>""")
        
        # Create hardware interface duplicates
        hw1_pkg = self.src_path / "robot_hardware"
        hw1_pkg.mkdir()
        (hw1_pkg / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>robot_hardware</name>
  <version>1.0.0</version>
  <description>Robot hardware interface</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>hardware_interface</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        src_dir1 = hw1_pkg / "src"
        src_dir1.mkdir()
        (src_dir1 / "hardware_interface.cpp").write_text("""
#include <hardware_interface/system_interface.hpp>

class RobotSystemInterface : public hardware_interface::SystemInterface {
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override {
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override {
    return hardware_interface::return_type::OK;
  }
  
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override {
    return hardware_interface::return_type::OK;
  }
};
""")
        
        hw2_pkg = self.src_path / "mecabridge_hardware"
        hw2_pkg.mkdir()
        (hw2_pkg / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>mecabridge_hardware</name>
  <version>1.0.0</version>
  <description>Mecabridge hardware interface</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>hardware_interface</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        src_dir2 = hw2_pkg / "src"
        src_dir2.mkdir()
        (src_dir2 / "mecabridge_interface.cpp").write_text("""
#include <hardware_interface/system_interface.hpp>

class MecabridgeSystemInterface : public hardware_interface::SystemInterface {
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override {
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override {
    return hardware_interface::return_type::OK;
  }
  
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override {
    return hardware_interface::return_type::OK;
  }
};
""")
        
        # Run duplicate detection
        duplicates = self.analyzer.detect_duplicates(similarity_threshold=0.3)
        
        # Should detect both types of duplicates
        self.assertGreaterEqual(len(duplicates), 2)
        
        # Check for robot description duplicates
        robot_desc_duplicates = [d for d in duplicates 
                               if (d.package1 in ["robot", "robot_description"] and 
                                   d.package2 in ["robot", "robot_description"])]
        self.assertGreater(len(robot_desc_duplicates), 0)
        
        # Check for hardware interface duplicates
        hw_duplicates = [d for d in duplicates 
                        if (d.package1 in ["robot_hardware", "mecabridge_hardware"] and 
                            d.package2 in ["robot_hardware", "mecabridge_hardware"])]
        self.assertGreater(len(hw_duplicates), 0)
    
    def test_false_positive_prevention(self):
        """Test that complementary packages are not marked as duplicates."""
        # Create packages that work together (not duplicates)
        
        # Robot description package
        desc_pkg = self.src_path / "robot_description"
        desc_pkg.mkdir()
        (desc_pkg / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>robot_description</name>
  <version>1.0.0</version>
  <description>Robot description</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        urdf_dir = desc_pkg / "urdf"
        urdf_dir.mkdir()
        (urdf_dir / "robot.urdf").write_text("""<?xml version="1.0"?>
<robot name="robot">
  <link name="base_link"/>
</robot>""")
        
        # Hardware interface package (implements interface)
        hw_pkg = self.src_path / "robot_hardware"
        hw_pkg.mkdir()
        (hw_pkg / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>robot_hardware</name>
  <version>1.0.0</version>
  <description>Robot hardware interface</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>hardware_interface</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        src_dir = hw_pkg / "src"
        src_dir.mkdir()
        (src_dir / "hardware_interface.cpp").write_text("""
#include <hardware_interface/system_interface.hpp>

class RobotSystemInterface : public hardware_interface::SystemInterface {
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override {
    return hardware_interface::return_type::OK;
  }
};
""")
        
        # Controller configuration package (configures controllers)
        ctrl_pkg = self.src_path / "robot_controllers"
        ctrl_pkg.mkdir()
        (ctrl_pkg / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>robot_controllers</name>
  <version>1.0.0</version>
  <description>Robot controller configurations</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        config_dir = ctrl_pkg / "config"
        config_dir.mkdir()
        (config_dir / "controllers.yaml").write_text("""
controller_manager:
  ros__parameters:
    mecanum_drive_controller:
      type: mecanum_drive_controller/MecanumDriveController
""")
        
        # Run duplicate detection
        duplicates = self.analyzer.detect_duplicates(similarity_threshold=0.3)
        
        # These complementary packages should NOT be marked as duplicates
        # (description + hardware + controllers work together)
        complementary_pairs = [
            ("robot_description", "robot_hardware"),
            ("robot_description", "robot_controllers"),
            ("robot_hardware", "robot_controllers")
        ]
        
        for pair in complementary_pairs:
            duplicate_found = any(
                (d.package1 == pair[0] and d.package2 == pair[1]) or
                (d.package1 == pair[1] and d.package2 == pair[0])
                for d in duplicates
            )
            self.assertFalse(duplicate_found, 
                           f"Complementary packages {pair} should not be marked as duplicates")


if __name__ == '__main__':
    unittest.main()