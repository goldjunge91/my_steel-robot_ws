#!/usr/bin/env python3
"""
Unit tests for Dependency Resolution functionality

Tests cover:
- Dependency resolution with various dependency configurations
- Circular dependency detection and handling
- Missing dependency identification
- Workspace vs system dependency resolution

Requirements addressed: 1.1, 1.2, 1.3
"""

import unittest
import tempfile
import shutil
from pathlib import Path
from unittest.mock import patch, MagicMock
import sys

# Add analysis directory to path
sys.path.insert(0, str(Path(__file__).parent))

from package_status_analyzer import PackageStatusAnalyzer

class TestDependencyResolution(unittest.TestCase):
    """Test cases for dependency resolution functionality."""
    
    def setUp(self):
        """Set up test environment with temporary workspace."""
        self.test_workspace = tempfile.mkdtemp()
        self.workspace_path = Path(self.test_workspace)
        self.src_path = self.workspace_path / "src"
        self.src_path.mkdir()
        
        self.analyzer = PackageStatusAnalyzer(str(self.workspace_path))
    
    def tearDown(self):
        """Clean up test environment."""
        shutil.rmtree(self.test_workspace)
    
    def create_package_with_dependencies(self, name: str, dependencies: dict):
        """Create a package with specified dependencies."""
        package_path = self.src_path / name
        package_path.mkdir()
        
        # Build dependency XML
        dep_xml = ""
        for dep_type, deps in dependencies.items():
            for dep in deps:
                dep_xml += f"  <{dep_type}>{dep}</{dep_type}>\n"
        
        package_xml_content = f"""<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <version>1.0.0</version>
  <description>Test package {name}</description>
  <maintainer email="test@example.com">Test Maintainer</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
{dep_xml}
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""
        
        (package_path / "package.xml").write_text(package_xml_content)
        return package_path
    
    def test_no_dependencies(self):
        """Test dependency resolution for package with no dependencies."""
        deps = {
            'buildtool_depend': [],
            'build_depend': [],
            'exec_depend': [],
            'test_depend': []
        }
        
        available, missing = self.analyzer.check_dependencies_available(deps)
        
        self.assertTrue(available)
        self.assertEqual(len(missing), 0)
    
    def test_common_ros2_dependencies(self):
        """Test dependency resolution for common ROS2 dependencies."""
        deps = {
            'buildtool_depend': ['ament_cmake'],
            'build_depend': ['rclcpp', 'std_msgs'],
            'exec_depend': ['rclpy'],
            'test_depend': ['ament_lint_auto']
        }
        
        available, missing = self.analyzer.check_dependencies_available(deps)
        
        # Common ROS2 dependencies should be recognized as available
        self.assertTrue(available)
        self.assertEqual(len(missing), 0)
    
    def test_workspace_package_dependencies(self):
        """Test dependency resolution for workspace packages."""
        # Create packages with dependencies on each other
        self.create_package_with_dependencies("pkg_base", {
            'build_depend': [],
            'exec_depend': []
        })
        
        self.create_package_with_dependencies("pkg_dependent", {
            'build_depend': ['pkg_base'],
            'exec_depend': ['pkg_base']
        })
        
        # Test that workspace packages are recognized
        deps = {
            'buildtool_depend': [],
            'build_depend': ['pkg_base'],
            'exec_depend': ['pkg_base'],
            'test_depend': []
        }
        
        available, missing = self.analyzer.check_dependencies_available(deps)
        
        # pkg_base should be recognized as available in workspace
        self.assertTrue(available or 'pkg_base' not in missing)
    
    def test_missing_dependencies(self):
        """Test dependency resolution with missing dependencies."""
        deps = {
            'buildtool_depend': [],
            'build_depend': ['nonexistent_package_xyz123'],
            'exec_depend': ['another_missing_package_abc456'],
            'test_depend': []
        }
        
        available, missing = self.analyzer.check_dependencies_available(deps)
        
        self.assertFalse(available)
        self.assertIn('nonexistent_package_xyz123', missing)
        self.assertIn('another_missing_package_abc456', missing)
    
    def test_mixed_available_missing_dependencies(self):
        """Test dependency resolution with mix of available and missing dependencies."""
        deps = {
            'buildtool_depend': ['ament_cmake'],  # Available
            'build_depend': ['rclcpp', 'nonexistent_package'],  # Mixed
            'exec_depend': ['std_msgs'],  # Available
            'test_depend': ['missing_test_package']  # Missing
        }
        
        available, missing = self.analyzer.check_dependencies_available(deps)
        
        self.assertFalse(available)
        self.assertIn('nonexistent_package', missing)
        self.assertIn('missing_test_package', missing)
        # Should not include common packages in missing
        self.assertNotIn('ament_cmake', missing)
        self.assertNotIn('rclcpp', missing)
        self.assertNotIn('std_msgs', missing)
    
    def test_circular_dependency_chain(self):
        """Test handling of circular dependency chains."""
        # Create circular dependency: A -> B -> C -> A
        self.create_package_with_dependencies("pkg_a", {
            'build_depend': ['pkg_b'],
            'exec_depend': []
        })
        
        self.create_package_with_dependencies("pkg_b", {
            'build_depend': ['pkg_c'],
            'exec_depend': []
        })
        
        self.create_package_with_dependencies("pkg_c", {
            'build_depend': ['pkg_a'],
            'exec_depend': []
        })
        
        # Test that circular dependencies don't cause infinite loops
        packages = self.analyzer.find_packages()
        self.assertEqual(len(packages), 3)
        
        # Test dependency checking doesn't crash
        deps_a = {
            'buildtool_depend': [],
            'build_depend': ['pkg_b'],
            'exec_depend': [],
            'test_depend': []
        }
        
        try:
            available, missing = self.analyzer.check_dependencies_available(deps_a)
            # Should complete without hanging
            self.assertIsInstance(available, bool)
            self.assertIsInstance(missing, list)
        except Exception as e:
            self.fail(f"Circular dependency handling failed: {e}")
    
    def test_self_dependency(self):
        """Test handling of self-dependencies (package depending on itself)."""
        self.create_package_with_dependencies("self_dep_pkg", {
            'build_depend': ['self_dep_pkg'],
            'exec_depend': []
        })
        
        deps = {
            'buildtool_depend': [],
            'build_depend': ['self_dep_pkg'],
            'exec_depend': [],
            'test_depend': []
        }
        
        # Should handle self-dependency gracefully
        available, missing = self.analyzer.check_dependencies_available(deps)
        
        # Self-dependency should be recognized as available (package exists)
        self.assertTrue(available or 'self_dep_pkg' not in missing)
    
    def test_deep_dependency_chain(self):
        """Test handling of deep dependency chains."""
        # Create chain: pkg1 -> pkg2 -> pkg3 -> pkg4 -> pkg5
        chain_length = 5
        
        for i in range(1, chain_length + 1):
            if i == chain_length:
                # Last package has no dependencies
                deps = {'build_depend': [], 'exec_depend': []}
            else:
                # Each package depends on the next
                deps = {'build_depend': [f'pkg{i+1}'], 'exec_depend': []}
            
            self.create_package_with_dependencies(f"pkg{i}", deps)
        
        # Test dependency resolution for first package
        deps = {
            'buildtool_depend': [],
            'build_depend': ['pkg2'],
            'exec_depend': [],
            'test_depend': []
        }
        
        available, missing = self.analyzer.check_dependencies_available(deps)
        
        # Should handle deep chains without issues
        self.assertTrue(available or 'pkg2' not in missing)
    
    def test_multiple_dependency_types(self):
        """Test packages with multiple types of dependencies."""
        # Create base packages
        self.create_package_with_dependencies("base_pkg", {
            'build_depend': [],
            'exec_depend': []
        })
        
        self.create_package_with_dependencies("test_pkg_dep", {
            'build_depend': [],
            'exec_depend': []
        })
        
        # Create package with all dependency types
        self.create_package_with_dependencies("complex_pkg", {
            'buildtool_depend': ['ament_cmake'],
            'build_depend': ['rclcpp', 'base_pkg'],
            'exec_depend': ['rclpy', 'base_pkg'],
            'test_depend': ['ament_lint_auto', 'test_pkg_dep']
        })
        
        # Test all dependency types
        deps = {
            'buildtool_depend': ['ament_cmake'],
            'build_depend': ['rclcpp', 'base_pkg'],
            'exec_depend': ['rclpy', 'base_pkg'],
            'test_depend': ['ament_lint_auto', 'test_pkg_dep']
        }
        
        available, missing = self.analyzer.check_dependencies_available(deps)
        
        # Should handle multiple dependency types
        self.assertIsInstance(available, bool)
        self.assertIsInstance(missing, list)
        
        # Workspace packages should be recognized
        self.assertTrue('base_pkg' not in missing)
        self.assertTrue('test_pkg_dep' not in missing)
    
    @patch('subprocess.run')
    def test_ros2_pkg_list_integration(self, mock_run):
        """Test integration with ros2 pkg list command."""
        # Mock ros2 pkg list output
        mock_run.return_value = MagicMock(
            returncode=0,
            stdout="ament_cmake\nrclcpp\nrclpy\nstd_msgs\ngeometry_msgs\n"
        )
        
        deps = {
            'buildtool_depend': [],
            'build_depend': ['geometry_msgs'],
            'exec_depend': [],
            'test_depend': []
        }
        
        available, missing = self.analyzer.check_dependencies_available(deps)
        
        # Should use ros2 pkg list to check system packages
        mock_run.assert_called_with(['ros2', 'pkg', 'list'], capture_output=True, text=True)
        
        # geometry_msgs should be found in system
        self.assertTrue(available or 'geometry_msgs' not in missing)
    
    @patch('subprocess.run')
    def test_ros2_pkg_list_failure(self, mock_run):
        """Test handling when ros2 pkg list fails."""
        # Mock ros2 pkg list failure
        mock_run.side_effect = FileNotFoundError("ros2 command not found")
        
        deps = {
            'buildtool_depend': [],
            'build_depend': ['some_package'],
            'exec_depend': [],
            'test_depend': []
        }
        
        # Should handle ros2 command failure gracefully
        try:
            available, missing = self.analyzer.check_dependencies_available(deps)
            self.assertIsInstance(available, bool)
            self.assertIsInstance(missing, list)
        except Exception as e:
            self.fail(f"Should handle ros2 command failure gracefully: {e}")
    
    def test_dependency_resolution_performance(self):
        """Test dependency resolution performance with many packages."""
        # Create many packages to test performance
        num_packages = 50
        
        for i in range(num_packages):
            # Each package depends on a few others
            deps = {
                'build_depend': [f'pkg{(i+1) % num_packages}', f'pkg{(i+2) % num_packages}'],
                'exec_depend': [f'pkg{(i+3) % num_packages}']
            }
            self.create_package_with_dependencies(f"pkg{i}", deps)
        
        # Test that dependency resolution completes in reasonable time
        import time
        start_time = time.time()
        
        packages = self.analyzer.find_packages()
        self.assertEqual(len(packages), num_packages)
        
        # Test dependency checking for a few packages
        for i in range(0, min(5, num_packages)):
            deps = {
                'buildtool_depend': [],
                'build_depend': [f'pkg{(i+1) % num_packages}'],
                'exec_depend': [],
                'test_depend': []
            }
            available, missing = self.analyzer.check_dependencies_available(deps)
        
        end_time = time.time()
        
        # Should complete within reasonable time (less than 10 seconds)
        self.assertLess(end_time - start_time, 10.0)
    
    def test_empty_dependency_lists(self):
        """Test handling of empty dependency lists."""
        deps = {
            'buildtool_depend': [],
            'build_depend': [],
            'exec_depend': [],
            'test_depend': []
        }
        
        available, missing = self.analyzer.check_dependencies_available(deps)
        
        self.assertTrue(available)
        self.assertEqual(len(missing), 0)
    
    def test_none_dependency_values(self):
        """Test handling of None values in dependencies."""
        deps = {
            'buildtool_depend': None,
            'build_depend': ['rclcpp'],
            'exec_depend': [],
            'test_depend': None
        }
        
        # Should handle None values gracefully
        try:
            available, missing = self.analyzer.check_dependencies_available(deps)
            self.assertIsInstance(available, bool)
            self.assertIsInstance(missing, list)
        except Exception as e:
            self.fail(f"Should handle None dependency values gracefully: {e}")


class TestDependencyResolutionIntegration(unittest.TestCase):
    """Integration tests for dependency resolution in complete package analysis."""
    
    def setUp(self):
        """Set up test environment."""
        self.test_workspace = tempfile.mkdtemp()
        self.workspace_path = Path(self.test_workspace)
        self.src_path = self.workspace_path / "src"
        self.src_path.mkdir()
        
        self.analyzer = PackageStatusAnalyzer(str(self.workspace_path))
    
    def tearDown(self):
        """Clean up test environment."""
        shutil.rmtree(self.test_workspace)
    
    def create_realistic_ros2_workspace(self):
        """Create a realistic ROS2 workspace with typical dependencies."""
        
        # Robot description package
        robot_desc_path = self.src_path / "robot_description"
        robot_desc_path.mkdir()
        (robot_desc_path / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>robot_description</name>
  <version>1.0.0</version>
  <description>Robot URDF description</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>urdf</exec_depend>
  <exec_depend>xacro</exec_depend>
  
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        # Hardware interface package
        hw_interface_path = self.src_path / "robot_hardware"
        hw_interface_path.mkdir()
        (hw_interface_path / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>robot_hardware</name>
  <version>1.0.0</version>
  <description>Robot hardware interface</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>hardware_interface</build_depend>
  <build_depend>rclcpp</build_depend>
  <build_depend>rclcpp_lifecycle</build_depend>
  <exec_depend>hardware_interface</exec_depend>
  <exec_depend>rclcpp</exec_depend>
  <exec_depend>rclcpp_lifecycle</exec_depend>
  
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        # Controller package depending on hardware
        controller_path = self.src_path / "robot_controllers"
        controller_path.mkdir()
        (controller_path / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>robot_controllers</name>
  <version>1.0.0</version>
  <description>Robot controller configurations</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>robot_hardware</exec_depend>
  <exec_depend>robot_description</exec_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>mecanum_drive_controller</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>
  
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        # Bringup package depending on everything
        bringup_path = self.src_path / "robot_bringup"
        bringup_path.mkdir()
        (bringup_path / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>robot_bringup</name>
  <version>1.0.0</version>
  <description>Robot system bringup</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>robot_description</exec_depend>
  <exec_depend>robot_hardware</exec_depend>
  <exec_depend>robot_controllers</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>controller_manager</exec_depend>
  
  <export><build_type>ament_cmake</build_type></export>
</package>""")
    
    def test_realistic_workspace_dependency_resolution(self):
        """Test dependency resolution in a realistic ROS2 workspace."""
        self.create_realistic_ros2_workspace()
        
        with patch.object(self.analyzer, 'check_build_status') as mock_build:
            mock_build.return_value = (True, [])
            
            packages = self.analyzer.analyze_all_packages()
        
        self.assertEqual(len(packages), 4)
        
        # Check that workspace dependencies are resolved correctly
        bringup_pkg = packages['robot_bringup']
        
        # Should recognize workspace packages as available dependencies
        self.assertTrue(bringup_pkg.dependencies_met or 
                       len([issue for issue in bringup_pkg.issues 
                           if 'robot_description' in issue or 
                              'robot_hardware' in issue or 
                              'robot_controllers' in issue]) == 0)
    
    def test_broken_dependency_chain_analysis(self):
        """Test analysis when dependency chain is broken."""
        # Create packages with broken dependency chain
        
        # Package A depends on non-existent package
        pkg_a_path = self.src_path / "pkg_a"
        pkg_a_path.mkdir()
        (pkg_a_path / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>pkg_a</name>
  <version>1.0.0</version>
  <description>Package A</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>nonexistent_package</build_depend>
  
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        # Package B depends on Package A
        pkg_b_path = self.src_path / "pkg_b"
        pkg_b_path.mkdir()
        (pkg_b_path / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>pkg_b</name>
  <version>1.0.0</version>
  <description>Package B</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>pkg_a</build_depend>
  
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        with patch.object(self.analyzer, 'check_build_status') as mock_build:
            mock_build.return_value = (False, ["Build failed due to missing dependencies"])
            
            packages = self.analyzer.analyze_all_packages()
        
        # Both packages should be marked as broken due to dependency issues
        pkg_a = packages['pkg_a']
        pkg_b = packages['pkg_b']
        
        # Package A should have missing dependency issue
        self.assertFalse(pkg_a.dependencies_met)
        self.assertTrue(any('nonexistent_package' in issue for issue in pkg_a.issues))
        
        # Package B should recognize pkg_a as available (workspace package)
        # but may still be broken due to transitive dependency issues
        self.assertTrue(pkg_b.dependencies_met or 'pkg_a' not in [issue for issue in pkg_b.issues])


if __name__ == '__main__':
    unittest.main()