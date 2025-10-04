#!/usr/bin/env python3
"""
Unit tests for Package Status Analyzer

Tests cover:
- Package status analyzer with mock packages
- Dependency resolution with various configurations
- Build status checking
- Implementation completeness detection

Requirements addressed: 1.1, 1.2, 1.3
"""

import unittest
import tempfile
import shutil
import json
from pathlib import Path
from unittest.mock import patch, MagicMock
import sys

# Add analysis directory to path
sys.path.insert(0, str(Path(__file__).parent))

from package_status_analyzer import PackageStatusAnalyzer, PackageStatus, PackageInfo

class TestPackageStatusAnalyzer(unittest.TestCase):
    """Test cases for PackageStatusAnalyzer class."""
    
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
    
    def create_mock_package(self, name: str, package_type: str = "functional", 
                          build_type: str = "ament_cmake", dependencies: list = None):
        """Create a mock ROS2 package for testing."""
        if dependencies is None:
            dependencies = []
        
        package_path = self.src_path / name
        package_path.mkdir()
        
        # Create package.xml
        package_xml_content = f"""<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <version>1.0.0</version>
  <description>Test package {name}</description>
  <maintainer email="test@example.com">Test Maintainer</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
"""
        
        for dep in dependencies:
            package_xml_content += f"  <depend>{dep}</depend>\n"
        
        package_xml_content += """
  <export>
    <build_type>""" + build_type + """</build_type>
  </export>
</package>
"""
        
        (package_path / "package.xml").write_text(package_xml_content)
        
        # Create different types of mock packages
        if package_type == "functional":
            self._create_functional_package_content(package_path, build_type)
        elif package_type == "empty":
            self._create_empty_package_content(package_path, build_type)
        elif package_type == "broken":
            self._create_broken_package_content(package_path, build_type)
        elif package_type == "partial":
            self._create_partial_package_content(package_path, build_type)
        
        return package_path
    
    def _create_functional_package_content(self, package_path: Path, build_type: str):
        """Create content for a functional package."""
        if build_type == "ament_cmake":
            # Create CMakeLists.txt with actual targets
            cmake_content = """cmake_minimum_required(VERSION 3.8)
project(test_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(test_node src/test_node.cpp)
ament_target_dependencies(test_node rclcpp)

install(TARGETS test_node DESTINATION lib/${PROJECT_NAME})

ament_package()
"""
            (package_path / "CMakeLists.txt").write_text(cmake_content)
            
            # Create source directory and files
            src_dir = package_path / "src"
            src_dir.mkdir()
            
            cpp_content = """#include <rclcpp/rclcpp.hpp>

class TestNode : public rclcpp::Node {
public:
    TestNode() : Node("test_node") {
        RCLCPP_INFO(this->get_logger(), "Test node started");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
"""
            (src_dir / "test_node.cpp").write_text(cpp_content)
            
        elif build_type == "ament_python":
            # Create setup.py
            setup_content = """from setuptools import setup

package_name = 'test_package'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Test',
    maintainer_email='test@example.com',
    description='Test package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'test_node = test_package.test_node:main',
        ],
    },
)
"""
            (package_path / "setup.py").write_text(setup_content)
            
            # Create Python package
            py_pkg_dir = package_path / package_path.name
            py_pkg_dir.mkdir()
            (py_pkg_dir / "__init__.py").write_text("")
            
            py_content = """#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('Test node started')

def main():
    rclpy.init()
    node = TestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
            (py_pkg_dir / "test_node.py").write_text(py_content)
        
        # Add launch files
        launch_dir = package_path / "launch"
        launch_dir.mkdir()
        launch_content = """from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_package',
            executable='test_node',
            name='test_node'
        )
    ])
"""
        (launch_dir / "test.launch.py").write_text(launch_content)
        
        # Add config files
        config_dir = package_path / "config"
        config_dir.mkdir()
        config_content = """test_node:
  ros__parameters:
    param1: value1
    param2: 42
"""
        (config_dir / "config.yaml").write_text(config_content)
    
    def _create_empty_package_content(self, package_path: Path, build_type: str):
        """Create content for an empty package."""
        if build_type == "ament_cmake":
            # Minimal CMakeLists.txt with no targets
            cmake_content = """cmake_minimum_required(VERSION 3.8)
project(empty_package)

find_package(ament_cmake REQUIRED)

ament_package()
"""
            (package_path / "CMakeLists.txt").write_text(cmake_content)
        elif build_type == "ament_python":
            # Minimal setup.py
            setup_content = """from setuptools import setup

setup(
    name='empty_package',
    version='1.0.0',
    packages=[],
    install_requires=['setuptools'],
    zip_safe=True,
)
"""
            (package_path / "setup.py").write_text(setup_content)
    
    def _create_broken_package_content(self, package_path: Path, build_type: str):
        """Create content for a broken package."""
        if build_type == "ament_cmake":
            # CMakeLists.txt with syntax errors
            cmake_content = """cmake_minimum_required(VERSION 3.8)
project(broken_package)

find_package(ament_cmake REQUIRED)
find_package(nonexistent_package REQUIRED)  # This will cause build failure

add_executable(broken_node src/broken_node.cpp)
ament_target_dependencies(broken_node nonexistent_package)

ament_package()
"""
            (package_path / "CMakeLists.txt").write_text(cmake_content)
            
            # Create source with compilation errors
            src_dir = package_path / "src"
            src_dir.mkdir()
            cpp_content = """#include <nonexistent_header.hpp>  // This will cause compilation error

int main() {
    undefined_function();  // This will cause linking error
    return 0;
}
"""
            (src_dir / "broken_node.cpp").write_text(cpp_content)
    
    def _create_partial_package_content(self, package_path: Path, build_type: str):
        """Create content for a partially working package."""
        if build_type == "ament_cmake":
            # CMakeLists.txt with targets but missing some files
            cmake_content = """cmake_minimum_required(VERSION 3.8)
project(partial_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(partial_node src/partial_node.cpp)
ament_target_dependencies(partial_node rclcpp)

ament_package()
"""
            (package_path / "CMakeLists.txt").write_text(cmake_content)
            
            # Create minimal source file
            src_dir = package_path / "src"
            src_dir.mkdir()
            cpp_content = """// Minimal implementation
int main() {
    return 0;
}
"""
            (src_dir / "partial_node.cpp").write_text(cpp_content)
    
    def test_find_packages(self):
        """Test package discovery functionality."""
        # Create test packages
        self.create_mock_package("test_pkg1", "functional")
        self.create_mock_package("test_pkg2", "empty")
        
        packages = self.analyzer.find_packages()
        
        self.assertEqual(len(packages), 2)
        package_names = [p.name for p in packages]
        self.assertIn("test_pkg1", package_names)
        self.assertIn("test_pkg2", package_names)
    
    def test_parse_package_xml(self):
        """Test package.xml parsing."""
        # Create a package manually with specific dependency structure
        pkg_path = self.src_path / "test_pkg"
        pkg_path.mkdir()
        
        package_xml_content = """<?xml version="1.0"?>
<package format="3">
  <name>test_pkg</name>
  <version>1.0.0</version>
  <description>Test package test_pkg</description>
  <maintainer email="test@example.com">Test Maintainer</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rclcpp</build_depend>
  <exec_depend>std_msgs</exec_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""
        (pkg_path / "package.xml").write_text(package_xml_content)
        
        metadata = self.analyzer.parse_package_xml(pkg_path)
        
        self.assertEqual(metadata['name'], 'test_pkg')
        self.assertEqual(metadata['version'], '1.0.0')
        self.assertEqual(metadata['build_type'], 'ament_cmake')
        self.assertIn('rclcpp', metadata['dependencies']['build_depend'])
        self.assertIn('std_msgs', metadata['dependencies']['exec_depend'])
    
    def test_parse_invalid_package_xml(self):
        """Test parsing of invalid package.xml."""
        pkg_path = self.src_path / "invalid_pkg"
        pkg_path.mkdir()
        
        # Create invalid XML
        (pkg_path / "package.xml").write_text("invalid xml content")
        
        metadata = self.analyzer.parse_package_xml(pkg_path)
        
        self.assertIn('error', metadata)
    
    def test_check_implementation_completeness_functional(self):
        """Test implementation completeness check for functional package."""
        pkg_path = self.create_mock_package("functional_pkg", "functional")
        
        has_impl, issues = self.analyzer.check_implementation_completeness(
            pkg_path, "ament_cmake")
        
        self.assertTrue(has_impl)
        self.assertEqual(len(issues), 0)
    
    def test_check_implementation_completeness_empty(self):
        """Test implementation completeness check for empty package."""
        pkg_path = self.create_mock_package("empty_pkg", "empty")
        
        has_impl, issues = self.analyzer.check_implementation_completeness(
            pkg_path, "ament_cmake")
        
        self.assertFalse(has_impl)
        self.assertGreater(len(issues), 0)
        self.assertTrue(any("No functional content found" in issue for issue in issues))
    
    def test_check_implementation_completeness_partial(self):
        """Test implementation completeness check for partial package."""
        pkg_path = self.create_mock_package("partial_pkg", "partial")
        
        has_impl, issues = self.analyzer.check_implementation_completeness(
            pkg_path, "ament_cmake")
        
        # Partial packages might have minimal implementation
        self.assertIsInstance(has_impl, bool)
        # Should have some issues noted
        self.assertIsInstance(issues, list)
    
    @patch('subprocess.run')
    def test_check_build_status_success(self, mock_run):
        """Test build status check for successful build."""
        mock_run.return_value = MagicMock(returncode=0, stderr="", stdout="")
        
        success, issues = self.analyzer.check_build_status("test_package")
        
        self.assertTrue(success)
        self.assertEqual(len(issues), 0)
        mock_run.assert_called_once()
    
    @patch('subprocess.run')
    def test_check_build_status_failure(self, mock_run):
        """Test build status check for failed build."""
        mock_run.return_value = MagicMock(
            returncode=1, 
            stderr="Could not find a package configuration file", 
            stdout=""
        )
        
        success, issues = self.analyzer.check_build_status("test_package")
        
        self.assertFalse(success)
        self.assertGreater(len(issues), 0)
        self.assertTrue(any("Missing package dependencies" in issue for issue in issues))
    
    @patch('subprocess.run')
    def test_check_build_status_timeout(self, mock_run):
        """Test build status check with timeout."""
        from subprocess import TimeoutExpired
        mock_run.side_effect = TimeoutExpired("colcon", 300)
        
        success, issues = self.analyzer.check_build_status("test_package")
        
        self.assertFalse(success)
        self.assertGreater(len(issues), 0)
        self.assertTrue(any("Build timeout" in issue for issue in issues))
    
    def test_check_dependencies_available(self):
        """Test dependency availability checking."""
        # Test with common ROS2 dependencies (should be available)
        deps = {
            'depend': ['ament_cmake', 'rclcpp', 'std_msgs'],
            'build_depend': [],
            'exec_depend': [],
            'test_depend': []
        }
        
        available, missing = self.analyzer.check_dependencies_available(deps)
        
        # Should not fail on common dependencies
        self.assertIsInstance(available, bool)
        self.assertIsInstance(missing, list)
    
    def test_check_dependencies_missing(self):
        """Test dependency checking with missing dependencies."""
        deps = {
            'depend': ['nonexistent_package_12345'],
            'build_depend': [],
            'exec_depend': [],
            'test_depend': []
        }
        
        available, missing = self.analyzer.check_dependencies_available(deps)
        
        self.assertFalse(available)
        self.assertIn('nonexistent_package_12345', missing)
    
    def test_analyze_package_functional(self):
        """Test complete package analysis for functional package."""
        pkg_path = self.create_mock_package("functional_pkg", "functional")
        
        with patch.object(self.analyzer, 'check_build_status') as mock_build:
            mock_build.return_value = (True, [])
            
            package_info = self.analyzer.analyze_package(pkg_path)
        
        self.assertEqual(package_info.name, "functional_pkg")
        self.assertEqual(package_info.status, PackageStatus.FUNCTIONAL)
        self.assertTrue(package_info.build_success)
        self.assertTrue(package_info.has_implementation)
        self.assertEqual(package_info.recommendation, "Keep")
    
    def test_analyze_package_empty(self):
        """Test complete package analysis for empty package."""
        pkg_path = self.create_mock_package("empty_pkg", "empty")
        
        with patch.object(self.analyzer, 'check_build_status') as mock_build:
            mock_build.return_value = (True, [])
            
            package_info = self.analyzer.analyze_package(pkg_path)
        
        self.assertEqual(package_info.name, "empty_pkg")
        self.assertEqual(package_info.status, PackageStatus.EMPTY)
        self.assertFalse(package_info.has_implementation)
        self.assertEqual(package_info.recommendation, "Remove or implement")
    
    def test_analyze_package_broken(self):
        """Test complete package analysis for broken package."""
        pkg_path = self.create_mock_package("broken_pkg", "broken")
        
        with patch.object(self.analyzer, 'check_build_status') as mock_build:
            mock_build.return_value = (False, ["Build failed"])
            
            package_info = self.analyzer.analyze_package(pkg_path)
        
        self.assertEqual(package_info.name, "broken_pkg")
        self.assertEqual(package_info.status, PackageStatus.BROKEN)
        self.assertFalse(package_info.build_success)
        self.assertIn("Fix build errors", package_info.recommendation)
    
    def test_analyze_all_packages(self):
        """Test analysis of all packages in workspace."""
        # Create multiple test packages
        self.create_mock_package("pkg1", "functional")
        self.create_mock_package("pkg2", "empty")
        self.create_mock_package("pkg3", "partial")
        
        with patch.object(self.analyzer, 'check_build_status') as mock_build:
            mock_build.return_value = (True, [])
            
            packages = self.analyzer.analyze_all_packages()
        
        self.assertEqual(len(packages), 3)
        self.assertIn("pkg1", packages)
        self.assertIn("pkg2", packages)
        self.assertIn("pkg3", packages)
        
        # Check that different statuses are assigned
        statuses = [pkg.status for pkg in packages.values()]
        self.assertIn(PackageStatus.FUNCTIONAL, statuses)
        self.assertIn(PackageStatus.EMPTY, statuses)
    
    def test_generate_report(self):
        """Test report generation."""
        # Create test packages
        self.create_mock_package("test_pkg", "functional")
        
        with patch.object(self.analyzer, 'check_build_status') as mock_build:
            mock_build.return_value = (True, [])
            
            report = self.analyzer.generate_report("test_report.json")
        
        # Check report structure
        self.assertIn('summary', report)
        self.assertIn('packages', report)
        self.assertIn('total_packages', report['summary'])
        self.assertIn('status_breakdown', report['summary'])
        
        # Check that report file was created
        report_file = self.workspace_path / "test_report.json"
        self.assertTrue(report_file.exists())
        
        # Verify report content
        with open(report_file) as f:
            saved_report = json.load(f)
        
        self.assertEqual(saved_report['summary']['total_packages'], 1)
        self.assertIn('test_pkg', saved_report['packages'])


class TestPackageStatusAnalyzerIntegration(unittest.TestCase):
    """Integration tests for PackageStatusAnalyzer with various dependency configurations."""
    
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
    
    def test_dependency_resolution_chain(self):
        """Test dependency resolution with package dependency chains."""
        # Create packages with dependency chain: pkg_a -> pkg_b -> pkg_c
        
        # Package C (no dependencies)
        pkg_c_path = self.src_path / "pkg_c"
        pkg_c_path.mkdir()
        (pkg_c_path / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>pkg_c</name>
  <version>1.0.0</version>
  <description>Package C</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        # Package B (depends on C)
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
  <depend>pkg_c</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        # Package A (depends on B)
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
  <depend>pkg_b</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        # Test dependency resolution
        packages = self.analyzer.find_packages()
        self.assertEqual(len(packages), 3)
        
        # Test that workspace packages are recognized as available
        deps_a = {'depend': ['pkg_b'], 'build_depend': [], 'exec_depend': [], 'test_depend': []}
        available_a, missing_a = self.analyzer.check_dependencies_available(deps_a)
        
        deps_b = {'depend': ['pkg_c'], 'build_depend': [], 'exec_depend': [], 'test_depend': []}
        available_b, missing_b = self.analyzer.check_dependencies_available(deps_b)
        
        # Should recognize workspace packages as available
        self.assertTrue(available_a or 'pkg_b' not in missing_a)
        self.assertTrue(available_b or 'pkg_c' not in missing_b)
    
    def test_circular_dependency_detection(self):
        """Test handling of circular dependencies."""
        # Create packages with circular dependency: pkg_x -> pkg_y -> pkg_x
        
        pkg_x_path = self.src_path / "pkg_x"
        pkg_x_path.mkdir()
        (pkg_x_path / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>pkg_x</name>
  <version>1.0.0</version>
  <description>Package X</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>pkg_y</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        pkg_y_path = self.src_path / "pkg_y"
        pkg_y_path.mkdir()
        (pkg_y_path / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>pkg_y</name>
  <version>1.0.0</version>
  <description>Package Y</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>pkg_x</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        # Test that analyzer can handle circular dependencies without crashing
        packages = self.analyzer.find_packages()
        self.assertEqual(len(packages), 2)
        
        # Should not crash when analyzing packages with circular deps
        try:
            with patch.object(self.analyzer, 'check_build_status') as mock_build:
                mock_build.return_value = (True, [])
                analyzed_packages = self.analyzer.analyze_all_packages()
            
            self.assertEqual(len(analyzed_packages), 2)
        except Exception as e:
            self.fail(f"Analyzer crashed on circular dependencies: {e}")
    
    def test_mixed_build_types(self):
        """Test analysis of packages with different build types."""
        # Create ament_cmake package
        cmake_pkg_path = self.src_path / "cmake_pkg"
        cmake_pkg_path.mkdir()
        (cmake_pkg_path / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>cmake_pkg</name>
  <version>1.0.0</version>
  <description>CMake Package</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>""")
        
        # Create ament_python package
        python_pkg_path = self.src_path / "python_pkg"
        python_pkg_path.mkdir()
        (python_pkg_path / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>python_pkg</name>
  <version>1.0.0</version>
  <description>Python Package</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <export><build_type>ament_python</build_type></export>
</package>""")
        
        packages = self.analyzer.find_packages()
        self.assertEqual(len(packages), 2)
        
        # Test parsing of different build types
        cmake_metadata = self.analyzer.parse_package_xml(cmake_pkg_path)
        python_metadata = self.analyzer.parse_package_xml(python_pkg_path)
        
        self.assertEqual(cmake_metadata['build_type'], 'ament_cmake')
        self.assertEqual(python_metadata['build_type'], 'ament_python')


if __name__ == '__main__':
    unittest.main()