#!/usr/bin/env python3
"""
Package Status Analyzer for ROS2 Workspace Cleanup

This script analyzes all ROS2 packages in the workspace to determine their status:
- Functional: Package builds successfully and has implementation
- Partial: Package has some functionality but may have issues
- Broken: Package fails to build due to missing dependencies or errors
- Empty: Package exists but has minimal or no implementation
- Duplicate: Package has overlapping functionality with other packages

Requirements addressed: 1.1, 1.2
"""

import os
import sys
import xml.etree.ElementTree as ET
import subprocess
import json
import glob
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, asdict
from enum import Enum

class PackageStatus(Enum):
    FUNCTIONAL = "functional"
    PARTIAL = "partial"
    BROKEN = "broken"
    EMPTY = "empty"
    DUPLICATE = "duplicate"
    UNKNOWN = "unknown"

@dataclass
class PackageInfo:
    name: str
    path: str
    status: PackageStatus
    build_success: bool
    dependencies_met: bool
    has_implementation: bool
    duplicate_of: Optional[str]
    issues: List[str]
    recommendation: str
    metadata: Dict
    
class PackageStatusAnalyzer:
    def __init__(self, workspace_root: str = "."):
        self.workspace_root = Path(workspace_root).resolve()
        self.src_dir = self.workspace_root / "src"
        self.packages = {}
        self.dependency_graph = {}
        
    def find_packages(self) -> List[Path]:
        """Find all ROS2 packages by locating package.xml files."""
        package_files = []
        for package_xml in self.src_dir.rglob("package.xml"):
            # Skip build artifacts
            if "build" not in str(package_xml) and "install" not in str(package_xml):
                package_files.append(package_xml.parent)
        return package_files
    
    def parse_package_xml(self, package_path: Path) -> Dict:
        """Parse package.xml and extract metadata."""
        package_xml = package_path / "package.xml"
        if not package_xml.exists():
            return {}
            
        try:
            tree = ET.parse(package_xml)
            root = tree.getroot()
            
            metadata = {
                'name': root.find('name').text if root.find('name') is not None else '',
                'version': root.find('version').text if root.find('version') is not None else '',
                'description': root.find('description').text if root.find('description') is not None else '',
                'maintainer': root.find('maintainer').text if root.find('maintainer') is not None else '',
                'license': root.find('license').text if root.find('license') is not None else '',
                'build_type': 'unknown'
            }
            
            # Get build type
            export = root.find('export')
            if export is not None:
                build_type = export.find('build_type')
                if build_type is not None:
                    metadata['build_type'] = build_type.text
            
            # Get dependencies
            dependencies = {
                'buildtool_depend': [dep.text for dep in root.findall('buildtool_depend')],
                'build_depend': [dep.text for dep in root.findall('build_depend')],
                'exec_depend': [dep.text for dep in root.findall('exec_depend')],
                'test_depend': [dep.text for dep in root.findall('test_depend')]
            }
            metadata['dependencies'] = dependencies
            
            return metadata
        except ET.ParseError as e:
            return {'error': f'XML parse error: {e}'}
    
    def check_implementation_completeness(self, package_path: Path, build_type: str) -> Tuple[bool, List[str]]:
        """Check if package has actual functional implementation beyond boilerplate."""
        issues = []
        has_implementation = False
        implementation_score = 0
        
        # Check for different types of functional content
        functional_content = {
            'source_code': False,
            'launch_files': False,
            'config_files': False,
            'urdf_files': False,
            'build_targets': False
        }
        
        if build_type == 'ament_cmake':
            # Check for CMakeLists.txt with actual targets
            cmake_file = package_path / "CMakeLists.txt"
            if cmake_file.exists():
                try:
                    cmake_content = cmake_file.read_text()
                    # Look for actual build targets
                    if any(keyword in cmake_content for keyword in [
                        'add_executable', 'add_library', 'ament_target_dependencies'
                    ]):
                        functional_content['build_targets'] = True
                        implementation_score += 2
                    else:
                        issues.append("CMakeLists.txt exists but has no build targets")
                except:
                    issues.append("Could not read CMakeLists.txt")
            else:
                issues.append("Missing CMakeLists.txt for ament_cmake package")
                
            # Check for meaningful source files
            src_files = list(package_path.rglob("*.cpp")) + list(package_path.rglob("*.c")) + list(package_path.rglob("*.hpp"))
            if src_files:
                # Check if source files have actual implementation
                for src_file in src_files[:3]:  # Check first 3 files
                    try:
                        content = src_file.read_text()
                        # Look for actual implementation, not just headers
                        if len(content.strip()) > 100 and any(keyword in content for keyword in [
                            'class ', 'void ', 'int ', 'return', 'if (', 'for ('
                        ]):
                            functional_content['source_code'] = True
                            implementation_score += 3
                            break
                    except:
                        pass
                
                if not functional_content['source_code']:
                    issues.append("Source files found but appear to be empty or boilerplate")
            else:
                # For ament_cmake packages, missing source files is only an issue if no other content
                pass
                
        elif build_type == 'ament_python':
            # Check for setup.py
            setup_file = package_path / "setup.py"
            if setup_file.exists():
                functional_content['build_targets'] = True
                implementation_score += 1
            else:
                issues.append("Missing setup.py for ament_python package")
                
            # Check for meaningful Python source files
            py_files = list(package_path.rglob("*.py"))
            # Filter out setup.py and __init__.py
            meaningful_py_files = [f for f in py_files if f.name not in ['setup.py', '__init__.py', 'conftest.py']]
            
            if meaningful_py_files:
                # Check if Python files have actual implementation
                for py_file in meaningful_py_files[:3]:  # Check first 3 files
                    try:
                        content = py_file.read_text()
                        # Look for actual implementation
                        if len(content.strip()) > 50 and any(keyword in content for keyword in [
                            'def ', 'class ', 'import ', 'from ', 'if __name__'
                        ]):
                            functional_content['source_code'] = True
                            implementation_score += 3
                            break
                    except:
                        pass
                
                if not functional_content['source_code']:
                    issues.append("Python files found but appear to be empty or boilerplate")
            else:
                # Python packages without source files are likely just configuration packages
                pass
        
        # Check for launch files (high value functional content)
        launch_files = list(package_path.rglob("*.launch.py")) + list(package_path.rglob("*.launch"))
        if launch_files:
            # Verify launch files have actual content
            for launch_file in launch_files[:2]:
                try:
                    content = launch_file.read_text()
                    if len(content.strip()) > 100 and 'LaunchDescription' in content:
                        functional_content['launch_files'] = True
                        implementation_score += 2
                        break
                except:
                    pass
            
        # Check for configuration files (medium value functional content)
        config_files = list(package_path.rglob("*.yaml")) + list(package_path.rglob("*.yml"))
        if config_files:
            # Verify config files have meaningful content
            for config_file in config_files[:2]:
                try:
                    content = config_file.read_text()
                    if len(content.strip()) > 20 and ':' in content:  # Basic YAML structure
                        functional_content['config_files'] = True
                        implementation_score += 1
                        break
                except:
                    pass
            
        # Check for URDF/XACRO files (high value functional content)
        urdf_files = list(package_path.rglob("*.urdf")) + list(package_path.rglob("*.xacro"))
        if urdf_files:
            # Verify URDF files have actual robot definition
            for urdf_file in urdf_files[:2]:
                try:
                    content = urdf_file.read_text()
                    if len(content.strip()) > 100 and any(tag in content for tag in ['<robot', '<link', '<joint']):
                        functional_content['urdf_files'] = True
                        implementation_score += 2
                        break
                except:
                    pass
        
        # Determine if package has sufficient implementation
        # Require at least score of 2 or multiple types of content
        functional_types_count = sum(1 for has_content in functional_content.values() if has_content)
        
        if implementation_score >= 2 or functional_types_count >= 2:
            has_implementation = True
        elif implementation_score == 0 and functional_types_count == 0:
            issues.append("No functional content found - package appears empty")
        elif implementation_score < 2:
            issues.append("Minimal functional content - may be incomplete implementation")
            
        return has_implementation, issues
    
    def check_build_status(self, package_name: str) -> Tuple[bool, List[str]]:
        """Attempt to build a single package and check for success."""
        issues = []
        
        try:
            # Try to build just this package
            result = subprocess.run([
                'colcon', 'build', '--packages-select', package_name,
                '--event-handlers', 'console_direct+'
            ], 
            cwd=self.workspace_root,
            capture_output=True, 
            text=True, 
            timeout=300  # 5 minute timeout
            )
            
            if result.returncode == 0:
                return True, []
            else:
                # Parse error output for common issues
                error_output = result.stderr + result.stdout
                
                if "Could not find a package configuration file" in error_output:
                    issues.append("Missing package dependencies")
                elif "No such file or directory" in error_output:
                    issues.append("Missing source files or incorrect paths")
                elif "undefined reference" in error_output:
                    issues.append("Linking errors - missing libraries")
                elif "error: " in error_output.lower():
                    issues.append("Compilation errors")
                else:
                    issues.append(f"Build failed: {error_output[:200]}...")
                    
                return False, issues
                
        except subprocess.TimeoutExpired:
            return False, ["Build timeout - package may have infinite loops or very slow compilation"]
        except FileNotFoundError:
            return False, ["colcon command not found - ROS2 environment not sourced"]
        except Exception as e:
            return False, [f"Build check failed: {str(e)}"]
    
    def check_dependencies_available(self, dependencies: Dict) -> Tuple[bool, List[str]]:
        """Check if package dependencies are available in the system."""
        missing_deps = []
        
        # Check workspace packages first
        workspace_packages = set()
        for pkg_path in self.find_packages():
            pkg_xml = pkg_path / "package.xml"
            if pkg_xml.exists():
                try:
                    tree = ET.parse(pkg_xml)
                    root = tree.getroot()
                    name_elem = root.find('name')
                    if name_elem is not None:
                        workspace_packages.add(name_elem.text)
                except:
                    pass
        
        # Check all dependency types
        all_deps = []
        for dep_type, deps in dependencies.items():
            if deps is not None:  # Handle None values gracefully
                all_deps.extend(deps)
        
        for dep in all_deps:
            # Skip system dependencies that are commonly available
            if dep in ['ament_cmake', 'ament_python', 'rclcpp', 'rclpy', 'std_msgs']:
                continue
                
            # Check if it's a workspace package
            if dep in workspace_packages:
                continue
                
            # For now, assume other dependencies might be missing
            # In a full implementation, we'd check rosdep and system packages
            try:
                result = subprocess.run(['ros2', 'pkg', 'list'], capture_output=True, text=True)
                if result.returncode == 0 and dep in result.stdout:
                    continue
            except:
                pass
                
            missing_deps.append(dep)
        
        return len(missing_deps) == 0, missing_deps
    
    def analyze_package(self, package_path: Path) -> PackageInfo:
        """Analyze a single package and determine its status."""
        metadata = self.parse_package_xml(package_path)
        
        if 'error' in metadata:
            return PackageInfo(
                name=package_path.name,
                path=str(package_path),
                status=PackageStatus.BROKEN,
                build_success=False,
                dependencies_met=False,
                has_implementation=False,
                duplicate_of=None,
                issues=[metadata['error']],
                recommendation="Fix or remove",
                metadata=metadata
            )
        
        package_name = metadata.get('name', package_path.name)
        build_type = metadata.get('build_type', 'unknown')
        dependencies = metadata.get('dependencies', {})
        
        # Check implementation completeness
        has_implementation, impl_issues = self.check_implementation_completeness(package_path, build_type)
        
        # Check dependencies
        deps_available, missing_deps = self.check_dependencies_available(dependencies)
        
        # Check build status
        build_success, build_issues = self.check_build_status(package_name)
        
        # Combine all issues
        all_issues = impl_issues + build_issues
        if missing_deps:
            all_issues.append(f"Missing dependencies: {', '.join(missing_deps)}")
        
        # Determine status
        if not has_implementation:
            status = PackageStatus.EMPTY
            recommendation = "Remove or implement"
        elif not deps_available:
            status = PackageStatus.BROKEN
            recommendation = "Fix dependencies or remove"
        elif not build_success:
            status = PackageStatus.BROKEN
            recommendation = "Fix build errors"
        elif build_success and has_implementation:
            status = PackageStatus.FUNCTIONAL
            recommendation = "Keep"
        else:
            status = PackageStatus.PARTIAL
            recommendation = "Review and fix issues"
        
        return PackageInfo(
            name=package_name,
            path=str(package_path),
            status=status,
            build_success=build_success,
            dependencies_met=deps_available,
            has_implementation=has_implementation,
            duplicate_of=None,  # Will be set by duplicate detection
            issues=all_issues,
            recommendation=recommendation,
            metadata=metadata
        )
    
    def analyze_all_packages(self) -> Dict[str, PackageInfo]:
        """Analyze all packages in the workspace."""
        print("🔍 Discovering packages...")
        package_paths = self.find_packages()
        print(f"Found {len(package_paths)} packages")
        
        packages = {}
        for i, package_path in enumerate(package_paths, 1):
            print(f"📦 Analyzing package {i}/{len(package_paths)}: {package_path.name}")
            package_info = self.analyze_package(package_path)
            packages[package_info.name] = package_info
        
        self.packages = packages
        return packages
    
    def generate_report(self, output_file: str = "package_analysis_report.json"):
        """Generate a comprehensive analysis report."""
        if not self.packages:
            self.analyze_all_packages()
        
        # Create summary statistics
        status_counts = {}
        for status in PackageStatus:
            status_counts[status.value] = sum(1 for pkg in self.packages.values() if pkg.status == status)
        
        report = {
            'summary': {
                'total_packages': len(self.packages),
                'status_breakdown': status_counts,
                'functional_packages': [name for name, pkg in self.packages.items() if pkg.status == PackageStatus.FUNCTIONAL],
                'broken_packages': [name for name, pkg in self.packages.items() if pkg.status == PackageStatus.BROKEN],
                'empty_packages': [name for name, pkg in self.packages.items() if pkg.status == PackageStatus.EMPTY]
            },
            'packages': {name: asdict(pkg) for name, pkg in self.packages.items()}
        }
        
        # Write report
        output_path = self.workspace_root / output_file
        with open(output_path, 'w') as f:
            json.dump(report, f, indent=2, default=str)
        
        print(f"\n📊 Analysis complete! Report saved to: {output_path}")
        return report
    
    def print_summary(self):
        """Print a summary of the analysis results."""
        if not self.packages:
            return
        
        print("\n" + "="*60)
        print("📋 PACKAGE ANALYSIS SUMMARY")
        print("="*60)
        
        status_counts = {}
        for status in PackageStatus:
            count = sum(1 for pkg in self.packages.values() if pkg.status == status)
            if count > 0:
                status_counts[status] = count
        
        for status, count in status_counts.items():
            print(f"{status.value.upper():12}: {count:2} packages")
        
        print(f"{'TOTAL':12}: {len(self.packages):2} packages")
        
        # Show problematic packages
        broken_packages = [pkg for pkg in self.packages.values() if pkg.status == PackageStatus.BROKEN]
        if broken_packages:
            print(f"\n🚨 BROKEN PACKAGES ({len(broken_packages)}):")
            for pkg in broken_packages:
                print(f"  • {pkg.name}: {', '.join(pkg.issues[:2])}")
        
        empty_packages = [pkg for pkg in self.packages.values() if pkg.status == PackageStatus.EMPTY]
        if empty_packages:
            print(f"\n📭 EMPTY PACKAGES ({len(empty_packages)}):")
            for pkg in empty_packages:
                print(f"  • {pkg.name}")

def main():
    """Main entry point for the package status analyzer."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Analyze ROS2 package status in workspace')
    parser.add_argument('--workspace', '-w', default='.', help='Workspace root directory')
    parser.add_argument('--output', '-o', default='package_analysis_report.json', help='Output report file')
    parser.add_argument('--quiet', '-q', action='store_true', help='Suppress progress output')
    
    args = parser.parse_args()
    
    analyzer = PackageStatusAnalyzer(args.workspace)
    
    try:
        analyzer.analyze_all_packages()
        analyzer.print_summary()
        analyzer.generate_report(args.output)
        
    except KeyboardInterrupt:
        print("\n❌ Analysis interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Analysis failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()