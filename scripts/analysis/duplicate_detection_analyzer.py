#!/usr/bin/env python3
"""
Duplicate Detection Analyzer for ROS2 Workspace Cleanup

This script identifies packages with overlapping or duplicate functionality by analyzing:
- Package contents and file types
- URDF/XACRO robot descriptions
- Hardware interface implementations
- Launch file similarities
- Source code patterns

Requirements addressed: 1.3
"""

import os
import sys
import json
import hashlib
from pathlib import Path
from typing import Dict, List, Set, Tuple, Optional
from dataclasses import dataclass, asdict
from collections import defaultdict
import difflib

@dataclass
class DuplicateMatch:
    package1: str
    package2: str
    similarity_score: float
    duplicate_type: str
    evidence: List[str]
    recommendation: str

@dataclass
class PackageFingerprint:
    name: str
    path: str
    file_types: Dict[str, int]  # Extension -> count
    urdf_files: List[str]
    launch_files: List[str]
    config_files: List[str]
    source_files: List[str]
    hardware_interfaces: List[str]
    content_hash: str
    functionality_keywords: Set[str]

class DuplicateDetectionAnalyzer:
    def __init__(self, workspace_root: str = "."):
        self.workspace_root = Path(workspace_root).resolve()
        self.src_dir = self.workspace_root / "src"
        self.package_fingerprints = {}
        self.duplicate_matches = []
        
        # Only include confirmed duplicates based on actual code analysis
        self.known_duplicates = {
            ('robot', 'robot_description'): 'URDF/robot description functionality overlap'
        }
        
    def find_packages(self) -> List[Path]:
        """Find all ROS2 packages by locating package.xml files."""
        package_dirs = []
        for package_xml in self.src_dir.rglob("package.xml"):
            # Skip build artifacts
            if "build" not in str(package_xml) and "install" not in str(package_xml):
                package_dirs.append(package_xml.parent)
        return package_dirs
    
    def extract_functionality_keywords(self, package_path: Path) -> Set[str]:
        """Extract keywords that indicate package functionality."""
        keywords = set()
        
        # Keywords from package name
        name_parts = package_path.name.lower().split('_')
        keywords.update(name_parts)
        
        # Keywords from file names and content
        for file_path in package_path.rglob("*"):
            if file_path.is_file():
                filename = file_path.name.lower()
                
                # Hardware-related keywords
                if any(hw_term in filename for hw_term in ['hardware', 'hw', 'interface', 'driver']):
                    keywords.add('hardware_interface')
                
                # Controller keywords
                if any(ctrl_term in filename for ctrl_term in ['controller', 'control', 'mecanum', 'diff_drive']):
                    keywords.add('controller')
                
                # Description keywords
                if any(desc_term in filename for desc_term in ['urdf', 'xacro', 'description', 'model']):
                    keywords.add('robot_description')
                
                # Gazebo/simulation keywords
                if any(sim_term in filename for sim_term in ['gazebo', 'simulation', 'sim', 'world']):
                    keywords.add('simulation')
                
                # Navigation keywords
                if any(nav_term in filename for nav_term in ['navigation', 'nav', 'localization', 'mapping']):
                    keywords.add('navigation')
                
                # Vision keywords
                if any(vis_term in filename for vis_term in ['vision', 'camera', 'image', 'opencv']):
                    keywords.add('vision')
                
                # Nerf launcher keywords
                if any(nerf_term in filename for nerf_term in ['nerf', 'launcher', 'dart', 'fire']):
                    keywords.add('nerf_launcher')
                
                # Autonomy keywords
                if any(auto_term in filename for auto_term in ['autonomy', 'autonomous', 'behavior']):
                    keywords.add('autonomy')
        
        return keywords
    
    def analyze_urdf_content(self, package_path: Path) -> List[str]:
        """Analyze URDF/XACRO files and extract robot model information."""
        urdf_info = []
        
        urdf_files = list(package_path.rglob("*.urdf")) + list(package_path.rglob("*.xacro"))
        
        for urdf_file in urdf_files:
            try:
                content = urdf_file.read_text()
                
                # Extract robot name
                if 'robot name=' in content:
                    robot_name = content.split('robot name=')[1].split('"')[1]
                    urdf_info.append(f"robot:{robot_name}")
                
                # Check for common components
                if 'mecanum' in content.lower():
                    urdf_info.append("drive:mecanum")
                elif 'differential' in content.lower() or 'diff_drive' in content.lower():
                    urdf_info.append("drive:differential")
                
                if 'lidar' in content.lower() or 'laser' in content.lower():
                    urdf_info.append("sensor:lidar")
                
                if 'camera' in content.lower():
                    urdf_info.append("sensor:camera")
                
                if 'imu' in content.lower():
                    urdf_info.append("sensor:imu")
                
                if 'manipulator' in content.lower() or 'arm' in content.lower():
                    urdf_info.append("component:manipulator")
                
                # Add file name for reference
                urdf_info.append(f"file:{urdf_file.name}")
                
            except Exception as e:
                urdf_info.append(f"error:Could not parse {urdf_file.name}")
        
        return urdf_info
    
    def analyze_hardware_interfaces(self, package_path: Path) -> List[str]:
        """Analyze hardware interface implementations."""
        interfaces = []
        
        # Look for C++ hardware interface files
        cpp_files = list(package_path.rglob("*.cpp")) + list(package_path.rglob("*.hpp"))
        
        for cpp_file in cpp_files:
            try:
                content = cpp_file.read_text()
                
                # Check for ros2_control interfaces
                if 'hardware_interface' in content:
                    interfaces.append("ros2_control_interface")
                
                if 'SystemInterface' in content:
                    interfaces.append("system_interface")
                
                if 'ActuatorInterface' in content:
                    interfaces.append("actuator_interface")
                
                if 'SensorInterface' in content:
                    interfaces.append("sensor_interface")
                
                # Check for specific hardware types
                if any(term in content.lower() for term in ['mecanum', 'omnidirectional']):
                    interfaces.append("mecanum_drive")
                
                if any(term in content.lower() for term in ['differential', 'diff_drive']):
                    interfaces.append("differential_drive")
                
                if 'micro_ros' in content.lower() or 'microros' in content.lower():
                    interfaces.append("micro_ros")
                
            except Exception:
                pass
        
        # Look for Python hardware interfaces
        py_files = list(package_path.rglob("*.py"))
        
        for py_file in py_files:
            try:
                content = py_file.read_text()
                
                if 'hardware_interface' in content:
                    interfaces.append("python_hardware_interface")
                
                if 'micro_ros' in content.lower():
                    interfaces.append("micro_ros_python")
                
            except Exception:
                pass
        
        return interfaces
    
    def calculate_content_hash(self, package_path: Path) -> str:
        """Calculate a hash of package content for similarity detection."""
        hasher = hashlib.md5()
        
        # Hash significant files
        significant_extensions = {'.cpp', '.hpp', '.py', '.urdf', '.xacro', '.yaml', '.yml', '.launch.py'}
        
        files_to_hash = []
        for ext in significant_extensions:
            files_to_hash.extend(package_path.rglob(f"*{ext}"))
        
        # Sort files for consistent hashing
        files_to_hash.sort(key=lambda x: str(x))
        
        for file_path in files_to_hash:
            try:
                if file_path.is_file() and file_path.stat().st_size < 1024 * 1024:  # Skip files > 1MB
                    content = file_path.read_bytes()
                    hasher.update(content)
            except Exception:
                pass
        
        return hasher.hexdigest()
    
    def create_package_fingerprint(self, package_path: Path) -> PackageFingerprint:
        """Create a comprehensive fingerprint of a package."""
        
        # Count file types
        file_types = defaultdict(int)
        urdf_files = []
        launch_files = []
        config_files = []
        source_files = []
        
        for file_path in package_path.rglob("*"):
            if file_path.is_file():
                ext = file_path.suffix.lower()
                file_types[ext] += 1
                
                if ext in ['.urdf', '.xacro']:
                    urdf_files.append(file_path.name)
                elif ext == '.py' and 'launch' in file_path.name:
                    launch_files.append(file_path.name)
                elif ext in ['.yaml', '.yml']:
                    config_files.append(file_path.name)
                elif ext in ['.cpp', '.hpp', '.py']:
                    source_files.append(file_path.name)
        
        return PackageFingerprint(
            name=package_path.name,
            path=str(package_path),
            file_types=dict(file_types),
            urdf_files=urdf_files,
            launch_files=launch_files,
            config_files=config_files,
            source_files=source_files,
            hardware_interfaces=self.analyze_hardware_interfaces(package_path),
            content_hash=self.calculate_content_hash(package_path),
            functionality_keywords=self.extract_functionality_keywords(package_path)
        )
    
    def analyze_code_logic_similarity(self, fp1: PackageFingerprint, fp2: PackageFingerprint) -> Tuple[float, List[str]]:
        """Analyze actual code logic and behavior similarity between packages."""
        evidence = []
        logic_overlap_score = 0.0
        
        # Check for identical functional purposes
        pkg1_path = Path(fp1.path)
        pkg2_path = Path(fp2.path)
        
        # 1. URDF/Robot Description Logic Analysis
        urdf_overlap = self.analyze_urdf_logic_overlap(pkg1_path, pkg2_path)
        if urdf_overlap['has_overlap']:
            logic_overlap_score += 0.4  # High weight for URDF overlap
            evidence.extend(urdf_overlap['evidence'])
        
        # 2. Hardware Interface Logic Analysis  
        hw_overlap = self.analyze_hardware_interface_logic(pkg1_path, pkg2_path)
        if hw_overlap['has_overlap']:
            logic_overlap_score += 0.3  # High weight for hardware interface overlap
            evidence.extend(hw_overlap['evidence'])
        
        # 3. Controller Logic Analysis
        controller_overlap = self.analyze_controller_logic(pkg1_path, pkg2_path)
        if controller_overlap['has_overlap']:
            logic_overlap_score += 0.2  # Medium weight for controller overlap
            evidence.extend(controller_overlap['evidence'])
        
        # 4. Launch File Functional Analysis
        launch_overlap = self.analyze_launch_functionality(pkg1_path, pkg2_path)
        if launch_overlap['has_overlap']:
            logic_overlap_score += 0.1  # Lower weight for launch file overlap
            evidence.extend(launch_overlap['evidence'])
        
        return min(logic_overlap_score, 1.0), evidence
    
    def analyze_urdf_logic_overlap(self, pkg1_path: Path, pkg2_path: Path) -> Dict:
        """Check if both packages define the same robot model/description."""
        pkg1_urdf_files = list(pkg1_path.rglob("*.urdf")) + list(pkg1_path.rglob("*.xacro"))
        pkg2_urdf_files = list(pkg2_path.rglob("*.urdf")) + list(pkg2_path.rglob("*.xacro"))
        
        if not pkg1_urdf_files or not pkg2_urdf_files:
            return {'has_overlap': False, 'evidence': []}
        
        # Check for robot definition overlap
        pkg1_robots = set()
        pkg2_robots = set()
        
        for urdf_file in pkg1_urdf_files:
            try:
                content = urdf_file.read_text()
                if '<robot' in content and 'name=' in content:
                    # Extract robot name
                    robot_name = content.split('name=')[1].split('"')[1] if 'name="' in content else "unknown"
                    pkg1_robots.add(robot_name)
            except:
                pass
        
        for urdf_file in pkg2_urdf_files:
            try:
                content = urdf_file.read_text()
                if '<robot' in content and 'name=' in content:
                    robot_name = content.split('name=')[1].split('"')[1] if 'name="' in content else "unknown"
                    pkg2_robots.add(robot_name)
            except:
                pass
        
        # Check for overlap in robot definitions
        common_robots = pkg1_robots & pkg2_robots
        if common_robots or (pkg1_robots and pkg2_robots and 'robot' in pkg1_robots and 'robot' in pkg2_robots):
            return {
                'has_overlap': True,
                'evidence': [f"Both packages define robot URDF: {pkg1_path.name} and {pkg2_path.name}"]
            }
        
        return {'has_overlap': False, 'evidence': []}
    
    def analyze_hardware_interface_logic(self, pkg1_path: Path, pkg2_path: Path) -> Dict:
        """Check if both packages implement the SAME hardware interface functionality."""
        
        # Look for actual hardware interface implementations (not just usage)
        pkg1_implements_hw_interface = False
        pkg2_implements_hw_interface = False
        
        # Check for SystemInterface implementation (actual hardware interface)
        for cpp_file in pkg1_path.rglob("*.cpp"):
            try:
                content = cpp_file.read_text()
                # Look for actual SystemInterface implementation, not just usage
                if 'SystemInterface' in content and any(method in content for method in ['on_init', 'on_configure', 'read(', 'write(']):
                    pkg1_implements_hw_interface = True
                    break
            except:
                pass
        
        for cpp_file in pkg2_path.rglob("*.cpp"):
            try:
                content = cpp_file.read_text()
                if 'SystemInterface' in content and any(method in content for method in ['on_init', 'on_configure', 'read(', 'write(']):
                    pkg2_implements_hw_interface = True
                    break
            except:
                pass
        
        # Only consider it duplicate if both actually IMPLEMENT hardware interfaces
        # (not just configure or use them)
        if pkg1_implements_hw_interface and pkg2_implements_hw_interface:
            return {
                'has_overlap': True,
                'evidence': [f"Both packages implement SystemInterface: {pkg1_path.name} and {pkg2_path.name}"]
            }
        
        return {'has_overlap': False, 'evidence': []}
    
    def analyze_controller_logic(self, pkg1_path: Path, pkg2_path: Path) -> Dict:
        """Check if both packages implement the SAME controller functionality (not complementary)."""
        
        # Distinguish between controller IMPLEMENTATION vs CONFIGURATION
        pkg1_implements_controllers = False
        pkg2_implements_controllers = False
        pkg1_configures_controllers = False
        pkg2_configures_controllers = False
        
        # Check for controller implementations (C++ code)
        for cpp_file in pkg1_path.rglob("*.cpp"):
            try:
                content = cpp_file.read_text()
                if any(term in content for term in ['Controller', 'controller_interface', 'DiffDriveController', 'MecanumDriveController']):
                    pkg1_implements_controllers = True
                    break
            except:
                pass
        
        for cpp_file in pkg2_path.rglob("*.cpp"):
            try:
                content = cpp_file.read_text()
                if any(term in content for term in ['Controller', 'controller_interface', 'DiffDriveController', 'MecanumDriveController']):
                    pkg2_implements_controllers = True
                    break
            except:
                pass
        
        # Check for main controller configurations (not examples)
        for yaml_file in pkg1_path.rglob("*.yaml"):
            try:
                filename = yaml_file.name.lower()
                # Skip example/test configuration files
                if any(skip_term in filename for skip_term in ['example', 'test', 'demo']):
                    continue
                    
                content = yaml_file.read_text()
                if 'controller_manager' in content and any(term in content for term in ['type:', 'DiffDriveController', 'MecanumDriveController']):
                    pkg1_configures_controllers = True
                    break
            except:
                pass
        
        for yaml_file in pkg2_path.rglob("*.yaml"):
            try:
                filename = yaml_file.name.lower()
                # Skip example/test configuration files
                if any(skip_term in filename for skip_term in ['example', 'test', 'demo']):
                    continue
                    
                content = yaml_file.read_text()
                if 'controller_manager' in content and any(term in content for term in ['type:', 'DiffDriveController', 'MecanumDriveController']):
                    pkg2_configures_controllers = True
                    break
            except:
                pass
        
        # Only consider it a duplicate if both packages do the SAME thing
        # Implementation + Implementation = duplicate
        # Configuration + Configuration = duplicate  
        # Implementation + Configuration = complementary (NOT duplicate)
        
        if pkg1_implements_controllers and pkg2_implements_controllers:
            return {
                'has_overlap': True,
                'evidence': [f"Both packages implement controller logic: {pkg1_path.name} and {pkg2_path.name}"]
            }
        elif pkg1_configures_controllers and pkg2_configures_controllers and not (pkg1_implements_controllers or pkg2_implements_controllers):
            return {
                'has_overlap': True,
                'evidence': [f"Both packages only configure controllers: {pkg1_path.name} and {pkg2_path.name}"]
            }
        
        # Implementation + Configuration = complementary, not duplicate
        return {'has_overlap': False, 'evidence': []}
    
    def analyze_launch_functionality(self, pkg1_path: Path, pkg2_path: Path) -> Dict:
        """Check if both packages provide the SAME primary launch functionality (not examples/tests)."""
        
        pkg1_main_purposes = set()
        pkg2_main_purposes = set()
        
        # Analyze launch files, but distinguish between main functionality vs examples
        for launch_file in pkg1_path.rglob("*.launch.py"):
            try:
                content = launch_file.read_text()
                filename = launch_file.name.lower()
                
                # Skip example/test launch files
                if any(skip_term in filename for skip_term in ['example', 'test', 'demo']):
                    continue
                
                launch_purpose = None
                
                # Determine if this is a main launch file (not just an example)
                if 'robot_state_publisher' in content and 'urdf' in content.lower():
                    # Check if this is the main robot launch (multiple launch files indicate main package)
                    launch_files_count = len(list(pkg1_path.rglob("*.launch.py")))
                    if launch_files_count >= 3:  # Main robot packages typically have multiple launch files
                        launch_purpose = 'main_robot_description_launch'
                elif 'gazebo' in content.lower() and 'world' in content.lower():
                    launch_purpose = 'main_simulation_launch'
                
                if launch_purpose:
                    pkg1_main_purposes.add(launch_purpose)
            except:
                pass
        
        for launch_file in pkg2_path.rglob("*.launch.py"):
            try:
                content = launch_file.read_text()
                filename = launch_file.name.lower()
                
                # Skip example/test launch files
                if any(skip_term in filename for skip_term in ['example', 'test', 'demo']):
                    continue
                
                launch_purpose = None
                
                if 'robot_state_publisher' in content and 'urdf' in content.lower():
                    launch_files_count = len(list(pkg2_path.rglob("*.launch.py")))
                    if launch_files_count >= 3:
                        launch_purpose = 'main_robot_description_launch'
                elif 'gazebo' in content.lower() and 'world' in content.lower():
                    launch_purpose = 'main_simulation_launch'
                
                if launch_purpose:
                    pkg2_main_purposes.add(launch_purpose)
            except:
                pass
        
        # Only consider duplicate if both packages serve the same PRIMARY launch purpose
        common_purposes = pkg1_main_purposes & pkg2_main_purposes
        if common_purposes and 'main_robot_description_launch' in common_purposes:
            return {
                'has_overlap': True,
                'evidence': [f"Both packages provide main robot description launch functionality"]
            }
        
        return {'has_overlap': False, 'evidence': []}
    
    def calculate_similarity_score(self, fp1: PackageFingerprint, fp2: PackageFingerprint) -> Tuple[float, List[str]]:
        """Calculate similarity score based on actual code logic and behavior overlap."""
        
        # Focus on actual functional overlap, not superficial similarities
        logic_score, logic_evidence = self.analyze_code_logic_similarity(fp1, fp2)
        
        # Only consider packages duplicates if they have significant logic overlap
        if logic_score >= 0.3:  # Require substantial functional overlap
            return logic_score, logic_evidence
        else:
            return 0.0, []
    
    def detect_duplicates(self, similarity_threshold: float = 0.3) -> List[DuplicateMatch]:
        """Detect duplicate packages based on similarity analysis."""
        
        print("🔍 Creating package fingerprints...")
        package_paths = self.find_packages()
        
        fingerprints = {}
        for package_path in package_paths:
            print(f"  📦 Analyzing {package_path.name}")
            fingerprint = self.create_package_fingerprint(package_path)
            fingerprints[fingerprint.name] = fingerprint
        
        self.package_fingerprints = fingerprints
        
        print(f"\n🔍 Comparing {len(fingerprints)} packages for duplicates...")
        
        duplicates = []
        package_names = list(fingerprints.keys())
        
        for i, pkg1 in enumerate(package_names):
            for j, pkg2 in enumerate(package_names[i+1:], i+1):
                fp1 = fingerprints[pkg1]
                fp2 = fingerprints[pkg2]
                
                similarity_score, evidence = self.calculate_similarity_score(fp1, fp2)
                
                if similarity_score >= similarity_threshold:
                    # Determine duplicate type and recommendation
                    duplicate_type = "general_similarity"
                    recommendation = "Review for consolidation"
                    
                    # Check known duplicate patterns
                    for (name1, name2), desc in self.known_duplicates.items():
                        if (pkg1 == name1 and pkg2 == name2) or (pkg1 == name2 and pkg2 == name1):
                            duplicate_type = desc
                            recommendation = f"Consolidate {name1} and {name2}"
                            break
                    
                    # Specific recommendations based on functionality
                    common_keywords = fp1.functionality_keywords & fp2.functionality_keywords
                    if 'robot_description' in common_keywords:
                        duplicate_type = "URDF/description overlap"
                        recommendation = "Merge into single robot_description package"
                    elif 'hardware_interface' in common_keywords:
                        duplicate_type = "Hardware interface overlap"
                        recommendation = "Consolidate hardware interfaces"
                    elif 'controller' in common_keywords:
                        duplicate_type = "Controller configuration overlap"
                        recommendation = "Merge controller configurations"
                    
                    duplicate_match = DuplicateMatch(
                        package1=pkg1,
                        package2=pkg2,
                        similarity_score=similarity_score,
                        duplicate_type=duplicate_type,
                        evidence=evidence,
                        recommendation=recommendation
                    )
                    
                    duplicates.append(duplicate_match)
                    print(f"  🔄 Found duplicate: {pkg1} ↔ {pkg2} (similarity: {similarity_score:.2f})")
        
        self.duplicate_matches = duplicates
        return duplicates
    
    def generate_report(self, output_file: str = "duplicate_analysis_report.json"):
        """Generate a comprehensive duplicate detection report."""
        
        if not self.duplicate_matches:
            self.detect_duplicates()
        
        # Group duplicates by type
        duplicates_by_type = defaultdict(list)
        for match in self.duplicate_matches:
            duplicates_by_type[match.duplicate_type].append(match)
        
        report = {
            'summary': {
                'total_packages_analyzed': len(self.package_fingerprints),
                'duplicate_pairs_found': len(self.duplicate_matches),
                'duplicate_types': {dtype: len(matches) for dtype, matches in duplicates_by_type.items()}
            },
            'duplicate_matches': [asdict(match) for match in self.duplicate_matches],
            'package_fingerprints': {name: asdict(fp) for name, fp in self.package_fingerprints.items()},
            'recommendations': self.generate_consolidation_recommendations()
        }
        
        # Write report
        output_path = self.workspace_root / output_file
        with open(output_path, 'w') as f:
            json.dump(report, f, indent=2, default=str)
        
        print(f"\n📊 Duplicate analysis complete! Report saved to: {output_path}")
        return report
    
    def generate_consolidation_recommendations(self) -> Dict[str, str]:
        """Generate specific consolidation recommendations."""
        recommendations = {}
        
        # Process each duplicate match
        for match in self.duplicate_matches:
            key = f"{match.package1}_vs_{match.package2}"
            recommendations[key] = match.recommendation
        
        # Add specific known recommendations
        recommendations.update({
            'robot_description_consolidation': 'Merge robot and robot_description packages into single robot_description',
            'hardware_interface_consolidation': 'Merge robot_hardware and mecabridge_hardware into unified robot_hardware',
            'controller_consolidation': 'Merge robot_controller and robot_controllers into single robot_controllers'
        })
        
        return recommendations
    
    def print_summary(self):
        """Print a summary of duplicate detection results."""
        if not self.duplicate_matches:
            return
        
        print("\n" + "="*60)
        print("🔄 DUPLICATE DETECTION SUMMARY")
        print("="*60)
        
        print(f"Packages analyzed: {len(self.package_fingerprints)}")
        print(f"Duplicate pairs found: {len(self.duplicate_matches)}")
        
        if self.duplicate_matches:
            print(f"\n🔄 DUPLICATE PACKAGES:")
            for match in sorted(self.duplicate_matches, key=lambda x: x.similarity_score, reverse=True):
                print(f"  • {match.package1} ↔ {match.package2}")
                print(f"    Similarity: {match.similarity_score:.2f} | Type: {match.duplicate_type}")
                print(f"    Recommendation: {match.recommendation}")
                if match.evidence:
                    print(f"    Evidence: {'; '.join(match.evidence[:2])}")
                print()
        else:
            print("\n✅ No duplicate packages detected")

def main():
    """Main entry point for the duplicate detection analyzer."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Detect duplicate ROS2 packages in workspace')
    parser.add_argument('--workspace', '-w', default='.', help='Workspace root directory')
    parser.add_argument('--output', '-o', default='duplicate_analysis_report.json', help='Output report file')
    parser.add_argument('--threshold', '-t', type=float, default=0.6, help='Similarity threshold (0.0-1.0)')
    parser.add_argument('--quiet', '-q', action='store_true', help='Suppress progress output')
    
    args = parser.parse_args()
    
    analyzer = DuplicateDetectionAnalyzer(args.workspace)
    
    try:
        analyzer.detect_duplicates(args.threshold)
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