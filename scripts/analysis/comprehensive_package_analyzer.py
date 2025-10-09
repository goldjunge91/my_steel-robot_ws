#!/usr/bin/env python3
"""
Comprehensive Package Analyzer for ROS2 Codebase Cleanup
Analyzes all packages in src/ directory to determine their status, functionality, and relationships.
"""

import os
import sys
import json
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Set, Optional
from dataclasses import dataclass, asdict
from enum import Enum

class PackageStatus(Enum):
    FUNCTIONAL = "functional"
    PARTIAL = "partial"
    BROKEN = "broken"
    EMPTY = "empty"
    DUPLICATE = "duplicate"

@dataclass
class PackageAnalysis:
    name: str
    path: str
    status: PackageStatus
    build_success: bool
    dependencies_met: bool
    has_implementation: bool
    duplicate_of: Optional[str]
    issues: List[str]
    recommendation: str
    description: str
    dependencies: List[str]
    file_count: int
    has_launch_files: bool
    has_config_files: bool
    has_source_code: bool
    has_tests: bool
    package_type: str  # ament_cmake, ament_python, etc.

class ComprehensivePackageAnalyzer:
    def __init__(self, workspace_root: str):
        self.workspace_root = Path(workspace_root)
        self.src_dir = self.workspace_root / "src"
        self.packages: Dict[str, PackageAnalysis] = {}
        self.dependency_graph: Dict[str, Set[str]] = {}
        
    def analyze_all_packages(self) -> Dict[str, PackageAnalysis]:
        """Analyze all packages in the src directory."""
        print("🔍 Starting comprehensive package analysis...")
        
        # Find all packages
        package_dirs = self._find_package_directories()
        print(f"Found {len(package_dirs)} potential packages")
        
        # Analyze each package
        for pkg_dir in package_dirs:
            try:
                analysis = self._analyze_package(pkg_dir)
                if analysis:
                    self.packages[analysis.name] = analysis
                    print(f"✅ Analyzed {analysis.name}: {analysis.status.value}")
                else:
                    print(f"❌ Failed to analyze {pkg_dir}")
            except Exception as e:
                print(f"❌ Error analyzing {pkg_dir}: {e}")
        
        # Detect duplicates
        self._detect_duplicates()
        
        # Build dependency graph
        self._build_dependency_graph()
        
        return self.packages
    
    def _find_package_directories(self) -> List[Path]:
        """Find all directories that contain package.xml files."""
        package_dirs = []
        
        for item in self.src_dir.iterdir():
            if item.is_dir():
                package_xml = item / "package.xml"
                if package_xml.exists():
                    package_dirs.append(item)
                else:
                    # Check if it's an empty directory or git repo only
                    contents = list(item.iterdir())
                    if not contents or all(c.name.startswith('.') for c in contents):
                        # Create a dummy analysis for empty directories
                        package_dirs.append(item)
        
        return sorted(package_dirs)
    
    def _analyze_package(self, pkg_dir: Path) -> Optional[PackageAnalysis]:
        """Analyze a single package directory."""
        package_xml = pkg_dir / "package.xml"
        
        # Initialize analysis
        analysis = PackageAnalysis(
            name=pkg_dir.name,
            path=str(pkg_dir.relative_to(self.workspace_root)),
            status=PackageStatus.EMPTY,
            build_success=False,
            dependencies_met=False,
            has_implementation=False,
            duplicate_of=None,
            issues=[],
            recommendation="ANALYZE",
            description="",
            dependencies=[],
            file_count=0,
            has_launch_files=False,
            has_config_files=False,
            has_source_code=False,
            has_tests=False,
            package_type="unknown"
        )
        
        # Check if package.xml exists
        if not package_xml.exists():
            analysis.issues.append("No package.xml found")
            analysis.status = PackageStatus.EMPTY
            analysis.recommendation = "REMOVE"
            return analysis
        
        # Parse package.xml
        try:
            tree = ET.parse(package_xml)
            root = tree.getroot()
            
            analysis.description = root.find('description').text.strip() if root.find('description') is not None else ""
            
            # Get dependencies
            for dep_type in ['depend', 'build_depend', 'exec_depend', 'buildtool_depend']:
                for dep in root.findall(dep_type):
                    if dep.text:
                        analysis.dependencies.append(dep.text.strip())
            
            # Get build type
            export = root.find('export')
            if export is not None:
                build_type = export.find('build_type')
                if build_type is not None:
                    analysis.package_type = build_type.text.strip()
            
        except ET.ParseError as e:
            analysis.issues.append(f"Invalid package.xml: {e}")
            analysis.status = PackageStatus.BROKEN
            analysis.recommendation = "FIX"
            return analysis
        
        # Analyze directory contents
        self._analyze_directory_contents(pkg_dir, analysis)
        
        # Determine package status
        self._determine_package_status(analysis)
        
        # Test build (if it looks functional)
        if analysis.status in [PackageStatus.FUNCTIONAL, PackageStatus.PARTIAL]:
            analysis.build_success = self._test_package_build(pkg_dir)
            if not analysis.build_success:
                analysis.issues.append("Build failed")
                if analysis.status == PackageStatus.FUNCTIONAL:
                    analysis.status = PackageStatus.PARTIAL
        
        return analysis
    
    def _analyze_directory_contents(self, pkg_dir: Path, analysis: PackageAnalysis):
        """Analyze the contents of a package directory."""
        
        # Count all files (excluding .git)
        all_files = []
        for root, dirs, files in os.walk(pkg_dir):
            # Skip .git directories
            dirs[:] = [d for d in dirs if d != '.git']
            for file in files:
                all_files.append(Path(root) / file)
        
        analysis.file_count = len(all_files)
        
        # Check for specific file types
        for file_path in all_files:
            rel_path = file_path.relative_to(pkg_dir)
            
            # Launch files
            if rel_path.suffix == '.py' and 'launch' in str(rel_path):
                analysis.has_launch_files = True
            
            # Config files
            if rel_path.suffix in ['.yaml', '.yml', '.xml', '.rviz']:
                analysis.has_config_files = True
            
            # Source code
            if rel_path.suffix in ['.cpp', '.c', '.hpp', '.h', '.py']:
                # Exclude test files from source code count
                if 'test' not in str(rel_path).lower():
                    analysis.has_source_code = True
            
            # Test files
            if 'test' in str(rel_path).lower() and rel_path.suffix in ['.cpp', '.py']:
                analysis.has_tests = True
        
        # Check for CMakeLists.txt or setup.py
        if (pkg_dir / "CMakeLists.txt").exists():
            analysis.package_type = "ament_cmake"
        elif (pkg_dir / "setup.py").exists():
            analysis.package_type = "ament_python"
    
    def _determine_package_status(self, analysis: PackageAnalysis):
        """Determine the overall status of a package."""
        
        # Empty package
        if analysis.file_count <= 3:  # Only package.xml, CMakeLists.txt, README
            analysis.status = PackageStatus.EMPTY
            analysis.recommendation = "REMOVE"
            return
        
        # Check for implementation
        if analysis.has_source_code or analysis.has_launch_files:
            analysis.has_implementation = True
        
        # Functional package
        if (analysis.has_source_code and analysis.has_launch_files and 
            analysis.has_config_files):
            analysis.status = PackageStatus.FUNCTIONAL
            analysis.recommendation = "KEEP"
        
        # Partial implementation
        elif analysis.has_implementation:
            analysis.status = PackageStatus.PARTIAL
            analysis.recommendation = "EVALUATE"
        
        # Empty or minimal
        else:
            analysis.status = PackageStatus.EMPTY
            analysis.recommendation = "REMOVE"
    
    def _test_package_build(self, pkg_dir: Path) -> bool:
        """Test if a package builds successfully."""
        try:
            # Change to workspace root
            original_cwd = os.getcwd()
            os.chdir(self.workspace_root)
            
            # Try to build just this package
            result = subprocess.run([
                'colcon', 'build', '--packages-select', pkg_dir.name,
                '--event-handlers', 'console_direct+'
            ], capture_output=True, text=True, timeout=120)
            
            os.chdir(original_cwd)
            return result.returncode == 0
            
        except (subprocess.TimeoutExpired, FileNotFoundError):
            if 'original_cwd' in locals():
                os.chdir(original_cwd)
            return False
    
    def _detect_duplicates(self):
        """Detect packages with duplicate functionality."""
        
        # Known duplicate patterns
        duplicate_patterns = [
            (['robot', 'robot_description'], 'robot_description'),
            (['robot_hardware', 'mecabridge_hardware'], 'mecabridge_hardware'),
            (['robot_controller', 'robot_controllers'], 'robot_controllers'),
        ]
        
        for pattern, preferred in duplicate_patterns:
            found_packages = [pkg for pkg in pattern if pkg in self.packages]
            
            if len(found_packages) > 1:
                for pkg_name in found_packages:
                    if pkg_name != preferred:
                        self.packages[pkg_name].status = PackageStatus.DUPLICATE
                        self.packages[pkg_name].duplicate_of = preferred
                        self.packages[pkg_name].recommendation = "MERGE"
    
    def _build_dependency_graph(self):
        """Build a dependency graph between packages."""
        for pkg_name, analysis in self.packages.items():
            self.dependency_graph[pkg_name] = set()
            
            for dep in analysis.dependencies:
                if dep in self.packages:
                    self.dependency_graph[pkg_name].add(dep)
    
    def generate_report(self) -> Dict:
        """Generate a comprehensive analysis report."""
        
        # Status summary
        status_counts = {}
        for status in PackageStatus:
            status_counts[status.value] = sum(1 for p in self.packages.values() if p.status == status)
        
        # Recommendation summary
        recommendation_counts = {}
        for analysis in self.packages.values():
            rec = analysis.recommendation
            recommendation_counts[rec] = recommendation_counts.get(rec, 0) + 1
        
        # Core package analysis
        core_packages = {}
        for pkg_name in ['robot', 'robot_description', 'robot_hardware', 'mecabridge_hardware', 
                        'robot_controller', 'robot_controllers', 'robot_bringup']:
            if pkg_name in self.packages:
                core_packages[pkg_name] = asdict(self.packages[pkg_name])
        
        # Functionality packages
        functionality_packages = {}
        for pkg_name in ['robot_gazebo', 'robot_vision', 'robot_nerf_launcher', 'robot_autonomy',
                        'robot_localization', 'robot_firmware', 'robot_utils']:
            if pkg_name in self.packages:
                functionality_packages[pkg_name] = asdict(self.packages[pkg_name])
        
        # External dependencies
        external_packages = {}
        for pkg_name in ['open_manipulator_x', 'robot-micro-ROS-Agent']:
            if pkg_name in self.packages:
                external_packages[pkg_name] = asdict(self.packages[pkg_name])
        
        report = {
            'summary': {
                'total_packages': len(self.packages),
                'status_counts': status_counts,
                'recommendation_counts': recommendation_counts,
                'analysis_timestamp': str(Path.cwd())
            },
            'core_packages': core_packages,
            'functionality_packages': functionality_packages,
            'external_packages': external_packages,
            'dependency_graph': {k: list(v) for k, v in self.dependency_graph.items()},
            'all_packages': {name: asdict(analysis) for name, analysis in self.packages.items()}
        }
        
        return report
    
    def print_summary(self):
        """Print a summary of the analysis."""
        print("\n" + "="*80)
        print("📊 COMPREHENSIVE PACKAGE ANALYSIS SUMMARY")
        print("="*80)
        
        # Status summary
        print("\n📈 Package Status Distribution:")
        for status in PackageStatus:
            count = sum(1 for p in self.packages.values() if p.status == status)
            print(f"  {status.value.upper():12}: {count:2d} packages")
        
        # Recommendations
        print("\n🎯 Recommendations:")
        rec_counts = {}
        for analysis in self.packages.values():
            rec = analysis.recommendation
            rec_counts[rec] = rec_counts.get(rec, 0) + 1
        
        for rec, count in sorted(rec_counts.items()):
            print(f"  {rec:12}: {count:2d} packages")
        
        # Core packages analysis
        print("\n🔧 Core Robot Packages Analysis:")
        core_packages = ['robot', 'robot_description', 'robot_hardware', 'mecabridge_hardware', 
                        'robot_controller', 'robot_controllers', 'robot_bringup']
        
        for pkg_name in core_packages:
            if pkg_name in self.packages:
                analysis = self.packages[pkg_name]
                status_icon = "✅" if analysis.status == PackageStatus.FUNCTIONAL else "⚠️" if analysis.status == PackageStatus.PARTIAL else "❌"
                print(f"  {status_icon} {pkg_name:20}: {analysis.status.value:10} - {analysis.recommendation}")
        
        # Duplicates
        duplicates = [p for p in self.packages.values() if p.status == PackageStatus.DUPLICATE]
        if duplicates:
            print("\n🔄 Duplicate Packages Found:")
            for dup in duplicates:
                print(f"  ❌ {dup.name:20}: duplicate of {dup.duplicate_of}")
        
        print("\n" + "="*80)

def main():
    if len(sys.argv) > 1:
        workspace_root = sys.argv[1]
    else:
        workspace_root = os.getcwd()
    
    analyzer = ComprehensivePackageAnalyzer(workspace_root)
    
    # Run analysis
    analyzer.analyze_all_packages()
    
    # Print summary
    analyzer.print_summary()
    
    # Generate detailed report
    report = analyzer.generate_report()
    
    # Save report
    output_file = Path("analysis_results") / "comprehensive_package_analysis.json"
    output_file.parent.mkdir(exist_ok=True)
    
    with open(output_file, 'w') as f:
        json.dump(report, f, indent=2)
    
    print(f"\n💾 Detailed report saved to: {output_file}")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())