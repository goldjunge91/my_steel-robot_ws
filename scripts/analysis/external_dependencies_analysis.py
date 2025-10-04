#!/usr/bin/env python3
"""
External Dependencies Analysis for Subtask 2.3
Analyzes open_manipulator_x, robot-micro-ROS-Agent, and robot_utils 
for necessity, integration status, and functionality.
"""

import os
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional
from dataclasses import dataclass, asdict

@dataclass
class ExternalPackageInfo:
    name: str
    path: str
    exists: bool
    is_external: bool
    has_package_xml: bool
    description: str
    version: str
    build_type: str
    dependencies: List[str]
    file_count: int
    has_source_code: bool
    has_launch_files: bool
    has_config_files: bool
    integration_status: str
    necessity_assessment: str
    recommendation: str
    issues: List[str]
    subpackages: List[str]

class ExternalDependenciesAnalyzer:
    def __init__(self, workspace_root: str):
        self.workspace_root = Path(workspace_root)
        self.src_dir = self.workspace_root / "src"
        
    def analyze_external_dependencies(self) -> Dict[str, ExternalPackageInfo]:
        """Analyze external dependencies for necessity and integration status."""
        
        # Define external packages to analyze
        external_packages = [
            "open_manipulator_x",
            "robot-micro-ROS-Agent", 
            "robot_utils"
        ]
        
        results = {}
        
        print("🔍 Analyzing External Dependencies...")
        print("="*50)
        
        for pkg_name in external_packages:
            pkg_info = self._analyze_package(pkg_name)
            results[pkg_name] = pkg_info
            
            status_icon = self._get_status_icon(pkg_info.integration_status)
            print(f"{status_icon} {pkg_name:25}: {pkg_info.integration_status:15} - {pkg_info.recommendation}")
            
            # Show subpackages if any
            if pkg_info.subpackages:
                for subpkg in pkg_info.subpackages:
                    print(f"    └── {subpkg}")
        
        return results
    
    def _get_status_icon(self, status: str) -> str:
        """Get appropriate icon for integration status."""
        if "INTEGRATED" in status:
            return "✅"
        elif "PARTIAL" in status:
            return "⚠️"
        elif "UNUSED" in status or "UNNECESSARY" in status:
            return "❌"
        else:
            return "🔍"
    
    def _analyze_package(self, pkg_name: str) -> ExternalPackageInfo:
        """Analyze a single external package."""
        
        pkg_path = self.src_dir / pkg_name
        
        pkg_info = ExternalPackageInfo(
            name=pkg_name,
            path=str(pkg_path.relative_to(self.workspace_root)),
            exists=pkg_path.exists(),
            is_external=True,
            has_package_xml=False,
            description="",
            version="",
            build_type="",
            dependencies=[],
            file_count=0,
            has_source_code=False,
            has_launch_files=False,
            has_config_files=False,
            integration_status="",
            necessity_assessment="",
            recommendation="",
            issues=[],
            subpackages=[]
        )
        
        if not pkg_info.exists:
            pkg_info.integration_status = "MISSING"
            pkg_info.necessity_assessment = "Package directory does not exist"
            pkg_info.recommendation = "EVALUATE - Check if needed"
            pkg_info.issues.append("Package directory not found")
            return pkg_info
        
        # Check if it's a multi-package repository
        self._check_for_subpackages(pkg_path, pkg_info)
        
        # If it has subpackages, analyze the main structure differently
        if pkg_info.subpackages:
            self._analyze_multi_package_repo(pkg_path, pkg_info)
        else:
            self._analyze_single_package(pkg_path, pkg_info)
        
        # Assess necessity and integration based on package purpose
        self._assess_necessity_and_integration(pkg_info)
        
        return pkg_info
    
    def _check_for_subpackages(self, pkg_path: Path, pkg_info: ExternalPackageInfo):
        """Check if this is a multi-package repository."""
        
        if not pkg_path.exists():
            return
            
        # Look for subdirectories with package.xml files
        for item in pkg_path.iterdir():
            if item.is_dir() and not item.name.startswith('.'):
                subpkg_xml = item / "package.xml"
                if subpkg_xml.exists():
                    pkg_info.subpackages.append(item.name)
    
    def _analyze_multi_package_repo(self, pkg_path: Path, pkg_info: ExternalPackageInfo):
        """Analyze a multi-package repository."""
        
        pkg_info.integration_status = "MULTI_PACKAGE"
        
        # Count total files across all subpackages
        total_files = 0
        has_any_source = False
        has_any_launch = False
        has_any_config = False
        
        for subpkg_name in pkg_info.subpackages:
            subpkg_path = pkg_path / subpkg_name
            
            # Analyze subpackage contents
            for root, dirs, files in os.walk(subpkg_path):
                dirs[:] = [d for d in dirs if d != '.git']
                for file in files:
                    total_files += 1
                    file_path = Path(root) / file
                    
                    try:
                        rel_path = file_path.relative_to(subpkg_path)
                        
                        # Check file types
                        if rel_path.suffix in ['.cpp', '.c', '.hpp', '.h', '.py']:
                            if 'test' not in str(rel_path).lower():
                                has_any_source = True
                        elif (rel_path.suffix == '.py' and 'launch' in str(rel_path)) or rel_path.suffix == '.launch':
                            has_any_launch = True
                        elif rel_path.suffix in ['.yaml', '.yml', '.xml', '.rviz', '.xacro']:
                            has_any_config = True
                            
                    except ValueError:
                        continue
        
        pkg_info.file_count = total_files
        pkg_info.has_source_code = has_any_source
        pkg_info.has_launch_files = has_any_launch
        pkg_info.has_config_files = has_any_config
        
        # Try to get description from first subpackage
        if pkg_info.subpackages:
            first_subpkg = pkg_path / pkg_info.subpackages[0] / "package.xml"
            if first_subpkg.exists():
                try:
                    tree = ET.parse(first_subpkg)
                    root = tree.getroot()
                    pkg_info.description = root.find('description').text.strip() if root.find('description') is not None else ""
                except ET.ParseError:
                    pass
    
    def _analyze_single_package(self, pkg_path: Path, pkg_info: ExternalPackageInfo):
        """Analyze a single package."""
        
        # Check for package.xml
        package_xml = pkg_path / "package.xml"
        pkg_info.has_package_xml = package_xml.exists()
        
        if pkg_info.has_package_xml:
            # Parse package.xml
            try:
                tree = ET.parse(package_xml)
                root = tree.getroot()
                
                pkg_info.description = root.find('description').text.strip() if root.find('description') is not None else ""
                pkg_info.version = root.find('version').text.strip() if root.find('version') is not None else ""
                
                # Get build type
                export = root.find('export')
                if export is not None:
                    build_type = export.find('build_type')
                    if build_type is not None:
                        pkg_info.build_type = build_type.text.strip()
                
                # Get dependencies
                for dep_type in ['depend', 'build_depend', 'exec_depend', 'buildtool_depend']:
                    for dep in root.findall(dep_type):
                        if dep.text:
                            pkg_info.dependencies.append(dep.text.strip())
                            
            except ET.ParseError as e:
                pkg_info.issues.append(f"Invalid package.xml: {e}")
        else:
            pkg_info.issues.append("No package.xml found")
        
        # Analyze directory contents
        self._analyze_directory_contents(pkg_path, pkg_info)
    
    def _analyze_directory_contents(self, pkg_path: Path, pkg_info: ExternalPackageInfo):
        """Analyze the contents of a package directory."""
        
        if not pkg_path.exists():
            return
            
        # Count files and analyze types
        all_files = []
        for root, dirs, files in os.walk(pkg_path):
            # Skip .git directories
            dirs[:] = [d for d in dirs if d != '.git']
            for file in files:
                all_files.append(Path(root) / file)
        
        pkg_info.file_count = len(all_files)
        
        # Check for specific file types
        for file_path in all_files:
            try:
                rel_path = file_path.relative_to(pkg_path)
                
                # Launch files
                if (rel_path.suffix == '.py' and 'launch' in str(rel_path)) or rel_path.suffix == '.launch':
                    pkg_info.has_launch_files = True
                
                # Config files
                if rel_path.suffix in ['.yaml', '.yml', '.xml', '.rviz', '.xacro']:
                    pkg_info.has_config_files = True
                
                # Source code
                if rel_path.suffix in ['.cpp', '.c', '.hpp', '.h', '.py']:
                    if 'test' not in str(rel_path).lower():
                        pkg_info.has_source_code = True
                        
            except ValueError:
                # Skip files that can't be made relative to pkg_path
                continue
    
    def _assess_necessity_and_integration(self, pkg_info: ExternalPackageInfo):
        """Assess necessity and integration status based on package purpose."""
        
        pkg_name = pkg_info.name
        
        if pkg_name == "open_manipulator_x":
            # Assess if manipulator arm is needed for the robot
            if pkg_info.subpackages:
                pkg_info.necessity_assessment = "Multi-package manipulator arm system"
                if len(pkg_info.subpackages) > 2:
                    pkg_info.integration_status = "INTEGRATED"
                    pkg_info.recommendation = "EVALUATE - Check if manipulator is needed for robot functionality"
                else:
                    pkg_info.integration_status = "PARTIAL"
                    pkg_info.recommendation = "EVALUATE - Incomplete manipulator integration"
            else:
                pkg_info.necessity_assessment = "Manipulator arm package"
                pkg_info.integration_status = "UNKNOWN"
                pkg_info.recommendation = "EVALUATE - Check necessity for robot design"
                
        elif pkg_name == "robot-micro-ROS-Agent":
            # Critical for Pico communication
            if pkg_info.has_source_code:
                pkg_info.necessity_assessment = "Essential for Raspberry Pi Pico communication"
                pkg_info.integration_status = "INTEGRATED"
                pkg_info.recommendation = "KEEP - Required for micro-ROS bridge"
            else:
                pkg_info.necessity_assessment = "micro-ROS communication bridge"
                pkg_info.integration_status = "INCOMPLETE"
                pkg_info.recommendation = "FIX - Essential for Pico integration"
                
        elif pkg_name == "robot_utils":
            # Utility functions and tools
            if pkg_info.has_source_code and pkg_info.file_count > 10:
                pkg_info.necessity_assessment = "Utility functions and development tools"
                pkg_info.integration_status = "INTEGRATED"
                pkg_info.recommendation = "KEEP - Provides utility functions"
            elif pkg_info.has_source_code:
                pkg_info.necessity_assessment = "Basic utility functions"
                pkg_info.integration_status = "PARTIAL"
                pkg_info.recommendation = "ENHANCE - Add more utility functions"
            else:
                pkg_info.necessity_assessment = "Utility package placeholder"
                pkg_info.integration_status = "MINIMAL"
                pkg_info.recommendation = "IMPLEMENT - Add utility functions"
        
        # Check for integration issues
        if not pkg_info.exists:
            pkg_info.integration_status = "MISSING"
        elif pkg_info.issues:
            if "INTEGRATED" in pkg_info.integration_status:
                pkg_info.integration_status = "INTEGRATED_WITH_ISSUES"
    
    def generate_report(self, results: Dict[str, ExternalPackageInfo]) -> Dict:
        """Generate external dependencies analysis report."""
        
        # Summary statistics
        total_packages = len(results)
        existing_packages = sum(1 for pkg in results.values() if pkg.exists)
        integrated_packages = sum(1 for pkg in results.values() if "INTEGRATED" in pkg.integration_status)
        missing_packages = sum(1 for pkg in results.values() if not pkg.exists)
        
        # Categorize by necessity
        essential_packages = []
        optional_packages = []
        unnecessary_packages = []
        
        for name, pkg in results.items():
            if "Essential" in pkg.necessity_assessment or "Required" in pkg.recommendation:
                essential_packages.append(name)
            elif "EVALUATE" in pkg.recommendation or "Check" in pkg.recommendation:
                optional_packages.append(name)
            elif "REMOVE" in pkg.recommendation or "UNNECESSARY" in pkg.integration_status:
                unnecessary_packages.append(name)
            else:
                optional_packages.append(name)
        
        report = {
            "analysis_type": "External Dependencies Analysis (Subtask 2.3)",
            "summary": {
                "total_packages_analyzed": total_packages,
                "existing_packages": existing_packages,
                "missing_packages": missing_packages,
                "integrated_packages": integrated_packages,
                "essential_packages": len(essential_packages),
                "optional_packages": len(optional_packages),
                "unnecessary_packages": len(unnecessary_packages)
            },
            "packages": {name: asdict(info) for name, info in results.items()},
            "categorization": {
                "essential": essential_packages,
                "optional": optional_packages,
                "unnecessary": unnecessary_packages
            },
            "integration_assessment": self._assess_overall_integration(results),
            "recommendations": self._generate_recommendations(results)
        }
        
        return report
    
    def _assess_overall_integration(self, results: Dict[str, ExternalPackageInfo]) -> Dict:
        """Assess overall integration status of external dependencies."""
        
        integration_summary = {
            "micro_ros_status": "Unknown",
            "manipulator_status": "Unknown", 
            "utilities_status": "Unknown",
            "overall_readiness": "Unknown"
        }
        
        # Check micro-ROS integration
        micro_ros = results.get("robot-micro-ROS-Agent")
        if micro_ros:
            if "INTEGRATED" in micro_ros.integration_status:
                integration_summary["micro_ros_status"] = "Ready"
            elif micro_ros.exists:
                integration_summary["micro_ros_status"] = "Needs work"
            else:
                integration_summary["micro_ros_status"] = "Missing"
        
        # Check manipulator integration
        manipulator = results.get("open_manipulator_x")
        if manipulator:
            if "INTEGRATED" in manipulator.integration_status:
                integration_summary["manipulator_status"] = "Available"
            elif manipulator.exists:
                integration_summary["manipulator_status"] = "Partial"
            else:
                integration_summary["manipulator_status"] = "Missing"
        
        # Check utilities
        utils = results.get("robot_utils")
        if utils:
            if "INTEGRATED" in utils.integration_status:
                integration_summary["utilities_status"] = "Available"
            elif utils.exists:
                integration_summary["utilities_status"] = "Basic"
            else:
                integration_summary["utilities_status"] = "Missing"
        
        # Overall readiness
        ready_count = sum(1 for status in integration_summary.values() 
                         if status in ["Ready", "Available"])
        
        if ready_count >= 2:
            integration_summary["overall_readiness"] = "Good"
        elif ready_count >= 1:
            integration_summary["overall_readiness"] = "Partial"
        else:
            integration_summary["overall_readiness"] = "Needs work"
        
        return integration_summary
    
    def _generate_recommendations(self, results: Dict[str, ExternalPackageInfo]) -> List[str]:
        """Generate overall recommendations for external dependencies."""
        
        recommendations = []
        
        # Check micro-ROS agent (critical)
        micro_ros = results.get("robot-micro-ROS-Agent")
        if micro_ros:
            if not micro_ros.exists:
                recommendations.append("CRITICAL: Install robot-micro-ROS-Agent for Pico communication")
            elif "INCOMPLETE" in micro_ros.integration_status:
                recommendations.append("HIGH PRIORITY: Fix robot-micro-ROS-Agent integration")
        
        # Check manipulator necessity
        manipulator = results.get("open_manipulator_x")
        if manipulator and manipulator.exists:
            recommendations.append("EVALUATE: Determine if open_manipulator_x is needed for robot design")
        
        # Check utilities
        utils = results.get("robot_utils")
        if utils:
            if "MINIMAL" in utils.integration_status:
                recommendations.append("ENHANCE: Expand robot_utils with development and utility functions")
            elif not utils.exists:
                recommendations.append("CREATE: Implement robot_utils for development tools")
        
        # Integration recommendations
        integrated_count = sum(1 for pkg in results.values() if "INTEGRATED" in pkg.integration_status)
        if integrated_count < len(results) // 2:
            recommendations.append("INTEGRATION: Improve external package integration and documentation")
        
        return recommendations

def main():
    analyzer = ExternalDependenciesAnalyzer(".")
    
    # Run analysis
    results = analyzer.analyze_external_dependencies()
    
    # Generate report
    report = analyzer.generate_report(results)
    
    # Print summary
    print(f"\n📊 EXTERNAL DEPENDENCIES ANALYSIS SUMMARY")
    print("="*50)
    print(f"Total packages analyzed: {report['summary']['total_packages_analyzed']}")
    print(f"Existing packages: {report['summary']['existing_packages']}")
    print(f"Missing packages: {report['summary']['missing_packages']}")
    print(f"Integrated packages: {report['summary']['integrated_packages']}")
    print(f"Essential packages: {report['summary']['essential_packages']}")
    print(f"Optional packages: {report['summary']['optional_packages']}")
    
    print(f"\n🔗 INTEGRATION STATUS:")
    integration = report['integration_assessment']
    print(f"micro-ROS: {integration['micro_ros_status']}")
    print(f"Manipulator: {integration['manipulator_status']}")
    print(f"Utilities: {integration['utilities_status']}")
    print(f"Overall readiness: {integration['overall_readiness']}")
    
    print(f"\n🎯 RECOMMENDATIONS:")
    for i, rec in enumerate(report['recommendations'], 1):
        print(f"{i}. {rec}")
    
    # Save report
    output_file = Path("analysis_results") / "external_dependencies_analysis.json"
    output_file.parent.mkdir(exist_ok=True)
    
    with open(output_file, 'w') as f:
        json.dump(report, f, indent=2)
    
    print(f"\n💾 Detailed report saved to: {output_file}")
    
    return 0

if __name__ == "__main__":
    exit(main())