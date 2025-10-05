#!/usr/bin/env python3
"""
Functionality Packages Analysis for Subtask 2.2
Analyzes robot_gazebo, robot_vision, robot_nerf_launcher, robot_autonomy, 
robot_localization, robot_firmware package completeness and functionality.
"""

import os
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional
from dataclasses import dataclass, asdict

@dataclass
class FunctionalityPackageInfo:
    name: str
    path: str
    exists: bool
    has_package_xml: bool
    description: str
    version: str
    build_type: str
    dependencies: List[str]
    file_count: int
    has_source_code: bool
    has_launch_files: bool
    has_config_files: bool
    has_tests: bool
    implementation_status: str
    functionality_assessment: str
    recommendation: str
    issues: List[str]

class FunctionalityPackagesAnalyzer:
    def __init__(self, workspace_root: str):
        self.workspace_root = Path(workspace_root)
        self.src_dir = self.workspace_root / "src"
        
    def analyze_functionality_packages(self) -> Dict[str, FunctionalityPackageInfo]:
        """Analyze functionality packages for completeness and implementation status."""
        
        # Define functionality packages to analyze
        functionality_packages = [
            "robot_gazebo",
            "robot_vision", 
            "robot_nerf_launcher",
            "robot_autonomy",
            "robot_localization",
            "robot_firmware"
        ]
        
        results = {}
        
        print("🔍 Analyzing Functionality Packages...")
        print("="*50)
        
        for pkg_name in functionality_packages:
            pkg_info = self._analyze_package(pkg_name)
            results[pkg_name] = pkg_info
            
            status_icon = self._get_status_icon(pkg_info.implementation_status)
            print(f"{status_icon} {pkg_name:20}: {pkg_info.implementation_status:12} - {pkg_info.recommendation}")
        
        return results
    
    def _get_status_icon(self, status: str) -> str:
        """Get appropriate icon for implementation status."""
        if "COMPLETE" in status:
            return "✅"
        elif "PARTIAL" in status:
            return "⚠️"
        elif "EMPTY" in status or "MISSING" in status:
            return "❌"
        else:
            return "🔍"
    
    def _analyze_package(self, pkg_name: str) -> FunctionalityPackageInfo:
        """Analyze a single functionality package."""
        
        pkg_path = self.src_dir / pkg_name
        
        pkg_info = FunctionalityPackageInfo(
            name=pkg_name,
            path=str(pkg_path.relative_to(self.workspace_root)),
            exists=pkg_path.exists(),
            has_package_xml=False,
            description="",
            version="",
            build_type="",
            dependencies=[],
            file_count=0,
            has_source_code=False,
            has_launch_files=False,
            has_config_files=False,
            has_tests=False,
            implementation_status="",
            functionality_assessment="",
            recommendation="",
            issues=[]
        )
        
        if not pkg_info.exists:
            pkg_info.implementation_status = "MISSING"
            pkg_info.functionality_assessment = "Package directory does not exist"
            pkg_info.recommendation = "CREATE - Package needs to be implemented"
            pkg_info.issues.append("Package directory not found")
            return pkg_info
        
        # Check for package.xml
        package_xml = pkg_path / "package.xml"
        pkg_info.has_package_xml = package_xml.exists()
        
        if not pkg_info.has_package_xml:
            pkg_info.issues.append("No package.xml found")
            pkg_info.implementation_status = "BROKEN"
            pkg_info.recommendation = "FIX - Add package.xml"
        else:
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
        
        # Analyze directory contents
        self._analyze_directory_contents(pkg_path, pkg_info)
        
        # Assess functionality based on package purpose
        self._assess_functionality(pkg_info)
        
        # Determine implementation status and recommendation
        self._determine_status_and_recommendation(pkg_info)
        
        return pkg_info
    
    def _analyze_directory_contents(self, pkg_path: Path, pkg_info: FunctionalityPackageInfo):
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
                    if 'test' in str(rel_path).lower():
                        pkg_info.has_tests = True
                    else:
                        pkg_info.has_source_code = True
                        
            except ValueError:
                # Skip files that can't be made relative to pkg_path
                continue
    
    def _assess_functionality(self, pkg_info: FunctionalityPackageInfo):
        """Assess functionality based on package purpose and expected components."""
        
        pkg_name = pkg_info.name
        
        if pkg_name == "robot_gazebo":
            # Should have world files, launch files, and gazebo configs
            expected_components = ["launch files", "world files", "gazebo configs"]
            if pkg_info.has_launch_files and pkg_info.has_config_files:
                pkg_info.functionality_assessment = "Has launch and config files for simulation"
            else:
                pkg_info.functionality_assessment = f"Missing expected components: {expected_components}"
                
        elif pkg_name == "robot_vision":
            # Should have computer vision nodes, camera configs
            expected_components = ["vision nodes", "camera configs", "OpenCV/YOLOv5 integration"]
            if pkg_info.has_source_code:
                pkg_info.functionality_assessment = "Has source code for vision processing"
            else:
                pkg_info.functionality_assessment = f"Missing expected components: {expected_components}"
                
        elif pkg_name == "robot_nerf_launcher":
            # Should have launcher control nodes, safety systems
            expected_components = ["launcher control nodes", "safety systems", "targeting logic"]
            if pkg_info.has_source_code and pkg_info.has_config_files:
                pkg_info.functionality_assessment = "Has source code and configs for Nerf launcher"
            else:
                pkg_info.functionality_assessment = f"Missing expected components: {expected_components}"
                
        elif pkg_name == "robot_autonomy":
            # Should have Nav2 configs, autonomous behaviors
            expected_components = ["Nav2 configs", "autonomous navigation", "behavior trees"]
            if pkg_info.has_config_files and pkg_info.has_launch_files:
                pkg_info.functionality_assessment = "Has configs and launch files for autonomy"
            else:
                pkg_info.functionality_assessment = f"Missing expected components: {expected_components}"
                
        elif pkg_name == "robot_localization":
            # Should have EKF configs, sensor fusion
            expected_components = ["EKF configuration", "sensor fusion", "odometry"]
            if pkg_info.has_config_files:
                pkg_info.functionality_assessment = "Has configuration for localization"
            else:
                pkg_info.functionality_assessment = f"Missing expected components: {expected_components}"
                
        elif pkg_name == "robot_firmware":
            # Should have Pico firmware, micro-ROS code
            expected_components = ["Pico firmware", "micro-ROS integration", "hardware drivers"]
            if pkg_info.has_source_code:
                pkg_info.functionality_assessment = "Has source code for firmware"
            else:
                pkg_info.functionality_assessment = f"Missing expected components: {expected_components}"
    
    def _determine_status_and_recommendation(self, pkg_info: FunctionalityPackageInfo):
        """Determine implementation status and recommendation."""
        
        if not pkg_info.exists:
            pkg_info.implementation_status = "MISSING"
            pkg_info.recommendation = "CREATE - Implement package"
            return
            
        if not pkg_info.has_package_xml:
            pkg_info.implementation_status = "BROKEN"
            pkg_info.recommendation = "FIX - Add package.xml"
            return
            
        if pkg_info.file_count <= 3:  # Only basic files
            pkg_info.implementation_status = "EMPTY"
            pkg_info.recommendation = "IMPLEMENT - Add functionality"
            return
        
        # Assess completeness based on expected functionality
        has_core_functionality = False
        
        if pkg_info.name in ["robot_gazebo", "robot_autonomy"]:
            has_core_functionality = pkg_info.has_launch_files and pkg_info.has_config_files
        elif pkg_info.name in ["robot_vision", "robot_nerf_launcher", "robot_firmware"]:
            has_core_functionality = pkg_info.has_source_code
        elif pkg_info.name == "robot_localization":
            has_core_functionality = pkg_info.has_config_files
        
        if has_core_functionality:
            if pkg_info.has_source_code and pkg_info.has_launch_files and pkg_info.has_config_files:
                pkg_info.implementation_status = "COMPLETE"
                pkg_info.recommendation = "KEEP - Fully implemented"
            else:
                pkg_info.implementation_status = "PARTIAL"
                pkg_info.recommendation = "ENHANCE - Add missing components"
        else:
            pkg_info.implementation_status = "MINIMAL"
            pkg_info.recommendation = "IMPLEMENT - Add core functionality"
    
    def generate_report(self, results: Dict[str, FunctionalityPackageInfo]) -> Dict:
        """Generate functionality packages analysis report."""
        
        # Summary statistics
        total_packages = len(results)
        existing_packages = sum(1 for pkg in results.values() if pkg.exists)
        complete_packages = sum(1 for pkg in results.values() if pkg.implementation_status == "COMPLETE")
        partial_packages = sum(1 for pkg in results.values() if pkg.implementation_status == "PARTIAL")
        empty_packages = sum(1 for pkg in results.values() if pkg.implementation_status in ["EMPTY", "MINIMAL"])
        missing_packages = sum(1 for pkg in results.values() if not pkg.exists)
        
        # Categorize by functionality area
        simulation_packages = [pkg for name, pkg in results.items() if "gazebo" in name]
        vision_packages = [pkg for name, pkg in results.items() if "vision" in name or "nerf" in name]
        navigation_packages = [pkg for name, pkg in results.items() if "autonomy" in name or "localization" in name]
        firmware_packages = [pkg for name, pkg in results.items() if "firmware" in name]
        
        report = {
            "analysis_type": "Functionality Packages Analysis (Subtask 2.2)",
            "summary": {
                "total_packages_analyzed": total_packages,
                "existing_packages": existing_packages,
                "missing_packages": missing_packages,
                "complete_packages": complete_packages,
                "partial_packages": partial_packages,
                "empty_minimal_packages": empty_packages,
                "implementation_readiness": f"{complete_packages}/{total_packages} packages complete"
            },
            "packages": {name: asdict(info) for name, info in results.items()},
            "functionality_areas": {
                "simulation": [asdict(pkg) for pkg in simulation_packages],
                "computer_vision": [asdict(pkg) for pkg in vision_packages], 
                "navigation": [asdict(pkg) for pkg in navigation_packages],
                "firmware": [asdict(pkg) for pkg in firmware_packages]
            },
            "recommendations": self._generate_recommendations(results)
        }
        
        return report
    
    def _generate_recommendations(self, results: Dict[str, FunctionalityPackageInfo]) -> List[str]:
        """Generate overall recommendations for functionality packages."""
        
        recommendations = []
        
        # Priority recommendations based on robot functionality
        missing_critical = []
        incomplete_critical = []
        
        for name, pkg in results.items():
            if not pkg.exists:
                missing_critical.append(name)
            elif pkg.implementation_status in ["EMPTY", "MINIMAL", "PARTIAL"]:
                incomplete_critical.append(name)
        
        if missing_critical:
            recommendations.append(f"HIGH PRIORITY: Create missing packages: {', '.join(missing_critical)}")
        
        if incomplete_critical:
            recommendations.append(f"MEDIUM PRIORITY: Complete implementation: {', '.join(incomplete_critical)}")
        
        # Specific functionality recommendations
        if "robot_vision" in results and results["robot_vision"].implementation_status != "COMPLETE":
            recommendations.append("IMPLEMENT: robot_vision package for face detection and computer vision")
        
        if "robot_nerf_launcher" in results and results["robot_nerf_launcher"].implementation_status != "COMPLETE":
            recommendations.append("IMPLEMENT: robot_nerf_launcher package for interactive Nerf dart system")
        
        if "robot_autonomy" in results and results["robot_autonomy"].implementation_status != "COMPLETE":
            recommendations.append("IMPLEMENT: robot_autonomy package for Nav2 autonomous navigation")
        
        return recommendations

def main():
    analyzer = FunctionalityPackagesAnalyzer(".")
    
    # Run analysis
    results = analyzer.analyze_functionality_packages()
    
    # Generate report
    report = analyzer.generate_report(results)
    
    # Print summary
    print(f"\n📊 FUNCTIONALITY PACKAGES ANALYSIS SUMMARY")
    print("="*50)
    print(f"Total packages analyzed: {report['summary']['total_packages_analyzed']}")
    print(f"Existing packages: {report['summary']['existing_packages']}")
    print(f"Missing packages: {report['summary']['missing_packages']}")
    print(f"Complete packages: {report['summary']['complete_packages']}")
    print(f"Partial packages: {report['summary']['partial_packages']}")
    print(f"Empty/minimal packages: {report['summary']['empty_minimal_packages']}")
    print(f"Implementation readiness: {report['summary']['implementation_readiness']}")
    
    print(f"\n🎯 RECOMMENDATIONS:")
    for i, rec in enumerate(report['recommendations'], 1):
        print(f"{i}. {rec}")
    
    # Save report
    output_file = Path("analysis_results") / "functionality_packages_analysis.json"
    output_file.parent.mkdir(exist_ok=True)
    
    with open(output_file, 'w') as f:
        json.dump(report, f, indent=2)
    
    print(f"\n💾 Detailed report saved to: {output_file}")
    
    return 0

if __name__ == "__main__":
    exit(main())