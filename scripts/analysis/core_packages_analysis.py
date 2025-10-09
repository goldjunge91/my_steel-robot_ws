#!/usr/bin/env python3
"""
Core Robot Packages Analysis for Subtask 2.1
Analyzes robot vs robot_description, robot_hardware vs mecabridge_hardware, 
and robot_controller vs robot_controllers for overlap and functionality.
"""

import os
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List
from dataclasses import dataclass, asdict

@dataclass
class PackageInfo:
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
    functionality_overlap: List[str]
    recommendation: str
    issues: List[str]

class CorePackagesAnalyzer:
    def __init__(self, workspace_root: str):
        self.workspace_root = Path(workspace_root)
        self.src_dir = self.workspace_root / "src"
        
    def analyze_core_packages(self) -> Dict[str, PackageInfo]:
        """Analyze core robot packages for overlap and functionality."""
        
        # Define core packages to analyze
        core_packages = [
            "robot",
            "robot_description", 
            "robot_hardware",
            "mecabridge_hardware",
            "robot_controller",
            "robot_controllers"
        ]
        
        results = {}
        
        print("🔍 Analyzing Core Robot Packages...")
        print("="*50)
        
        for pkg_name in core_packages:
            pkg_info = self._analyze_package(pkg_name)
            results[pkg_name] = pkg_info
            
            status_icon = "✅" if pkg_info.exists else "❌"
            print(f"{status_icon} {pkg_name:25}: {pkg_info.recommendation}")
        
        # Analyze overlaps and duplicates
        self._analyze_overlaps(results)
        
        return results
    
    def _analyze_package(self, pkg_name: str) -> PackageInfo:
        """Analyze a single package."""
        
        pkg_path = self.src_dir / pkg_name
        
        # Handle special case for robot_controllers (it's a directory with subpackages)
        if pkg_name == "robot_controllers":
            pkg_path = self.src_dir / "robot_controllers" / "mecanum_drive_controller"
            if not pkg_path.exists():
                pkg_path = self.src_dir / "robot_controllers"
        
        pkg_info = PackageInfo(
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
            functionality_overlap=[],
            recommendation="",
            issues=[]
        )
        
        if not pkg_info.exists:
            pkg_info.recommendation = "MISSING - Package directory does not exist"
            pkg_info.issues.append("Package directory not found")
            return pkg_info
        
        # Check for package.xml
        package_xml = pkg_path / "package.xml"
        pkg_info.has_package_xml = package_xml.exists()
        
        if not pkg_info.has_package_xml:
            pkg_info.issues.append("No package.xml found")
            pkg_info.recommendation = "BROKEN - Missing package.xml"
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
        
        # Determine recommendation
        self._determine_recommendation(pkg_info)
        
        return pkg_info
    
    def _analyze_directory_contents(self, pkg_path: Path, pkg_info: PackageInfo):
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
                    # Exclude test files from source code count
                    if 'test' not in str(rel_path).lower():
                        pkg_info.has_source_code = True
                        
            except ValueError:
                # Skip files that can't be made relative to pkg_path
                continue
    
    def _determine_recommendation(self, pkg_info: PackageInfo):
        """Determine recommendation for the package."""
        
        if not pkg_info.exists:
            pkg_info.recommendation = "MISSING"
            return
            
        if not pkg_info.has_package_xml:
            pkg_info.recommendation = "BROKEN - No package.xml"
            return
            
        if pkg_info.file_count <= 3:  # Only basic files
            pkg_info.recommendation = "EMPTY - Consider removal"
            return
            
        if pkg_info.has_source_code or pkg_info.has_launch_files:
            if pkg_info.has_launch_files and pkg_info.has_config_files:
                pkg_info.recommendation = "FUNCTIONAL - Keep"
            else:
                pkg_info.recommendation = "PARTIAL - Needs completion"
        else:
            pkg_info.recommendation = "MINIMAL - Evaluate necessity"
    
    def _analyze_overlaps(self, results: Dict[str, PackageInfo]):
        """Analyze functionality overlaps between packages."""
        
        print("\n🔄 Analyzing Package Overlaps...")
        print("="*50)
        
        # robot vs robot_description overlap
        robot_pkg = results.get("robot")
        robot_desc_pkg = results.get("robot_description")
        
        if robot_pkg and robot_desc_pkg and robot_pkg.exists and robot_desc_pkg.exists:
            # Check for URDF/description overlap
            robot_has_urdf = any("urdf" in dep or "xacro" in dep for dep in robot_pkg.dependencies)
            robot_desc_has_urdf = any("urdf" in dep or "xacro" in dep for dep in robot_desc_pkg.dependencies)
            
            if robot_has_urdf and robot_desc_has_urdf:
                overlap = "Both contain URDF/description functionality"
                robot_pkg.functionality_overlap.append(overlap)
                robot_desc_pkg.functionality_overlap.append(overlap)
                print(f"⚠️  OVERLAP: robot ↔ robot_description - {overlap}")
        
        # robot_hardware vs mecabridge_hardware overlap  
        robot_hw_pkg = results.get("robot_hardware")
        mecabridge_pkg = results.get("mecabridge_hardware")
        
        if robot_hw_pkg and mecabridge_pkg:
            if not robot_hw_pkg.exists and mecabridge_pkg.exists:
                print(f"✅ robot_hardware is MISSING, mecabridge_hardware is FUNCTIONAL")
                robot_hw_pkg.recommendation = "MISSING - Use mecabridge_hardware instead"
                mecabridge_pkg.functionality_overlap.append("Replaces robot_hardware functionality")
            elif robot_hw_pkg.exists and mecabridge_pkg.exists:
                overlap = "Both provide hardware interface functionality"
                robot_hw_pkg.functionality_overlap.append(overlap)
                mecabridge_pkg.functionality_overlap.append(overlap)
                print(f"⚠️  OVERLAP: robot_hardware ↔ mecabridge_hardware - {overlap}")
        
        # robot_controller vs robot_controllers overlap
        robot_ctrl_pkg = results.get("robot_controller")  
        robot_ctrls_pkg = results.get("robot_controllers")
        
        if robot_ctrl_pkg and robot_ctrls_pkg and robot_ctrl_pkg.exists and robot_ctrls_pkg.exists:
            # Check for controller overlap
            ctrl_overlap = "Both contain controller functionality"
            robot_ctrl_pkg.functionality_overlap.append(ctrl_overlap)
            robot_ctrls_pkg.functionality_overlap.append(ctrl_overlap)
            print(f"⚠️  OVERLAP: robot_controller ↔ robot_controllers - {ctrl_overlap}")
    
    def generate_report(self, results: Dict[str, PackageInfo]) -> Dict:
        """Generate analysis report."""
        
        # Summary statistics
        total_packages = len(results)
        existing_packages = sum(1 for pkg in results.values() if pkg.exists)
        functional_packages = sum(1 for pkg in results.values() if "FUNCTIONAL" in pkg.recommendation)
        broken_packages = sum(1 for pkg in results.values() if "BROKEN" in pkg.recommendation)
        missing_packages = sum(1 for pkg in results.values() if not pkg.exists)
        
        # Overlap analysis
        overlapping_packages = sum(1 for pkg in results.values() if pkg.functionality_overlap)
        
        report = {
            "analysis_type": "Core Robot Packages Analysis (Subtask 2.1)",
            "summary": {
                "total_packages_analyzed": total_packages,
                "existing_packages": existing_packages,
                "missing_packages": missing_packages,
                "functional_packages": functional_packages,
                "broken_packages": broken_packages,
                "packages_with_overlaps": overlapping_packages
            },
            "packages": {name: asdict(info) for name, info in results.items()},
            "overlap_analysis": {
                "robot_vs_robot_description": self._get_overlap_summary(results, "robot", "robot_description"),
                "robot_hardware_vs_mecabridge_hardware": self._get_overlap_summary(results, "robot_hardware", "mecabridge_hardware"),
                "robot_controller_vs_robot_controllers": self._get_overlap_summary(results, "robot_controller", "robot_controllers")
            },
            "recommendations": self._generate_recommendations(results)
        }
        
        return report
    
    def _get_overlap_summary(self, results: Dict[str, PackageInfo], pkg1: str, pkg2: str) -> Dict:
        """Get overlap summary between two packages."""
        
        p1 = results.get(pkg1)
        p2 = results.get(pkg2)
        
        if not p1 or not p2:
            return {"status": "Cannot analyze - one or both packages missing from analysis"}
        
        return {
            "package1": {"name": pkg1, "exists": p1.exists, "recommendation": p1.recommendation},
            "package2": {"name": pkg2, "exists": p2.exists, "recommendation": p2.recommendation},
            "overlaps": p1.functionality_overlap + p2.functionality_overlap,
            "analysis": self._analyze_pair_relationship(p1, p2)
        }
    
    def _analyze_pair_relationship(self, pkg1: PackageInfo, pkg2: PackageInfo) -> str:
        """Analyze relationship between two packages."""
        
        if not pkg1.exists and not pkg2.exists:
            return "Both packages missing"
        elif not pkg1.exists and pkg2.exists:
            return f"{pkg2.name} should replace {pkg1.name}"
        elif pkg1.exists and not pkg2.exists:
            return f"{pkg1.name} is the only implementation"
        elif pkg1.functionality_overlap or pkg2.functionality_overlap:
            return "Potential duplicate functionality - consolidation needed"
        else:
            return "No significant overlap detected"
    
    def _generate_recommendations(self, results: Dict[str, PackageInfo]) -> List[str]:
        """Generate overall recommendations."""
        
        recommendations = []
        
        # Check robot vs robot_description
        robot = results.get("robot")
        robot_desc = results.get("robot_description")
        if robot and robot_desc and robot.exists and robot_desc.exists:
            if robot.functionality_overlap:
                recommendations.append("CONSOLIDATE: Merge robot and robot_description packages to eliminate duplication")
        
        # Check hardware packages
        robot_hw = results.get("robot_hardware") 
        mecabridge = results.get("mecabridge_hardware")
        if robot_hw and mecabridge:
            if not robot_hw.exists and mecabridge.exists:
                recommendations.append("REMOVE: robot_hardware directory is empty, use mecabridge_hardware")
            elif robot_hw.exists and mecabridge.exists and robot_hw.functionality_overlap:
                recommendations.append("CONSOLIDATE: Choose between robot_hardware and mecabridge_hardware")
        
        # Check controller packages
        robot_ctrl = results.get("robot_controller")
        robot_ctrls = results.get("robot_controllers") 
        if robot_ctrl and robot_ctrls and robot_ctrl.exists and robot_ctrls.exists:
            if robot_ctrl.functionality_overlap:
                recommendations.append("CONSOLIDATE: Merge robot_controller and robot_controllers functionality")
        
        return recommendations

def main():
    analyzer = CorePackagesAnalyzer(".")
    
    # Run analysis
    results = analyzer.analyze_core_packages()
    
    # Generate report
    report = analyzer.generate_report(results)
    
    # Print summary
    print(f"\n📊 CORE PACKAGES ANALYSIS SUMMARY")
    print("="*50)
    print(f"Total packages analyzed: {report['summary']['total_packages_analyzed']}")
    print(f"Existing packages: {report['summary']['existing_packages']}")
    print(f"Missing packages: {report['summary']['missing_packages']}")
    print(f"Functional packages: {report['summary']['functional_packages']}")
    print(f"Broken packages: {report['summary']['broken_packages']}")
    print(f"Packages with overlaps: {report['summary']['packages_with_overlaps']}")
    
    print(f"\n🎯 RECOMMENDATIONS:")
    for i, rec in enumerate(report['recommendations'], 1):
        print(f"{i}. {rec}")
    
    # Save report
    output_file = Path("analysis_results") / "core_packages_analysis.json"
    output_file.parent.mkdir(exist_ok=True)
    
    with open(output_file, 'w') as f:
        json.dump(report, f, indent=2)
    
    print(f"\n💾 Detailed report saved to: {output_file}")
    
    return 0

if __name__ == "__main__":
    exit(main())