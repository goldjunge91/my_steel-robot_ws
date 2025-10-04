#!/usr/bin/env python3
"""
Main Analysis Runner for ROS2 Workspace Cleanup

This script orchestrates the complete analysis of the ROS2 workspace by running:
1. Package status analysis
2. Duplicate detection analysis  
3. Build testing automation
4. Dependency mapping

Requirements addressed: 1.1, 1.2
"""

import os
import sys
import json
import subprocess
from pathlib import Path
from datetime import datetime

# Add the analysis directory to Python path
sys.path.insert(0, str(Path(__file__).parent))

from package_status_analyzer import PackageStatusAnalyzer
from duplicate_detection_analyzer import DuplicateDetectionAnalyzer

class WorkspaceAnalysisRunner:
    def __init__(self, workspace_root: str = "."):
        self.workspace_root = Path(workspace_root).resolve()
        self.output_dir = self.workspace_root / "analysis_results"
        self.output_dir.mkdir(exist_ok=True)
        
        # Initialize analyzers
        self.package_analyzer = PackageStatusAnalyzer(workspace_root)
        self.duplicate_analyzer = DuplicateDetectionAnalyzer(workspace_root)
        
    def setup_environment(self):
        """Setup ROS2 environment for analysis."""
        print("🔧 Setting up ROS2 environment...")
        
        # Check if ROS2 is sourced
        try:
            result = subprocess.run(['ros2', '--help'], capture_output=True, text=True)
            if result.returncode == 0:
                print(f"✅ ROS2 detected: {result.stdout.strip()}")
                return True
            else:
                print("❌ ROS2 not found in PATH")
                return False
        except FileNotFoundError:
            print("❌ ROS2 not found - please source ROS2 environment")
            return False
    
    def run_build_test_all(self):
        """Run a complete build test of all packages."""
        print("\n🔨 Running complete workspace build test...")
        
        try:
            # Clean previous build
            build_dir = self.workspace_root / "build"
            install_dir = self.workspace_root / "install"
            
            if build_dir.exists():
                print("  🧹 Cleaning previous build...")
                subprocess.run(['rm', '-rf', str(build_dir)], check=True)
            
            if install_dir.exists():
                subprocess.run(['rm', '-rf', str(install_dir)], check=True)
            
            # Run colcon build
            print("  🔨 Building all packages...")
            result = subprocess.run([
                'colcon', 'build', '--symlink-install',
                '--event-handlers', 'console_direct+'
            ], 
            cwd=self.workspace_root,
            capture_output=True, 
            text=True,
            timeout=1800  # 30 minute timeout
            )
            
            build_success = result.returncode == 0
            
            # Save build log
            build_log_file = self.output_dir / "complete_build_log.txt"
            with open(build_log_file, 'w') as f:
                f.write(f"Build completed at: {datetime.now()}\n")
                f.write(f"Return code: {result.returncode}\n\n")
                f.write("STDOUT:\n")
                f.write(result.stdout)
                f.write("\nSTDERR:\n")
                f.write(result.stderr)
            
            if build_success:
                print("  ✅ Complete build successful!")
            else:
                print("  ❌ Complete build failed - see build log for details")
            
            return build_success, str(build_log_file)
            
        except subprocess.TimeoutExpired:
            print("  ⏰ Build timeout - build process took too long")
            return False, "Build timeout"
        except Exception as e:
            print(f"  ❌ Build test failed: {e}")
            return False, str(e)
    
    def create_dependency_map(self):
        """Create a dependency map of all packages."""
        print("\n🗺️  Creating dependency map...")
        
        packages = self.package_analyzer.packages
        if not packages:
            print("  ⚠️  No package analysis available - run package analysis first")
            return {}
        
        dependency_map = {}
        
        for pkg_name, pkg_info in packages.items():
            dependencies = pkg_info.metadata.get('dependencies', {})
            
            # Flatten all dependency types
            all_deps = []
            for dep_type, deps in dependencies.items():
                all_deps.extend(deps)
            
            # Filter to only workspace packages
            workspace_deps = []
            for dep in all_deps:
                if dep in packages:
                    workspace_deps.append(dep)
            
            dependency_map[pkg_name] = {
                'depends_on': workspace_deps,
                'depended_by': []
            }
        
        # Calculate reverse dependencies
        for pkg_name, deps in dependency_map.items():
            for dep in deps['depends_on']:
                if dep in dependency_map:
                    dependency_map[dep]['depended_by'].append(pkg_name)
        
        # Save dependency map
        dep_map_file = self.output_dir / "dependency_map.json"
        with open(dep_map_file, 'w') as f:
            json.dump(dependency_map, f, indent=2)
        
        print(f"  📊 Dependency map saved to: {dep_map_file}")
        return dependency_map
    
    def run_complete_analysis(self):
        """Run the complete workspace analysis."""
        print("🚀 Starting complete ROS2 workspace analysis...")
        print(f"📁 Workspace: {self.workspace_root}")
        print(f"📊 Results will be saved to: {self.output_dir}")
        
        # Setup environment
        if not self.setup_environment():
            print("❌ Environment setup failed - continuing with limited analysis")
        
        analysis_results = {
            'timestamp': datetime.now().isoformat(),
            'workspace_root': str(self.workspace_root),
            'analysis_steps': {}
        }
        
        try:
            # Step 1: Package Status Analysis
            print("\n" + "="*60)
            print("📦 STEP 1: Package Status Analysis")
            print("="*60)
            
            self.package_analyzer.analyze_all_packages()
            self.package_analyzer.print_summary()
            
            package_report = self.package_analyzer.generate_report(
                str(self.output_dir / "package_status_report.json")
            )
            analysis_results['analysis_steps']['package_status'] = {
                'status': 'completed',
                'report_file': 'package_status_report.json',
                'summary': package_report['summary']
            }
            
            # Step 2: Duplicate Detection Analysis
            print("\n" + "="*60)
            print("🔄 STEP 2: Duplicate Detection Analysis")
            print("="*60)
            
            self.duplicate_analyzer.detect_duplicates()
            self.duplicate_analyzer.print_summary()
            
            duplicate_report = self.duplicate_analyzer.generate_report(
                str(self.output_dir / "duplicate_analysis_report.json")
            )
            analysis_results['analysis_steps']['duplicate_detection'] = {
                'status': 'completed',
                'report_file': 'duplicate_analysis_report.json',
                'summary': duplicate_report['summary']
            }
            
            # Step 3: Dependency Mapping
            print("\n" + "="*60)
            print("🗺️  STEP 3: Dependency Mapping")
            print("="*60)
            
            dependency_map = self.create_dependency_map()
            analysis_results['analysis_steps']['dependency_mapping'] = {
                'status': 'completed',
                'report_file': 'dependency_map.json',
                'total_packages': len(dependency_map)
            }
            
            # Step 4: Complete Build Test
            print("\n" + "="*60)
            print("🔨 STEP 4: Complete Build Test")
            print("="*60)
            
            build_success, build_log = self.run_build_test_all()
            analysis_results['analysis_steps']['build_test'] = {
                'status': 'completed' if build_success else 'failed',
                'build_success': build_success,
                'build_log': build_log
            }
            
            # Generate final summary report
            self.generate_final_report(analysis_results)
            
            print("\n" + "="*60)
            print("✅ ANALYSIS COMPLETE")
            print("="*60)
            print(f"📊 All results saved to: {self.output_dir}")
            
        except Exception as e:
            print(f"\n❌ Analysis failed: {e}")
            analysis_results['error'] = str(e)
            
            # Save partial results
            summary_file = self.output_dir / "analysis_summary.json"
            with open(summary_file, 'w') as f:
                json.dump(analysis_results, f, indent=2, default=str)
            
            raise
    
    def generate_final_report(self, analysis_results):
        """Generate a final summary report combining all analysis results."""
        
        # Load individual reports
        package_report_file = self.output_dir / "package_status_report.json"
        duplicate_report_file = self.output_dir / "duplicate_analysis_report.json"
        dependency_map_file = self.output_dir / "dependency_map.json"
        
        final_report = {
            'analysis_summary': analysis_results,
            'recommendations': [],
            'next_steps': []
        }
        
        # Generate recommendations based on analysis results
        recommendations = []
        
        # Package status recommendations
        if package_report_file.exists():
            with open(package_report_file) as f:
                pkg_report = json.load(f)
            
            broken_count = pkg_report['summary']['status_breakdown'].get('broken', 0)
            empty_count = pkg_report['summary']['status_breakdown'].get('empty', 0)
            
            if broken_count > 0:
                recommendations.append(f"Fix or remove {broken_count} broken packages")
            if empty_count > 0:
                recommendations.append(f"Remove or implement {empty_count} empty packages")
        
        # Duplicate detection recommendations
        if duplicate_report_file.exists():
            with open(duplicate_report_file) as f:
                dup_report = json.load(f)
            
            duplicate_count = dup_report['summary']['duplicate_pairs_found']
            if duplicate_count > 0:
                recommendations.append(f"Consolidate {duplicate_count} duplicate package pairs")
        
        final_report['recommendations'] = recommendations
        
        # Next steps
        next_steps = [
            "Review package status report and fix broken packages",
            "Review duplicate detection report and plan consolidations",
            "Create backup branches before making changes",
            "Implement package consolidations incrementally",
            "Update all configuration files and references",
            "Run final build validation after cleanup"
        ]
        final_report['next_steps'] = next_steps
        
        # Save final report
        summary_file = self.output_dir / "analysis_summary.json"
        with open(summary_file, 'w') as f:
            json.dump(final_report, f, indent=2, default=str)
        
        print(f"\n📋 Final summary report saved to: {summary_file}")
        
        # Print key recommendations
        if recommendations:
            print(f"\n🎯 KEY RECOMMENDATIONS:")
            for i, rec in enumerate(recommendations, 1):
                print(f"  {i}. {rec}")

def main():
    """Main entry point for the workspace analysis runner."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Run complete ROS2 workspace analysis')
    parser.add_argument('--workspace', '-w', default='.', help='Workspace root directory')
    parser.add_argument('--skip-build', action='store_true', help='Skip the complete build test')
    
    args = parser.parse_args()
    
    runner = WorkspaceAnalysisRunner(args.workspace)
    
    try:
        runner.run_complete_analysis()
        
    except KeyboardInterrupt:
        print("\n❌ Analysis interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Analysis failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()