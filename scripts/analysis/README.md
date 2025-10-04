# ROS2 Workspace Analysis Tools

This directory contains analysis tools for the ROS2 workspace cleanup process. These tools help identify package status, detect duplicates, and provide recommendations for codebase consolidation.

## Tools Overview

### 1. Package Status Analyzer (`package_status_analyzer.py`)
Analyzes all ROS2 packages in the workspace to determine their status:
- **Functional**: Package builds successfully and has implementation
- **Partial**: Package has some functionality but may have issues  
- **Broken**: Package fails to build due to missing dependencies or errors
- **Empty**: Package exists but has minimal or no implementation
- **Duplicate**: Package has overlapping functionality with other packages

### 2. Duplicate Detection Analyzer (`duplicate_detection_analyzer.py`)
Identifies packages with overlapping or duplicate functionality by analyzing:
- Package contents and file types
- URDF/XACRO robot descriptions
- Hardware interface implementations
- Launch file similarities
- Source code patterns

### 3. Analysis Runner (`run_analysis.py`)
Orchestrates the complete analysis process by running all analyzers and generating comprehensive reports.

## Usage

### Quick Start - Run Complete Analysis
```bash
# Run complete workspace analysis
cd /path/to/workspace
python3 scripts/analysis/run_analysis.py

# Results will be saved to analysis_results/ directory
```

### Individual Tool Usage

#### Package Status Analysis
```bash
# Analyze package status
python3 scripts/analysis/package_status_analyzer.py

# With custom output file
python3 scripts/analysis/package_status_analyzer.py -o my_package_report.json

# Specify workspace directory
python3 scripts/analysis/package_status_analyzer.py -w /path/to/workspace
```

#### Duplicate Detection
```bash
# Detect duplicate packages
python3 scripts/analysis/duplicate_detection_analyzer.py

# With custom similarity threshold (0.0-1.0)
python3 scripts/analysis/duplicate_detection_analyzer.py -t 0.7

# With custom output file
python3 scripts/analysis/duplicate_detection_analyzer.py -o my_duplicate_report.json
```

## Output Files

The analysis tools generate several output files in the `analysis_results/` directory:

- `package_status_report.json` - Detailed package status analysis
- `duplicate_analysis_report.json` - Duplicate detection results
- `dependency_map.json` - Package dependency relationships
- `complete_build_log.txt` - Full workspace build log
- `analysis_summary.json` - Combined summary and recommendations

## Requirements

- Python 3.6+
- ROS2 environment sourced
- `colcon` build tool available
- Workspace with ROS2 packages in `src/` directory

## Analysis Process

1. **Package Discovery**: Finds all packages by locating `package.xml` files
2. **Metadata Extraction**: Parses package.xml for dependencies and build type
3. **Implementation Check**: Analyzes source files, launch files, and configurations
4. **Build Testing**: Attempts to build each package individually
5. **Duplicate Detection**: Compares packages for similar functionality
6. **Report Generation**: Creates comprehensive JSON reports with recommendations

## Actual Duplicate Patterns Found

Based on manual verification of package contents and code logic:

**Confirmed Duplicates:**
- `robot` vs `robot_description` - Both provide robot URDF/description functionality
  - `robot/description/` contains custom URDF implementation with local xacro files
  - `robot_description/urdf/` uses husarion_components_description approach
  - Both packages serve the same purpose with different architectures

**NOT Duplicates (Incorrectly Assumed):**
- `robot_hardware` vs `mecabridge_hardware` - mecabridge_hardware is completely empty
- `robot_controller` vs `robot_controllers` - Complementary (config vs implementation)
- `robot_hardware` vs `robot_hardware_interfaces` - Different (planned vs implemented)

**Note:** Duplicate detection should focus on actual code logic overlap, not just similar naming patterns.

## Troubleshooting

### ROS2 Environment Issues
```bash
# Make sure ROS2 is sourced
source /opt/ros/humble/setup.bash

# Verify ROS2 is available
ros2 --version
```

### Build Failures
```bash
# Install missing dependencies
rosdep install --from-paths src --ignore-src -y --rosdistro=humble

# Clean build artifacts
rm -rf build install log
```

### Permission Issues
```bash
# Make scripts executable
chmod +x scripts/analysis/*.py
```

## Integration with Cleanup Workflow

These analysis tools support the codebase cleanup workflow by providing:

1. **Requirements 1.1, 1.2**: Systematic package evaluation and dependency checking
2. **Requirement 1.3**: Duplicate functionality identification
3. **Data for Requirements 2.x**: Information needed for consolidation planning
4. **Validation for Requirements 5.x**: Build status verification

The analysis results inform the subsequent cleanup phases and help ensure no working functionality is lost during consolidation.