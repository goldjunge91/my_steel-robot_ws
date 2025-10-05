# Robot Package Restructuring Plan

## Overview

This document details the specific plan for restructuring the legacy `robot` package into a modern repo collection format. The restructuring will eliminate overlap with robot_description while maintaining essential system orchestration functionality.

## Current State Analysis

### Existing robot Package Structure
```
robot/
├── config/
│   ├── my_steel/
│   │   ├── mecanum_drive_controller.yaml
│   │   └── diff_drive_controller.yaml
│   └── robot_xl/
│       ├── mecanum_drive_controller.yaml
│       └── diff_drive_controller.yaml
├── description/
│   ├── my_steel.urdf.xacro
│   ├── robot_xl.urdf.xacro
│   └── components/
├── launch/
│   ├── bringup.launch.py
│   ├── microros.launch.py
│   └── simulation.launch.py
├── worlds/
│   ├── empty_world.world
│   └── test_world.world
├── scripts/
├── docs/
└── package.xml
```

### Issues Identified
1. **Overlap with robot_description**: URDF files duplicated
2. **Configuration scatter**: Controller configs should be in robot_controller
3. **Legacy structure**: Not following modern ROS2 repo collection patterns
4. **Unclear purpose**: Package serves multiple conflicting roles

## Target Structure Design

### New robot Package Structure
```
robot/
├── CMakeLists.txt
├── package.xml
├── manipulator.repos
├── robot_hardware.repos
├── robot_simulation.repos
├── nerf_launcher.repos
├── launch/
│   └── system_orchestration.launch.py
└── README.md
```

### Repo Collection Files Content

#### manipulator.repos
```yaml
# Manipulator-related repositories for my_steel robot
repositories:
  open_manipulator_x:
    type: git
    url: https://github.com/goldjunge91/open_manipulator_x.git
    version: main
```

#### robot_hardware.repos
```yaml
# Hardware interface repositories for my_steel robot
repositories:
  robot_hardware_interfaces:
    type: git
    url: https://github.com/goldjunge91/robot_hardware_interfaces.git
    version: main
  robot-micro-ROS-Agent:
    type: git
    url: https://github.com/goldjunge91/robot-micro-ROS-Agent.git
    version: humble
```

#### robot_simulation.repos
```yaml
# Simulation-related repositories for my_steel robot
repositories:
  robot_gazebo:
    type: git
    url: https://github.com/goldjunge91/robot_gazebo.git
    version: humble
```

#### nerf_launcher.repos
```yaml
# Nerf launcher related repositories for my_steel robot
# Note: Purpose and content to be clarified based on project requirements
repositories:
  # Potential candidates (if implemented):
  # robot_nerf_launcher:
  #   type: git
  #   url: https://github.com/goldjunge91/robot_nerf_launcher.git
  #   version: main
  # robot_vision:
  #   type: git
  #   url: https://github.com/goldjunge91/robot_vision.git
  #   version: main
```

## Migration Strategy

### Step 1: Content Analysis and Redistribution

#### URDF/Description Files
- **Action**: Remove from robot package
- **Destination**: Ensure robot_description has all necessary models
- **Validation**: Verify robot_description contains my_steel.urdf.xacro and robot_xl.urdf.xacro

#### Configuration Files
- **Action**: Move to robot_controller package
- **Source**: `robot/config/`
- **Destination**: `robot_controller/config/`
- **Update**: Launch file references to new locations

#### Launch Files
- **Keep**: System orchestration launch files
- **Remove**: Specific functionality launch files (move to appropriate packages)
- **Update**: References to moved configurations and descriptions

#### World Files
- **Action**: Move to robot_gazebo package
- **Source**: `robot/worlds/`
- **Destination**: `robot_gazebo/worlds/`
- **Update**: Simulation launch file references

### Step 2: Package.xml Transformation

#### Current package.xml Issues
- Generic description and maintainer information
- Dependencies that should be managed by repo collections
- Build type appropriate for legacy structure

#### New package.xml Structure
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot</name>
  <version>1.0.0</version>
  <description>Repo collection manager for my_steel robot system</description>
  <maintainer email="[maintainer_email]">[Maintainer Name]</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Repo collection dependencies -->
  <exec_depend>vcs</exec_depend>
  
  <!-- System orchestration dependencies -->
  <exec_depend>robot_description</exec_depend>
  <exec_depend>robot_bringup</exec_depend>
  <exec_depend>robot_controller</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Step 3: Integration with robot_description

#### Verification Checklist
- [ ] robot_description contains my_steel.urdf.xacro
- [ ] robot_description contains robot_xl.urdf.xacro  
- [ ] robot_description has all necessary component descriptions
- [ ] robot_description meshes and configurations are complete

#### Integration Actions
1. **Audit robot_description content** against robot package descriptions
2. **Identify missing components** and add to robot_description
3. **Remove duplicate content** from robot package
4. **Update launch file references** to use robot_description models
5. **Test model loading** from robot_description

### Step 4: Launch File Updates

#### System Orchestration Launch File
Create `launch/system_orchestration.launch.py`:
```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    """
    System orchestration launch file for my_steel robot.
    
    This launch file coordinates the startup of all robot subsystems
    using the repo collection structure.
    """
    
    # Include robot bringup
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'launch',
                'bringup.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        robot_bringup,
    ])
```

## Clarification Requirements

### nerf_launcher.repos Purpose

**Current Status**: Unclear what repositories should be included

**Questions to Resolve**:
1. Should robot_nerf_launcher be implemented as a separate package?
2. Should robot_vision be included for targeting functionality?
3. Are there other Nerf launcher related components?
4. How does this integrate with robot_hardware_interfaces?

**Recommended Approach**:
1. **Phase 1**: Leave nerf_launcher.repos empty with TODO comments
2. **Phase 2**: Clarify requirements with stakeholders
3. **Phase 3**: Implement based on clarified requirements

### Integration Points

#### With robot_controller
- **Configuration Management**: Ensure robot_controller handles all controller configs
- **Launch Coordination**: Coordinate launch file hierarchy
- **Parameter Management**: Clear parameter ownership

#### With robot_hardware_interfaces  
- **Hardware Abstraction**: Ensure proper hardware interface integration
- **Nerf Launcher Hardware**: Clarify if Nerf launcher hardware belongs here
- **Sensor Integration**: Coordinate sensor interface management

#### With robot_gazebo
- **World Files**: Transfer Gazebo world files
- **Simulation Launch**: Coordinate simulation startup
- **Model Integration**: Ensure proper model loading in simulation

## Implementation Steps

### Phase 1: Preparation
1. **Create backup branch** for robot repository
2. **Audit current content** and document all files
3. **Verify robot_description completeness** 
4. **Plan content redistribution** to appropriate packages

### Phase 2: Content Migration
1. **Move configuration files** to robot_controller
2. **Move world files** to robot_gazebo  
3. **Remove duplicate URDF files** (keep in robot_description)
4. **Update file references** in launch files

### Phase 3: Structure Transformation
1. **Create repo collection files** (manipulator.repos, etc.)
2. **Update package.xml** to reflect new purpose
3. **Create system orchestration launch file**
4. **Update CMakeLists.txt** for new structure

### Phase 4: Integration Testing
1. **Test repo collection import** with vcs tool
2. **Verify launch file functionality** 
3. **Test system orchestration** launch
4. **Validate integration** with other packages

### Phase 5: Documentation
1. **Create comprehensive README.md**
2. **Document repo collection usage**
3. **Update architecture documentation**
4. **Create migration guide** for users

## Validation Criteria

### Technical Validation
- [ ] All repo collection files import successfully with vcs
- [ ] System orchestration launch file executes without errors
- [ ] No duplicate content between robot and robot_description
- [ ] All moved configurations accessible from new locations
- [ ] Integration with robot_controller and robot_gazebo functional

### Functional Validation  
- [ ] Robot system starts up correctly using new structure
- [ ] All hardware interfaces remain functional
- [ ] Simulation launches properly with moved world files
- [ ] Controller configurations load from new locations
- [ ] No regression in system functionality

### Documentation Validation
- [ ] README.md clearly explains repo collection usage
- [ ] All file movements documented
- [ ] Integration points with other packages documented
- [ ] Migration guide available for users
- [ ] Architecture documentation updated

## Post-Restructuring Maintenance

### Ongoing Responsibilities
- **Repo Collection Management**: Maintain .repos files with current repository versions
- **System Orchestration**: Maintain high-level launch files for system startup
- **Integration Coordination**: Ensure proper integration between subsystem packages

### Development Workflow
1. **Repository Updates**: Update .repos files when adding/removing repositories
2. **Version Management**: Coordinate version updates across repo collections
3. **Launch Coordination**: Update system orchestration for new subsystems
4. **Documentation**: Keep README.md current with repo collection changes

### Quality Assurance
- Regular testing of repo collection imports
- Validation of system orchestration launch files
- Integration testing with all subsystem packages
- Documentation review and updates