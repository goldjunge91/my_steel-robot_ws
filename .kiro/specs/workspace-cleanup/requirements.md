# Requirements Document

## Introduction

This feature focuses on systematically cleaning up the workspace by identifying and removing unnecessary files, directories, and artifacts that are no longer needed. The goal is to reduce workspace clutter, improve build times, and make the project structure clearer while ensuring no critical files or configurations are accidentally deleted.

## Requirements

### Requirement 1: Identify Obsolete Code and Directories

**User Story:** As a developer, I want to identify and remove obsolete code directories, so that the workspace only contains actively maintained code.

#### Acceptance Criteria

1. WHEN scanning the workspace THEN the system SHALL identify directories marked as obsolete (e.g., `_obsolet/`)
2. WHEN obsolete directories are found THEN the system SHALL verify they are not referenced by active code
3. WHEN verification is complete THEN the system SHALL provide a list of safe-to-delete obsolete directories
4. IF obsolete code is referenced by active code THEN the system SHALL flag it for manual review

### Requirement 2: Clean Build Artifacts

**User Story:** As a developer, I want to remove regenerable build artifacts, so that I can reclaim disk space and ensure clean builds.

#### Acceptance Criteria

1. WHEN cleaning build artifacts THEN the system SHALL identify `build/`, `install/`, and `log/` directories
2. WHEN build directories are identified THEN the system SHALL verify they can be regenerated via build commands
3. WHEN cleaning firmware builds THEN the system SHALL identify `firmware/build/` and `firmware/build_release/` directories
4. WHEN cleaning is complete THEN the system SHALL preserve release artifacts in `firmware/releases/`
5. IF build artifacts are in use THEN the system SHALL warn before deletion

### Requirement 3: Remove Unused External Dependencies

**User Story:** As a developer, I want to identify unused external dependencies, so that the workspace only contains necessary libraries.

#### Acceptance Criteria

1. WHEN scanning dependencies THEN the system SHALL check git submodules in `lib/` directory
2. WHEN checking submodules THEN the system SHALL verify if they are referenced in CMakeLists.txt or build files
3. WHEN unused submodules are found THEN the system SHALL provide a list for removal
4. WHEN removing submodules THEN the system SHALL update `.gitmodules` accordingly

### Requirement 4: Clean Temporary and Log Files

**User Story:** As a developer, I want to remove temporary files and old logs, so that the workspace remains clean and organized.

#### Acceptance Criteria

1. WHEN cleaning temporary files THEN the system SHALL identify pytest cache directories (`.pytest_cache/`)
2. WHEN cleaning logs THEN the system SHALL identify old log directories in `log/`
3. WHEN cleaning is requested THEN the system SHALL preserve the most recent log entries
4. WHEN temporary files are removed THEN the system SHALL not affect gitignored patterns

### Requirement 5: Identify Duplicate or Redundant Files

**User Story:** As a developer, I want to identify duplicate configuration files, so that I can consolidate and maintain a single source of truth.

#### Acceptance Criteria

1. WHEN scanning for duplicates THEN the system SHALL identify multiple `.env` example files
2. WHEN duplicates are found THEN the system SHALL compare their content
3. WHEN consolidation is possible THEN the system SHALL suggest which files to keep
4. IF files have different purposes THEN the system SHALL preserve all variants

### Requirement 6: Safe Deletion with Backup Strategy

**User Story:** As a developer, I want a backup strategy before deletion, so that I can recover files if needed.

#### Acceptance Criteria

1. WHEN performing cleanup THEN the system SHALL create a list of files to be deleted
2. WHEN deletion is requested THEN the system SHALL require explicit user confirmation
3. WHEN deleting files THEN the system SHALL provide an option to create a backup archive
4. WHEN backup is created THEN the system SHALL store it in a designated backup location
5. IF deletion fails THEN the system SHALL report which files could not be removed

### Requirement 7: Verify Workspace Integrity After Cleanup

**User Story:** As a developer, I want to verify the workspace still builds correctly after cleanup, so that I know no critical files were removed.

#### Acceptance Criteria

1. WHEN cleanup is complete THEN the system SHALL provide commands to verify workspace integrity
2. WHEN verification is requested THEN the system SHALL attempt to build ROS2 packages
3. WHEN verification is requested THEN the system SHALL attempt to build firmware
4. IF builds fail THEN the system SHALL report what is missing
5. WHEN all builds succeed THEN the system SHALL confirm workspace integrity
