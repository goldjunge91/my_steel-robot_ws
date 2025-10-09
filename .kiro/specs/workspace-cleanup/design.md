# Workspace Cleanup Design Document

## Overview

The workspace cleanup system provides a safe, systematic approach to identifying and removing unnecessary files from the ROS2 workspace. The design emphasizes safety through analysis, user confirmation, and optional backup before any deletion occurs.

## Architecture

### Component Structure

```
workspace-cleanup/
├── analysis/
│   ├── obsolete_detector.py      # Identifies obsolete directories
│   ├── build_artifact_scanner.py # Scans build artifacts
│   ├── dependency_checker.py     # Checks unused dependencies
│   ├── duplicate_finder.py       # Finds duplicate files
│   └── temp_file_scanner.py      # Identifies temporary files
├── cleanup/
│   ├── cleanup_manager.py        # Orchestrates cleanup operations
│   ├── backup_manager.py         # Handles backup creation
│   └── deletion_executor.py      # Executes safe deletion
├── verification/
│   └── integrity_checker.py      # Verifies workspace after cleanup
└── cleanup_tool.py               # Main CLI entry point
```

### Design Principles

1. **Safety First**: Never delete without explicit confirmation
2. **Dry Run by Default**: Show what would be deleted before doing it
3. **Backup Option**: Always offer to create backups
4. **Verification**: Provide tools to verify workspace integrity
5. **Transparency**: Clear reporting of all actions

## Components and Interfaces

### 1. Analysis Components

#### ObsoleteDetector
**Purpose**: Identifies directories and files marked as obsolete

**Interface**:
```python
class ObsoleteDetector:
    def scan_workspace(self, workspace_path: str) -> List[ObsoleteItem]
    def check_references(self, item: ObsoleteItem) -> bool
    def is_safe_to_delete(self, item: ObsoleteItem) -> bool
```

**Logic**:
- Scans for directories with names like `_obsolet`, `deprecated`, `old_`
- Checks if any active code imports or references these directories
- Uses grep/ripgrep to search for references in source files
- Returns list of items with safety status

#### BuildArtifactScanner
**Purpose**: Identifies regenerable build artifacts

**Interface**:
```python
class BuildArtifactScanner:
    def find_build_directories(self, workspace_path: str) -> List[BuildArtifact]
    def estimate_space_savings(self, artifacts: List[BuildArtifact]) -> int
    def is_currently_in_use(self, artifact: BuildArtifact) -> bool
```

**Logic**:
- Identifies: `build/`, `install/`, `log/`, `firmware/build/`, `firmware/build_release/`
- Preserves: `firmware/releases/` (release artifacts)
- Checks for lock files or active processes
- Calculates disk space that would be freed

#### DependencyChecker
**Purpose**: Identifies unused external dependencies

**Interface**:
```python
class DependencyChecker:
    def scan_submodules(self, workspace_path: str) -> List[Submodule]
    def check_usage(self, submodule: Submodule) -> UsageInfo
    def find_unused(self) -> List[Submodule]
```

**Logic**:
- Parses `.gitmodules` file
- Searches CMakeLists.txt files for references
- Checks package.xml files for dependencies
- Identifies submodules not referenced anywhere

#### DuplicateFinder
**Purpose**: Finds duplicate or redundant files

**Interface**:
```python
class DuplicateFinder:
    def find_duplicates(self, patterns: List[str]) -> List[DuplicateGroup]
    def compare_content(self, files: List[str]) -> bool
    def suggest_consolidation(self, group: DuplicateGroup) -> str
```

**Logic**:
- Focuses on configuration files (`.env*`, `*.yaml`, `*.json`)
- Compares file content using hash or diff
- Groups similar files together
- Suggests which to keep based on naming conventions

#### TempFileScanner
**Purpose**: Identifies temporary and cache files

**Interface**:
```python
class TempFileScanner:
    def find_temp_files(self, workspace_path: str) -> List[TempFile]
    def find_old_logs(self, log_dir: str, keep_recent: int) -> List[str]
    def is_gitignored(self, file_path: str) -> bool
```

**Logic**:
- Identifies: `.pytest_cache/`, `__pycache__/`, `*.pyc`, `.DS_Store`
- Finds old log directories (keeps most recent N)
- Respects `.gitignore` patterns
- Never deletes files tracked by git

### 2. Cleanup Components

#### CleanupManager
**Purpose**: Orchestrates the entire cleanup process

**Interface**:
```python
class CleanupManager:
    def analyze(self, options: CleanupOptions) -> CleanupPlan
    def execute(self, plan: CleanupPlan, dry_run: bool) -> CleanupResult
    def generate_report(self, result: CleanupResult) -> str
```

**Logic**:
- Coordinates all analysis components
- Builds comprehensive cleanup plan
- Manages execution flow
- Generates detailed reports

#### BackupManager
**Purpose**: Creates backups before deletion

**Interface**:
```python
class BackupManager:
    def create_backup(self, files: List[str], backup_dir: str) -> str
    def compress_backup(self, backup_path: str) -> str
    def verify_backup(self, backup_path: str) -> bool
```

**Logic**:
- Creates timestamped backup directory
- Copies files preserving directory structure
- Optionally compresses to tar.gz
- Verifies backup integrity

#### DeletionExecutor
**Purpose**: Safely executes file and directory deletion

**Interface**:
```python
class DeletionExecutor:
    def delete_files(self, files: List[str], confirm: bool) -> DeletionResult
    def delete_directories(self, dirs: List[str], confirm: bool) -> DeletionResult
    def handle_errors(self, errors: List[DeletionError]) -> None
```

**Logic**:
- Requires explicit confirmation for each category
- Handles permission errors gracefully
- Tracks successful and failed deletions
- Provides rollback information

### 3. Verification Component

#### IntegrityChecker
**Purpose**: Verifies workspace integrity after cleanup

**Interface**:
```python
class IntegrityChecker:
    def check_ros2_packages(self, workspace_path: str) -> BuildResult
    def check_firmware(self, firmware_path: str) -> BuildResult
    def verify_dependencies(self) -> DependencyResult
    def generate_verification_report(self) -> str
```

**Logic**:
- Attempts `colcon build` for ROS2 packages
- Attempts `make build` for firmware
- Checks that all required dependencies are present
- Reports any missing files or broken builds

### 4. Main CLI Tool

#### CleanupTool
**Purpose**: Command-line interface for workspace cleanup

**Interface**:
```bash
# Analyze workspace (dry run)
python3 scripts/cleanup_tool.py analyze

# Analyze specific categories
python3 scripts/cleanup_tool.py analyze --categories build,temp

# Execute cleanup with backup
python3 scripts/cleanup_tool.py cleanup --backup

# Execute cleanup without confirmation (dangerous!)
python3 scripts/cleanup_tool.py cleanup --yes --no-backup

# Verify workspace integrity
python3 scripts/cleanup_tool.py verify
```

## Data Models

### CleanupItem
```python
@dataclass
class CleanupItem:
    path: str
    category: str  # 'obsolete', 'build', 'dependency', 'duplicate', 'temp'
    size: int
    safe_to_delete: bool
    reason: str
    references: List[str]
```

### CleanupPlan
```python
@dataclass
class CleanupPlan:
    items: List[CleanupItem]
    total_size: int
    categories: Dict[str, List[CleanupItem]]
    warnings: List[str]
    timestamp: datetime
```

### CleanupResult
```python
@dataclass
class CleanupResult:
    deleted_items: List[CleanupItem]
    failed_items: List[Tuple[CleanupItem, str]]
    space_freed: int
    backup_path: Optional[str]
    duration: float
```

## Error Handling

### Error Categories

1. **Permission Errors**: File/directory cannot be deleted due to permissions
   - Action: Skip and report, suggest manual deletion with sudo

2. **In-Use Errors**: File is currently in use by a process
   - Action: Skip and report, suggest closing processes

3. **Reference Errors**: File is referenced by active code
   - Action: Do not delete, flag for manual review

4. **Backup Errors**: Backup creation fails
   - Action: Abort cleanup, report error

5. **Verification Errors**: Workspace fails to build after cleanup
   - Action: Report what's missing, suggest restoring from backup

### Error Recovery

- All errors are logged with full context
- Failed deletions are reported separately
- Backup can be used to restore if needed
- Verification step catches missing critical files

## Testing Strategy

### Unit Tests

1. **ObsoleteDetector Tests**
   - Test detection of obsolete directories
   - Test reference checking
   - Test false positive handling

2. **BuildArtifactScanner Tests**
   - Test identification of build directories
   - Test space calculation
   - Test in-use detection

3. **DependencyChecker Tests**
   - Test submodule parsing
   - Test usage detection
   - Test unused identification

4. **DuplicateFinder Tests**
   - Test duplicate detection
   - Test content comparison
   - Test consolidation suggestions

5. **TempFileScanner Tests**
   - Test temp file identification
   - Test log retention
   - Test gitignore respect

### Integration Tests

1. **Full Cleanup Workflow**
   - Create test workspace with known cleanup targets
   - Run analysis
   - Execute cleanup
   - Verify results

2. **Backup and Restore**
   - Create backup
   - Delete files
   - Restore from backup
   - Verify restoration

3. **Verification**
   - Clean workspace
   - Run verification
   - Ensure builds succeed

### Manual Testing

1. **Dry Run Testing**
   - Run analysis on real workspace
   - Verify no false positives
   - Check all categories

2. **Selective Cleanup**
   - Clean only build artifacts
   - Verify workspace still works
   - Clean other categories incrementally

3. **Full Cleanup**
   - Create backup
   - Run full cleanup
   - Verify workspace integrity
   - Test all functionality

## Configuration

### Cleanup Configuration File

Location: `.kiro/cleanup_config.yaml`

```yaml
# Workspace Cleanup Configuration

# Directories to always preserve
preserve:
  - src/
  - firmware/src/
  - docs/
  - scripts/
  - config/
  - .git/
  - .kiro/

# Directories to always clean
always_clean:
  - build/
  - install/
  - log/
  - firmware/build/
  - firmware/build_release/
  - .pytest_cache/

# Patterns for obsolete detection
obsolete_patterns:
  - _obsolet*
  - deprecated*
  - old_*
  - backup_*

# Patterns for temporary files
temp_patterns:
  - "*.pyc"
  - "__pycache__"
  - ".DS_Store"
  - "*.swp"
  - "*~"

# Log retention
log_retention:
  keep_recent: 5  # Keep 5 most recent log directories

# Backup settings
backup:
  enabled: true
  location: .kiro/backups/
  compress: true
  max_backups: 10  # Keep only 10 most recent backups

# Verification settings
verification:
  check_ros2_build: true
  check_firmware_build: true
  check_dependencies: true
```

## Implementation Notes

### Dependencies

- **Python 3.8+**: Main implementation language
- **pathlib**: Path manipulation
- **subprocess**: Running build commands
- **tarfile**: Backup compression
- **argparse**: CLI argument parsing
- **dataclasses**: Data models
- **typing**: Type hints

### Performance Considerations

1. **Large Workspaces**: Use generators for file scanning to avoid memory issues
2. **Parallel Analysis**: Can run multiple scanners in parallel
3. **Incremental Cleanup**: Support cleaning one category at a time
4. **Progress Reporting**: Show progress for long-running operations

### Security Considerations

1. **Path Traversal**: Validate all paths are within workspace
2. **Symlink Handling**: Detect and handle symlinks carefully
3. **Permission Checks**: Check permissions before attempting deletion
4. **Confirmation**: Always require confirmation for destructive operations

## Future Enhancements

1. **Interactive Mode**: TUI for selecting what to clean
2. **Scheduled Cleanup**: Automatic cleanup on schedule
3. **Cloud Backup**: Upload backups to cloud storage
4. **Cleanup Profiles**: Predefined cleanup configurations
5. **Git Integration**: Clean untracked files using git
6. **Size Visualization**: Show disk usage with charts
7. **Undo Feature**: Restore specific files from backup
8. **Cleanup History**: Track cleanup operations over time
