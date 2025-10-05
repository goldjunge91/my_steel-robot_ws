#!/bin/bash

# Backup Strategy Script for Codebase Cleanup
# This script creates backup branches for all repositories before making changes

set -e  # Exit on any error

# Configuration
BACKUP_DATE=$(date +%Y%m%d_%H%M%S)
BACKUP_BRANCH="backup_before_cleanup_${BACKUP_DATE}"
WORKSPACE_ROOT="$(pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# List of repositories to backup (based on ros2.repos analysis)
REPOS=(
    "robot"
    "mecabridge_hardware" 
    "robot_controller"
    "robot_controllers"
    "robot_hardware_interfaces"
    "robot_gazebo"
    "robot_localization"
    "robot_utils"
    "robot_description"
    "robot_bringup"
    "robot_autonomy"
    "robot_vision"
    "robot_nerf_launcher"
    "robot_firmware"
    "open_manipulator_x"
    "robot-micro-ROS-Agent"
)

# Function to create backup for a single repository
backup_repository() {
    local repo_name="$1"
    local repo_path="src/$repo_name"
    
    if [ ! -d "$repo_path" ]; then
        log_warning "Repository $repo_name not found at $repo_path, skipping..."
        return 0
    fi
    
    log_info "Creating backup for $repo_name..."
    
    # Navigate to repository
    cd "$repo_path"
    
    # Check if it's a git repository
    if [ ! -d ".git" ]; then
        log_warning "$repo_name is not a git repository, skipping..."
        cd "$WORKSPACE_ROOT"
        return 0
    fi
    
    # Get current branch
    current_branch=$(git branch --show-current)
    if [ -z "$current_branch" ]; then
        log_warning "$repo_name is in detached HEAD state, using HEAD for backup"
        current_branch="HEAD"
    fi
    
    # Check if repository has uncommitted changes
    if ! git diff-index --quiet HEAD --; then
        log_warning "$repo_name has uncommitted changes. Consider committing before backup."
        read -p "Continue with backup anyway? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            log_info "Skipping $repo_name due to uncommitted changes"
            cd "$WORKSPACE_ROOT"
            return 0
        fi
    fi
    
    # Create backup branch
    if git checkout -b "$BACKUP_BRANCH" 2>/dev/null; then
        log_success "Created backup branch $BACKUP_BRANCH for $repo_name"
        
        # Push backup branch to remote (if remote exists)
        if git remote get-url origin >/dev/null 2>&1; then
            if git push origin "$BACKUP_BRANCH" 2>/dev/null; then
                log_success "Pushed backup branch to remote for $repo_name"
            else
                log_warning "Failed to push backup branch to remote for $repo_name"
            fi
        else
            log_warning "No remote origin found for $repo_name, backup branch created locally only"
        fi
        
        # Return to original branch
        git checkout "$current_branch" 2>/dev/null || git checkout main 2>/dev/null || git checkout master 2>/dev/null
        
    else
        log_error "Failed to create backup branch for $repo_name"
        cd "$WORKSPACE_ROOT"
        return 1
    fi
    
    # Return to workspace root
    cd "$WORKSPACE_ROOT"
    
    return 0
}

# Function to create workspace backup
create_workspace_backup() {
    log_info "Creating workspace backup..."
    
    local backup_filename="workspace_backup_${BACKUP_DATE}.tar.gz"
    
    # Create compressed backup excluding build artifacts and git directories
    if tar -czf "$backup_filename" \
        --exclude=build \
        --exclude=install \
        --exclude=log \
        --exclude=.git \
        --exclude=__pycache__ \
        --exclude="*.pyc" \
        --exclude=".pytest_cache" \
        --exclude="node_modules" \
        . ; then
        
        log_success "Workspace backup created: $backup_filename"
        
        # Display backup size
        local backup_size=$(du -h "$backup_filename" | cut -f1)
        log_info "Backup size: $backup_size"
        
    else
        log_error "Failed to create workspace backup"
        return 1
    fi
    
    return 0
}

# Function to verify backups
verify_backups() {
    log_info "Verifying backups..."
    
    local failed_repos=()
    
    for repo in "${REPOS[@]}"; do
        local repo_path="src/$repo"
        
        if [ ! -d "$repo_path" ]; then
            continue  # Skip non-existent repositories
        fi
        
        cd "$repo_path"
        
        if [ -d ".git" ]; then
            # Check if backup branch exists
            if git branch | grep -q "$BACKUP_BRANCH"; then
                log_success "Backup verified for $repo"
            else
                log_error "Backup branch not found for $repo"
                failed_repos+=("$repo")
            fi
        fi
        
        cd "$WORKSPACE_ROOT"
    done
    
    if [ ${#failed_repos[@]} -eq 0 ]; then
        log_success "All backups verified successfully"
        return 0
    else
        log_error "Backup verification failed for: ${failed_repos[*]}"
        return 1
    fi
}

# Function to display backup summary
display_backup_summary() {
    log_info "Backup Summary"
    echo "=============================================="
    echo "Backup Date: $(date)"
    echo "Backup Branch: $BACKUP_BRANCH"
    echo "Workspace Backup: workspace_backup_${BACKUP_DATE}.tar.gz"
    echo ""
    echo "Repositories backed up:"
    
    for repo in "${REPOS[@]}"; do
        local repo_path="src/$repo"
        
        if [ -d "$repo_path/.git" ]; then
            cd "$repo_path"
            if git branch | grep -q "$BACKUP_BRANCH"; then
                echo "  ✓ $repo"
            else
                echo "  ✗ $repo (backup failed)"
            fi
            cd "$WORKSPACE_ROOT"
        elif [ -d "$repo_path" ]; then
            echo "  - $repo (not a git repository)"
        else
            echo "  - $repo (not found)"
        fi
    done
    
    echo ""
    echo "To restore from backup:"
    echo "  git checkout $BACKUP_BRANCH"
    echo ""
    echo "To remove backup branches after successful cleanup:"
    echo "  git branch -D $BACKUP_BRANCH"
    echo "  git push origin --delete $BACKUP_BRANCH"
}

# Main execution
main() {
    log_info "Starting backup process for codebase cleanup..."
    log_info "Backup branch name: $BACKUP_BRANCH"
    
    # Check if we're in the right directory
    if [ ! -f "src/ros2.repos" ]; then
        log_error "This script must be run from the workspace root directory"
        log_error "Expected to find src/ros2.repos file"
        exit 1
    fi
    
    # Create workspace backup first
    if ! create_workspace_backup; then
        log_error "Failed to create workspace backup, aborting..."
        exit 1
    fi
    
    # Create repository backups
    local backup_count=0
    local failed_count=0
    
    for repo in "${REPOS[@]}"; do
        if backup_repository "$repo"; then
            ((backup_count++))
        else
            ((failed_count++))
        fi
    done
    
    log_info "Backup process completed"
    log_info "Successful backups: $backup_count"
    log_info "Failed backups: $failed_count"
    
    # Verify backups
    if ! verify_backups; then
        log_error "Backup verification failed"
        exit 1
    fi
    
    # Display summary
    display_backup_summary
    
    if [ $failed_count -eq 0 ]; then
        log_success "All backups completed successfully!"
        log_info "You can now proceed with the codebase cleanup"
    else
        log_warning "Some backups failed. Review the errors above before proceeding."
        exit 1
    fi
}

# Script usage information
usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Creates backup branches for all repositories before codebase cleanup"
    echo ""
    echo "Options:"
    echo "  -h, --help     Show this help message"
    echo "  -v, --verify   Only verify existing backups"
    echo ""
    echo "This script should be run from the workspace root directory."
}

# Parse command line arguments
case "${1:-}" in
    -h|--help)
        usage
        exit 0
        ;;
    -v|--verify)
        verify_backups
        exit $?
        ;;
    "")
        main
        ;;
    *)
        log_error "Unknown option: $1"
        usage
        exit 1
        ;;
esac