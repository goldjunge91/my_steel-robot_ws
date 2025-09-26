#!/usr/bin/env bash
set -euo pipefail

# create_github_repos.sh
# - Creates a list of GitHub repos (public) under the authenticated user (or provided owner)
# - Ensures entries exist in src/ros2.repos
# - Pushes local folder contents to the created remote (initializes git & commits if needed)

REPOS=(
        test1
)

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_REPOS_FILE="$ROOT_DIR/src/ros2.repos"

# Optional flags
CMD_OWNER=""

usage() {
  echo "Usage: $0 [--owner OWNER]"
  exit 1
}

# parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    --owner)
      CMD_OWNER="$2"; shift 2;;
    -h|--help)
      usage;;
    *)
      echo "Unknown arg: $1"; usage;;
  esac
done

if ! command -v gh >/dev/null 2>&1; then
  echo "Error: gh (GitHub CLI) is not installed or not on PATH. Install/enable it and authenticate (gh auth login)."
  exit 1
fi

if [ -n "$CMD_OWNER" ]; then
  owner="$CMD_OWNER"
else
  owner=$(gh api user --jq .login) || { echo "Failed to determine GitHub username via 'gh api user'"; exit 1; }
fi

echo "Using GitHub owner: $owner"

mkdir -p "$(dirname "$ROS2_REPOS_FILE")"

# Ensure ros2.repos starts with repositories: if missing
if [ ! -f "$ROS2_REPOS_FILE" ]; then
  cat > "$ROS2_REPOS_FILE" <<EOF
repositories:
EOF
fi

for repo in "${REPOS[@]}"; do
  echo "\n--- Processing: $repo ---"
  remote_spec="${owner}/${repo}"
  remote_url="git@github.com:${remote_spec}.git"

  # Create GitHub repo if missing
  if gh repo view "$remote_spec" >/dev/null 2>&1; then
    echo "Remote $remote_spec already exists on GitHub"
  else
    echo "Creating GitHub repo $remote_spec (public)..."
    gh repo create "$repo" --public --confirm || { echo "Failed to create $remote_spec"; exit 1; }
  fi

  # Ensure ros2.repos contains an entry for the repo
  if grep -E "^[[:space:]]*${repo}:" "$ROS2_REPOS_FILE" >/dev/null 2>&1; then
    echo "Entry for $repo already exists in $ROS2_REPOS_FILE"
  else
    cat >> "$ROS2_REPOS_FILE" <<EOF
  ${repo}:
    type: git
    url: ${remote_url}
    version: main
EOF
    echo "Appended entry for $repo to $ROS2_REPOS_FILE"
  fi

  # Prepare and push local folder contents (no cloning)
  local_dir="$ROOT_DIR/src/$repo"
  if [ -d "$local_dir" ]; then
    echo "Preparing local folder $local_dir to push to $remote_url"
    if [ -d "$local_dir/.git" ]; then
      echo "Local git exists. Setting origin to $remote_url"
      git -C "$local_dir" remote remove origin 2>/dev/null || true
      git -C "$local_dir" remote add origin "$remote_url" 2>/dev/null || git -C "$local_dir" remote set-url origin "$remote_url"

      # If the repo has no commits yet, create a README and commit so we can push
      if ! git -C "$local_dir" rev-parse --verify HEAD >/dev/null 2>&1; then
        echo "No commits found in $local_dir; creating README and making initial commit"
        if [ ! -f "$local_dir/README.md" ]; then
          echo "# $repo" > "$local_dir/README.md"
          git -C "$local_dir" add README.md || true
        fi
        git -C "$local_dir" commit -m "initial commit" || true
        git -C "$local_dir" branch -M main || true
      fi
    else
      echo "Initializing git in $local_dir and committing files"
      git -C "$local_dir" init
      git -C "$local_dir" add -A || true
      # If there are no commits yet, create a README to allow an initial commit
      if git -C "$local_dir" rev-parse --verify HEAD >/dev/null 2>&1; then
        echo "Existing commits detected."
      else
        if [ ! -f "$local_dir/README.md" ]; then
          echo "# $repo" > "$local_dir/README.md"
          git -C "$local_dir" add README.md || true
        fi
        git -C "$local_dir" commit -m "initial commit" || true
      fi
      git -C "$local_dir" branch -M main || true
      git -C "$local_dir" remote add origin "$remote_url" || git -C "$local_dir" remote set-url origin "$remote_url"
    fi

    branch=$(git -C "$local_dir" rev-parse --abbrev-ref HEAD 2>/dev/null || true)
    if [ -z "$branch" ] || [ "$branch" = "HEAD" ]; then
      branch=main
    fi
    echo "Pushing branch '$branch' from $local_dir to $remote_url"
    git -C "$local_dir" push -u origin "$branch" || echo "Push failed for $repo; check remote and credentials."
  else
    echo "Local directory $local_dir missing; skipping push for $repo"
  fi

done


echo "\nAll done. Summary:"
for repo in "${REPOS[@]}"; do
  printf " - %s: " "$repo"
  if gh repo view "${owner}/${repo}" >/dev/null 2>&1; then
    echo "exists"
  else
    echo "missing"
  fi
done

echo "\nYou can import the repos into src/ with vcstool, for example:\n  vcs import src < src/ros2.repos\n"

exit 0
