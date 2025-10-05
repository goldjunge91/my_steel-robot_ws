#!/bin/bash
set -e

# Ensure we're in the workspace directory
cd /github/workspace || { echo "Failed to cd to /github/workspace"; exit 1; }

# Set AMENT_TRACE_SETUP_FILES to avoid unset variable error with set -u
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-0}

echo "Running setup script..."
bash ./setup.sh

echo "Building workspace..."
bash ./build.sh

echo "Running tests..."
bash ./test.sh
