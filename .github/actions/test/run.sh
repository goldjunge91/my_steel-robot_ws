#!/bin/bash
set -e

# Ensure we're in the workspace directory
cd /github/workspace || { echo "Failed to cd to /github/workspace"; exit 1; }

echo "Running setup script..."
bash ./setup.sh

echo "Building workspace..."
bash ./build.sh

echo "Running tests..."
bash ./test.sh
