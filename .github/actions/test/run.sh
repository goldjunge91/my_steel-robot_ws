#!/bin/bash
set -e

echo "Running setup script..."
./setup.sh

echo "Building workspace..."
./build.sh

echo "Running tests..."
./test.sh
