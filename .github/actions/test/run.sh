#!/bin/bash
set -e

echo "Starting test workflow"
sleep 5
echo "Running setup script"
./setup.sh
wait 
echo "Running build script"
./build.sh
echo "Running test script"
./test.sh
