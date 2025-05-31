#!/bin/bash

# Script to install dependencies for ground_remover

# Update package list
sudo apt-get update

# Install PCL
sudo apt-get install -y libpcl-dev

# Install CMake
sudo apt-get install -y cmake

echo "Installation of dependencies complete."
echo "You might need to install other PCL visualization dependencies if you encounter issues with the PCLViewer, such as:"
echo "sudo apt-get install -y libvtk-qt-dev"
