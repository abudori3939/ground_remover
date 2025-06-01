#!/bin/bash

# Script to install dependencies for ground_remover

# Update package list
sudo apt-get update

# Install PCL
sudo apt install -y libpcl-dev
sudo apt install -y libvtk-qt-dev

# Install CMake
sudo apt install -y cmake
