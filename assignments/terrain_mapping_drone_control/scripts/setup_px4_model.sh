#!/bin/bash

# Exit on any error
set -e

# Check if PX4-Autopilot exists in home directory
if [ ! -d "$HOME/PX4-Autopilot" ]; then
    echo "Error: PX4-Autopilot directory not found in home directory"
    exit 1
fi

# Define paths
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PACKAGE_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"
PX4_DIR="$HOME/PX4-Autopilot"

# Create necessary directories if they don't exist
mkdir -p "$PACKAGE_DIR/models/px4_models/airframes"
mkdir -p "$PACKAGE_DIR/models/px4_models/gz_models"

# Copy files from PX4-Autopilot to our package
echo "Copying PX4 model files to package..."

# Copy airframe file
cp "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/4021_gz_x500_depth_mono" \
   "$PACKAGE_DIR/models/px4_models/airframes/"

# Copy CMakeLists.txt if it exists
if [ -f "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt" ]; then
    cp "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt" \
       "$PACKAGE_DIR/models/px4_models/airframes/"
fi

# Copy Gazebo model
if [ -d "$PX4_DIR/Tools/simulation/gz/models/x500_depth_mono" ]; then
    cp -r "$PX4_DIR/Tools/simulation/gz/models/x500_depth_mono" \
          "$PACKAGE_DIR/models/px4_models/gz_models/"
else
    echo "Warning: x500_depth_mono model not found in PX4-Autopilot"
fi

echo "Setup complete! Model files have been copied to the package." 