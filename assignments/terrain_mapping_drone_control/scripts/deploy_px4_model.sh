#!/bin/bash

# Exit on any error
set -e

# Default PX4 directory
DEFAULT_PX4_DIR="$HOME/PX4-Autopilot"

# Parse command line arguments
PX4_DIR=$DEFAULT_PX4_DIR
SHOW_HELP=false

# Process command line arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        -p|--px4-dir)
            PX4_DIR="$2"
            shift # past argument
            shift # past value
            ;;
        -h|--help)
            SHOW_HELP=true
            shift # past argument
            ;;
        *)    # unknown option
            echo "Unknown option: $1"
            SHOW_HELP=true
            shift # past argument
            ;;
    esac
done

# Show help message
if [ "$SHOW_HELP" = true ]; then
    echo "Usage: $0 [OPTIONS]"
    echo "Deploy PX4 model files to PX4-Autopilot directory"
    echo ""
    echo "Options:"
    echo "  -p, --px4-dir PATH    Path to PX4-Autopilot directory (default: $DEFAULT_PX4_DIR)"
    echo "  -h, --help            Show this help message"
    exit 0
fi

# Check if PX4-Autopilot exists in specified directory
if [ ! -d "$PX4_DIR" ]; then
    echo "Error: PX4-Autopilot directory not found at: $PX4_DIR"
    echo "Please specify the correct path using the -p or --px4-dir option"
    echo "Example: $0 --px4-dir /path/to/PX4-Autopilot"
    exit 1
fi

# Define paths
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PACKAGE_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"

# Check if our model files exist
if [ ! -d "$PACKAGE_DIR/models/px4_models" ]; then
    echo "Error: PX4 model files not found in package"
    echo "Please run setup_px4_model.sh first if you're developing"
    exit 1
fi

# Display paths being used
echo "Using PX4-Autopilot directory: $PX4_DIR"
echo "Using package directory: $PACKAGE_DIR"

# Copy files to PX4-Autopilot
echo "Deploying model files to PX4-Autopilot..."

# Copy airframe files
echo "Copying airframe files..."
cp "$PACKAGE_DIR/models/px4_models/airframes/4022_gz_x500_depth_mono" \
   "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/"
# cp "$PACKAGE_DIR/models/px4_models/airframes/4022_gz_x500_oaklite" \
#    "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/"

# Copy CMakeLists.txt if it exists
if [ -f "$PACKAGE_DIR/models/px4_models/airframes/CMakeLists.txt" ]; then
    cp "$PACKAGE_DIR/models/px4_models/airframes/CMakeLists.txt" \
       "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/"
fi

# Copy Gazebo models
echo "Copying Gazebo models..."

# Copy x500_depth_mono model
if [ -d "$PACKAGE_DIR/models/px4_models/gz_models/x500_depth_mono" ]; then
    cp -r "$PACKAGE_DIR/models/px4_models/gz_models/x500_depth_mono" \
          "$PX4_DIR/Tools/simulation/gz/models/"
else
    echo "Warning: x500_depth_mono model not found in package"
fi

# Copy OakD-Lite model
if [ -d "$PACKAGE_DIR/models/OakD-Lite-Modify" ]; then
    cp -r "$PACKAGE_DIR/models/OakD-Lite-Modify" \
          "$PX4_DIR/Tools/simulation/gz/models/"
else
    echo "Warning: OakD-Lite model not found in package"
fi



# Copy x500_oaklite model
# if [ -d "$PACKAGE_DIR/models/px4_models/gz_models/x500_oaklite" ]; then
#     cp -r "$PACKAGE_DIR/models/px4_models/gz_models/x500_oaklite" \
#           "$PX4_DIR/Tools/simulation/gz/models/"
# else
#     echo "Warning: x500_oaklite model not found in package"
# fi

echo "Deployment complete! Model files have been copied to PX4-Autopilot"
echo "Please rebuild PX4-Autopilot for changes to take effect" 