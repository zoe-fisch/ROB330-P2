#!/bin/bash

# Exit on any error, undefined variable, or pipe failure
set -e

# Set your paths
export FIRMWARE_PATH="$HOME/mbot_firmware_ros"
export PICO_SDK_PATH="$HOME/pico-sdk"  
export UROS_WS="$HOME/microros_ws"
export MBOT_WS="$HOME/mbot_ws"

# Verify required paths exist
echo "Verifying required paths..."
if [ ! -d "$FIRMWARE_PATH" ]; then
    echo "ERROR: Firmware path not found: $FIRMWARE_PATH"
    exit 1
fi

if [ ! -d "$PICO_SDK_PATH" ]; then
    echo "ERROR: Pico SDK path not found: $PICO_SDK_PATH"
    exit 1
fi

if [ ! -d "$UROS_WS" ]; then
    echo "ERROR: micro-ROS workspace not found: $UROS_WS"
    exit 1
fi

######## Init ########
echo "Initializing micro-ROS workspace..."
# Creates a firmware workspace for library generation
cd $UROS_WS

source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash

ros2 run micro_ros_setup create_firmware_ws.sh generate_lib

######## Adding extra packages ########
echo "Adding extra packages..."
pushd firmware/mcu_ws > /dev/null # Now in ~/microros_ws/firmware/mcu_ws

    # Workaround: Copy just tf2_msgs
    git clone -b jazzy https://github.com/ros2/geometry2
    cp -R geometry2/tf2_msgs ros2/tf2_msgs
    rm -rf geometry2

    # Import user defined packages
    mkdir -p extra_packages
    pushd extra_packages > /dev/null # Now in ~/microros_ws/firmware/mcu_ws/extra_packages
        # Copy custom messages from ROS workspace
        if [ -d "$MBOT_WS/src/mbot_interfaces" ]; then
            echo "Found mbot_interfaces, copying to build workspace..."
            cp -R $MBOT_WS/src/mbot_interfaces .
        else
            echo "ERROR: mbot_interfaces not found at $MBOT_WS/src/mbot_interfaces"
            exit 1
        fi
        
        # Copy other extra packages if they exist
        if [ -d "$FIRMWARE_PATH/microros_static_library/library_generation/extra_packages" ]; then
            echo "Copying additional extra packages..."
            cp -R $FIRMWARE_PATH/microros_static_library/library_generation/extra_packages/* . 2>/dev/null || true
        fi
        
        # Import from repos file if it exists
        if [ -f "extra_packages.repos" ]; then
            echo "Importing packages from extra_packages.repos..."
            vcs import --input extra_packages.repos
        fi
    popd > /dev/null

popd > /dev/null

######## Clean and source ########
echo "Cleaning up micro-ROS build artifacts..."
rm -rf $FIRMWARE_PATH/libmicroros/ 2>/dev/null || true

######## Build for Raspberry Pi Pico SDK  ########
echo "Building micro-ROS library..."
rm -rf firmware/build

export PICO_SDK_PATH=$PICO_SDK_PATH
ros2 run micro_ros_setup build_firmware.sh $FIRMWARE_PATH/microros_static_library/library_generation/toolchain.cmake $FIRMWARE_PATH/microros_static_library/library_generation/colcon.meta

# Verify build succeeded
if [ ! -f "firmware/build/libmicroros.a" ]; then
    echo "ERROR: Build failed - libmicroros.a not generated"
    exit 1
fi

find firmware/build/include/ -name "*.c" -delete 2>/dev/null || true
mkdir -p $FIRMWARE_PATH/libmicroros/include
cp -R firmware/build/include/* $FIRMWARE_PATH/libmicroros/include

cp firmware/build/libmicroros.a $FIRMWARE_PATH/libmicroros/libmicroros.a

######## Fix include paths  ########
echo "Fixing include paths..."
pushd firmware/mcu_ws > /dev/null
    INCLUDE_ROS2_PACKAGES=$(colcon list | awk '{print $1}' | awk -v d=" " '{s=(NR==1?s:s d)$0}END{print s}')
popd > /dev/null

# Install rsync if not present
if ! command -v rsync &> /dev/null; then
    echo "Installing rsync..."
    sudo apt -y install rsync
fi

for var in ${INCLUDE_ROS2_PACKAGES}; do
    if [ -d "$FIRMWARE_PATH/libmicroros/include/${var}/${var}" ]; then
        rsync -r $FIRMWARE_PATH/libmicroros/include/${var}/${var}/* $FIRMWARE_PATH/libmicroros/include/${var}/
        rm -rf $FIRMWARE_PATH/libmicroros/include/${var}/${var}
    fi
done

######## Generate extra files ########
echo "Generating metadata files..."
find firmware/mcu_ws/ros2 \( -name "*.srv" -o -name "*.msg" -o -name "*.action" \) | awk -F"/" '{print $(NF-2)"/"$NF}' > $FIRMWARE_PATH/available_ros2_types
find firmware/mcu_ws/extra_packages \( -name "*.srv" -o -name "*.msg" -o -name "*.action" \) | awk -F"/" '{print $(NF-2)"/"$NF}' >> $FIRMWARE_PATH/available_ros2_types
# sort it so that the result order is reproducible
sort -o $FIRMWARE_PATH/available_ros2_types $FIRMWARE_PATH/available_ros2_types

cd firmware
echo "" > $FIRMWARE_PATH/built_packages
for f in $(find $(pwd) -name .git -type d); do 
    pushd $f > /dev/null
    echo $(git config --get remote.origin.url) $(git rev-parse HEAD) >> $FIRMWARE_PATH/built_packages
    popd > /dev/null
done
# sort it so that the result order is reproducible
sort -o $FIRMWARE_PATH/built_packages $FIRMWARE_PATH/built_packages

######## Fix permissions ########
echo "Fixing permissions..."
chmod -R 755 $FIRMWARE_PATH/microros_static_library 2>/dev/null || true

echo "Build complete! Library available at: $FIRMWARE_PATH/libmicroros/libmicroros.a"
echo "Headers available at: $FIRMWARE_PATH/libmicroros/include/"