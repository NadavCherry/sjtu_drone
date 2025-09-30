#!/bin/bash

ROS_DISTRO=humble
XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority
IMAGE_NAME="sjtu_drone:humble_ros2"

# Usage info
if [ -z "$1" ]; then
    echo "Usage: $0 <world_file>"
    echo ""
    echo "Available worlds:"
    echo "  - hospital.world"
    echo "  - hospital_two_floors.world"
    echo "  - hospital_three_floors.world"
    echo ""
    echo "Examples:"
    echo "  $0 hospital.world"
    echo "  $0 hospital_two_floors.world"
    echo ""
    exit 1
fi

WORLD_FILE=$1

# Check if world file exists
WORLD_PATH="/root/drone_workspace/aws-robomaker-hospital-world/worlds/${WORLD_FILE}"
WORLD_BASE=$(basename "${WORLD_FILE}" .world)

echo "Using world: ${WORLD_PATH}"

# Enable X11 forwarding for Docker
xhost +local:docker

docker run \
    -it --rm \
    --gpus all \
    -v ${XSOCK}:${XSOCK} \
    -v ${XAUTH}:${XAUTH} \
    -e DISPLAY=${DISPLAY} \
    -e XAUTHORITY=${XAUTH} \
    --env=QT_X11_NO_MITSHM=1 \
    --privileged \
    --net=host \
    -v $HOME/drone_workspace:/root/drone_workspace:rw \
    --name="sjtu_drone_${WORLD_BASE}" \
    ${IMAGE_NAME} \
    bash -c "
        # Source ROS2 environment
        source /opt/ros/${ROS_DISTRO}/setup.bash

        # Force use of available RMW implementation
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

        # Setup Gazebo model paths - CRITICAL FIX HERE
        # Start with system models
        export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models

        # Add the AWS RoboMaker Hospital models directory
        # This is the key fix - we need to add the 'models' directory itself, not its subdirectories
        export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/root/drone_workspace/aws-robomaker-hospital-world/models

        # Also check for fuel_models if it exists (in case you download additional models)
        if [ -d \"/root/drone_workspace/aws-robomaker-hospital-world/fuel_models\" ]; then
            export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/root/drone_workspace/aws-robomaker-hospital-world/fuel_models

            # Copy shared textures to reduce warnings (optional cosmetic fix)
            if [ -d \"/root/drone_workspace/aws-robomaker-hospital-world/fuel_models/backup_meshes\" ]; then
                for dir in /root/drone_workspace/aws-robomaker-hospital-world/fuel_models/*/meshes/; do
                    if [ -d \"\$dir\" ]; then
                        cp -n /root/drone_workspace/aws-robomaker-hospital-world/fuel_models/backup_meshes/*.png \"\$dir\" 2>/dev/null || true
                        cp -n /root/drone_workspace/aws-robomaker-hospital-world/fuel_models/backup_meshes/*.jpg \"\$dir\" 2>/dev/null || true
                    fi
                done
            fi
        fi

        # Add SJTU drone models
        export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/root/drone_workspace/sjtu_drone/sjtu_drone_description
        export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/root/drone_workspace/sjtu_drone/models

        # Setup Gazebo resource paths for finding world files and resources
        export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
        export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:/root/drone_workspace/aws-robomaker-hospital-world/worlds
        export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:/root/drone_workspace/aws-robomaker-hospital-world

        # Clear model database URI to prevent online model fetching
        export GAZEBO_MODEL_DATABASE_URI=

        # Set Gazebo master URI
        export GAZEBO_MASTER_URI=http://localhost:11345

        # Setup plugin paths
        export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:\$GAZEBO_PLUGIN_PATH

        # Suppress ALSA audio errors since we're running in a container without audio
        export ALSA_CARD=0
        export GAZEBO_AUDIO_DEVICE=null

        # Enable verbose Gazebo output to help debug model loading
        export GAZEBO_VERBOSE=1

        echo '================================'
        echo 'Environment Setup Complete'
        echo 'World: ${WORLD_FILE}'
        echo '================================'
        echo ''
        echo 'GAZEBO_MODEL_PATH:'
        echo \$GAZEBO_MODEL_PATH | tr ':' '\n'
        echo ''

        # Check if world file exists
        if [ ! -f \"${WORLD_PATH}\" ]; then
            echo \"ERROR: World file not found: ${WORLD_PATH}\"
            echo \"Available world files:\"
            ls -la /root/drone_workspace/aws-robomaker-hospital-world/worlds/ 2>/dev/null || echo \"Worlds directory not found\"
            exit 1
        fi

        # Check and list available models to help debug
        echo 'Checking for hospital world models...'
        if [ -d \"/root/drone_workspace/aws-robomaker-hospital-world/models\" ]; then
            echo 'Found models directory with the following models:'
            ls /root/drone_workspace/aws-robomaker-hospital-world/models/ | head -10
            TOTAL_MODELS=\$(ls /root/drone_workspace/aws-robomaker-hospital-world/models/ | wc -l)
            echo \"Total models found: \$TOTAL_MODELS\"
            echo ''
        else
            echo 'WARNING: Hospital world models directory not found!'
            echo 'The world will load without furniture and medical equipment.'
        fi

        # Install missing dependencies if needed (optional - remove if image is properly built)
        echo 'Checking for required ROS2 packages...'
        if ! ros2 pkg list | grep -q rmw_fastrtps_cpp; then
            echo 'Installing missing RMW implementation...'
            apt-get update > /dev/null 2>&1
            apt-get install -y ros-${ROS_DISTRO}-rmw-fastrtps-cpp > /dev/null 2>&1
            source /opt/ros/${ROS_DISTRO}/setup.bash
        fi

        # Build the workspace including AWS hospital world
        echo 'Building ROS2 workspace...'
        cd /root/drone_workspace

        # Clean any previous failed builds
        rm -rf build/sjtu_drone_* install/sjtu_drone_* 2>/dev/null

        # Build SJTU drone packages with explicit RMW implementation
        echo 'Building SJTU Drone packages...'
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        colcon build --packages-select sjtu_drone_bringup sjtu_drone_description sjtu_drone_control --cmake-args -DBUILD_TESTING=OFF -DRMW_IMPLEMENTATION=rmw_fastrtps_cpp

        # Source the workspace
        source install/setup.bash 2>/dev/null || source install/local_setup.bash 2>/dev/null

        # Launch the drone with the hospital world
        echo 'Launching SJTU Drone in Hospital World...'
        echo 'This may take a while as Gazebo loads all the models...'
        ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py \
            world:=${WORLD_PATH} &

        LAUNCH_PID=\$!

        # Wait for Gazebo to fully load
        echo 'Waiting for Gazebo to start...'
        for i in {1..60}; do
            if pgrep -x \"gzserver\" > /dev/null; then
                echo 'Gazebo server is running!'
                break
            fi
            if [ \$i -eq 60 ]; then
                echo 'ERROR: Gazebo server failed to start after 60 seconds'
                exit 1
            fi
            sleep 1
        done

        # Additional wait for models to load (increased time for loading all hospital models)
        echo 'Waiting for models to load...'
        sleep 10

        # Check if launch is still running
        if ! kill -0 \$LAUNCH_PID 2>/dev/null; then
            echo 'ERROR: Launch process died. Check the error messages above.'
            exit 1
        fi

        echo ''
        echo '================================'
        echo 'SJTU Drone Successfully Launched!'
        echo '================================'
        echo ''
        echo 'NOTE: If you do not see furniture and equipment in the hospital,'
        echo 'check the Gazebo terminal output for model loading errors.'
        echo ''
        echo 'Available ROS2 topics:'
        ros2 topic list | grep -E '(drone|simple_drone|cmd_vel|takeoff|land)' || echo 'Waiting for topics...'
        echo ''
        echo 'Control Commands (verify topic names with ros2 topic list):'
        echo ''
        echo 'For /drone namespace:'
        echo '  Takeoff: ros2 topic pub /drone/takeoff std_msgs/msg/Empty \"{}\" --once'
        echo '  Land:    ros2 topic pub /drone/land std_msgs/msg/Empty \"{}\" --once'
        echo '  Move:    ros2 topic pub /drone/cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\" --rate 10'
        echo ''
        echo 'For /simple_drone namespace (if available):'
        echo '  Takeoff: ros2 topic pub /simple_drone/takeoff std_msgs/msg/Empty \"{}\" --once'
        echo '  Land:    ros2 topic pub /simple_drone/land std_msgs/msg/Empty \"{}\" --once'
        echo '  Move:    ros2 topic pub /simple_drone/cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\" --rate 10'
        echo ''
        echo 'View camera feeds (if available):'
        echo '  Front: ros2 run image_view image_view --ros-args -r image:=/drone/front_camera/image_raw'
        echo '  Down:  ros2 run image_view image_view --ros-args -r image:=/drone/down_camera/image_raw'
        echo ''
        echo 'Other useful commands:'
        echo '  List all topics:     ros2 topic list'
        echo '  List all services:   ros2 service list'
        echo '  Show topic info:     ros2 topic info /drone/cmd_vel'
        echo '  Monitor topic:       ros2 topic echo /drone/cmd_vel'
        echo ''
        echo 'Press Ctrl+C to exit'
        echo ''

        # Keep the container running
        wait \$LAUNCH_PID
        "

# Disable X11 forwarding after container exits
xhost -local:docker

echo "Container exited."