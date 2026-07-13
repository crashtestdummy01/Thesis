#!/bin/bash

# Set default ROS_DISTRO if not set (default to humble for this workspace)
ROS_DISTRO=${ROS_DISTRO:-humble}

# Move to the root of the project
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
SRC_DIR="$ROOT_DIR/src"

mkdir -p "$SRC_DIR"
cd "$SRC_DIR"

echo "Using ROS_DISTRO: $ROS_DISTRO"
echo "Working in: $SRC_DIR"

# Helper function to clone OR pull
sync_repo() {
    local repo_url=$1
    local target_dir=$2
    local branch=$3

    if [ ! -d "$target_dir" ]; then
        echo "Cloning $target_dir..."
        if [ -n "$branch" ]; then
            git clone -b "$branch" "$repo_url" "$target_dir"
        else
            git clone "$repo_url" "$target_dir"
        fi
    else
        echo "$target_dir already exists, pulling latest changes..."
        cd "$target_dir"
        git pull
        cd ..
    fi
}

# 1. CRISP Controllers
sync_repo "https://github.com/utiasDSL/crisp_controllers.git" "crisp_controllers"

# 2. Franka ROS 2
if [ ! -d "franka_ros2" ]; then
    echo "Cloning franka_ros2 ($ROS_DISTRO)..."
    git clone -b "$ROS_DISTRO" https://github.com/frankarobotics/franka_ros2.git ./franka_ros2
    
    # Remove nodes not used in this pipeline
    rm -rf ./franka_ros2/franka_gazebo ./franka_ros2/franka_fr3_moveit_config ./franka_ros2/franka_mobile_example_controllers ./franka_ros2/franka_mobile_sensors ./franka_ros2/franka_gazebo_bringup
    rm -rf ./franka_ros2/libfranka ./franka_ros2/franka_robot_state_broadcaster
    
    cd ./franka_ros2/ 
    git clone --recurse-submodules https://suvert@git.ias.informatik.tu-darmstadt.de/ros2/franka/libfranka.git ./franka_ros2/libfranka
    git clone https://suvert@git.ias.informatik.tu-darmstadt.de/ros2/franka/franka_robot_state_broadcaster.git ./franka_ros2/franka_robot_state_broadcaster
    cd ..
    
    vcs import ./franka_ros2 < ./franka_ros2/dependency.repos --recursive --skip-existing
    rosdep install --from-paths ./franka_ros2 --ignore-src --rosdistro "$ROS_DISTRO" --skip-keys "ignition-plugin franka_ign_ros2_control" -y
else
    echo "franka_ros2 already exists, pulling changes for sub-repos..."
    cd franka_ros2
    git pull
    cd libfranka && git pull && git submodule update --init --recursive && cd ..
    cd franka_robot_state_broadcaster && git pull && cd ..
    cd ..
fi

echo "Setup and sync complete!"
