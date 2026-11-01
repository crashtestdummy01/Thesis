# bota_payload_utils_ros2 

This repository contains the **bota_payload_utils** package source code. It is built on top of the **Bota Payload Utils for C++**.

Updated documentation about the **Bota Payload Utils for ROS2** can be found in [Bota FT Stack documentation](https://code.botasys.com/en/gen_a/layer1/payload_utils/payload_utils_ros2.html).

# About
- **Author**: Bota Systems AG (support@botasys.com) 
- **Contributors**: Raul Cruz-Oliver
- **Date**: 14 November 2025 
- **Place**: Zurich, Switzerland 

# Contents
- **/bota_payload_utils_cpp**: C++ implementation of the Bota Payload Utils
- **/include/controllers**: Header files for the bota_payload_compensator_controller and bota_payload_estimator_controller
- **/include/nodes**: Header files for the bota_payload_compensator_node and bota_payload_estimator_node
- **/include/input_block.hpp**: Header only implementation for the InputBlock class
- **/resources**: Plugin description for ros2_control
- **/src/controllers**: Implementation files for the bota_payload_compensator_controller and bota_payload_estimator_controller
- **/src/nodes**: Implementation files for the bota_payload_compensator_node and bota_payload_estimator_node
- **CMakeLists.txt**: CMake build configuration
- **LICENSE:** License information
- **package.xml:** ROS2 package configuration
- **README.md:** This readme file

# Package Description

The **bota_payload_utils** package provides a ROS2 interface for the Bota Payload Utils, enabling seamless integration with ROS2-based robotic systems.

The package includes two variants:

1. **bota_payload_compensator_node** and **bota_payload_estimator_node**: Standard ROS2 nodes providing topics and services.
3. **BotaPayloadCompensatorController** and **BotaPayloadEstimatorController**: *ros2_control* controllers for integration with the *ros2_control* framework.

The choice between the variant to be used depends on the specific requirements of your application. See the [Bota FT Stack documentation](https://code.botasys.com/en/gen_a/layer1/payload_utils/payload_utils_ros2.html) for more details.


# Getting Started

1. Clone this repository into your ROS2 workspace's `src` directory:

```bash
# go to your workspace src folder
cd <WORKSPACE_DIR>/src

# clone the repository 
git clone https://gitlab.com/botasys/control_toolkit/bota_payload_utils_ros2.git
```

Example workspace structure after cloning:

```text
~/ros2_ws/
├── src/
│   ├── bota_payload_utils_ros2/       # cloned repository
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── README.md
│   │   ├── src/
│   │   └── ...
│   └── other_package/
├── build/
├── install/
└── log/
```

2. Build your workspace using `colcon build`.

```bash
# go to your workspace root folder
cd <WORKSPACE_DIR>

# source ROS2
source /opt/ros/<DISTRO>/setup.bash

# build the workspace, by default it only builds the standard nodes
colcon build 
```

If you want to build also the **ros2_control controllers**, use the following command:

```bash
# go to your workspace root folder
cd <WORKSPACE_DIR>

# source ROS2
source /opt/ros/<DISTRO>/setup.bash

# make sure to have the neccesary dependencies installed
sudo apt install ros-${ROS_DISTRO}-ros2-control 

# build the workspace, indicating to build the hardware interface
colcon build --cmake-args -DBUILD_CONTROLLERS=TRUE
```

If you only want to explicitly only build the **ros2_control controllers**, you can use:

```bash
# build the workspace, indicating to not build the nodes, but build the hardware interface
colcon build --cmake-args -DBUILD_NODES=FALSE -DBUILD_CONTROLLERS=TRUE
```

3. Explore the examples:
- For standard ROS 2, see the [bota_demo_ros2](https://gitlab.com/botasys/control_toolkit/bota_demo_ros2)
- For ros2_control, see the [bota_demo_ros2_control](https://gitlab.com/botasys/control_toolkit/bota_demo_ros2_control)


# Updates and version pinning

If you want to update the package to the latest version, you can pull the latest changes from the repository:

```bash
# go to your workspace src folder
cd <WORKSPACE_DIR>/src/bota_payload_utils_ros2

# pull the latest changes
git pull
```

If you want a specific version, you can checkout the desired tag:

```bash
# go to your workspace src folder
cd <WORKSPACE_DIR>/src/bota_payload_utils_ros2

# checkout the desired tag
git checkout <tag_name>
```

# License
See [LICENSE](LICENSE) file.