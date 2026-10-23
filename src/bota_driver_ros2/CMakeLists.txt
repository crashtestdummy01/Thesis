cmake_minimum_required(VERSION 3.8)
project(bota_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Build configuration options
option(BUILD_NODES "Enable building ROS2 nodes" TRUE)
option(BUILD_HARDWARE_INTERFACE "Enable building ros2_control hardware interface" FALSE)
option(BOTA_DRIVER_CHECK_LATEST "Check the remote for a newer bota_driver_cpp release on every configure, even if a local copy already exists" OFF)

# ============================================================
# Platform detection
#
# This package only supports Linux, on x86_64 or aarch64.
# ============================================================
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "[bota_driver] Only Linux is supported for this package (got: ${CMAKE_SYSTEM_NAME}).")
endif()

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(PLATFORM "linux_aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
  set(PLATFORM "linux_x86_64")
else()
  message(FATAL_ERROR "[bota_driver] Unsupported processor: ${CMAKE_SYSTEM_PROCESSOR} (only x86_64 and aarch64 are supported).")
endif()

message(STATUS "[bota_driver] Platform: ${PLATFORM}")
message(STATUS "[bota_driver] BOTA_DRIVER_CHECK_LATEST = ${BOTA_DRIVER_CHECK_LATEST}")

# ============================================================
# Fetch bota_driver_cpp (see cmake/FetchBotaLibraries.cmake for
# get_latest_version()/fetch_library() - same helper used by the
# bota_driver_cpp demo repo)
# ============================================================
include(cmake/FetchBotaLibraries.cmake)

set(BOTA_DRIVER_RELEASE_PAGE_URL "https://botasys.gitlab.io/drivers/priv_bota_driver")

# LIBS_DIR + LIB_NAME are what fetch_library() combines into its
# destination ("${LIBS_DIR}/${LIB_NAME}"); using PLATFORM as the
# LIB_NAME keeps the existing on-disk layout unchanged:
# bota_driver_cpp/<platform>/{include,lib,VERSION,LICENSE}.
set(LIBS_DIR "${CMAKE_CURRENT_LIST_DIR}/bota_driver_cpp")
set(BOTA_DRIVER_VERSION_FILE "${LIBS_DIR}/${PLATFORM}/VERSION")

# Zip naming: both supported platforms use "bota_driver_release_<platform>_<version>.zip"
# (checked against the live release page - no "public" special case for x86_64 anymore).
function(_bota_driver_zip_name VERSION OUT_VAR)
  set(${OUT_VAR} "bota_driver_release_${PLATFORM}_${VERSION}.zip" PARENT_SCOPE)
endfunction()

# ------------------------------------------------------------
# bota_ensure_driver_library(<out_version_var>)
#
# Implements BOTA_DRIVER_CHECK_LATEST:
#   OFF + local copy present -> use it, never call get_latest_version()
#                                 (no network access at all).
#   OFF + nothing local yet  -> fetch latest anyway.
#   ON                       -> always call get_latest_version(), then
#                                fetch_library(); fetch_library() itself
#                                already skips the download when the
#                                on-disk VERSION matches, so this is a
#                                cheap "is there anything newer?" check
#                                whenever a copy is already present.
# ------------------------------------------------------------
function(bota_ensure_driver_library OUT_VERSION_VAR)
  set(_have_local FALSE)
  set(_local_version "")
  if(EXISTS "${BOTA_DRIVER_VERSION_FILE}")
    file(READ "${BOTA_DRIVER_VERSION_FILE}" _local_version)
    string(STRIP "${_local_version}" _local_version)
    if(_local_version)
      set(_have_local TRUE)
    endif()
  endif()

  if(_have_local AND NOT BOTA_DRIVER_CHECK_LATEST)
    message(STATUS "[bota_driver] bota_driver_cpp: using local version ${_local_version} "
                    "(BOTA_DRIVER_CHECK_LATEST=OFF, skipping remote check)")
    set(${OUT_VERSION_VAR} "${_local_version}" PARENT_SCOPE)
    return()
  endif()

  if(NOT _have_local)
    message(STATUS "[bota_driver] bota_driver_cpp: no local copy found, fetching latest release")
  endif()

  get_latest_version(${BOTA_DRIVER_RELEASE_PAGE_URL} _remote_version)
  _bota_driver_zip_name("${_remote_version}" _zip_name)

  # fetch_library() prints "already up to date, skipping" or
  # "version changed X -> Y, re-downloading" on its own.
  fetch_library("${PLATFORM}" "${_remote_version}" ${BOTA_DRIVER_RELEASE_PAGE_URL} "${_zip_name}")

  set(${OUT_VERSION_VAR} "${_remote_version}" PARENT_SCOPE)
endfunction()

bota_ensure_driver_library(BOTA_DRIVER_VERSION)
message(STATUS "[bota_driver] Using bota_driver_cpp version: ${BOTA_DRIVER_VERSION}")

# find dependencies
find_package(ament_cmake REQUIRED)

# Core dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ROS2 message generation
find_package(rosidl_default_generators REQUIRED)

# type_description_interfaces is only available in Iron+ distributions
if("$ENV{ROS_DISTRO}" STREQUAL "humble")
  set(TYPE_DESCRIPTION_DEPS)
else()
  find_package(type_description_interfaces REQUIRED)
  set(TYPE_DESCRIPTION_DEPS type_description_interfaces)
endif()

# Conditional dependencies
if(BUILD_NODES)
  # Lifecycle dependencies
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
endif()

if(BUILD_HARDWARE_INTERFACE)
  # ROS2 control dependencies
  find_package(hardware_interface REQUIRED)
  find_package(pluginlib REQUIRED)
endif()

# BotaDriverExposed cpp library
find_library(BOTADRIVER_LIBRARY NAMES BotaDriver
             PATHS ${CMAKE_CURRENT_LIST_DIR}/bota_driver_cpp/${PLATFORM}/lib)

if(NOT BOTADRIVER_LIBRARY)
  message(FATAL_ERROR "BotaDriverExposed library not found!")
endif()
message(STATUS "Found BotaDriverExposed: ${BOTADRIVER_LIBRARY}")

##################################
####### bota_msgs ############
##################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BotaFrame.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs ${TYPE_DESCRIPTION_DEPS}
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_NODES)
##################################
####### bota_driver_node
##################################

add_executable(bota_driver_node src/bota_driver_node.cpp)

set_target_properties(bota_driver_node PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)

target_include_directories(bota_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${CMAKE_CURRENT_LIST_DIR}/bota_driver_cpp/${PLATFORM}/include)

# ament_target_dependencies is deprecated from kilted+
if("$ENV{ROS_DISTRO}" STREQUAL "humble" OR "$ENV{ROS_DISTRO}" STREQUAL "jazzy")

  ament_target_dependencies(bota_driver_node
    PUBLIC
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    std_srvs)

else()

  target_link_libraries(bota_driver_node PUBLIC
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs_library
  )

endif()

target_link_libraries(bota_driver_node PRIVATE 
      ${BOTADRIVER_LIBRARY}
      ${cpp_typesupport_target})

install(
  TARGETS bota_driver_node
  DESTINATION lib/${PROJECT_NAME})

##################################
####### bota_driver_lc_node
##################################

add_executable(bota_driver_lcnode src/bota_driver_lcnode.cpp)

set_target_properties(bota_driver_lcnode PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)

target_include_directories(bota_driver_lcnode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${CMAKE_CURRENT_LIST_DIR}/bota_driver_cpp/${PLATFORM}/include)

# ament_target_dependencies is deprecated from kilted+
if("$ENV{ROS_DISTRO}" STREQUAL "humble" OR "$ENV{ROS_DISTRO}" STREQUAL "jazzy")

  ament_target_dependencies(bota_driver_lcnode
    PUBLIC
    rclcpp
    lifecycle_msgs
    rclcpp_lifecycle
    std_msgs
    geometry_msgs
    sensor_msgs
    std_srvs)

else()

    target_link_libraries(bota_driver_lcnode PUBLIC
      ${geometry_msgs_TARGETS}
      ${lifecycle_msgs_TARGETS}
      ${sensor_msgs_TARGETS}
      ${std_msgs_TARGETS}
      ${std_srvs_TARGETS}
      rclcpp::rclcpp
      rclcpp_lifecycle::rclcpp_lifecycle
      sensor_msgs::sensor_msgs_library
    )

endif()


target_link_libraries(bota_driver_lcnode 
  PRIVATE 
  ${BOTADRIVER_LIBRARY}
  ${cpp_typesupport_target})

# Intall the lifecycle managed node
install(
  TARGETS bota_driver_lcnode
  DESTINATION lib/${PROJECT_NAME})

endif() # BUILD_NODES

if(BUILD_HARDWARE_INTERFACE)
##################################
####### bota_ft_sensor - ros2_control hardware interface
##################################

add_library(bota_ft_sensor SHARED hardware/bota_ft_sensor_interface.cpp)

set_target_properties(bota_ft_sensor PROPERTIES
  INSTALL_RPATH "$ORIGIN/bota_driver"
)

target_include_directories(bota_ft_sensor 
  PRIVATE 
  include/${PROJECT_NAME}
  ${CMAKE_CURRENT_LIST_DIR}/bota_driver_cpp/${PLATFORM}/include)



# ament_target_dependencies is deprecated from kilted+
if("$ENV{ROS_DISTRO}" STREQUAL "humble" OR "$ENV{ROS_DISTRO}" STREQUAL "jazzy")

  ament_target_dependencies(bota_ft_sensor 
    PUBLIC
    hardware_interface 
    pluginlib)

else()

  target_link_libraries(bota_ft_sensor PUBLIC
    hardware_interface::hardware_interface
    hardware_interface::mock_components
    pluginlib::pluginlib
  )

endif()


target_link_libraries(bota_ft_sensor PRIVATE ${BOTADRIVER_LIBRARY})

# Intall the ros2_control library - wrapped as a plugin
install(
  TARGETS bota_ft_sensor
  LIBRARY DESTINATION lib)

# Install the plugin
install(FILES resources/bota_ft_sensor_plugin.xml
  DESTINATION share/${PROJECT_NAME}/resources)

# Export plugin
pluginlib_export_plugin_description_file(hardware_interface resources/bota_ft_sensor_plugin.xml)

endif() # BUILD_HARDWARE_INTERFACE

#######################
#### INSTALLATION #####
#######################

# Install the BotaDriver cpp-core shared library
install(
  FILES ${BOTADRIVER_LIBRARY}
  DESTINATION lib
)

# Install the urdf directory  
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install the mesh directory
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install the scripts for enabling EtherCAT access (to be used when upload to ROS2 build factory)
# install(
#   PROGRAMS scripts/set_ethercat_network_capabilities.sh
#   DESTINATION lib/${PROJECT_NAME}
#   RENAME set_ethercat_network_capabilities
# )

# install(
#   PROGRAMS scripts/remove_ethercat_network_capabilities.sh
#   DESTINATION lib/${PROJECT_NAME}
#   RENAME remove_ethercat_network_capabilities
# )

##########################
## Local testing installation

option(TEST_LOCAL "Enable local testing installation" FALSE)

if(TEST_LOCAL)
  # Install the launch directory
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
  )

  # Install the urdf directory  
  install(DIRECTORY urdf_test
    DESTINATION share/${PROJECT_NAME}
  )

  # Install the config directory
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )

  # Install the rviz directory
  install(DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
  )

endif()

############
# TESTING ##
############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()