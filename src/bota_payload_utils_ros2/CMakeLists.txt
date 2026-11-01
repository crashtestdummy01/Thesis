cmake_minimum_required(VERSION 3.18)   # file(ARCHIVE_EXTRACT), find_library(REQUIRED)
project(bota_payload_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_NODES       "Build the ROS2 nodes"                        ON)
option(BUILD_CONTROLLERS "Build the ros2_control controllers"          ON)
option(BOTA_CHECK_LATEST "Check for newer Bota libraries on every configure" OFF)

# ============================================================
# Platform (Linux x86_64 / aarch64 only)
# ============================================================
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "[bota_payload_utils] Only Linux is supported (got: ${CMAKE_SYSTEM_NAME})")
endif()

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(PLATFORM "linux_aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
  set(PLATFORM "linux_x86_64")
else()
  message(FATAL_ERROR "[bota_payload_utils] Unsupported processor: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

message(STATUS "[bota_payload_utils] Platform: ${PLATFORM}, BOTA_CHECK_LATEST=${BOTA_CHECK_LATEST}")

# ============================================================
# Fetch Bota libraries into bota_libraries/<platform>/<library>
# ============================================================
include(cmake/FetchBotaLibraries.cmake)

set(LIBS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/bota_libraries/${PLATFORM}")   # used by fetch_library()

set(BOTA_CONTROL_URL "https://botasys.gitlab.io/control_toolkit/priv_bota_control")
set(BOTA_PAYLOAD_URL "https://botasys.gitlab.io/control_toolkit/priv_bota_payload_utils")

# Zip names as published on the release pages (@VERSION@ is filled in at fetch time)
set(BOTA_CONTROL_ZIP "bota_control_release_${PLATFORM}_@VERSION@.zip")
if(PLATFORM STREQUAL "linux_x86_64")
  set(BOTA_PAYLOAD_ZIP "bota_payload_utils_release_public_@VERSION@.zip")
else()
  set(BOTA_PAYLOAD_ZIP "bota_payload_utils_release_${PLATFORM}_@VERSION@.zip")
endif()

# bota_fetch(<name> <release_url> <zip_template>)
#   Uses the local copy if present, unless BOTA_CHECK_LATEST is ON.
function(bota_fetch NAME URL ZIP_TEMPLATE)
  set(VERSION_FILE "${LIBS_DIR}/${NAME}/VERSION")

  if(EXISTS "${VERSION_FILE}" AND NOT BOTA_CHECK_LATEST)
    file(STRINGS "${VERSION_FILE}" VERSION LIMIT_COUNT 1)
    message(STATUS "[bota_payload_utils] ${NAME}: using local v${VERSION}")
    return()
  endif()

  get_latest_version("${URL}" VERSION)
  string(REPLACE "@VERSION@" "${VERSION}" ZIP_NAME "${ZIP_TEMPLATE}")
  fetch_library("${NAME}" "${VERSION}" "${URL}" "${ZIP_NAME}")
endfunction()

bota_fetch(bota_control       "${BOTA_CONTROL_URL}" "${BOTA_CONTROL_ZIP}")
bota_fetch(bota_payload_utils "${BOTA_PAYLOAD_URL}" "${BOTA_PAYLOAD_ZIP}")

# ============================================================
# Imported targets for the fetched libraries
#   (include/ and lib/ may sit at the top level or in per-component subfolders)
# ============================================================
set(CONTROL_DIR "${LIBS_DIR}/bota_control")
set(PAYLOAD_DIR "${LIBS_DIR}/bota_payload_utils")

file(GLOB CONTROL_INCLUDE_DIRS LIST_DIRECTORIES true "${CONTROL_DIR}/include" "${CONTROL_DIR}/*/include" "${CONTROL_DIR}/*/*/include" "${CONTROL_DIR}/*/*/*/include")
file(GLOB CONTROL_LIB_DIRS     LIST_DIRECTORIES true "${CONTROL_DIR}/lib" "${CONTROL_DIR}/*/lib" "${CONTROL_DIR}/*/*/lib" "${CONTROL_DIR}/*/*/*/lib")
file(GLOB CONTROL_LIBRARIES "${CONTROL_DIR}/lib/*.so" "${CONTROL_DIR}/*/lib/*.so" "${CONTROL_DIR}/*/*/lib/*.so" "${CONTROL_DIR}/*/*/*/lib/*.so")
file(GLOB PAYLOAD_INCLUDE_DIRS LIST_DIRECTORIES true "${PAYLOAD_DIR}/include" "${PAYLOAD_DIR}/*/include" "${PAYLOAD_DIR}/*/*/include" "${PAYLOAD_DIR}/*/*/*/include")
file(GLOB PAYLOAD_LIB_DIRS     LIST_DIRECTORIES true "${PAYLOAD_DIR}/lib" "${PAYLOAD_DIR}/*/lib" "${PAYLOAD_DIR}/*/*/lib" "${PAYLOAD_DIR}/*/*/*/lib")
# bota::control - headers plus any shared libraries it ships
add_library(bota::control INTERFACE IMPORTED)
target_include_directories(bota::control INTERFACE ${CONTROL_INCLUDE_DIRS})
target_link_libraries(bota::control INTERFACE ${CONTROL_LIBRARIES})

# bota_import_payload_library(<target> <library_name>)
function(bota_import_payload_library TARGET LIB_NAME)
  find_library(${LIB_NAME}_PATH NAMES ${LIB_NAME} PATHS ${PAYLOAD_LIB_DIRS} NO_DEFAULT_PATH REQUIRED)
  message(STATUS "[bota_payload_utils] Found ${LIB_NAME}: ${${LIB_NAME}_PATH}")

  add_library(${TARGET} UNKNOWN IMPORTED)
  set_target_properties(${TARGET} PROPERTIES IMPORTED_LOCATION "${${LIB_NAME}_PATH}")
  target_include_directories(${TARGET} INTERFACE ${PAYLOAD_INCLUDE_DIRS})
  target_link_libraries(${TARGET} INTERFACE bota::control)
endfunction()

bota_import_payload_library(bota::payload_compensator BotaPayloadCompensator)
bota_import_payload_library(bota::payload_estimator   BotaPayloadEstimator)

# ============================================================
# ROS dependencies
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(bota_driver REQUIRED)   # custom Bota messages

if(BUILD_CONTROLLERS)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(controller_interface REQUIRED)
endif()

# ament_target_dependencies() is deprecated from Kilted on
function(bota_link_ros_deps TARGET)
  if("$ENV{ROS_DISTRO}" STREQUAL "humble" OR "$ENV{ROS_DISTRO}" STREQUAL "jazzy")
    ament_target_dependencies(${TARGET} PUBLIC ${ARGN})
  else()
    foreach(DEP IN LISTS ARGN)
      target_link_libraries(${TARGET} PUBLIC ${${DEP}_TARGETS})
    endforeach()
  endif()
endfunction()

# ============================================================
# Nodes
# ============================================================
if(BUILD_NODES)
  function(bota_add_node NAME SOURCE BOTA_LIBRARY)
    add_executable(${NAME} ${SOURCE})
    set_target_properties(${NAME} PROPERTIES INSTALL_RPATH "$ORIGIN")   # vendor .so next to the node
    target_include_directories(${NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    bota_link_ros_deps(${NAME} rclcpp std_msgs std_srvs geometry_msgs sensor_msgs bota_driver)
    target_link_libraries(${NAME} PRIVATE ${BOTA_LIBRARY} Eigen3::Eigen)
    install(TARGETS ${NAME} DESTINATION lib/${PROJECT_NAME})
  endfunction()

  bota_add_node(bota_payload_compensator_node src/nodes/bota_payload_compensator_node.cpp bota::payload_compensator)
  bota_add_node(bota_payload_estimator_node   src/nodes/bota_payload_estimator_node.cpp   bota::payload_estimator)
endif()

# ============================================================
# ros2_control controllers
# ============================================================
if(BUILD_CONTROLLERS)
  set(CONTROLLERS ${PROJECT_NAME}_controllers)

  add_library(${CONTROLLERS} SHARED
    src/controllers/bota_payload_compensator_controller.cpp
    src/controllers/bota_payload_estimator_controller.cpp)

  set_target_properties(${CONTROLLERS} PROPERTIES INSTALL_RPATH "$ORIGIN/${PROJECT_NAME}")
  target_include_directories(${CONTROLLERS} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  bota_link_ros_deps(${CONTROLLERS} pluginlib controller_interface rclcpp rclcpp_lifecycle std_srvs)
  target_link_libraries(${CONTROLLERS} PRIVATE bota::payload_compensator bota::payload_estimator Eigen3::Eigen)

  install(TARGETS ${CONTROLLERS} LIBRARY DESTINATION lib)
  install(FILES resources/bota_payload_utils_controllers.xml DESTINATION share/${PROJECT_NAME}/resources)
  pluginlib_export_plugin_description_file(controller_interface resources/bota_payload_utils_controllers.xml)
endif()

# ============================================================
# Install the fetched shared libraries next to the nodes
# ============================================================
foreach(DIR IN LISTS CONTROL_LIB_DIRS PAYLOAD_LIB_DIRS)
  install(DIRECTORY "${DIR}/" DESTINATION lib/${PROJECT_NAME} FILES_MATCHING PATTERN "*.so*")
endforeach()

# ============================================================
# ONLY FOR LOCAL DEVELOPMENT TESTING - Install launch files, configs and URDF - 
# ============================================================
# install(DIRECTORY launch config bota_config urdf
#   DESTINATION share/${PROJECT_NAME})

# ============================================================
# Testing
# ============================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
