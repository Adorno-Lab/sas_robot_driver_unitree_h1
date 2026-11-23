cmake_minimum_required(VERSION 3.12)

project(sas_robot_driver_unitree_h1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


####################################
# Dependencies
####################################

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(sas_core REQUIRED)
find_package(sas_common REQUIRED)
find_package(sas_robot_driver REQUIRED)

find_package(unitree_sdk2 REQUIRED)
find_package(unitree_drivers REQUIRED)
find_package(sas_legged_msgs REQUIRED)

####################################
# DQ Robotics
####################################

find_library(DQROBOTICS_LIBRARY
  NAMES dqrobotics
  REQUIRED
)

####################################
# Driver shared library
####################################

# Collect all sas_tools implementation files, including subdirectories.
file(GLOB_RECURSE SAS_TOOLS_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/sas_tools/sas_tools/src/*.cpp"
)

add_library(${PROJECT_NAME} SHARED
  src/DriverUnitreeH1.cpp
  src/RobotDriverUnitreeH1Limb.cpp
  src/sas_robot_driver_unitree_h1.cpp
  ${SAS_TOOLS_SOURCES}
)


ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sas_core
  sas_common
  sas_robot_driver
  sas_legged_msgs
)


target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/sas_tools/sas_tools/include>
    $<INSTALL_INTERFACE:include>
)


target_link_libraries(${PROJECT_NAME}
  unitree_sdk2
  ${DQROBOTICS_LIBRARY}
  unitree_drivers::loco_client
  unitree_drivers::arm_sdk
  unitree_drivers::low_state
)


####################################
# Export library
####################################

ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)


ament_export_dependencies(
  Eigen3
  rclcpp
  sas_core
  sas_common
  sas_robot_driver
  sas_legged_msgs
)


####################################
# Install
####################################

install(
  DIRECTORY include/
  DESTINATION include
)


install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


# ----------------------------------
# Node executable
# ----------------------------------

add_executable(sas_robot_driver_unitree_h1_node
  src/sas_robot_driver_unitree_h1_node_main.cpp
)

target_link_libraries(sas_robot_driver_unitree_h1_node
  ${PROJECT_NAME}
)

ament_target_dependencies(sas_robot_driver_unitree_h1_node
  rclcpp
  sas_core
  sas_common
  sas_robot_driver
  sas_legged_msgs
)

install(TARGETS sas_robot_driver_unitree_h1_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


####################################
# Launch files
####################################
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


####################################
# Examples
####################################

function(add_h1_example target source)
  add_executable(${target} ${source})
  target_link_libraries(${target} ${PROJECT_NAME})
endfunction()

add_h1_example(movement_example
  examples/standalone_examples/movement_example.cpp
)

add_h1_example(printer_example
  examples/standalone_examples/printer_example.cpp
)

add_h1_example(ROS2_movement_example
  examples/ROS2_examples/movement_example.cpp
)

ament_target_dependencies(ROS2_movement_example
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

####################################
# Testing / linting
####################################

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

endif()


####################################
# Finalise ament package
####################################

ament_package()