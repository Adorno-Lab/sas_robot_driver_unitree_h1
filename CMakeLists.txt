cmake_minimum_required(VERSION 3.8)

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

find_package(sas_core REQUIRED)
find_package(sas_common REQUIRED)
find_package(sas_robot_driver REQUIRED)

find_package(unitree_sdk2 REQUIRED)
find_package(unitree_drivers REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS
  core
  imgcodecs
  imgproc
  highgui
)

####################################
# DQ Robotics
####################################
# Installed system library:
#   /usr/lib/libdqrobotics.so
# Headers:
#   /usr/include/dqrobotics

find_library(DQROBOTICS_LIBRARY
  NAMES dqrobotics
  REQUIRED
)


# ####################################
# # Unitree SDK2
# ####################################

# add_subdirectory(
#   ${CMAKE_CURRENT_SOURCE_DIR}/src/unitree_sdk2
# )

####################################
# Driver shared library
####################################

add_library(${PROJECT_NAME} SHARED
  src/DriverUnitreeH1.cpp
  src/sas_robot_driver_unitree_h1.cpp
  src/sas_tools/src/LeggedRobotDriver.cpp
)


ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sas_core
  sas_common
  sas_robot_driver
)


target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/sas_tools/include>
    $<INSTALL_INTERFACE:include>

  # PRIVATE
  #   ${CMAKE_CURRENT_SOURCE_DIR}/src/unitree_sdk2/include
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
# Node executables
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
# Test executables
####################################

add_executable(basic_test
  src/standalone_scripts/basic_test.cpp
)

target_link_libraries(basic_test
  ${PROJECT_NAME}
)


add_executable(velocity_test
  src/standalone_scripts/velocity_test.cpp
)

target_link_libraries(velocity_test
  ${PROJECT_NAME}
)


add_executable(printer_test
  src/standalone_scripts/printer_test.cpp
)

target_link_libraries(printer_test
  ${PROJECT_NAME}
)


add_executable(torque_test
  src/standalone_scripts/torque_test.cpp
)

target_link_libraries(torque_test
  ${PROJECT_NAME}
)

add_executable(standing_test
  src/standalone_scripts/standing_test.cpp
)

target_link_libraries(standing_test
  ${PROJECT_NAME}
)

add_executable(mode_test
  src/standalone_scripts/mode_test.cpp
)

target_link_libraries(mode_test
  ${PROJECT_NAME}
)

add_executable(limit_test
  src/standalone_scripts/limit_test.cpp
)

target_link_libraries(limit_test
  ${PROJECT_NAME}
)

add_executable(ROS2_basic_test
  src/ROS2_scripts/ROS2_basic_test.cpp
)

target_link_libraries(ROS2_basic_test
  ${PROJECT_NAME}
)

add_executable(ROS2_velocity_test
  src/ROS2_scripts/ROS2_velocity_test.cpp
)

target_link_libraries(ROS2_velocity_test
  ${PROJECT_NAME}
)

add_executable(camera_test
  src/standalone_scripts/camera_test.cpp
)

target_include_directories(camera_test PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(camera_test
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
)

# Camera publisher
add_executable(ROS2_camera_publisher
  src/ROS2_scripts/ROS2_camera_publisher.cpp
)

target_compile_features(ROS2_camera_publisher PRIVATE cxx_std_17)

ament_target_dependencies(ROS2_camera_publisher
  rclcpp
  sensor_msgs
)

target_include_directories(ROS2_camera_publisher PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(ROS2_camera_publisher
  unitree_sdk2
  ${OpenCV_LIBS}
)


# Camera subscriber/viewer
add_executable(ROS2_camera_viewer
  src/ROS2_scripts/ROS2_camera_viewer.cpp
)

target_compile_features(ROS2_camera_viewer PRIVATE cxx_std_17)

ament_target_dependencies(ROS2_camera_viewer
  rclcpp
  sensor_msgs
)

target_include_directories(ROS2_camera_viewer PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(ROS2_camera_viewer
  ${OpenCV_LIBS}
)


install(TARGETS
  ROS2_camera_publisher
  ROS2_camera_viewer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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