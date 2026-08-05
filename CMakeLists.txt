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


####################################
# Unitree SDK2
####################################

add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/unitree_sdk2
)


####################################
# Driver shared library
####################################

add_library(${PROJECT_NAME} SHARED
  src/DriverUnitreeH1.cpp
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

  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/unitree_sdk2/include
)


target_link_libraries(${PROJECT_NAME}
  unitree_sdk2
  ${DQROBOTICS_LIBRARY}
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


####################################
# Test executables
####################################

add_executable(basic_test
  src/scripts/basic_test.cpp
)

target_link_libraries(basic_test
  ${PROJECT_NAME}
)


add_executable(velocity_test
  src/scripts/velocity_test.cpp
)

target_link_libraries(velocity_test
  ${PROJECT_NAME}
)


add_executable(printer_test
  src/scripts/printer_test.cpp
)

target_link_libraries(printer_test
  ${PROJECT_NAME}
)


add_executable(torque_test
  src/scripts/torque_test.cpp
)

target_link_libraries(torque_test
  ${PROJECT_NAME}
)

add_executable(standing_test
  src/scripts/standing_test.cpp
)

target_link_libraries(standing_test
  ${PROJECT_NAME}
)

add_executable(mode_test
  src/scripts/mode_test.cpp
)

target_link_libraries(mode_test
  ${PROJECT_NAME}
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