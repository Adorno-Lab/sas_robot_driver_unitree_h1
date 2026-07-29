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
# Unitree SDK (build as subproject)
####################################
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/unitree_sdk2)

####################################
# Shared Library
####################################
add_library(${PROJECT_NAME} SHARED
  src/DriverUnitreeH1.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  sas_core
  sas_common
  sas_robot_driver
  rclcpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/unitree_sdk2/include>
)

# Link Unitree SDK properly (no file paths)
target_link_libraries(${PROJECT_NAME}
  unitree_sdk2
)

####################################
# Export
####################################
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  Eigen3
  rclcpp
  sas_core
  sas_common
  sas_robot_driver
)

####################################
# Install library + headers
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
# Script executables
####################################

add_executable(basic_test
    src/basic_test.cpp
    src/DriverUnitreeH1.cpp
    src/DriverUnitreeH1.hpp
)
target_link_libraries(basic_test unitree_sdk2)

add_executable(velocity_test
    src/velocity_test.cpp
    src/DriverUnitreeH1.cpp
    src/DriverUnitreeH1.hpp
)
target_link_libraries(velocity_test unitree_sdk2)

add_executable(printer_test
    src/printer_test.cpp
    src/DriverUnitreeH1.cpp
    src/DriverUnitreeH1.hpp
)
target_link_libraries(printer_test unitree_sdk2)

####################################
# Node executable
####################################
# set(RCLCPP_LOCAL_BINARY_NAME sas_robot_driver_unitree_h1_node)

# add_executable(${RCLCPP_LOCAL_BINARY_NAME}
#   src/sas_robot_driver_unitree_h1_node.cpp
# )

# ament_target_dependencies(${RCLCPP_LOCAL_BINARY_NAME}
#   rclcpp
#   sas_core
#   sas_common
#   sas_robot_driver
# )

# target_include_directories(${RCLCPP_LOCAL_BINARY_NAME}
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
# )

# target_link_libraries(${RCLCPP_LOCAL_BINARY_NAME}
#   ${PROJECT_NAME}
# )

# target_compile_features(${RCLCPP_LOCAL_BINARY_NAME} PUBLIC c_std_99 cxx_std_17)

# install(
#   TARGETS ${RCLCPP_LOCAL_BINARY_NAME}
#   DESTINATION lib/${PROJECT_NAME}
# )

# unset(RCLCPP_LOCAL_BINARY_NAME)

####################################
# Launch files
####################################
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

####################################
# Scripts
####################################
# install(
#   DIRECTORY scripts/
#   FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
#   DESTINATION lib/${PROJECT_NAME}
# )

####################################
# Testing
####################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

####################################
# Final ament package
####################################
ament_package()