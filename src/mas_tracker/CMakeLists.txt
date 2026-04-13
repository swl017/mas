cmake_minimum_required(VERSION 3.8)
project(mas_tracker)

# Default to C++17. ROS 2 Humble and later generally use C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mas_msgs REQUIRED)

# Find Eigen3
find_package(Eigen3 3 REQUIRED NO_MODULE) # NO_MODULE uses Eigen3Config.cmake

# Find MAV_MSGS (if a ROS 2 version is available and has a CMake config)
# find_package(mav_msgs QUIET) # Or mav_msgs_ros2, etc.
# if(mav_msgs_FOUND)
#   ament_target_dependencies(${PROJECT_NAME}_node PUBLIC mav_msgs::mav_msgs) # Example
#   target_include_directories(${PROJECT_NAME}_node PUBLIC $<TARGET_PROPERTY:mav_msgs::mav_msgs,INTERFACE_INCLUDE_DIRECTORIES>) # Example
# else()
#   message(WARNING "mav_msgs not found. Parts of the code might not compile or work correctly.")
# endif()

# Include directories
include_directories(include) # Local include directory


# --- Library for core tracking logic (Hungarian, Kalman, Sort3D) ---
add_library(${PROJECT_NAME}_lib
  src/hungarian.cpp
  src/sort3d.cpp
  # kalman_filter.h is header-only.
)
# Link Eigen3 to the library
target_link_libraries(${PROJECT_NAME}_lib PUBLIC Eigen3::Eigen)
target_include_directories(${PROJECT_NAME}_lib INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Eigen3_INCLUDE_DIRS} # Make Eigen available via the library
)
# Specify dependencies for the library that are needed by its headers
ament_target_dependencies(${PROJECT_NAME}_lib PUBLIC
  vision_msgs # For vision_msgs::msg::* types in sort3d.h
  geometry_msgs # For geometry_msgs::msg::* types in sort3d.h
)


# --- Executable for the ROS 2 node ---
add_executable(${PROJECT_NAME}_node src/sort3d_node.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE
    ${Eigen3_INCLUDE_DIRS} # Also for the node directly if needed
)

# Specify dependencies for the executable
ament_target_dependencies(${PROJECT_NAME}_node PUBLIC
  rclcpp
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  vision_msgs
  visualization_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  mas_msgs
)

# Link the node executable against our library and Eigen
target_link_libraries(${PROJECT_NAME}_node PUBLIC
  ${PROJECT_NAME}_lib
  Eigen3::Eigen
)

# --- Installation ---
# Install the library
install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin # Only if it were an executable library
)
# Install public headers from the 'include' directory
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" # Ensure only headers are installed
)
# Install the executable
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)
# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

# --- Exporting for other packages ---
# Export the library targets
ament_export_targets(export_${PROJECT_NAME}_lib
  HAS_LIBRARY_TARGET # Indicates that this export provides a library
)
# Export include directories for other packages to use this library
ament_export_include_directories(include)
# Export dependencies that other packages will need if they link against this library
ament_export_dependencies(
  rclcpp
  vision_msgs
  geometry_msgs
  Eigen3 # Export Eigen3 dependency
  # mav_msgs # If mav_msgs is a public dependency
)

# --- Linting (Optional but Recommended) ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
