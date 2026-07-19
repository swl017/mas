cmake_minimum_required(VERSION 3.8)
project(mas_multiview_fgo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mas_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)   # standalone C++ dep (4.2.x) at /usr/local; ROS-distro-independent

# GTSAM 4.2 was built against Eigen 3.4.0 (/usr/local), but the ROS message packages pull the
# system Eigen 3.3.7 (/usr/include) into the include path -> GTSAM's version static-assert fails.
# Force GTSAM's Eigen ahead of the system one (prepended per-target with BEFORE).
set(GTSAM_EIGEN_INCLUDE_DIR "${EIGEN3_INCLUDE_DIR}")
if(EXISTS "/usr/local/include/eigen3/Eigen/src/Core/util/Macros.h")
  set(GTSAM_EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")
endif()

# Header-only cooperative smoother (ticket 024) — lib/coop_smoother/include.

# ---- Ticket 024 S1: ROS2 node (cooperative smoother -> coop_loc pose+twist) ----
add_executable(coop_smoother_node src/coop_smoother_node.cpp)
ament_target_dependencies(coop_smoother_node
  rclcpp mas_msgs geometry_msgs sensor_msgs vision_msgs std_msgs nav_msgs)
target_include_directories(coop_smoother_node SYSTEM BEFORE PRIVATE ${GTSAM_EIGEN_INCLUDE_DIR})
target_include_directories(coop_smoother_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/coop_smoother/include)
target_include_directories(coop_smoother_node SYSTEM PRIVATE ${GTSAM_INCLUDE_DIR})
target_link_libraries(coop_smoother_node gtsam gtsam_unstable)
install(TARGETS coop_smoother_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/ OPTIONAL)

# ---- Ticket 024 S0: offline smoother unit test (GTSAM, GPU-free) ----
add_executable(test_coop_smoother lib/coop_smoother/test/test_coop_smoother.cpp)
target_include_directories(test_coop_smoother SYSTEM BEFORE PRIVATE ${GTSAM_EIGEN_INCLUDE_DIR})
target_include_directories(test_coop_smoother PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/coop_smoother/include)
target_include_directories(test_coop_smoother SYSTEM PRIVATE ${GTSAM_INCLUDE_DIR})
target_link_libraries(test_coop_smoother gtsam)

# ---- Ticket 024 S3 track B: offline replay tool (GTSAM, GPU-free) ----
# Drives the PRODUCTION CoopSmoother on recorded factor inputs (from s3_replay_extract.py) so
# the live divergence can be replayed + counterfactuals compared on identical inputs (rev1 §11.6).
add_executable(replay_coop_smoother tools/replay_coop_smoother.cpp)
target_include_directories(replay_coop_smoother SYSTEM BEFORE PRIVATE ${GTSAM_EIGEN_INCLUDE_DIR})
target_include_directories(replay_coop_smoother PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/coop_smoother/include)
target_include_directories(replay_coop_smoother SYSTEM PRIVATE ${GTSAM_INCLUDE_DIR})
target_link_libraries(replay_coop_smoother gtsam gtsam_unstable)

install(DIRECTORY lib/coop_smoother/include/ DESTINATION include)
install(TARGETS test_coop_smoother replay_coop_smoother DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # Run the S0 smoother test under `colcon test`.
  add_test(NAME test_coop_smoother COMMAND test_coop_smoother)
endif()

ament_package()
