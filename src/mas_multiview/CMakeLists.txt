cmake_minimum_required(VERSION 3.8)
project(mas_multiview)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(mas_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)  # not used by triangulation_node
# find_package(image_transport REQUIRED)  # not used by triangulation_node
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

add_subdirectory(lib/multiview_triangulation)

add_executable(triangulation_node src/triangulation_node.cpp)
ament_target_dependencies(triangulation_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  vision_msgs
  visualization_msgs
  mas_msgs
  Eigen3
  Ceres
)
target_include_directories(triangulation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/multiview_triangulation/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(triangulation_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(triangulation_node
  Eigen3::Eigen
  Ceres::ceres
  multiview_triangulation
)

install(TARGETS triangulation_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
