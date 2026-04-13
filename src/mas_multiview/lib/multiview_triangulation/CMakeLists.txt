cmake_minimum_required(VERSION 3.10)
project(multiview_triangulation)

# Set C++ standard
add_compile_options(-std=c++17)
find_package(Eigen3 REQUIRED)

# Only find Ceres if not already found by parent
if(NOT Ceres_FOUND)
  find_package(Ceres REQUIRED)
endif()

# Add executable
add_library(${PROJECT_NAME}
    src/multiview_triangulation.cpp
    src/covariance_propagation.cpp
)
# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${Ceres_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PRIVATE 
    Eigen3::Eigen
    Ceres::ceres
)

# add_executable(cartesian_product test/cartesian_product.cpp)

# Make the library installable
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 2)
    # Export the targets
    ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
    ament_export_dependencies(Eigen3 Ceres)
    ament_export_include_directories(include)
    ament_export_libraries(${PROJECT_NAME})
  endif()
endif()