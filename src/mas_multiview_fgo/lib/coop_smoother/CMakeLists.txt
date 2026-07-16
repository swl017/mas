cmake_minimum_required(VERSION 3.10)
project(coop_smoother)

add_compile_options(-std=c++17)

find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

# Header-only smoother library (ticket 024 S0). Only the test compiles a TU.
add_executable(test_coop_smoother test/test_coop_smoother.cpp)
target_include_directories(test_coop_smoother PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${GTSAM_INCLUDE_DIR}
)
target_link_libraries(test_coop_smoother PRIVATE gtsam Eigen3::Eigen)
