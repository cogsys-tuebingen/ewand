cmake_minimum_required (VERSION 3.10) 
project(wand_calibration
  LANGUAGES CXX C
  VERSION 0.1.0) 

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(
  OpenCV 4.7
  COMPONENTS imgproc highgui video features2d core calib3d
  REQUIRED)

find_package(Eigen3 REQUIRED NO_MODULE) 

find_package(Ceres REQUIRED)

find_package(Boost COMPONENTS program_options REQUIRED)

find_package(OpenMP REQUIRED)

set(BUILD_SOPHUS_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
set(SOPHUS_INSTALL ON CACHE BOOL "" FORCE)
add_subdirectory(libs/Sophus)

add_subdirectory(src)

add_subdirectory(test)
