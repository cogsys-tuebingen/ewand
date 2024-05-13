add_library(
  ${PROJECT_NAME}
  Utils.cpp SelfCalibration.cpp Initialization.cpp BundleAdjustment.cpp
  WandCalibration.cpp)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_link_libraries(
  ${PROJECT_NAME}
  Eigen3::Eigen
  Ceres::ceres
  Sophus::Sophus
  opencv_features2d
  opencv_imgcodecs
  opencv_calib3d
  OpenMP::OpenMP_CXX)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_compile_options(
  ${PROJECT_NAME}
  PRIVATE $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:
          -pipe
          -march=native
          -Wall
          -Wextra
          $<$<CONFIG:Release>:-O3>
          $<$<CONFIG:Debug>:-Og
          -g
          -ggdb3
          >>
          $<$<CXX_COMPILER_ID:MSVC>:
          $<$<CONFIG:Debug>:/Od
          /Wall
          /Zi>>)

add_executable(${PROJECT_NAME}_main main.cpp)

target_link_libraries(${PROJECT_NAME}_main ${PROJECT_NAME}
                      Boost::program_options)

target_include_directories(
  ${PROJECT_NAME}_main
  PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_compile_features(${PROJECT_NAME}_main PUBLIC cxx_std_17)

target_compile_options(
  ${PROJECT_NAME}_main
  PRIVATE $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:
          -pipe
          -march=native
          -Wall
          -Wextra
          $<$<CONFIG:Release>:-O3>
          $<$<CONFIG:Debug>:-Og
          -g
          -ggdb3
          >>
          $<$<CXX_COMPILER_ID:MSVC>:
          $<$<CONFIG:Debug>:/Od
          /Wall
          /Zi>>)

# BA for 3d points for the benchmark with optitrack
add_executable(${PROJECT_NAME}_point_ba point_ba.cpp)

target_link_libraries(${PROJECT_NAME}_point_ba ${PROJECT_NAME}
                      Boost::program_options)

target_include_directories(
  ${PROJECT_NAME}_point_ba
  PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
         $<INSTALL_INTERFACE:include> ${catkin_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CERES_INCLUDE_DIRS})

target_compile_features(${PROJECT_NAME}_point_ba PUBLIC cxx_std_17)

target_compile_options(
  ${PROJECT_NAME}_point_ba
  PRIVATE $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:
          -pipe
          -march=native
          -Wall
          -Wextra
          $<$<CONFIG:Release>:-O3>>
          $<$<CONFIG:Debug>:-Og
          -g
          -ggdb3
          >>
          $<$<CXX_COMPILER_ID:MSVC>:
          $<$<CONFIG:Debug>:/Od
          /Wall
          /Zi>>)

include(GenerateExportHeader)
generate_export_header(${PROJECT_NAME})

set_target_properties(
  ${PROJECT_NAME}
  PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
             LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
             RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include(GNUInstallDirs)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/wand_calibration
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
  EXPORT ${PROJECT_NAME}-targets
  FILE ${PROJECT_NAME}-targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/wand_calibration)

include(CMakePackageConfigHelpers)

configure_package_config_file(
  ${CMAKE_SOURCE_DIR}/cmake/wand_calibration-config.cmake.in
  ${CMAKE_BINARY_DIR}/cmake/wand_calibration-config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/wand_calibration)

write_basic_package_version_file(
  ${CMAKE_BINARY_DIR}/cmake/wand_calibration-config-version.cmake
  VERSION ${POLY_EXPLORATION_VERSION}
  COMPATIBILITY AnyNewerVersion)

install(FILES ${CMAKE_BINARY_DIR}/cmake/wand_calibration-config.cmake
              ${CMAKE_BINARY_DIR}/cmake/wand_calibration-config-version.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/wand_calibration)

export(
  EXPORT wand_calibration-targets
  FILE ${CMAKE_BINARY_DIR}/cmake/wand_calibration-targets.cmake
  NAMESPACE ${PROJECT_NAME}::)
