find_package(GTest)

add_executable(wand_calibration-test
  main.cpp
  test.cpp
  ${PROJECT_SOURCE_DIR}/src/WandCalibration.cpp
)

target_link_libraries(wand_calibration-test
  PRIVATE
    gtest_main
    GTest::GTest
    GTest::Main
    ${GLOG_LIBRARIES}
    wand_calibration::wand_calibration
)


target_compile_features(wand_calibration-test PUBLIC cxx_std_17)
target_compile_options(wand_calibration-test PRIVATE
    $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:
        -pipe -march=native
        $<$<CONFIG:Debug>:-g -ggdb3 -O0 -Wall -Wextra>>
    $<$<CXX_COMPILER_ID:MSVC>:
        $<$<CONFIG:Debug>:/Od /Wall /Zi>>
)

gtest_add_tests(TARGET wand_calibration-test
                SOURCES
                main.cpp
                test.cpp
)
