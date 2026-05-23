cmake_minimum_required(VERSION 3.20)

project(ballistics_cli CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(ballistics STATIC src/ballistics.cpp)
add_executable(ballistics_cli src/main.cpp)
target_link_libraries(ballistics_cli PRIVATE ballistics)
target_include_directories(ballistics PUBLIC include/)
# окремий CMake target для тестів, підключення до бібліотеки
# з доменною логікою і реєстрація у CTest
if(NOT CMAKE_CROSSCOMPILING)
    include(FetchContent)
    include(GoogleTest)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(googletest)
    add_executable(ballistics_tests tests/ballistics_tests.cpp)
    target_link_libraries(ballistics_tests PRIVATE ballistics GTest::gtest_main GTest::gmock_main)
    gtest_discover_tests(ballistics_tests DISCOVERY_MODE PRE_TEST)
endif()