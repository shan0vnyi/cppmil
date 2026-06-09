cmake_minimum_required(VERSION 3.20)

project(ballistics_json CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
FetchContent_MakeAvailable(json)

add_executable(ballistics_json src/main.cpp)
target_link_libraries(ballistics_json PRIVATE nlohmann_json::nlohmann_json)


target_compile_options(ballistics_json PRIVATE -Wall -Wextra -pedantic)