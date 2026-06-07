cmake_minimum_required(VERSION 3.20)

# Окремий CMake-проєкт для домашньої роботи 02. Його можна збирати як
# через root CMakeLists.txt, так і напряму з цієї директорії.
project(ballistics_base CXX)

# Код домашньої роботи збирається з тим самим стандартом, що й лекційні приклади.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Один executable для задачі балістичної задачі з вибором цілі.
add_executable(ballistics_base src/main.cpp)

# Базові compiler warnings допомагають побачити підозрілі місця до runtime.
target_compile_options(ballistics_base PRIVATE -Wall -Wextra -pedantic)