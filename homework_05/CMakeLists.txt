cmake_minimum_required(VERSION 3.20)

# Окремий CMake-проєкт для домашньої роботи 05. Його можна збирати як
# через root CMakeLists.txt, так і напряму з цієї директорії.
project(telemetry_check CXX)

# Код домашньої роботи збирається з тим самим стандартом, що й лекційні приклади.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Створюємо статичнку бібліотеку 'telemetry' з src/telemetry.cpp
add_library(telemetry STATIC src/telemetry.cpp)
# Один executable для задачі з telemetry check.
add_executable(telemetry_check src/main.cpp)
# Пов'язуємо біблікотеку з виконавчим файлом
target_link_libraries(telemetry_check PRIVATE telemetry)
# Кажемо CMake-у де знайти файли заголовку для цієї бібліотеки
target_include_directories(telemetry PUBLIC include/)

# Базові compiler warnings допомагають побачити підозрілі місця до runtime.
target_compile_options(telemetry_check PRIVATE -Wall -Wextra -pedantic)
