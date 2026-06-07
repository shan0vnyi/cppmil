#!/bin/bash

# Одна точка входу для локальної перевірки перед PR
# Команди повторюють ручний зикл з заняття 2.6.

CXX_FILES=(
    homework_06/include/ballistics.hpp
    homework_06/src/ballistics.cpp
    homework_06/src/main.cpp
    homework_06/tests/ballistics_tests.cpp
)

CMAKE_FILES=(
    homework_06/CMakeLists.txt
)

configure() {
    cmake --preset debug
}

build() {
    configure
    cmake --build --preset debug
}

format() {
    clang-format -style=file:.devcontainer/.clang-format -i "${CXX_FILES[@]}"
    cmake-format --config-file .devcontainer/.cmake-format.json -i "${CMAKE_FILES[@]}"
}

test_all() {
    build
    ctest --test-dir build/debug --output-on-failure
}

lint() {
    build
    local report_dir="build/debug/reports"
    local raw_report="${report_dir}/clang-tidy.raw.txt"
    # local report="${report_dir}/clang-tidy.txt"
    # local diagnostics="${report_dir}/clang-tidy.diagnostics.txt"

    mkdir -p "${report_dir}"

    set +e
    
    clang-tidy --quiet --config-file=.devcontainer/.clang-tidy -p build/debug \
        homework_06/src/ballistics.cpp \
        homework_06/src/main.cpp \
        homework_06/tests/ballistics_tests.cpp > "${raw_report}" 2>1
}
"$@"