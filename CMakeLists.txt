cmake_minimum_required(VERSION 3.20)

project(modeled_c LANGUAGES CXX)

add_library(modeled_c SHARED
    src/main.cpp
    src/ik.cpp
    src/object_tracking.cpp
    src/pipeline.cpp
)

set_target_properties(modeled_c PROPERTIES
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)

target_compile_features(modeled_c PUBLIC cxx_std_17)

target_include_directories(modeled_c PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)