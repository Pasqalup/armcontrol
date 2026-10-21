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

option(MODELED_BUILD_JNI "Build the JNI bridge shared library" OFF)

if(ANDROID)
    set(MODELED_BUILD_JNI ON CACHE BOOL
        "Build the JNI bridge shared library" FORCE)
endif()

if(MODELED_BUILD_JNI)
    add_library(modeled_jni SHARED
        src/main.cpp
        src/ik.cpp
        src/object_tracking.cpp
        src/pipeline.cpp
        src/jni/modeled_jni.cpp
    )

    target_compile_features(modeled_jni PUBLIC cxx_std_17)

    target_include_directories(modeled_jni PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    if(ANDROID)
        target_compile_options(modeled_jni PRIVATE -fexceptions)
        target_link_options(modeled_jni PRIVATE -Wl,--no-undefined)
    else()
        find_package(JNI REQUIRED)
        target_include_directories(modeled_jni PRIVATE ${JNI_INCLUDE_DIRS})
    endif()
endif()