cmake_minimum_required(VERSION 3.16)
project(LiveVoxelMapper VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Output to ./build/bin for convenience
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Find OpenMP (optional - works fine single-threaded on Pi Zero as well)
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message(STATUS "OpenMP found and enabled")
else()
    message(WARNING "OpenMP not found - parallelization will be disabled (single-threaded fallback)")
endif()

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Common headers
set(COMMON_HEADERS
    math.hpp
    grid.hpp
    pose.hpp
    motion.hpp
    ray_marching.hpp
    clustering.hpp
    navigation.hpp
)

#------------------------------------------------
# Core executable: live voxel mapper
#------------------------------------------------
add_executable(embedded_voxel_mapper
    live_voxel_mapper.cpp
    ${COMMON_HEADERS}
)
target_include_directories(embedded_voxel_mapper PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
if(OpenMP_CXX_FOUND)
    target_link_libraries(embedded_voxel_mapper OpenMP::OpenMP_CXX)
endif()

#------------------------------------------------
# Optional legacy demo: ray_voxel
# Requires nlohmann/json.hpp and stb_image.h in third_party/
#------------------------------------------------
include(CMakePushCheckState)
include(CheckIncludeFileCXX)

set(CMAKE_REQUIRED_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
check_include_file_cxx("nlohmann/json.hpp" HAVE_NLOHMANN_JSON)
check_include_file_cxx("stb_image.h" HAVE_STB_IMAGE)
set(CMAKE_REQUIRED_INCLUDES "")

if(HAVE_NLOHMANN_JSON AND HAVE_STB_IMAGE)
    add_executable(ray_voxel_legacy ray_voxel.cpp)
    target_include_directories(ray_voxel_legacy PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
    if(OpenMP_CXX_FOUND)
        target_link_libraries(ray_voxel_legacy OpenMP::OpenMP_CXX)
    endif()
    message(STATUS "Building ray_voxel_legacy (third_party deps available)")
else()
    message(STATUS "Skipping ray_voxel_legacy - third_party deps (nlohmann/json, stb) not found")
    message(STATUS "  -> See third_party/README.md for how to fetch them if you want this target")
endif()

#------------------------------------------------
# Optional Python bindings (pybind11)
# Requires: pybind11 (pip install pybind11 OR apt install pybind11-dev)
# Enable with: cmake -DBUILD_PYTHON_BINDINGS=ON ..
#------------------------------------------------
option(BUILD_PYTHON_BINDINGS "Build Python pybind11 module (process_image_cpp)" OFF)
if(BUILD_PYTHON_BINDINGS)
    find_package(pybind11 CONFIG QUIET)
    if(pybind11_FOUND)
        pybind11_add_module(process_image_cpp process_image.cpp)
        target_include_directories(process_image_cpp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
        if(OpenMP_CXX_FOUND)
            target_link_libraries(process_image_cpp PRIVATE OpenMP::OpenMP_CXX)
        endif()
        message(STATUS "Building process_image_cpp python module")
    else()
        message(WARNING "BUILD_PYTHON_BINDINGS=ON but pybind11 not found - skipping python module")
        message(STATUS "  Install with: pip install pybind11  (or  apt install pybind11-dev)")
    endif()
endif()

#------------------------------------------------
# Install targets
#------------------------------------------------
install(TARGETS embedded_voxel_mapper
    RUNTIME DESTINATION bin
)
install(FILES ${COMMON_HEADERS}
    DESTINATION include/voxel_mapper
)

# Testing
enable_testing()
add_test(NAME EmbeddedMapperSmokeTest
    COMMAND embedded_voxel_mapper
)

# Custom target for cleaning generated files
add_custom_target(clean-all
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Cleaning all build artifacts"
)

# Print configuration summary
message(STATUS "---------------- LiveVoxelMapper ----------------")
message(STATUS "Build type:        ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ compiler:      ${CMAKE_CXX_COMPILER}")
message(STATUS "C++ flags:         ${CMAKE_CXX_FLAGS}")
message(STATUS "OpenMP:            ${OpenMP_CXX_FOUND}")
message(STATUS "Python bindings:   ${BUILD_PYTHON_BINDINGS}")
message(STATUS "Target arch:       ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "------------------------------------------------")