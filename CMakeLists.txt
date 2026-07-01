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
# Tests: cross-header smoke tests + two-TU ODR regression
#------------------------------------------------
add_executable(smoke_test tests/smoke_test.cpp)
target_include_directories(smoke_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
if(OpenMP_CXX_FOUND)
    target_link_libraries(smoke_test OpenMP::OpenMP_CXX)
endif()

# Two-TU ODR regression: link two translation units that both include every
# header. Before the inline fixes this failed to link (multiply-defined free
# functions). Keeps the gate closed on a whole class of regressions.
add_executable(odr_two_tu tests/odr_two_tu_main.cpp tests/odr_two_tu_other.cpp)
target_include_directories(odr_two_tu PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
if(OpenMP_CXX_FOUND)
    target_link_libraries(odr_two_tu OpenMP::OpenMP_CXX)
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
    COMMAND embedded_voxel_mapper --duration 1 --grid 32 --out ${CMAKE_BINARY_DIR}/ci_grid.bin
)
add_test(NAME SmokeUnitTest  COMMAND smoke_test)
add_test(NAME OdrTwoTuTest   COMMAND odr_two_tu)

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
message(STATUS "Target arch:       ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "------------------------------------------------")