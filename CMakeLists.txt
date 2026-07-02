cmake_minimum_required(VERSION 3.16)
project(dronectl LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags. Use generator expressions so gcc/clang get -Wall -Wextra
# while MSVC gets /W4 (gcc-only -Wextra would otherwise be mis-parsed by
# cl.exe as /Wextra -> D8021 invalid numeric argument). Do NOT override
# CMAKE_CXX_FLAGS_<CONFIG> here: CMake already supplies toolchain-correct
# defaults (gcc -O3 -DNDEBUG for Release, MSVC /O2 /DNDEBUG) and putting
# generator expressions with list separators (';') into those string
# variables leaks the ';' as a shell command separator and corrupts the
# compiler-depend .ts filename.
add_compile_options(
    $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>:-Wall>
    $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>:-Wextra>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# Output to ./build/bin for convenience
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

#------------------------------------------------
# Option gates (manual Part 9.2)
#
# The reference manual lists `find_package(MAVSDK REQUIRED)` and
# `find_package(onnxruntime REQUIRED)` as unconditional. To keep a bare
# `cmake -B build .` configuring and building with zero external deps on a
# fresh checkout, the MAVSDK and onnxruntime lookups are gated behind the
# CMake options below. They default OFF; a wrapped build only turns them on
# when the corresponding optional targets are wanted:
#
#   cmake -B build -DDRONECTL_ENABLE_FC_BRIDGE=ON .
#   cmake -B build -DDRONECTL_ENABLE_CFC=ON .
#
# With both OFF (the default), this script configures and builds: voxelcore
# (header-only lib), embedded_voxel_mapper, smoke_test, odr_two_tu,
# failsafe_test, perception_compile_test, observation_test, cfc_safety_test,
# the pilot_main stub, and the optional ray_voxel_legacy target.
#------------------------------------------------
option(DRONECTL_ENABLE_FC_BRIDGE "Link the MAVSDK flight-controller bridge TU into pilot_main (requires libmavsdk)" OFF)
option(DRONECTL_ENABLE_CFC       "Link the CfC ONNX pilot into pilot_main (requires onnxruntime)" OFF)

#------------------------------------------------
# Find OpenMP (optional - works fine single-threaded on Pi Zero as well)
#------------------------------------------------
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message(STATUS "OpenMP found and enabled")
else()
    message(WARNING "OpenMP not found - parallelization will be disabled (single-threaded fallback)")
endif()

# Optional external packages; guarded by the explicit options above.
if(DRONECTL_ENABLE_FC_BRIDGE)
    find_package(MAVSDK REQUIRED)
    message(STATUS "DRONECTL_ENABLE_FC_BRIDGE=ON -> MAVSDK linked into pilot_main")
endif()
if(DRONECTL_ENABLE_CFC)
    # Prefer find_package; fall back to an ORT_ROOT hint (manual Part 9.2).
    find_package(onnxruntime QUIET)
    if(onnxruntime_FOUND)
        message(STATUS "DRONECTL_ENABLE_CFC=ON -> onnxruntime linked into pilot_main (find_package)")
    elseif(DEFINED ENV{ORT_ROOT})
        set(ORT_INCLUDE_DIR "$ENV{ORT_ROOT}/include")
        set(ORT_LIB_DIR "$ENV{ORT_ROOT}/lib")
        if(EXISTS "${ORT_INCLUDE_DIR}/onnxruntime_cxx_api.h" AND
           EXISTS "${ORT_LIB_DIR}/libonnxruntime.so")
            set(ORT_ROOT_FOUND TRUE)
            message(STATUS "DRONECTL_ENABLE_CFC=ON -> onnxruntime linked via ORT_ROOT=$ENV{ORT_ROOT}")
        else()
            message(FATAL_ERROR "DRONECTL_ENABLE_CFC=ON but onnxruntime not found via find_package or ORT_ROOT")
        endif()
    else()
        message(FATAL_ERROR "DRONECTL_ENABLE_CFC=ON but onnxruntime not found (install it or set ORT_ROOT)")
    endif()
endif()

#------------------------------------------------
# Header-only interface library: voxelcore
# Phase-0 headers (math/grid/pose/motion/ray_marching/clustering/navigation)
# live in core/. Including "math.hpp" etc. bare works from any TU because core/
# is on the include path.
#------------------------------------------------
add_library(voxelcore INTERFACE)
target_include_directories(voxelcore INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/core)

# Common headers (now under core/)
set(COMMON_HEADERS
    core/math.hpp
    core/grid.hpp
    core/pose.hpp
    core/motion.hpp
    core/ray_marching.hpp
    core/clustering.hpp
    core/navigation.hpp
)

#------------------------------------------------
# Core executable: live voxel mapper
#------------------------------------------------
add_executable(embedded_voxel_mapper
    core/live_voxel_mapper.cpp
    ${COMMON_HEADERS}
)
target_link_libraries(embedded_voxel_mapper PRIVATE voxelcore)
if(OpenMP_CXX_FOUND)
    target_link_libraries(embedded_voxel_mapper PRIVATE OpenMP::OpenMP_CXX)
endif()

#------------------------------------------------
# onboard pilot_main: Phase-3 onboard loop. Composes DronePoseProvider +
# FcBridge (optional MAVSDK) + CfCPilot (optional onnxruntime) + Failsafe +
# DualVoxelGrid. Header-only CfC pieces (control/cfc_pilot/observation.hpp +
# cfc_pilot.hpp) are pulled in via the include path. With both feature
# options OFF (default) the pilot main still builds and produces a working
# binary where the CfC is a no-op HOLD and the FcBridge TU is omitted.
#------------------------------------------------
set(PILOT_SOURCES
    control/pilot_main.cpp
    perception/drone_pose_provider.cpp
)
if(DRONECTL_ENABLE_FC_BRIDGE)
    list(APPEND PILOT_SOURCES control/fc_bridge/fc_bridge.cpp)
endif()

add_executable(pilot_main ${PILOT_SOURCES}
    control/cfc_pilot/observation.hpp
    control/cfc_pilot/cfc_pilot.hpp
)
target_link_libraries(pilot_main PRIVATE voxelcore)
target_include_directories(pilot_main PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/control/fc_bridge
    ${CMAKE_CURRENT_SOURCE_DIR}/control/cfc_pilot
)
if(DRONECTL_ENABLE_FC_BRIDGE)
    target_compile_definitions(pilot_main PRIVATE DRONECTL_HAVE_MAVSDK)
    target_link_libraries(pilot_main PRIVATE MAVSDK::mavsdk)
    message(STATUS "Building pilot_main with FcBridge (MAVSDK found)")
else()
    message(STATUS "Building pilot_main without FcBridge (DRONECTL_ENABLE_FC_BRIDGE=OFF)")
endif()
if(DRONECTL_ENABLE_CFC)
    target_compile_definitions(pilot_main PRIVATE DRONECTL_HAVE_ORT)
    if(onnxruntime_FOUND)
        target_link_libraries(pilot_main PRIVATE onnxruntime::onnxruntime)
    elseif(ORT_ROOT_FOUND)
        target_include_directories(pilot_main PRIVATE ${ORT_INCLUDE_DIR})
        target_link_directories(pilot_main PRIVATE ${ORT_LIB_DIR})
        target_link_libraries(pilot_main PRIVATE onnxruntime)
    endif()
    message(STATUS "Linking CfC pilot (onnxruntime found)")
endif()
if(OpenMP_CXX_FOUND)
    target_link_libraries(pilot_main PRIVATE OpenMP::OpenMP_CXX)
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
    add_executable(ray_voxel_legacy core/ray_voxel.cpp)
    target_link_libraries(ray_voxel_legacy PRIVATE voxelcore)
    target_include_directories(ray_voxel_legacy PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
    if(OpenMP_CXX_FOUND)
        target_link_libraries(ray_voxel_legacy PRIVATE OpenMP::OpenMP_CXX)
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
target_link_libraries(smoke_test PRIVATE voxelcore)
if(OpenMP_CXX_FOUND)
    target_link_libraries(smoke_test PRIVATE OpenMP::OpenMP_CXX)
endif()

# Two-TU ODR regression: link two translation units that both include every
# header. Before the inline fixes this failed to link (multiply-defined free
# functions). Keeps the gate closed on a whole class of regressions.
add_executable(odr_two_tu tests/odr_two_tu_main.cpp tests/odr_two_tu_other.cpp)
target_link_libraries(odr_two_tu PRIVATE voxelcore)
if(OpenMP_CXX_FOUND)
    target_link_libraries(odr_two_tu PRIVATE OpenMP::OpenMP_CXX)
endif()

# Failsafe state-machine unit tests (Part 7). Header-only failsafe.hpp, so the
# test is a single self-contained TU just like smoke_test.
add_executable(failsafe_test tests/failsafe_test.cpp)
target_link_libraries(failsafe_test PRIVATE voxelcore)
if(OpenMP_CXX_FOUND)
    target_link_libraries(failsafe_test PRIVATE OpenMP::OpenMP_CXX)
endif()

# Phase 1 perception compile/link gate. Links tests/perception_compile_test.cpp
# against the perception TU (perception/drone_pose_provider.cpp) so the new
# headers + TU are exercised. Implicitly also a small two-TU ODR check for
# the perception code.
add_executable(perception_compile_test
    tests/perception_compile_test.cpp
    perception/drone_pose_provider.cpp
)
target_link_libraries(perception_compile_test PRIVATE voxelcore)
if(OpenMP_CXX_FOUND)
    target_link_libraries(perception_compile_test PRIVATE OpenMP::OpenMP_CXX)
endif()

# Phase 3 (CfC observation builder + safety filter unit tests). Both are
# header-only-depending tests; the safety-filter test deliberately does
# NOT need onnxruntime (the geometric veto is decoupled from inference).
add_executable(observation_test tests/observation_test.cpp)
target_link_libraries(observation_test PRIVATE voxelcore)
if(OpenMP_CXX_FOUND)
    target_link_libraries(observation_test PRIVATE OpenMP::OpenMP_CXX)
endif()

add_executable(cfc_safety_test tests/cfc_safety_test.cpp)
target_link_libraries(cfc_safety_test PRIVATE voxelcore)
if(OpenMP_CXX_FOUND)
    target_link_libraries(cfc_safety_test PRIVATE OpenMP::OpenMP_CXX)
endif()

#------------------------------------------------
# Phase 5: per-drone ground coordinator (manual Part 6.1).
# POSIX-sockets + JSON-line TCP client to the Phase 4 tether-agent on the
# drone side, plus a read-only /fleet HTTP scrape on 127.0.0.1:8090. Pure
# C++17, no HTTP framework dep. Built as an OPTIONAL self-test binary —
# NOT part of the default `all`/`test` flow (mirrors the tether-agent
# scaffold). `make coordinator` / `cmake --build . --target coordinator`
# builds it on demand.
#------------------------------------------------
add_executable(coordinator ground/coordinator/coordinator.cpp)
target_link_libraries(coordinator PRIVATE voxelcore)
if(OpenMP_CXX_FOUND)
    target_link_libraries(coordinator PRIVATE OpenMP::OpenMP_CXX)
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
add_test(NAME FailsafeTest   COMMAND failsafe_test)
add_test(NAME PerceptionCompileTest COMMAND perception_compile_test)
add_test(NAME ObservationTest COMMAND observation_test)
add_test(NAME CfcSafetyTest   COMMAND cfc_safety_test)

# Custom target for cleaning generated files
add_custom_target(clean-all
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Cleaning all build artifacts"
)

# Print configuration summary
message(STATUS "---------------- dronectl ----------------")
message(STATUS "Build type:        ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ compiler:      ${CMAKE_CXX_COMPILER}")
message(STATUS "C++ flags:         ${CMAKE_CXX_FLAGS}")
message(STATUS "OpenMP:            ${OpenMP_CXX_FOUND}")
message(STATUS "FC bridge:         ${DRONECTL_ENABLE_FC_BRIDGE}")
message(STATUS "CfC pilot:         ${DRONECTL_ENABLE_CFC}")
message(STATUS "Target arch:       ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "------------------------------------------")