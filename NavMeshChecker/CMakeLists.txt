cmake_minimum_required(VERSION 3.10)
project(NavMeshChecker)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the recastnavigation installation
set(RECASTNAVIGATION_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")

# Debug: Print paths
message(STATUS "RECASTNAVIGATION_ROOT: ${RECASTNAVIGATION_ROOT}")
message(STATUS "Looking for Recast headers in: ${RECASTNAVIGATION_ROOT}/Recast/Include")
message(STATUS "Looking for Detour headers in: ${RECASTNAVIGATION_ROOT}/Detour/Include")

# Check if directories exist
if(EXISTS "${RECASTNAVIGATION_ROOT}/Recast/Include")
    message(STATUS "Found Recast Include directory")
else()
    message(FATAL_ERROR "Recast Include directory not found at: ${RECASTNAVIGATION_ROOT}/Recast/Include")
endif()

if(EXISTS "${RECASTNAVIGATION_ROOT}/Detour/Include")
    message(STATUS "Found Detour Include directory")
else()
    message(FATAL_ERROR "Detour Include directory not found at: ${RECASTNAVIGATION_ROOT}/Detour/Include")
endif()

# Include directories
include_directories(
    ${RECASTNAVIGATION_ROOT}/Recast/Include
    ${RECASTNAVIGATION_ROOT}/Detour/Include
    ${RECASTNAVIGATION_ROOT}/RecastDemo/Include
)

# Check for libraries
set(RECAST_LIB_PATH "${RECASTNAVIGATION_ROOT}/build/Recast/Release/Recast.lib")
set(DETOUR_LIB_PATH "${RECASTNAVIGATION_ROOT}/build/Detour/Release/Detour.lib")

if(EXISTS ${RECAST_LIB_PATH})
    message(STATUS "Found Recast library: ${RECAST_LIB_PATH}")
else()
    message(WARNING "Recast library not found at: ${RECAST_LIB_PATH}")
    # Try alternative paths
    if(EXISTS "${RECASTNAVIGATION_ROOT}/build/Recast/Recast.lib")
        set(RECAST_LIB_PATH "${RECASTNAVIGATION_ROOT}/build/Recast/Recast.lib")
        message(STATUS "Found Recast library at alternative path: ${RECAST_LIB_PATH}")
    endif()
endif()

if(EXISTS ${DETOUR_LIB_PATH})
    message(STATUS "Found Detour library: ${DETOUR_LIB_PATH}")
else()
    message(WARNING "Detour library not found at: ${DETOUR_LIB_PATH}")
    # Try alternative paths
    if(EXISTS "${RECASTNAVIGATION_ROOT}/build/Detour/Detour.lib")
        set(DETOUR_LIB_PATH "${RECASTNAVIGATION_ROOT}/build/Detour/Detour.lib")
        message(STATUS "Found Detour library at alternative path: ${DETOUR_LIB_PATH}")
    endif()
endif()

# Add executables
add_executable(NavMeshChecker 
    navmesh_checker.cpp
    ${RECASTNAVIGATION_ROOT}/RecastDemo/Source/ChunkyTriMesh.cpp
)

add_executable(NavMeshLoader 
    navmesh_loader.cpp
)

# Link libraries using full paths
target_link_libraries(NavMeshChecker 
    ${RECAST_LIB_PATH}
    ${DETOUR_LIB_PATH}
)

target_link_libraries(NavMeshLoader 
    ${DETOUR_LIB_PATH}
)

# Set output directory
set_target_properties(NavMeshChecker NavMeshLoader PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin
)