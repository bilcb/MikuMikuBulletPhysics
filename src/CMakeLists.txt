# MMP Physics — CMake Build Configuration
cmake_minimum_required(VERSION 3.20)
project(mmp_physics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)

# Bullet3 options
set(BUILD_BULLET2_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_OPENGL3_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_CPU_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(INSTALL_LIBS ON CACHE BOOL "" FORCE)
set(USE_MSVC_RUNTIME_LIBRARY_DLL ON CACHE BOOL "" FORCE)
set(USE_GLUT OFF CACHE BOOL "" FORCE)
set(BUILD_EGL OFF CACHE BOOL "" FORCE)

add_subdirectory(bullet3 EXCLUDE_FROM_ALL)

# ── core/math ──────────────────────────────────────────────────────────
add_library(mmp_core_math OBJECT
    core/math/converter.cpp
)
target_include_directories(mmp_core_math PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/core/math"
    "${CMAKE_CURRENT_SOURCE_DIR}/bullet3/src"
)

# ── core/bridge/adapter ────────────────────────────────────────────────
add_library(mmp_bridge_adapter OBJECT
    core/bridge/adapter/BlenderAdapter.cpp
)
target_include_directories(mmp_bridge_adapter PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/core/bridge/adapter"
    "${CMAKE_CURRENT_SOURCE_DIR}/bullet3/src"
)
target_link_libraries(mmp_bridge_adapter PUBLIC mmp_core_math)

# ── core/bridge/engine ─────────────────────────────────────────────────
add_library(mmp_bridge_engine STATIC
    core/bridge/engine/Engine.cpp
    core/bridge/engine/PhysicsPipeline.cpp
)
target_include_directories(mmp_bridge_engine PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(mmp_bridge_engine PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/bullet3/src")

# ── core library ───────────────────────────────────────────────────────
set(MMP_CORE_SOURCES
    core/pmx/parser.cpp
    core/vmd/parser.cpp
    core/model/model.cpp
    core/node/transform.cpp
    core/ik/solver.cpp
    core/anim/evaluator.cpp
    core/physics/world.cpp
    core/util/encoding.cpp
)

add_library(mmp_physics SHARED
    ${MMP_CORE_SOURCES}
    core/bridge/capi/CAPI.cpp
    $<TARGET_OBJECTS:mmp_core_math>
    $<TARGET_OBJECTS:mmp_bridge_adapter>
)

target_include_directories(mmp_physics PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/bullet3/src"
)

target_link_libraries(mmp_physics PRIVATE
    mmp_bridge_engine
    BulletDynamics BulletCollision LinearMath
)

set_target_properties(mmp_physics PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

if(MINGW)
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -static-libgcc -static-libstdc++ -static")
endif()
