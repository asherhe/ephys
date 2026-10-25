cmake_minimum_required(VERSION 3.10)

project(ephys VERSION 0.1)

set(EPHYS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/)
set(EPHYS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/includes/)

set(EPHYS_SOURCES
  collision.cpp
  contacts.cpp
  forcegen.cpp
  math.cpp
  particle.cpp
  pcontacts.cpp
  pforcegen.cpp
  plinks.cpp
  pworld.cpp
  rigidbody.cpp
  world.cpp
)
list(TRANSFORM EPHYS_SOURCES PREPEND ${EPHYS_SRC_DIR})
file(GLOB EPHYS_HEADER_FILES ${EPHYS_INCLUDE_DIR}/ephys/*.h)

# wasm target: ephys
if(DEFINED ENV{EMSDK})
  add_subdirectory(web)
endif()

# native target: ephys_desktop
if(NOT EMSCRIPTEN)
  add_library(ephys_core STATIC ${EPHYS_SOURCES})
  target_include_directories(ephys_core PUBLIC ${EPHYS_INCLUDE_DIR})
  target_compile_features(ephys_core PUBLIC cxx_std_17)

  add_subdirectory(desktop)
endif()
