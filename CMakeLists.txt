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

#################### wasm target: ephys ####################
# calls em++ directly, so it works from both an emcmake and a native configure
if(DEFINED ENV{EMSDK})
  set(EMSDK $ENV{EMSDK})
  set(WEBIDL_BINDER ${EMSDK}/upstream/emscripten/tools/webidl_binder.py)
  find_program(EMXX NAMES em++ em++.bat PATHS ${EMSDK}/upstream/emscripten REQUIRED)
  set(IDL_FILE ${CMAKE_CURRENT_SOURCE_DIR}/ephys.idl)
  set(EPHYS_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/ephys-headers.h)
  set(GLUE_BASE ${CMAKE_CURRENT_BINARY_DIR}/glue)
  set(GLUE_JS ${GLUE_BASE}.js)
  set(GLUE_CPP ${GLUE_BASE}.cpp)
  set(GLUE_O ${GLUE_BASE}.o)
  set(OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/demos/js/ephys)
  set(EPHYS_JS ${OUTPUT_DIR}/ephys.js)
  set(EPHYS_WASM ${OUTPUT_DIR}/ephys.wasm)

  set(EMCC_GLUE_ARGS
    -c
    -include ${EPHYS_HEADERS}
    -I${EPHYS_INCLUDE_DIR}
  )

  if("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    set(EMCC_ARGS
      -O3
      --no-entry
      -s MODULARIZE
      -s EXPORT_NAME="ephys"
      -s ASSERTIONS
      -I${EPHYS_INCLUDE_DIR}
      --post-js ${GLUE_JS}
    )
  else()
    set(EMCC_ARGS
      -O0
      --no-entry
      -s MODULARIZE
      -s EXPORT_NAME="ephys"
      -s LLD_REPORT_UNDEFINED
      -s ASSERTIONS
      -g
      -I${EPHYS_INCLUDE_DIR}
      --post-js ${GLUE_JS}
    )
  endif()

  add_custom_command(
    OUTPUT ${GLUE_JS} ${GLUE_CPP}
    DEPENDS ${IDL_FILE}
    COMMAND python ${WEBIDL_BINDER} ${IDL_FILE} ${GLUE_BASE}
    COMMENT "generating webidl bindings"
    VERBATIM
  )
  add_custom_command(
    OUTPUT ${GLUE_O}
    DEPENDS ${GLUE_CPP} ${EPHYS_HEADERS} ${EPHYS_HEADER_FILES}
    COMMAND ${EMXX} ${GLUE_CPP} ${EMCC_GLUE_ARGS} -o ${GLUE_O}
    COMMENT "compiling bindings"
    VERBATIM
  )
  add_custom_command(
    OUTPUT ${EPHYS_JS} ${EPHYS_WASM}
    DEPENDS ${GLUE_JS} ${GLUE_O} ${EPHYS_SOURCES} ${EPHYS_HEADER_FILES}
    COMMAND ${EMXX} ${GLUE_O} ${EPHYS_SOURCES} ${EMCC_ARGS} -o ${EPHYS_JS}
    COMMENT "compiling ephys code to wasm"
    VERBATIM
  )
  add_custom_target(ephys DEPENDS ${EPHYS_JS} ${EPHYS_WASM})
endif()

#################### native target: ephys_desktop ####################
if(NOT EMSCRIPTEN)
  add_library(ephys_core STATIC ${EPHYS_SOURCES})
  target_include_directories(ephys_core PUBLIC ${EPHYS_INCLUDE_DIR})
  target_compile_features(ephys_core PUBLIC cxx_std_17)
endif()
