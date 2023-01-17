cmake_minimum_required(VERSION 3.10)

project(ephys VERSION 0.1)

set(EMSDK $ENV{EMSDK})
set(WEBIDL_BINDER ${EMSDK}/upstream/emscripten/tools/webidl_binder.py)
set(EPHYS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/)
set(EPHYS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/includes/)
set(IDL_FILE ${CMAKE_CURRENT_SOURCE_DIR}/ephys.idl)
set(EPHYS_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/ephys-headers.h)
set(GLUE_BASE ${CMAKE_CURRENT_BINARY_DIR}/glue)
set(GLUE_JS ${GLUE_BASE}.js)
set(GLUE_CPP ${GLUE_BASE}.cpp)
set(GLUE_O ${GLUE_BASE}.o)
set(GLUE_WRAPPER ${CMAKE_CURRENT_SOURCE_DIR}/glue-wrapper.cpp)
set(OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/demos/js/ephys)
set(EPHYS_JS ${OUTPUT_DIR}/ephys.js)
set(EPHYS_WASM ${OUTPUT_DIR}/ephys.wasm)

set(EPHYS_SOURCES
  math.cpp
  rigidbody.cpp
  world.cpp
  forcegen.cpp
  particle.cpp
  pworld.cpp
  pcontacts.cpp
  pforcegen.cpp
  plinks.cpp
)
list(TRANSFORM EPHYS_SOURCES PREPEND ${EPHYS_SRC_DIR})

set(EMCC_GLUE_ARGS
  -c
  -include ${EPHYS_HEADERS}
  -I${EPHYS_INCLUDE_DIR}
)
set(EMCC_EPHYS_ARGS
  -O3
  --no-entry
  -s MODULARIZE
  -s EXPORT_NAME="ephys"
  -s LLD_REPORT_UNDEFINED
  -s ASSERTIONS
  -g
  -I${EPHYS_INCLUDE_DIR}
  --post-js ${GLUE_JS}
)

add_custom_command(
  OUTPUT ${GLUE_JS} ${GLUE_CPP}
  DEPENDS ${IDL_FILE}
  COMMAND python ${WEBIDL_BINDER} ${IDL_FILE} ${GLUE_BASE}
  COMMENT "generating webidl bindings"
  VERBATIM
)
add_custom_command(
  OUTPUT ${GLUE_O}
  DEPENDS ${GLUE_CPP}
  COMMAND emcc ${GLUE_CPP} ${EMCC_GLUE_ARGS} -o ${GLUE_O}
  COMMENT "compiling bindings"
  VERBATIM
)
add_custom_command(
  OUTPUT ${EPHYS_JS} ${EPHYS_WASM}
  DEPENDS ${GLUE_JS} ${GLUE_O} ${SOURCE_DIR}
  COMMAND emcc ${GLUE_O} ${EPHYS_SOURCES} ${EMCC_EPHYS_ARGS} -o ${EPHYS_JS}
  COMMENT "compiling ephys code to wasm"
  VERBATIM
)
add_custom_target(ephys ALL DEPENDS ${EPHYS_JS} ${EPHYS_WASM})
