# wasm target: ephys
# calls em++ directly, so it works from both an emcmake and a native configure

# configured with the Emscripten toolchain file, which sets EMSCRIPTEN_ROOT_PATH and makes the C++ compiler em++
set(WEBIDL_BINDER ${EMSCRIPTEN_ROOT_PATH}/tools/webidl_binder.py)
set(EMXX ${CMAKE_CXX_COMPILER})
set(IDL_FILE ${CMAKE_CURRENT_SOURCE_DIR}/ephys.idl)
set(EPHYS_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/ephys-headers.h)
set(GLUE_BASE ${CMAKE_CURRENT_BINARY_DIR}/glue)
set(GLUE_JS ${GLUE_BASE}.js)
set(GLUE_CPP ${GLUE_BASE}.cpp)
set(GLUE_O ${GLUE_BASE}.o)
set(DIST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/dist)
set(OUTPUT_DIR ${DIST_DIR}/js/ephys)
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
  COMMAND ${CMAKE_COMMAND} -E make_directory ${OUTPUT_DIR}
  COMMAND ${EMXX} ${GLUE_O} ${EPHYS_SOURCES} ${EMCC_ARGS} -o ${EPHYS_JS}
  COMMENT "compiling ephys code to wasm"
  VERBATIM
)
add_custom_target(ephys ALL DEPENDS ${EPHYS_JS} ${EPHYS_WASM})

# copy static site files into dist alongside the wasm output
add_custom_command(
  TARGET ephys POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/css ${DIST_DIR}/css
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/img ${DIST_DIR}/img
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/js ${DIST_DIR}/js
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/index.html ${DIST_DIR}/index.html
  COMMENT "copying web files to dist"
  VERBATIM
)
