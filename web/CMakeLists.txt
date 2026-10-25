# wasm build of the SDL3 + ImGui app (target ephys_app, defined in desktop/)
# ephys.idl and ephys-headers.h (WebIDL JS bindings) are kept for reference but no longer built

set(DIST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/dist)
set(OUTPUT_DIR ${DIST_DIR}/js/ephys)

# emits dist/js/ephys/ephys.{js,wasm}, loaded by js/main.js
set_target_properties(ephys_app PROPERTIES
  OUTPUT_NAME ephys
  SUFFIX ".js"
  RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR}
)
target_link_options(ephys_app PRIVATE
  -sMODULARIZE
  -sEXPORT_NAME=ephys
  -sENVIRONMENT=web
  -sALLOW_MEMORY_GROWTH
  -sASSERTIONS
)
if(NOT "${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  target_link_options(ephys_app PRIVATE -sLLD_REPORT_UNDEFINED -g)
endif()

# copy static site files into dist alongside the wasm output
# (a separate target, since POST_BUILD only works on targets defined in this directory)
add_custom_target(ephys_site ALL
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/css ${DIST_DIR}/css
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/img ${DIST_DIR}/img
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/js ${DIST_DIR}/js
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/index.html ${DIST_DIR}/index.html
  COMMENT "copying web files to dist"
  VERBATIM
)
