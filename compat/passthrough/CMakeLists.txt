# SPDX-License-Identifier: GPL-3.0-or-later
find_program(XROS_CARGO cargo REQUIRED)
set(XROS_CARGO_OUT "$ENV{XR_CARGO_TARGET_DIR}")
if(NOT XROS_CARGO_OUT)
  set(XROS_CARGO_OUT "${CMAKE_CURRENT_BINARY_DIR}/cargo")
endif()
set(XROS_CARGO_TARGET "$ENV{XR_CARGO_TARGET}")
set(XROS_CARGO_ARGS)
set(XROS_LIBRARY "${XROS_CARGO_OUT}/release/libxros_passthrough.a")
if(XROS_CARGO_TARGET)
  list(APPEND XROS_CARGO_ARGS --target "${XROS_CARGO_TARGET}")
  set(XROS_LIBRARY "${XROS_CARGO_OUT}/${XROS_CARGO_TARGET}/release/libxros_passthrough.a")
endif()
add_custom_command(OUTPUT "${XROS_LIBRARY}"
  COMMAND "${CMAKE_COMMAND}" -E env "CARGO_TARGET_DIR=${XROS_CARGO_OUT}"
    "${XROS_CARGO}" build --release --locked --offline ${XROS_CARGO_ARGS}
    --manifest-path "${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml"
  DEPENDS Cargo.toml Cargo.lock build.rs src/lib.rs src/gpu.rs src/camera.rs src/renderer.rs
    shaders/passthrough.comp shaders/background.vert shaders/background.frag
  VERBATIM)
add_custom_target(xros_camera DEPENDS "${XROS_LIBRARY}")
add_library(xros_passthrough SHARED layer.cpp)
add_dependencies(xros_passthrough xros_camera)
target_compile_features(xros_passthrough PRIVATE cxx_std_17)
target_include_directories(xros_passthrough PRIVATE "${PROJECT_SOURCE_DIR}/src/external/openxr_includes")
target_compile_options(xros_passthrough PRIVATE -fvisibility=hidden)
target_link_libraries(xros_passthrough PRIVATE "${XROS_LIBRARY}" pthread dl m)
target_link_options(xros_passthrough PRIVATE -Wl,--exclude-libs,ALL -Wl,-z,defs)
set_target_properties(xros_passthrough PROPERTIES OUTPUT_NAME xros-passthrough)
set(XROS_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/xr/passthrough")
set(XROS_LAYER_LIBRARY "${CMAKE_INSTALL_PREFIX}/${XROS_INSTALL_DIR}/libxros-passthrough.so")
configure_file(layer.json.in layer.json @ONLY)
install(TARGETS xros_passthrough LIBRARY DESTINATION "${XROS_INSTALL_DIR}" COMPONENT XROSPassthrough)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/layer.json"
  DESTINATION "${XROS_INSTALL_DIR}/explicit.d" COMPONENT XROSPassthrough)
