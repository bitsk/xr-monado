# SPDX-License-Identifier: MIT
find_package(Python3 COMPONENTS Interpreter REQUIRED)
find_path(XR_COMPAT_OPENXR_INCLUDE openxr/openxr.h
  HINTS "${PROJECT_SOURCE_DIR}/src/external/openxr_includes" REQUIRED)
set(XR_COMPAT_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(XR_COMPAT_PREFIX "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/xr/native-interop")
separate_arguments(XR_COMPAT_FLAGS NATIVE_COMMAND "${CMAKE_CXX_FLAGS}")
set(XR_COMPAT_ARGS)
foreach(flag IN LISTS XR_COMPAT_FLAGS)
  list(APPEND XR_COMPAT_ARGS "--cxx-option=${flag}")
endforeach()
add_custom_command(
  OUTPUT "${XR_COMPAT_OUTPUT}/root/lib/libqiyu-interop.so"
         "${XR_COMPAT_OUTPUT}/root/lib/libqiyu-xr-lifecycle.so"
  COMMAND "${CMAKE_COMMAND}" -E rm -rf "${XR_COMPAT_OUTPUT}"
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/build.py"
    --output "${XR_COMPAT_OUTPUT}" --openxr-include "${XR_COMPAT_OPENXR_INCLUDE}"
    --cxx "${CMAKE_CXX_COMPILER}" --prefix "${XR_COMPAT_PREFIX}" ${XR_COMPAT_ARGS}
  DEPENDS build.py interop-layer.cpp lifecycle-layer.cpp LICENSE-MIT
  VERBATIM)
add_custom_target(xr_native_compat DEPENDS
  "${XR_COMPAT_OUTPUT}/root/lib/libqiyu-interop.so"
  "${XR_COMPAT_OUTPUT}/root/lib/libqiyu-xr-lifecycle.so")
install(DIRECTORY "${XR_COMPAT_OUTPUT}/root/lib/"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/xr/native-interop/lib" COMPONENT XRNativeInterop)
install(DIRECTORY "${XR_COMPAT_OUTPUT}/root/share/"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/xr/native-interop/share" COMPONENT XRNativeInterop)
install(FILES "${XR_COMPAT_OUTPUT}/root/profile.json"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/xr/native-interop" COMPONENT XRNativeInterop)
