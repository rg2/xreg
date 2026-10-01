
add_subdirectory(cmaes)

if (NOT XREG_USE_SYSTEM_FMT)
  add_subdirectory(fmt-12.2.0)

  # fmt's PUBLIC_HEADER entries are relative to its source dir, but they are
  # resolved relative to the dir of the install() call (top-level), so make them absolute
  get_target_property(XREG_FMT_HEADERS fmt PUBLIC_HEADER)
  list(TRANSFORM XREG_FMT_HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/fmt-12.2.0/")
  set_target_properties(fmt PROPERTIES PUBLIC_HEADER "${XREG_FMT_HEADERS}")
endif ()

if (NOT XREG_USE_SYSTEM_OPENCL)
  add_subdirectory(OpenCL-ICD-Loader-2020.06.16)
endif ()

