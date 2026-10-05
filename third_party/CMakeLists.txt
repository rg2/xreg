
add_subdirectory(cmaes)

if (NOT XREG_USE_SYSTEM_FMT)
  add_subdirectory(fmt-12.2.0)

  # fmt's PUBLIC_HEADER entries are relative to its source dir, but they are
  # resolved relative to the dir of the install() call (top-level), so make them absolute
  get_target_property(XREG_FMT_HEADERS fmt PUBLIC_HEADER)
  list(TRANSFORM XREG_FMT_HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/fmt-12.2.0/")
  set_target_properties(fmt PROPERTIES PUBLIC_HEADER "${XREG_FMT_HEADERS}")
endif ()

if (NOT XREG_USE_SYSTEM_NLOPT)
  # Only the core C library is needed: disable the C++ only algorithms and all bindings
  set(NLOPT_CXX OFF)
  set(NLOPT_FORTRAN OFF)
  set(NLOPT_PYTHON OFF)
  set(NLOPT_OCTAVE OFF)
  set(NLOPT_MATLAB OFF)
  set(NLOPT_GUILE OFF)
  set(NLOPT_JAVA OFF)
  set(NLOPT_SWIG OFF)
  set(NLOPT_TESTS OFF)

  # NLopt's option() defaults BUILD_SHARED_LIBS to ON, which would be added to the
  # cache and change how xreg is built, so build NLopt the same way as xreg
  if (NOT DEFINED BUILD_SHARED_LIBS)
    set(BUILD_SHARED_LIBS OFF)
  endif ()

  add_subdirectory(nlopt-2.11.0)

  # NLopt caches this as TRUE, do not let it change the install RPATH of xreg's targets
  unset(CMAKE_INSTALL_RPATH_USE_LINK_PATH CACHE)
endif ()

if (NOT XREG_USE_SYSTEM_OPENCL)
  add_subdirectory(OpenCL-ICD-Loader-2020.06.16)
endif ()

