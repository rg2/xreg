
add_subdirectory(cmaes)

# NLopt's option() and oneTBB both default BUILD_SHARED_LIBS to ON, which would
# change how xreg is built (NLopt adds it to the cache), so build them the same way as xreg
if (NOT DEFINED BUILD_SHARED_LIBS)
  set(BUILD_SHARED_LIBS OFF)
endif ()

if (NOT XREG_USE_SYSTEM_TBB)
  # Only the core tbb library is needed: no tests, examples, or tbbmalloc
  set(TBB_TEST OFF)
  set(TBB_EXAMPLES OFF)
  set(TBB_STRICT OFF)
  set(TBBMALLOC_BUILD OFF)

  # NOTE: oneTBB warns that static builds are "not supported", however they build and
  #       work (dynamic loading and tbbbind/NUMA support are disabled). When building
  #       statically, ensure no other copy of TBB is linked into the same process
  #       (e.g. by a VTK or OpenCV built against another TBB).
  add_subdirectory(oneTBB-2023.1.0)

  if (NOT BUILD_SHARED_LIBS)
    # A static TBB with ITT notify (VTune instrumentation) support exports the __itt_*
    # symbols, which conflict with other static libraries embedding ittnotify (e.g. OpenCV)
    get_target_property(XREG_TBB_COMPILE_DEFS tbb COMPILE_DEFINITIONS)
    list(REMOVE_ITEM XREG_TBB_COMPILE_DEFS __TBB_USE_ITT_NOTIFY)
    set_target_properties(tbb PROPERTIES COMPILE_DEFINITIONS "${XREG_TBB_COMPILE_DEFS}")
  endif ()
endif ()

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

  add_subdirectory(nlopt-2.11.0)

  # NLopt caches this as TRUE, do not let it change the install RPATH of xreg's targets
  unset(CMAKE_INSTALL_RPATH_USE_LINK_PATH CACHE)
endif ()

if (NOT XREG_USE_SYSTEM_OPENCL)
  add_subdirectory(OpenCL-ICD-Loader-2020.06.16)
endif ()

