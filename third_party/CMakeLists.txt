
add_subdirectory(cmaes)

if (NOT XREG_USE_SYSTEM_FMT)
  add_subdirectory(fmt-12.2.0)
endif ()

if (NOT XREG_USE_SYSTEM_OPENCL)
  add_subdirectory(OpenCL-ICD-Loader-2020.06.16)
endif ()

