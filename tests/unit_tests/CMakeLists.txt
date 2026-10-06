
if (XREG_USE_METAL)
  add_subdirectory(metal_devices)
  add_subdirectory(metal_prog_opts)
  add_subdirectory(metal_sync_buf)
endif ()

if (APPLE)
  add_subdirectory(apple_avf_video)
endif ()
