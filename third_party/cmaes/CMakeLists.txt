
if (WIN32)
	set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif ()

add_library(cmaes boundary_transformation.c cmaes.c)
target_include_directories(cmaes PUBLIC
                                   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                   $<INSTALL_INTERFACE:${XREG_INC_INSTALL_DIR}>)

