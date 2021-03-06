#cmake_minimum_required (VERSION 2.8.11)

project (OPENCL_ICD_LOADER)
include (GNUInstallDirs)
find_package (Threads REQUIRED)

# The option below allows building the ICD Loader library as a shared library
# (ON, default) or a static library (OFF).
#
# Khronos OpenCL Working Group strongly recommends building and using the ICD
# loader as a shared library due to the following benefits:
#
# 1. The shared library can be updated independent of the application. This
#    allows releasing new fixes and features in the ICD loader without updating
#    the application.
#
#    In rare cases when there are backward-incompatible changes to the ICD
#    loader (due to platform requirements, for instance), using a shared
#    library allows updating the library to make the transition seamless to
#    installed applications.
#
# 2. On platforms that require the ICD mechanism there are multiple vendors
#    shipping their OpenCL implementations. The vendor installers collaborate
#    to make sure that the installed ICD shared library version is suitable for
#    working with all vendor implementations installed on the system.
#
#    If applications statically link to ICD Loader then that version of the ICD
#    loader may not work with one or more installed vendor implementations.
#
# Using the OpenCL ICD loader as a static library is NOT recommended for
# end-user installations in general. However in some controlled environments it
# may be useful to simplify the build and distribution of the application. E.g.
# in test farms, or in cases where the end-user system configs are known in
# advance. Use it with discretion.
#option (BUILD_SHARED_LIBS "Build shared libs" ON)
# RG: This will be forced at the call to add_library()

# This option enables support for OpenCL 3.0 Provisional in the ICD loader.  It
# is currently off by default while the specification is provisional, as it may
# change.
#option (ENABLE_OPENCL30_PROVISIONAL "Enable 3.0 provisional entry points" OFF)
# RG: we do not need this as an option
set (ENABLE_OPENCL30_PROVISIONAL OFF)

include(CheckFunctionExists)
check_function_exists(secure_getenv HAVE_SECURE_GETENV)
check_function_exists(__secure_getenv HAVE___SECURE_GETENV)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/loader/icd_cmake_config.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/icd_cmake_config.h)

set (OPENCL_ICD_LOADER_SOURCES
    loader/icd.c
    loader/icd.h
    loader/icd_dispatch.c
    loader/icd_dispatch.h
    loader/icd_envvars.h
    loader/icd_platform.h)

if (WIN32)
    # By default don't include OpenCL 3.0 symbol definitions (i.e. comment them
    # out), but include them for OpenCL 3.0 builds.  Once the symbols are no
    # longer provisional then they may be included unconditionally.
    set(ENABLE_OPENCL30_SYMBOLS ";")
    if (ENABLE_OPENCL30_PROVISIONAL)
      set(ENABLE_OPENCL30_SYMBOLS "")
    endif ()
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/loader/windows/OpenCL.def.in
        ${CMAKE_CURRENT_BINARY_DIR}/loader/windows/OpenCL.def)

    list (APPEND OPENCL_ICD_LOADER_SOURCES 
        loader/windows/adapter.h
        loader/windows/icd_windows.c
        loader/windows/icd_windows.h
        loader/windows/icd_windows_dxgk.c
        loader/windows/icd_windows_dxgk.h
        loader/windows/icd_windows_envvars.c
        loader/windows/icd_windows_hkr.c
        loader/windows/icd_windows_hkr.h
        loader/windows/icd_windows_apppackage.cpp
        loader/windows/icd_windows_apppackage.h
        ${CMAKE_CURRENT_BINARY_DIR}/loader/windows/OpenCL.def
        loader/windows/OpenCL.rc)
    # Only add the DXSDK include directory if the environment variable is
    # defined.  Since the DXSDK has merged into the Windows SDK, this is
    # only required in rare cases.
    if (DEFINED ENV{DXSDK_DIR} AND NOT (MINGW OR MSYS OR CYGWIN))
        include_directories ($ENV{DXSDK_DIR}/Include)
    endif ()
else ()
    # By default don't include OpenCL 3.0 symbol definitions (i.e. comment them
    # out), but include them for OpenCL 3.0 builds.  Once the symbols are no
    # longer provisional then they may be included unconditionally.
    set(ENABLE_OPENCL30_SYMBOLS_START "/*")
    set(ENABLE_OPENCL30_SYMBOLS_END "*/")
    if (ENABLE_OPENCL30_PROVISIONAL)
      set(ENABLE_OPENCL30_SYMBOLS_START "")
      set(ENABLE_OPENCL30_SYMBOLS_END "")
    endif ()
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/loader/linux/icd_exports.map.in
        ${CMAKE_CURRENT_BINARY_DIR}/loader/linux/icd_exports.map)

    list (APPEND OPENCL_ICD_LOADER_SOURCES
        loader/linux/icd_linux.c
        loader/linux/icd_linux_envvars.c
        ${CMAKE_CURRENT_BINARY_DIR}/loader/linux/icd_exports.map)
endif ()

#set (OPENCL_ICD_LOADER_HEADERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc CACHE PATH "Path to OpenCL Headers")
# RG taking this out of the cache
set (OPENCL_ICD_LOADER_HEADERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../OpenCL-Headers-2020.06.16)

add_library (OpenCL SHARED ${OPENCL_ICD_LOADER_SOURCES})
set_target_properties (OpenCL PROPERTIES VERSION "1.2" SOVERSION "1")

if (WIN32)
    target_link_libraries (OpenCL cfgmgr32.lib runtimeobject.lib)

    # RG: we need 1.2 to run on NVIDIA and MacOS - so remove this option
    #option (OPENCL_ICD_LOADER_DISABLE_OPENCLON12 "Disable support for OpenCLOn12. Support for OpenCLOn12 should only be disabled when building an import lib to link with, and must be enabled when building an ICD loader for distribution!" OFF)
    #if (OPENCL_ICD_LOADER_DISABLE_OPENCLON12)
    #    target_compile_definitions(OpenCL PRIVATE OPENCL_ICD_LOADER_DISABLE_OPENCLON12)
    #endif()

    if(NOT USE_DYNAMIC_VCXX_RUNTIME)
        string(REPLACE "/MD" "/MT" CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
        string(REPLACE "/MD" "/MT" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
        string(REPLACE "/MD" "/MT" CMAKE_C_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL}")
        string(REPLACE "/MD" "/MT" CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL}")
        string(REPLACE "/MD" "/MT" CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO}")
        string(REPLACE "/MD" "/MT" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
        string(REPLACE "/MDd" "/MTd" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
        string(REPLACE "/MDd" "/MTd" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
    endif()
else()
    if (APPLE)
        target_link_libraries (OpenCL ${CMAKE_THREAD_LIBS_INIT})
    else ()
        set_target_properties (OpenCL PROPERTIES LINK_FLAGS "-Wl,--version-script -Wl,${CMAKE_CURRENT_BINARY_DIR}/loader/linux/icd_exports.map")
        target_link_libraries (OpenCL ${CMAKE_THREAD_LIBS_INIT})
    endif ()
endif ()

#include_directories (${OPENCL_ICD_LOADER_HEADERS_DIR}) # RG
if (ENABLE_OPENCL30_PROVISIONAL)
  add_definitions (-DCL_TARGET_OPENCL_VERSION=300)
else()
  #RG add_definitions (-DCL_TARGET_OPENCL_VERSION=220)
  target_compile_definitions (OpenCL PUBLIC CL_TARGET_OPENCL_VERSION=220)
endif()

target_include_directories (OpenCL PRIVATE ${CMAKE_CURRENT_BINARY_DIR} loader)
target_include_directories (OpenCL PUBLIC
                                    $<BUILD_INTERFACE:${OPENCL_ICD_LOADER_HEADERS_DIR}>
                                    $<INSTALL_INTERFACE:${XREG_INC_INSTALL_DIR}>) # RG
target_link_libraries (OpenCL ${CMAKE_DL_LIBS})

# RG: no testing
#include (CTest)
#if (BUILD_TESTING)
#    add_subdirectory (test)
#endif()

# RG: handle the install in the root CMakeLists
#install (TARGETS OpenCL
#    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
#    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
