# MIT License
#
# Copyright (c) 2020-2021 Robert Grupp
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cmake_minimum_required(VERSION 3.13.0 FATAL_ERROR)

#if (APPLE)
#  # Target MacOS 10.14, Mojave
#  set (CMAKE_OSX_DEPLOYMENT_TARGET "10.14" CACHE STRING "MacOS Deployment Target")
#endif ()

# Version format is YYYY.MM.DD for a release and YYYY.MM.DD.1 for devel after the release
project(xreg VERSION 2021.09.19.1)

set (CMAKE_CXX_STANDARD "11" CACHE STRING "C++ Standard (needs at least 11)")
mark_as_advanced(CMAKE_CXX_STANDARD)

set (XREG_TOP_LEVEL_PROJ FALSE)
if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  set (XREG_TOP_LEVEL_PROJ TRUE)
endif ()

option(XREG_BUILD_APPS "Build xreg applications in addition to the library" ${XREG_TOP_LEVEL_PROJ})

option(XREG_INCLUDE_GIT_HASH_IN_VER_STR "Include the git hash in the version string that is embedded into the library" OFF)
mark_as_advanced(XREG_INCLUDE_GIT_HASH_IN_VER_STR)

if (NOT APPLE)
  set (XREG_USE_SYSTEM_OPENCL_DEFAULT OFF)
else ()
  set (XREG_USE_SYSTEM_OPENCL_DEFAULT ON)
endif ()

option(XREG_USE_SYSTEM_OPENCL "Use a system install of OpenCL instead of the bundled loader" ${XREG_USE_SYSTEM_OPENCL_DEFAULT})
mark_as_advanced(XREG_USE_SYSTEM_OPENCL)

if (APPLE OR WIN32)
  set(XREG_USE_LEGACY_FIND_TBB_DEFAULT OFF)
else ()
  set(XREG_USE_LEGACY_FIND_TBB_DEFAULT ON)
endif ()

option(XREG_USE_LEGACY_FIND_TBB "Use a legacy FindTBB.cmake which works better for tbb installed on Ubuntu through the system package manager - it does not work on Windows w/ VS 2019" ${XREG_USE_LEGACY_FIND_TBB_DEFAULT})
mark_as_advanced(XREG_USE_LEGACY_FIND_TBB)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake" ${CMAKE_MODULE_PATH})
if (XREG_USE_LEGACY_FIND_TBB)
  set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/tbb" ${CMAKE_MODULE_PATH})
  set(XREG_TBB_LIB tbb)
else ()
  set(XREG_TBB_LIB TBB::tbb)
endif ()

set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

include(GNUInstallDirs)

set (XREG_INC_INSTALL_DIR "${CMAKE_INSTALL_INCLUDEDIR}/xreg")

if (XREG_USE_SYSTEM_OPENCL)
  find_package(OpenCL REQUIRED)
  set (XREG_EXT_LIBS_TO_LINK OpenCL::OpenCL)
endif ()

find_path(VIENNACL_INC_DIR viennacl/matrix.hpp PATHS ${ViennaCL_DIR} ${ViennaCL_DIR}/..)
if (${VIENNACL_INC_DIR} STREQUAL "VIENNACL_INC_DIR-NOTFOUND")
  message(SEND_ERROR "ViennaCL not found!")
endif ()

find_package(TBB REQUIRED)

find_package(Boost REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(fmt REQUIRED)

find_package(ITK REQUIRED)
include(${ITK_USE_FILE})

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(OpenCV REQUIRED)

find_package(NLopt REQUIRED)

add_subdirectory(third_party)

set(XREG_EXT_LIB_INC_DIRS ${Boost_INCLUDE_DIRS}
                          ${EIGEN3_INCLUDE_DIR}
                          ${FMT_INCLUDE_DIRS}
                          ${ITK_INCLUDE_DIRS}
                          ${ITK_INCLUDE_DIRS}/itkhdf5
                          ${OpenCV_INCLUDE_DIRS}
                          ${VTK_INCLUDE_DIRS}
                          ${VIENNACL_INC_DIR})

set(XREG_EXT_LIBS_TO_LINK ${XREG_EXT_LIBS_TO_LINK}
                          fmt::fmt
                          ${ITK_LIBRARIES}
                          ${OpenCV_LIBS}
                          ${XREG_TBB_LIB}
                          ${VTK_LIBRARIES}
                          ${NLOPT_LIBRARIES})

if (XREG_INCLUDE_GIT_HASH_IN_VER_STR)
  include(GetGitRevisionDescription)
  get_git_head_revision(GIT_REFSPEC XREG_GIT_SHA1)
  set(XREG_HAS_GIT_SHA1 true)
else ()
  set(XREG_GIT_SHA1 "")
  set(XREG_HAS_GIT_SHA1 false)
endif ()

add_subdirectory(lib)

set(XREG_EXE_LIBS_TO_LINK ${XREG_EXT_LIBS_TO_LINK} xreg)

# Prefix of each application executable filename
set(XREG_EXE_PREFIX "xreg-")

if (XREG_BUILD_APPS)
  add_subdirectory(apps)
endif ()

# only install targets and config when building the library
# as its own top-level project. This is needed for when the
# library file tree is located within another project and added
# via a call to add_subdirectory()
if (XREG_TOP_LEVEL_PROJ)
  # Even though the cmaes target may be static library with a private
  # relationship to xreg, CMake still requires it to be exported!
  set (XREG_TARGETS_TO_EXPORT xreg cmaes)

  if (NOT XREG_USE_SYSTEM_OPENCL)
    set (XREG_TARGETS_TO_EXPORT ${XREG_TARGETS_TO_EXPORT} OpenCL)
    install(DIRECTORY third_party/OpenCL-Headers-2020.06.16/CL DESTINATION ${XREG_INC_INSTALL_DIR})
  endif ()


  install(TARGETS ${XREG_TARGETS_TO_EXPORT} EXPORT xreg-targets
          RUNTIME       DESTINATION ${CMAKE_INSTALL_BINDIR}
          LIBRARY       DESTINATION ${CMAKE_INSTALL_LIBDIR}
          ARCHIVE       DESTINATION ${CMAKE_INSTALL_LIBDIR}
          PUBLIC_HEADER DESTINATION ${XREG_INC_INSTALL_DIR})

  include(CMakePackageConfigHelpers)

  set(XREG_CMAKE_CONFIG_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/xreg)

  configure_package_config_file(cmake/xregConfig.cmake.in
                                ${PROJECT_BINARY_DIR}/xregConfig.cmake
                                INSTALL_DESTINATION ${XREG_CMAKE_CONFIG_INSTALL_DIR})

  write_basic_package_version_file(${PROJECT_BINARY_DIR}/xregConfigVersion.cmake COMPATIBILITY AnyNewerVersion)

  export(TARGETS ${XREG_TARGETS_TO_EXPORT} NAMESPACE xreg:: FILE ${PROJECT_BINARY_DIR}/xreg-targets.cmake)

  install(FILES ${PROJECT_BINARY_DIR}/xregConfig.cmake ${PROJECT_BINARY_DIR}/xregConfigVersion.cmake
          DESTINATION ${XREG_CMAKE_CONFIG_INSTALL_DIR})

  install(EXPORT xreg-targets NAMESPACE xreg:: DESTINATION ${XREG_CMAKE_CONFIG_INSTALL_DIR})
  
  if (WIN32)
    install(FILES dist/setup-xreg-vars.bat DESTINATION ${CMAKE_INSTALL_BINDIR})
  endif ()
endif ()

