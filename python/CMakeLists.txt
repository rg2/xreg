# MIT License
#
# Copyright (c) 2026 Robert Grupp
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

# Python bindings for xreg using nanobind.
#
# A single extension module (xreg._xreg) is built, which statically links the xreg
# library. The module targets the Python stable ABI (abi3) of Python 3.12, so that
# one binary may be used with Python 3.12 and all newer versions.
#
# nanobind is located using the Python interpreter found by CMake, e.g. it should
# be pip installed into the Python environment used for building. When building
# via scikit-build-core (pip install .) nanobind is provided as a build requirement.
# nanobind_DIR may be set to use a specific copy of nanobind instead.

if (CMAKE_VERSION VERSION_LESS 3.26)
  message(FATAL_ERROR "The xreg Python bindings require CMake 3.26 or newer "
                      "(needed for stable ABI support in FindPython).")
endif ()

option(XREG_PY_STABLE_ABI "Build the xreg Python extension against the Python stable ABI (abi3)" ON)

if (CMAKE_CROSSCOMPILING)
  set(XREG_PY_BUILD_STUBS_DEFAULT OFF)
else ()
  set(XREG_PY_BUILD_STUBS_DEFAULT ON)
endif ()

# Stub generation imports the extension module, so it needs to be disabled when
# the module cannot be run on the build machine (e.g. cross compiling).
option(XREG_PY_BUILD_STUBS "Generate type stubs (.pyi) for the xreg Python extension" ${XREG_PY_BUILD_STUBS_DEFAULT})

set(XREG_PY_COMPONENTS Interpreter Development.Module)
if (XREG_PY_STABLE_ABI)
  list(APPEND XREG_PY_COMPONENTS Development.SABIModule)
endif ()

find_package(Python 3.12 REQUIRED COMPONENTS ${XREG_PY_COMPONENTS})

if (NOT nanobind_DIR)
  execute_process(COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
                  OUTPUT_STRIP_TRAILING_WHITESPACE
                  OUTPUT_VARIABLE XREG_NANOBIND_CMAKE_DIR
                  RESULT_VARIABLE XREG_NANOBIND_FIND_RES)

  if (NOT XREG_NANOBIND_FIND_RES EQUAL 0)
    message(FATAL_ERROR "Failed to locate nanobind using ${Python_EXECUTABLE} - "
                        "is nanobind installed in this Python environment?")
  endif ()

  set(nanobind_DIR "${XREG_NANOBIND_CMAKE_DIR}")
endif ()

find_package(nanobind CONFIG REQUIRED)

set(XREG_PY_SRCS src/xreg_py_module.cpp
                 src/xreg_py_image.cpp
                 src/xreg_py_proj_data.cpp
                 src/xreg_py_io.cpp
                 src/xreg_py_backend.cpp
                 src/xreg_py_ray_cast.cpp
                 src/xreg_py_sim_metrics.cpp
                 src/xreg_py_opt.cpp
                 src/xreg_py_regi.cpp
                 src/xreg_py_replay.cpp
                 src/xreg_py_hip.cpp)

set(XREG_PY_MODULE_OPTS NB_STATIC)
if (XREG_PY_STABLE_ABI)
  list(APPEND XREG_PY_MODULE_OPTS STABLE_ABI)
endif ()

nanobind_add_module(_xreg ${XREG_PY_MODULE_OPTS} ${XREG_PY_SRCS})

# nanobind silently falls back to a version specific build when the stable ABI
# cannot be used - make this an error, since a mis-tagged wheel would result.
if (XREG_PY_STABLE_ABI AND NOT TARGET nanobind-static-abi3)
  message(FATAL_ERROR "A stable ABI build of the xreg Python extension was requested, but is not "
                      "possible with this Python (${Python_VERSION}, ${Python_INTERPRETER_ID}). "
                      "Set XREG_PY_STABLE_ABI=OFF to build a version specific extension.")
endif ()

target_link_libraries(_xreg PRIVATE xreg)

# The pure Python portion of the package
set(XREG_PY_PKG_FILES xreg/__init__.py
                      xreg/_image_interop.py)

# Assemble an importable package in the build tree, e.g. use with:
#   PYTHONPATH=<build dir>/python python -c "import xreg"
set(XREG_PY_BUILD_PKG_DIR "${CMAKE_CURRENT_BINARY_DIR}/xreg")

set_target_properties(_xreg PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${XREG_PY_BUILD_PKG_DIR}")

# multi-config generators (e.g. Visual Studio) append the configuration name
# to the output directory unless the per-configuration properties are set
foreach (XREG_PY_CFG ${CMAKE_CONFIGURATION_TYPES})
  string(TOUPPER "${XREG_PY_CFG}" XREG_PY_CFG_UPPER)
  set_target_properties(_xreg PROPERTIES
                        LIBRARY_OUTPUT_DIRECTORY_${XREG_PY_CFG_UPPER} "${XREG_PY_BUILD_PKG_DIR}")
endforeach ()

set(XREG_PY_BUILD_PKG_FILES)
foreach (XREG_PY_FILE ${XREG_PY_PKG_FILES})
  set(XREG_PY_DST_FILE "${CMAKE_CURRENT_BINARY_DIR}/${XREG_PY_FILE}")
  add_custom_command(OUTPUT "${XREG_PY_DST_FILE}"
                     COMMAND ${CMAKE_COMMAND} -E copy_if_different
                             "${CMAKE_CURRENT_SOURCE_DIR}/${XREG_PY_FILE}" "${XREG_PY_DST_FILE}"
                     DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${XREG_PY_FILE}"
                     COMMENT "Copying ${XREG_PY_FILE} to build tree package")
  list(APPEND XREG_PY_BUILD_PKG_FILES "${XREG_PY_DST_FILE}")
endforeach ()

add_custom_target(xreg_py_pkg_files ALL DEPENDS ${XREG_PY_BUILD_PKG_FILES})
add_dependencies(_xreg xreg_py_pkg_files)

# Install rule used when building a wheel with scikit-build-core; the pure Python
# files are added to the wheel by scikit-build-core (see pyproject.toml).
# The component is used to avoid installing the C++ library, headers, etc. into the wheel.
install(TARGETS _xreg LIBRARY DESTINATION xreg COMPONENT xreg_python)

if (XREG_PY_BUILD_STUBS)
  # The extension module is imported directly (not through the xreg package)
  nanobind_add_stub(xreg_py_stub
                    MODULE _xreg
                    OUTPUT "${XREG_PY_BUILD_PKG_DIR}/_xreg.pyi"
                    MARKER_FILE "${XREG_PY_BUILD_PKG_DIR}/py.typed"
                    PYTHON_PATH "${XREG_PY_BUILD_PKG_DIR}"
                    DEPENDS _xreg)

  install(FILES "${XREG_PY_BUILD_PKG_DIR}/_xreg.pyi" "${XREG_PY_BUILD_PKG_DIR}/py.typed"
          DESTINATION xreg COMPONENT xreg_python)
endif ()
