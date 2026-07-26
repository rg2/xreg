# MIT License
#
# Copyright (c) 2022 Robert Grupp
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

# Builds the `pyxreg` Python extension module (pybind11) against the xreg
# library target. Enabled from the top-level CMakeLists via XREG_BUILD_PYTHON.

# pybind11 needs at least C++14 for its current releases; bump if the project is
# still on C++11 (xreg itself only requires C++11).
if (CMAKE_CXX_STANDARD LESS 14)
  message(STATUS "xreg-python: raising CMAKE_CXX_STANDARD to 14 for pybind11")
  set(CMAKE_CXX_STANDARD 14)
endif ()

# Prefer an installed pybind11 (e.g. from pip/conda: `pybind11-config --cmakedir`
# or `find_package`), and fall back to fetching a pinned release. Set
# XREG_PYBIND11_FETCH=OFF to require a system/installed pybind11 instead.
option(XREG_PYBIND11_FETCH "Fetch pybind11 with FetchContent if not found" ON)

find_package(pybind11 CONFIG QUIET)

if (NOT pybind11_FOUND)
  if (XREG_PYBIND11_FETCH)
    message(STATUS "xreg-python: pybind11 not found, fetching a pinned release")
    include(FetchContent)
    FetchContent_Declare(pybind11
      GIT_REPOSITORY https://github.com/pybind/pybind11.git
      GIT_TAG        v2.11.1)
    FetchContent_MakeAvailable(pybind11)
  else ()
    message(FATAL_ERROR
      "xreg-python: pybind11 not found and XREG_PYBIND11_FETCH is OFF. "
      "Install pybind11 (pip install pybind11) or set XREG_PYBIND11_FETCH=ON.")
  endif ()
endif ()

set(XREGPY_SRCS
    src/xregpy_module.cpp
    src/xregpy_convert.cpp
    src/xregpy_transforms.cpp
    src/xregpy_landmarks.cpp
    src/xregpy_image_io.cpp
    src/xregpy_mesh.cpp
    src/xregpy_proj_data.cpp)

pybind11_add_module(xreg_python MODULE ${XREGPY_SRCS})

# Import name is `pyxreg` (the CMake target is xreg_python to avoid colliding
# with the `xreg` C++ library target).
set_target_properties(xreg_python PROPERTIES OUTPUT_NAME pyxreg)

# XREG_EXE_LIBS_TO_LINK == external libs + the `xreg` target, which publicly
# propagates all of xreg's include directories and external dependencies.
target_link_libraries(xreg_python PRIVATE ${XREG_EXE_LIBS_TO_LINK})

# Install the extension next to the pure-Python package if a destination is set.
if (NOT DEFINED XREG_PYTHON_INSTALL_DIR)
  set(XREG_PYTHON_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/python/xreg")
endif ()

install(TARGETS xreg_python LIBRARY DESTINATION ${XREG_PYTHON_INSTALL_DIR})
