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

# Embeds a Metal shader source file into a C++ source file as a string, so
# that the shader may be compiled at runtime.
#
# Run with: cmake -DXREG_METAL_SRC_IN=<.metal file> -DXREG_METAL_SRC_OUT=<.cpp file>
#                 -DXREG_METAL_SRC_NAME=<name of string variable in namespace xreg>
#                 -P xregEmbedMetalSrc.cmake

file(READ "${XREG_METAL_SRC_IN}" XREG_METAL_SRC_CONTENTS)

set(XREG_RAW_STR_DELIM "xregmsl")

string(FIND "${XREG_METAL_SRC_CONTENTS}" ")${XREG_RAW_STR_DELIM}\"" XREG_DELIM_POS)
if (NOT XREG_DELIM_POS EQUAL -1)
  message(FATAL_ERROR "${XREG_METAL_SRC_IN} contains the raw string delimiter: )${XREG_RAW_STR_DELIM}\"")
endif ()

get_filename_component(XREG_METAL_SRC_IN_NAME "${XREG_METAL_SRC_IN}" NAME)

set(XREG_METAL_SRC_CPP
"// Generated from ${XREG_METAL_SRC_IN_NAME} by xregEmbedMetalSrc.cmake - DO NOT EDIT

namespace xreg
{

extern const char* const ${XREG_METAL_SRC_NAME};

const char* const ${XREG_METAL_SRC_NAME} = R\"${XREG_RAW_STR_DELIM}(${XREG_METAL_SRC_CONTENTS})${XREG_RAW_STR_DELIM}\";

}  // xreg
")

# only write when changed to avoid unnecessary re-compilation
set(XREG_WRITE_OUT TRUE)
if (EXISTS "${XREG_METAL_SRC_OUT}")
  file(READ "${XREG_METAL_SRC_OUT}" XREG_EXISTING_OUT)
  if (XREG_EXISTING_OUT STREQUAL XREG_METAL_SRC_CPP)
    set(XREG_WRITE_OUT FALSE)
  endif ()
endif ()

if (XREG_WRITE_OUT)
  file(WRITE "${XREG_METAL_SRC_OUT}" "${XREG_METAL_SRC_CPP}")
endif ()
