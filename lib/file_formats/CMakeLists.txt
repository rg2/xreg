# MIT License
#
# Copyright (c) 2020-2022 Robert Grupp
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

file(GLOB_RECURSE XREG_CUR_LIB_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

set(xreg_file_formats_src ${XREG_CUR_LIB_HEADERS}
                          xregDICOMUtils.cpp
                          xregCSVUtils.cpp
                          xregCIOSFusionDICOM.cpp
                          xregACSVUtils.cpp
                          xregFCSVUtils.cpp
                          xregSTLMeshIO.cpp
                          xregH5MeshIO.cpp
                          xregMeshIO.cpp
                          xregH5PAOIO.cpp
                          xregPAOIO.cpp
                          xregH5CamModelIO.cpp
                          xregH5ProjDataIO.cpp
                          xregH5SE3OptVarsIO.cpp
                          xregWriteVideo.cpp
                          xregRadRawProj.cpp
                          xregStaRawVol.cpp
                          xregReadProjDataFromDICOM.cpp
                          xregRTKGeom.cpp
                          xregSlicerMarkupsJSON.cpp
                          xregLandmarkFiles.cpp)

if (APPLE)
  set(xreg_file_formats_src ${xreg_file_formats_src}
                            xregAppleAVFoundation.mm)
endif ()

add_library(xreg_file_formats OBJECT ${xreg_file_formats_src})

