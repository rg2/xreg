/*
 * MIT License
 *
 * Copyright (c) 2022 Robert Grupp
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "xregpy_convert.h"

#include "xregLandmarkFiles.h"

// Bindings for landmark / point-cloud file I/O (FCSV and 3D Slicer Markups
// JSON). These back the print_lands and xform_fcsv command-line tools. Points
// cross the boundary as Eigen Pt3 (float32 (3,) NumPy arrays) via
// pybind11/eigen + pybind11/stl.
namespace xregpy
{

void RegisterLandmarks(py::module_& parent)
{
  auto m = parent.def_submodule("landmarks",
      "Landmark / point-cloud file I/O (FCSV, 3D Slicer Markups JSON).");

  m.def("is_supported_pts_file", &xreg::IsSupportedLandmarksFilePts,
        py::arg("path"),
        "True if a list of 3D points can be read from this file (by extension).");

  m.def("is_supported_name_pt_map_file", &xreg::IsSupportedLandmarksFileNamePtMap,
        py::arg("path"),
        "True if a name -> 3D point map can be read from this file (by extension).");

  m.def("read_pts",
        [](const std::string& path, const bool output_in_lps)
        {
          return xreg::ReadLandmarksFilePts(path, output_in_lps);
        },
        py::arg("path"), py::arg("output_in_lps") = true,
        "Read a landmark file into a list of 3D points ((3,) float32 arrays). "
        "LPS coordinates when output_in_lps is True, RAS otherwise.");

  m.def("read_name_pt_map",
        [](const std::string& path, const bool output_in_lps)
        {
          return xreg::ReadLandmarksFileNamePtMap(path, output_in_lps);
        },
        py::arg("path"), py::arg("output_in_lps") = true,
        "Read a landmark file into a dict mapping name -> 3D point. "
        "LPS coordinates when output_in_lps is True, RAS otherwise.");
}

}  // namespace xregpy
