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

#include <cstring>

#include "xregPerspectiveXform.h"
#include "xregProjData.h"
#include "xregH5ProjDataIO.h"

// Bindings for the xreg HDF5 "proj-data" format: the central container for 2D
// fluoroscopy/radiograph projections plus their pinhole CameraModel and 2D
// landmarks. Backs convert_*_to_proj_data, proj_cat, extract_nii_from_proj_data,
// and is a prerequisite for the 2D/3D registration pipeline.
namespace xregpy
{

namespace
{

using Proj2D = itk::Image<float,2>;

// Copy an ITK 2D float image into a (rows, cols) float32 NumPy array. ITK 2D
// buffers are column-fastest, matching a C-order (ny, nx) array, so this is a
// single contiguous memcpy.
py::array_t<float> Proj2DToNumpy(const Proj2D* img)
{
  const auto size = img->GetLargestPossibleRegion().GetSize();  // [0]=cols, [1]=rows
  const py::ssize_t ncols = static_cast<py::ssize_t>(size[0]);
  const py::ssize_t nrows = static_cast<py::ssize_t>(size[1]);

  py::array_t<float> arr({ nrows, ncols });
  std::memcpy(arr.mutable_data(), img->GetBufferPointer(),
              static_cast<std::size_t>(nrows) * static_cast<std::size_t>(ncols) * sizeof(float));
  return arr;
}

// A single projection as it crosses the boundary.
struct Projection
{
  py::array_t<float>          pixels;     // (rows, cols) float32; empty if not read
  xreg::CameraModel           cam;
  std::unordered_map<std::string, xreg::Pt2> landmarks;  // name -> (col, row) index
};

Projection ProjDataF32ToProjection(const xreg::ProjDataF32& pd)
{
  Projection out;
  out.cam = pd.cam;
  if (pd.img)
  {
    out.pixels = Proj2DToNumpy(pd.img.GetPointer());
  }
  for (const auto& kv : pd.landmarks)
  {
    out.landmarks[kv.first] = kv.second;
  }
  return out;
}

}  // namespace

void RegisterProjData(py::module_& parent)
{
  auto m = parent.def_submodule("proj_data",
      "xreg HDF5 proj-data (2D projections + camera models) I/O.");

  py::enum_<xreg::CameraModel::CameraCoordFrame>(m, "CameraCoordFrame")
    .value("ORIGIN_AT_FOCAL_PT_DET_POS_Z", xreg::CameraModel::kORIGIN_AT_FOCAL_PT_DET_POS_Z)
    .value("ORIGIN_AT_FOCAL_PT_DET_NEG_Z", xreg::CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z)
    .value("ORIGIN_ON_DETECTOR",           xreg::CameraModel::kORIGIN_ON_DETECTOR);

  py::class_<xreg::CameraModel>(m, "CameraModel",
      "Pinhole camera / projection geometry model.")
    .def(py::init<>())
    .def_readwrite("intrins",         &xreg::CameraModel::intrins, "3x3 intrinsic matrix")
    .def_readwrite("intrins_inv",     &xreg::CameraModel::intrins_inv)
    .def_property("extrins",
        [](const xreg::CameraModel& c) -> xreg::Mat4x4 { return c.extrins.matrix(); },
        [](xreg::CameraModel& c, const xreg::Mat4x4& m) { c.extrins = Mat4x4ToFrameTransform(m); },
        "4x4 extrinsic (world -> camera) matrix")
    .def_property_readonly("extrins_inv",
        [](const xreg::CameraModel& c) -> xreg::Mat4x4 { return c.extrins_inv.matrix(); },
        "4x4 extrinsic (camera -> world) matrix")
    .def_readwrite("pinhole_pt",      &xreg::CameraModel::pinhole_pt, "pinhole point in world coords")
    .def_readwrite("focal_len",       &xreg::CameraModel::focal_len)
    .def_readwrite("num_det_rows",    &xreg::CameraModel::num_det_rows)
    .def_readwrite("num_det_cols",    &xreg::CameraModel::num_det_cols)
    .def_readwrite("det_row_spacing", &xreg::CameraModel::det_row_spacing)
    .def_readwrite("det_col_spacing", &xreg::CameraModel::det_col_spacing)
    .def_readwrite("coord_frame_type",&xreg::CameraModel::coord_frame_type)
    .def("setup",
        [](xreg::CameraModel& c, const xreg::CoordScalar focal_len,
           const xreg::size_type nr, const xreg::size_type nc,
           const xreg::CoordScalar rs, const xreg::CoordScalar cs)
        { c.setup(focal_len, nr, nc, rs, cs); },
        py::arg("focal_len"), py::arg("num_rows"), py::arg("num_cols"),
        py::arg("row_spacing"), py::arg("col_spacing"),
        "Initialize a trivial pinhole geometry (principal point centered).");

  py::class_<Projection>(m, "Projection",
      "A single 2D projection: pixels (rows, cols) + CameraModel + 2D landmarks.")
    .def(py::init<>())
    .def_readonly("pixels",    &Projection::pixels)
    .def_readonly("cam",       &Projection::cam)
    .def_readonly("landmarks", &Projection::landmarks,
                  "name -> (col, row) continuous pixel index");

  m.def("read_proj_data_f32",
        [](const std::string& path, const bool read_pixels)
        {
          const auto pd_list = xreg::ReadProjDataH5F32FromDisk(path, read_pixels);
          std::vector<Projection> out;
          out.reserve(pd_list.size());
          for (const auto& pd : pd_list)
          {
            out.push_back(ProjDataF32ToProjection(pd));
          }
          return out;
        },
        py::arg("path"), py::arg("read_pixels") = true,
        "Read a float32 proj-data HDF5 file into a list of Projection objects.");

  m.def("read_cam_models",
        [](const std::string& path)
        {
          return xreg::ReadCamModelsFromProjDataFromDisk(path);
        },
        py::arg("path"),
        "Read just the camera models from a proj-data HDF5 file.");

  m.def("num_projs",
        [](const std::string& path)
        {
          xreg::DeferredProjReader reader(path);
          return reader.num_projs_on_disk();
        },
        py::arg("path"),
        "Return the number of projections stored in a proj-data HDF5 file.");
}

}  // namespace xregpy
