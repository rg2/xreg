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

#include "xregITKIOUtils.h"
#include "xregRigidUtils.h"

// Bindings for rigid/affine transform I/O and SE(3) helpers. These back the
// print_rigid_transform and xform_fcsv command-line tools. Transforms cross the
// boundary as 4x4 float32 NumPy arrays (pybind11/eigen handles Mat4x4 natively).
namespace xregpy
{

void RegisterTransforms(py::module_& parent)
{
  auto m = parent.def_submodule("transforms",
      "Rigid/affine transform I/O and SE(3) utilities.");

  m.def("read_itk_affine_transform",
        [](const std::string& path) -> xreg::Mat4x4
        {
          return xreg::ReadITKAffineTransformFromFile(path).matrix();
        },
        py::arg("path"),
        "Read a 3D affine transform from an ITK-supported file; returns a 4x4 array.");

  m.def("read_slicer_affine_transform",
        [](const std::string& path) -> xreg::Mat4x4
        {
          return xreg::ReadSlicerAffineTransformFromFile(path).matrix();
        },
        py::arg("path"),
        "Read a 3D Slicer affine transform (stored inverted) and return the 4x4 forward transform.");

  m.def("write_itk_affine_transform",
        [](const std::string& path, const xreg::Mat4x4& xform)
        {
          xreg::WriteITKAffineTransform(path, Mat4x4ToFrameTransform(xform));
        },
        py::arg("path"), py::arg("xform"),
        "Write a 4x4 affine transform to disk in an ITK-supported format.");

  m.def("rot_ang_trans_mag",
        [](const xreg::Mat4x4& xform)
        {
          xreg::CoordScalar ang = 0;
          xreg::CoordScalar mag = 0;
          std::tie(ang, mag) = xreg::ComputeRotAngTransMag(Mat4x4ToFrameTransform(xform));
          return py::make_tuple(ang, mag);
        },
        py::arg("xform"),
        "Return (rotation_angle_rad, translation_magnitude) of a rigid transform.");

  m.def("frame_diff_rot_ang_trans_mag",
        [](const xreg::Mat4x4& a, const xreg::Mat4x4& b)
        {
          xreg::CoordScalar ang = 0;
          xreg::CoordScalar mag = 0;
          std::tie(ang, mag) = xreg::FrameDiffRotAngTransMag(
              Mat4x4ToFrameTransform(a), Mat4x4ToFrameTransform(b));
          return py::make_tuple(ang, mag);
        },
        py::arg("a"), py::arg("b"),
        "Return (rotation_angle_rad, translation_magnitude) of a * inv(b).");

  m.def("se3_inv", &xreg::SE3Inv, py::arg("T"),
        "Inverse of a 4x4 SE(3) rigid transform (without a matrix inverse).");

  m.def("se3_exp",
        [](const xreg::Pt6& x) { return xreg::ExpSE3(x); },
        py::arg("x"),
        "se(3) 6-vector -> SE(3) 4x4 exponential map.");

  m.def("se3_log", &xreg::LogSE3ToMat4x4, py::arg("T"),
        "SE(3) 4x4 -> se(3) 4x4 logarithm map.");
}

}  // namespace xregpy
