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

#ifndef XREGPY_CONVERT_H_
#define XREGPY_CONVERT_H_

#include <array>
#include <cstdint>
#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <itkImage.h>

#include "xregCommon.h"
#include "xregMesh.h"

// Shared type marshaling between xreg's C++ types and NumPy/Eigen. This is the
// foundation every binding depends on: it keeps the ITK <-> NumPy and
// Eigen-transform conversions in one place so the per-feature binding files stay
// thin. See python/README.md for the design rationale.
namespace xregpy
{

namespace py = pybind11;

/// 3D volume pixel type used throughout xreg (float voxels).
using Vol3D = itk::Image<xreg::RayCastPixelScalar,3>;

/// \brief A 3D image/volume as it crosses the Python boundary.
///
/// Pixels are a C-contiguous float32 NumPy array indexed (z, y, x) to match the
/// ITK in-memory buffer ordering (x is the fastest varying index). The physical
/// metadata (spacing/origin/direction) is carried alongside so a round trip
/// through Python preserves the volume geometry.
struct Volume
{
  py::array_t<float> pixels;                       ///< shape (nz, ny, nx), float32
  Eigen::Vector3d    spacing   = Eigen::Vector3d::Ones();   ///< (sx, sy, sz)
  Eigen::Vector3d    origin    = Eigen::Vector3d::Zero();   ///< (ox, oy, oz)
  Eigen::Matrix3d    direction = Eigen::Matrix3d::Identity();

  /// Convenience: (nz, ny, nx)
  std::array<py::ssize_t,3> shape() const;
};

/// \brief A triangle mesh as it crosses the Python boundary.
///
/// Vertices are (N, 3) float32, faces are (M, 3) uint64 offsets into vertices.
struct Mesh
{
  py::array_t<float>            vertices;  ///< shape (N, 3), float32
  py::array_t<std::uint64_t>    faces;     ///< shape (M, 3), uint64
  py::array_t<float>            normals;   ///< shape (M, 3) or empty when not computed

  bool has_normals() const;
};

// ITK volume <-> Volume ------------------------------------------------------

/// \brief Copy an ITK float volume into a Volume (NumPy-backed).
Volume ItkVolToVolume(const Vol3D* img);

/// \brief Build an ITK float volume from a Volume (NumPy-backed).
Vol3D::Pointer VolumeToItkVol(const Volume& vol);

// Eigen transform helpers ----------------------------------------------------

/// \brief Convert a 4x4 (float) homogeneous matrix into an xreg FrameTransform.
xreg::FrameTransform Mat4x4ToFrameTransform(const xreg::Mat4x4& m);

// TriMesh <-> Mesh -----------------------------------------------------------

/// \brief Copy an xreg TriMesh into a Mesh (NumPy-backed).
Mesh TriMeshToMesh(const xreg::TriMesh& src);

/// \brief Build an xreg TriMesh from a Mesh (NumPy-backed).
xreg::TriMesh MeshToTriMesh(const Mesh& src);

// Registration of the shared value types (Volume, Mesh) on the module. Called
// once from the module entry point before the feature submodules are wired up.
void RegisterConvertTypes(py::module_& m);

}  // namespace xregpy

#endif
