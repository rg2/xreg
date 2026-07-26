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
#include <stdexcept>

namespace xregpy
{

std::array<py::ssize_t,3> Volume::shape() const
{
  const auto info = pixels.request();
  if (info.ndim != 3)
  {
    throw std::runtime_error("Volume.pixels must be a 3D array");
  }
  return { info.shape[0], info.shape[1], info.shape[2] };
}

bool Mesh::has_normals() const
{
  return normals.size() > 0;
}

Volume ItkVolToVolume(const Vol3D* img)
{
  const auto region = img->GetLargestPossibleRegion();
  const auto size   = region.GetSize();  // size[0]=x, size[1]=y, size[2]=z

  const py::ssize_t nx = static_cast<py::ssize_t>(size[0]);
  const py::ssize_t ny = static_cast<py::ssize_t>(size[1]);
  const py::ssize_t nz = static_cast<py::ssize_t>(size[2]);

  Volume vol;

  // NumPy array indexed (z, y, x) which matches the ITK buffer's linear order
  // (x fastest), so the copy below is a single contiguous memcpy.
  vol.pixels = py::array_t<float>({ nz, ny, nx });

  const std::size_t num_vox = static_cast<std::size_t>(nx) *
                              static_cast<std::size_t>(ny) *
                              static_cast<std::size_t>(nz);

  std::memcpy(vol.pixels.mutable_data(), img->GetBufferPointer(),
              num_vox * sizeof(float));

  const auto sp  = img->GetSpacing();
  const auto org = img->GetOrigin();
  const auto dir = img->GetDirection();

  for (int i = 0; i < 3; ++i)
  {
    vol.spacing(i) = sp[i];
    vol.origin(i)  = org[i];
    for (int j = 0; j < 3; ++j)
    {
      vol.direction(i,j) = dir(i,j);
    }
  }

  return vol;
}

Vol3D::Pointer VolumeToItkVol(const Volume& vol)
{
  const auto info = vol.pixels.request();
  if (info.ndim != 3)
  {
    throw std::runtime_error("Volume.pixels must be a 3D array of shape (nz, ny, nx)");
  }

  // Ensure a C-contiguous float32 buffer we can memcpy verbatim.
  py::array_t<float, py::array::c_style | py::array::forcecast> pix(vol.pixels);
  const auto pinfo = pix.request();

  const std::size_t nz = static_cast<std::size_t>(pinfo.shape[0]);
  const std::size_t ny = static_cast<std::size_t>(pinfo.shape[1]);
  const std::size_t nx = static_cast<std::size_t>(pinfo.shape[2]);

  auto img = Vol3D::New();

  Vol3D::RegionType region;
  Vol3D::SizeType   size;
  Vol3D::IndexType  start;
  start.Fill(0);
  size[0] = nx;
  size[1] = ny;
  size[2] = nz;
  region.SetSize(size);
  region.SetIndex(start);

  img->SetRegions(region);
  img->Allocate();

  Vol3D::SpacingType   sp;
  Vol3D::PointType     org;
  Vol3D::DirectionType dir;
  for (int i = 0; i < 3; ++i)
  {
    sp[i]  = vol.spacing(i);
    org[i] = vol.origin(i);
    for (int j = 0; j < 3; ++j)
    {
      dir(i,j) = vol.direction(i,j);
    }
  }
  img->SetSpacing(sp);
  img->SetOrigin(org);
  img->SetDirection(dir);

  std::memcpy(img->GetBufferPointer(), pinfo.ptr,
              nx * ny * nz * sizeof(float));

  return img;
}

xreg::FrameTransform Mat4x4ToFrameTransform(const xreg::Mat4x4& m)
{
  xreg::FrameTransform xf;
  xf.matrix() = m;
  return xf;
}

Mesh TriMeshToMesh(const xreg::TriMesh& src)
{
  Mesh dst;

  const py::ssize_t num_verts = static_cast<py::ssize_t>(src.vertices.size());
  const py::ssize_t num_faces = static_cast<py::ssize_t>(src.faces.size());

  dst.vertices = py::array_t<float>({ num_verts, py::ssize_t(3) });
  {
    auto v = dst.vertices.mutable_unchecked<2>();
    for (py::ssize_t i = 0; i < num_verts; ++i)
    {
      v(i,0) = src.vertices[i](0);
      v(i,1) = src.vertices[i](1);
      v(i,2) = src.vertices[i](2);
    }
  }

  dst.faces = py::array_t<std::uint64_t>({ num_faces, py::ssize_t(3) });
  {
    auto f = dst.faces.mutable_unchecked<2>();
    for (py::ssize_t i = 0; i < num_faces; ++i)
    {
      f(i,0) = static_cast<std::uint64_t>(src.faces[i][0]);
      f(i,1) = static_cast<std::uint64_t>(src.faces[i][1]);
      f(i,2) = static_cast<std::uint64_t>(src.faces[i][2]);
    }
  }

  if (src.normals_valid && (src.normals.size() == src.faces.size()))
  {
    dst.normals = py::array_t<float>({ num_faces, py::ssize_t(3) });
    auto n = dst.normals.mutable_unchecked<2>();
    for (py::ssize_t i = 0; i < num_faces; ++i)
    {
      n(i,0) = src.normals[i](0);
      n(i,1) = src.normals[i](1);
      n(i,2) = src.normals[i](2);
    }
  }

  return dst;
}

xreg::TriMesh MeshToTriMesh(const Mesh& src)
{
  xreg::TriMesh dst;

  py::array_t<float, py::array::c_style | py::array::forcecast> verts(src.vertices);
  const auto vinfo = verts.request();
  if ((vinfo.ndim != 2) || (vinfo.shape[1] != 3))
  {
    throw std::runtime_error("Mesh.vertices must have shape (N, 3)");
  }

  const py::ssize_t num_verts = vinfo.shape[0];
  dst.vertices.resize(num_verts);
  {
    auto v = verts.unchecked<2>();
    for (py::ssize_t i = 0; i < num_verts; ++i)
    {
      dst.vertices[i] = xreg::Pt3(v(i,0), v(i,1), v(i,2));
    }
  }

  py::array_t<std::uint64_t, py::array::c_style | py::array::forcecast> faces(src.faces);
  const auto finfo = faces.request();
  if ((finfo.ndim != 2) || (finfo.shape[1] != 3))
  {
    throw std::runtime_error("Mesh.faces must have shape (M, 3)");
  }

  const py::ssize_t num_faces = finfo.shape[0];
  dst.faces.resize(num_faces);
  {
    auto f = faces.unchecked<2>();
    for (py::ssize_t i = 0; i < num_faces; ++i)
    {
      dst.faces[i] = { static_cast<xreg::size_type>(f(i,0)),
                       static_cast<xreg::size_type>(f(i,1)),
                       static_cast<xreg::size_type>(f(i,2)) };
    }
  }

  return dst;
}

void RegisterConvertTypes(py::module_& m)
{
  py::class_<Volume>(m, "Volume",
      "A 3D image/volume: float32 pixels indexed (z, y, x) with physical "
      "spacing/origin/direction metadata.")
    .def(py::init<>())
    .def_readwrite("pixels",    &Volume::pixels,    "(nz, ny, nx) float32 array")
    .def_readwrite("spacing",   &Volume::spacing,   "(sx, sy, sz)")
    .def_readwrite("origin",    &Volume::origin,    "(ox, oy, oz)")
    .def_readwrite("direction", &Volume::direction, "3x3 direction cosine matrix")
    .def_property_readonly("shape", &Volume::shape, "(nz, ny, nx)")
    .def("__repr__", [](const Volume& v)
       {
         const auto s = v.shape();
         return "<xreg.Volume shape=(" + std::to_string(s[0]) + ", " +
                std::to_string(s[1]) + ", " + std::to_string(s[2]) + ")>";
       });

  py::class_<Mesh>(m, "Mesh",
      "A triangle mesh: (N, 3) float32 vertices and (M, 3) uint64 faces.")
    .def(py::init<>())
    .def_readwrite("vertices", &Mesh::vertices, "(N, 3) float32 array")
    .def_readwrite("faces",    &Mesh::faces,    "(M, 3) uint64 array")
    .def_readwrite("normals",  &Mesh::normals,  "(M, 3) float32 per-face normals or empty")
    .def_property_readonly("has_normals", &Mesh::has_normals)
    .def("__repr__", [](const Mesh& m)
       {
         const auto vi = m.vertices.request();
         const auto fi = m.faces.request();
         const auto nv = (vi.ndim == 2) ? vi.shape[0] : 0;
         const auto nf = (fi.ndim == 2) ? fi.shape[0] : 0;
         return "<xreg.Mesh verts=" + std::to_string(nv) +
                " faces=" + std::to_string(nf) + ">";
       });
}

}  // namespace xregpy
