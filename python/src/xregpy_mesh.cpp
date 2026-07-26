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

#include "xregMeshIO.h"

// Bindings for surface mesh file I/O. Meshes cross the boundary as an xreg.Mesh
// (NumPy vertices/faces). Backs show_mesh / create_mesh output handling.
namespace xregpy
{

void RegisterMesh(py::module_& parent)
{
  auto m = parent.def_submodule("mesh", "Surface mesh file I/O.");

  m.def("read_mesh",
        [](const std::string& path)
        {
          return TriMeshToMesh(xreg::ReadMeshFromDisk(path));
        },
        py::arg("path"),
        "Read a surface mesh from disk into an xreg.Mesh.");

  m.def("write_mesh",
        [](const Mesh& mesh, const std::string& path, const bool prefer_ascii)
        {
          xreg::WriteMeshToDisk(MeshToTriMesh(mesh), path, prefer_ascii);
        },
        py::arg("mesh"), py::arg("path"), py::arg("prefer_ascii") = false,
        "Write an xreg.Mesh to disk; format is chosen by the file extension.");
}

}  // namespace xregpy
