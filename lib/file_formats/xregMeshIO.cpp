/*
 * MIT License
 *
 * Copyright (c) 2020 Robert Grupp
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

#include "xregMeshIO.h"

#include "xregFilesystemUtils.h"
#include "xregStringUtils.h"
#include "xregSTLMeshIO.h"
#include "xregH5MeshIO.h"
#include "xregVTKMeshUtils.h"

xreg::TriMesh xreg::ReadMeshFromDisk(const std::string& path)
{
  const std::string file_ext = ToLowerCase(Path(path).file_extension());

  TriMesh mesh;

  if (file_ext == ".stl")
  {
    mesh = ReadSTLMesh(path);
  }
  else if ((file_ext == ".h5") || (file_ext == ".hdf5"))
  {
    mesh = ReadMeshH5File(path);
  }
  else if (file_ext == ".ply")
  {
    mesh = ReadPLYMesh(path);
  }
  else if (file_ext == ".vtk")
  {
    mesh = ReadVTKMesh(path);
  }
  else
  {
    xregThrow("Unsupported file extension/format: %s", file_ext.c_str());
  }

  return mesh;
}

void xreg::WriteMeshToDisk(const TriMesh& mesh, const std::string& path, const bool prefer_ascii)
{
  const std::string file_ext = ToLowerCase(Path(path).file_extension());

  if (file_ext == ".stl")
  {
    WriteSTLMesh(path, mesh, prefer_ascii);
  }
  else if ((file_ext == ".h5") || (file_ext == ".hdf5"))
  {
    WriteMeshH5File(mesh, path);
  }
  else if (file_ext == ".ply")
  {
    WritePLYMesh(mesh, path, prefer_ascii);
  }
  else if (file_ext == ".vtk")
  {
    WriteVTKMesh(mesh, path, prefer_ascii);
  }
  else
  {
    xregThrow("Unsupported file extension/format: %s", file_ext.c_str());
  }
}

