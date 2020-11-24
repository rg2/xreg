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

#include "xregH5MeshIO.h"

#include "xregHDF5.h"

xreg::TriMesh xreg::ReadMeshH5(const H5::Group& h5)
{
  TriMesh mesh;

  mesh.vertices = ReadListOfPointsFromMatrixH5Pt3("vertices", h5);
  
  mesh.faces = ReadListOfArraysFromMatrixH5Sizes3("faces", h5);

  if (ObjectInGroupH5("face-normals", h5))
  {
    mesh.normals = ReadListOfPointsFromMatrixH5Pt3("face-normals", h5);
    mesh.normals_valid = true;
  }

  return mesh;
}

xreg::TriMesh xreg::ReadMeshH5File(const std::string& path)
{
  return ReadMeshH5(H5::H5File(path, H5F_ACC_RDONLY));
}

void xreg::WriteMeshH5(const TriMesh& mesh, H5::Group* h5, const bool compress)
{
  WriteListOfPointsAsMatrixH5("vertices", mesh.vertices, h5, compress);
  WriteListOfArraysToMatrixH5("faces", mesh.faces, h5, compress);
  
  if (mesh.normals_valid)
  {
    WriteListOfPointsAsMatrixH5("face-normals", mesh.normals, h5, compress);
  }
}

void xreg::WriteMeshH5File(const TriMesh& mesh, const std::string& path, const bool compress)
{
  H5::H5File h5(path, H5F_ACC_TRUNC);

  WriteMeshH5(mesh, &h5, compress);

  h5.flush(H5F_SCOPE_GLOBAL);
  h5.close();
}

