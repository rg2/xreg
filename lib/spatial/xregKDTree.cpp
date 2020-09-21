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

#include "xregKDTree.h"

#include "xregSpatialPrimitives.h"
#include "xregMesh.h"
  
xreg::size_type xreg::KDTreeTri::size() const
{
  return centroid.size();
}

const xreg::CoordScalar& xreg::KDTreeTri::operator[](const size_type& dim) const
{
  return centroid[dim];
}

std::tuple<xreg::Pt3,xreg::CoordScalar> xreg::KDTreeTri::closest_point(const Pt3& x) const
{
  const Pt3 tri_pt = FindClosestPtInTri(x, *(verts[0]), *(verts[1]), *(verts[2]));

  return std::make_tuple(tri_pt, (x - tri_pt).norm());
}

std::tuple<xreg::CoordScalar,xreg::CoordScalar>
xreg::KDTreeTri::bounds(const size_type& dim) const
{
  const CoordScalar& x1 = (*(verts[0]))[dim];
  const CoordScalar& x2 = (*(verts[1]))[dim];
  const CoordScalar& x3 = (*(verts[2]))[dim];

  return std::make_tuple(std::min(std::min(x1, x2), x3), std::max(std::max(x1, x2), x3));
}

std::vector<xreg::KDTreeTri> xreg::CreateTrisForKDTree(const TriMesh& mesh)
{
  const size_type num_tris = mesh.faces.size();

  std::vector<KDTreeTri> kd_tris(num_tris);
  
  const auto centroids = mesh.face_centroids();
 
  for (size_type i = 0; i < num_tris; ++i)
  {
    auto& cur_kd_tri = kd_tris[i];
    
    cur_kd_tri.centroid = centroids[i];

    const auto& cur_face = mesh.faces[i];

    cur_kd_tri.verts[0] = &mesh.vertices[cur_face[0]];
    cur_kd_tri.verts[1] = &mesh.vertices[cur_face[1]];
    cur_kd_tri.verts[2] = &mesh.vertices[cur_face[2]];
  }

  return kd_tris;
}

