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

#include "xregMesh.h"

#include <queue>
#include <unordered_set>

#include "xregAssert.h"
#include "xregCompareUtils.h"
#include "xregPointCloudUtils.h"
#include "xregTBBUtils.h"
#include "xregSpatialPrimitives.h"

std::tuple<xreg::Pt3,xreg::Pt3> xreg::TriMesh::find_bounds() const
{
  auto vit = vertices.begin();

  Scalar cur_min_x = vit->operator[](0);
  Scalar cur_max_x = cur_min_x;
  Scalar cur_min_y = vit->operator[](1);
  Scalar cur_max_y = cur_min_y;
  Scalar cur_min_z = vit->operator[](2);
  Scalar cur_max_z = cur_min_z;

  ++vit;
  for (; vit != vertices.end(); ++vit)
  {
    const Vertex& v = *vit;

    if (v[0] < cur_min_x)
    {
      cur_min_x = v[0];
    }
    else if (v[0] > cur_max_x)
    {
      cur_max_x = v[0];
    }

    if (v[1] < cur_min_y)
    {
      cur_min_y = v[1];
    }
    else if (v[1] > cur_max_y)
    {
      cur_max_y = v[1];
    }

    if (v[2] < cur_min_z)
    {
      cur_min_z = v[2];
    }
    else if (v[2] > cur_max_z)
    {
      cur_max_z = v[2];
    }
  }

  Pt3 min_pt;
  min_pt[0] = cur_min_x;
  min_pt[1] = cur_min_y;
  min_pt[2] = cur_min_z;

  Pt3 max_pt;
  max_pt[0] = cur_max_x;
  max_pt[1] = cur_max_y;
  max_pt[2] = cur_max_z;

  return std::make_tuple(min_pt, max_pt);
}

void xreg::TriMesh::reverse_vertex_ordering()
{
  const size_type num_tris = faces.size();
  for (size_type i = 0; i < num_tris; ++i)
  {
    std::swap(faces[i][0], faces[i][2]);
  }
}

void xreg::TriMesh::flip_normals()
{
  if (normals_valid)
  {
    const size_type num_normals = normals.size();
    xregASSERT(num_normals == faces.size());

    for (size_type i = 0; i < num_normals; ++i)
    {
      normals[i][0] *= -1;
      normals[i][1] *= -1;
      normals[i][2] *= -1;
    }
  }
}

void xreg::TriMesh::compute_normals(const bool inward_facing)
{
  const size_type num_faces = faces.size();

  normals.resize(num_faces);

  for (size_type i = 0; i < num_faces; ++i)
  {
    const Vertex& v1 = vertices[faces[i][0]];
    const Vertex& v2 = vertices[faces[i][1]];
    const Vertex& v3 = vertices[faces[i][2]];

    normals[i] = (v2 - v1).cross(v3 - v1);
    normals[i].normalize();

    if (!inward_facing)
    {
      normals[i] *= -1;
    }
  }

  normals_valid = true;
}

void xreg::TriMesh::compute_vertex_normals(const bool inward_facing)
{
	compute_normals(inward_facing);
	const size_type num_vertices = vertices.size();

	vertex_normals.resize(num_vertices);
    
  IndexList f;

	for (size_type i = 0; i < num_vertices; ++i)
	{
		vertex_normals[i][0] = 0 ; vertex_normals[i][1] = 0 ; vertex_normals[i][2] = 0 ;
		
		find_faces(i, &f);
		
    for (size_type j = 0 ; j < f.size() ; j++)
		{
			vertex_normals[i] += normals[ f[j] ];
		}
		vertex_normals[i].normalize() ;
	}

	normals_valid = true;
}

void xreg::TriMesh::find_faces(const size_type vert_index, IndexList* face_inds)
{
  xregASSERT(vert_index < vertices.size());

  face_inds->clear();

  const size_type num_faces = faces.size();

  for (size_type face_idx = 0; face_idx < num_faces; ++face_idx)
  {
    if ((faces[face_idx][0] == vert_index) || (faces[face_idx][1] == vert_index) ||
        (faces[face_idx][2] == vert_index))
    {
      face_inds->push_back(face_idx);
    }
  }
}

void xreg::TriMesh::populate_face_indices_for_vertices()
{
  const size_type num_verts = vertices.size();
  const size_type num_faces = faces.size();

  vertex_face_inds.resize(num_verts);

  for (size_type face_idx = 0; face_idx < num_faces; ++face_idx)
  {
    vertex_face_inds[faces[face_idx][0]].push_back(face_idx);
    vertex_face_inds[faces[face_idx][1]].push_back(face_idx);
    vertex_face_inds[faces[face_idx][2]].push_back(face_idx);
  }
}

void xreg::TriMesh::compute_vertex_neighbors()
{
  compute_vertex_neighbors(&neighbors);
}

void xreg::TriMesh::compute_vertex_neighbors(VertexNeighborSetList* neighbors) const
{
  VertexNeighborSetList& vn = *neighbors;

  vn.clear();
  vn.resize(vertices.size());

  const size_type num_tris = faces.size();

  for (size_type i = 0; i < num_tris; ++i)
  {
    vn[faces[i][0]].insert(faces[i][1]);
    vn[faces[i][0]].insert(faces[i][2]);

    vn[faces[i][1]].insert(faces[i][0]);
    vn[faces[i][1]].insert(faces[i][2]);

    vn[faces[i][2]].insert(faces[i][0]);
    vn[faces[i][2]].insert(faces[i][1]);
  }
}

void xreg::TriMesh::find_boundary_verts(const VertexMask& orig_mask,
                                        VertexMask* boundary_mask_ptr) const
{
  xregASSERT(orig_mask.size() == vertices.size());

  const size_type num_faces = faces.size();

  VertexMask& bound_mask = *boundary_mask_ptr;

  // initialize to an empty boundary
  bound_mask.assign(vertices.size(), false);

  for (size_type i = 0; i < num_faces; ++i)
  {
    const Triangle& cur_tri = faces[i];

    // check each vertex, if a vertex is masked and at least one of its
    // neighbors is not masked, than it must be on the boundary

    if (!orig_mask[cur_tri[0]] && (orig_mask[cur_tri[1]] || orig_mask[cur_tri[2]]))
    {
      bound_mask[cur_tri[0]] = true;
    }

    if (!orig_mask[cur_tri[1]] && (orig_mask[cur_tri[0]] || orig_mask[cur_tri[2]]))
    {
      bound_mask[cur_tri[1]] = true;
    }

    if (!orig_mask[cur_tri[2]] && (orig_mask[cur_tri[0]] || orig_mask[cur_tri[1]]))
    {
      bound_mask[cur_tri[2]] = true;
    }
  }
}

void xreg::TriMesh::find_boundary_verts(const VertexMask& orig_mask,
                                        const VertexNeighborSetList& neighbors,
                                        VertexMask* boundary_mask_ptr) const
{
  const size_type num_verts = vertices.size();

  xregASSERT(neighbors.size() == num_verts);
  xregASSERT(orig_mask.size() == num_verts);

  VertexMask& bound_mask = *boundary_mask_ptr;

  // initialize to an empty boundary
  bound_mask.assign(num_verts, false);

  for (size_type cur_vert = 0; cur_vert < num_verts; ++cur_vert)
  {
    if (!orig_mask[cur_vert])
    {
      // the current vertex is masked out, check to see if any of its neighbors
      // are not

      const NeighborSet& cur_neighbors = neighbors[cur_vert];

      for (auto it = cur_neighbors.begin(); it != cur_neighbors.end(); ++it)
      {
        if (orig_mask[*it])
        {
          // a neighbor is not masked, therefore the current vertex is on the
          // boundary and we can stop looking at the other neighbors
          bound_mask[cur_vert] = true;
          break;
        }
      }
    }
  }
}

void xreg::TriMesh::find_verts_near_mask(const VertexMask& verts_mask,
                                         const size_type max_depth,
                                         VertexDistanceList* vert_dists_from_mask_ptr)
{
  using VertexQueue = std::queue<size_type>;

  const size_type num_verts = vertices.size();

  VertexDistanceList& vert_dists_from_mask = *vert_dists_from_mask_ptr;
  vert_dists_from_mask.assign(num_verts, kMAX_VERT_DIST);  // initialize to max distance

  if (max_depth == 0)  // trivial case
  {
    return;
  }

  VertexNeighborSetList neighbors;
  compute_vertex_neighbors(&neighbors);

  VertexMask boundary;
  find_boundary_verts(verts_mask, neighbors, &boundary);

  VertexQueue vert_queues[2];

  VertexQueue* cur_verts  = &vert_queues[0];
  VertexQueue* next_verts = &vert_queues[1];

  // build up the initial queue of vertices to start searching FROM
  // look through all of the vertices and start from the masked-out-boundary
  // vertices
  for (size_type vert = 0; vert < num_verts; ++vert)
  {
    if (boundary[vert])
    {
      vert_dists_from_mask[vert] = 0;
      cur_verts->push(vert);
    }
  }

  for (size_type cur_depth = 1; cur_depth <= max_depth; ++cur_depth)
  {
    while (!cur_verts->empty())
    {
      const size_type root_vert = cur_verts->front();

      const NeighborSet& root_neighbors = neighbors[root_vert];

      for (auto it = root_neighbors.begin(); it != root_neighbors.end(); ++it)
      {
        const size_type next_vert = *it;

        if ((vert_dists_from_mask[next_vert] == kMAX_VERT_DIST) && verts_mask[next_vert])
        {
          // the neighbor has not already been visited and is not masked-out
          vert_dists_from_mask[next_vert] = cur_depth;
          next_verts->push(next_vert);
        }
      }

      cur_verts->pop();
    }

    std::swap(cur_verts, next_verts);
  }
}

void xreg::TriMesh::find_verts_within_radius_local(const size_type start_vert_index,
                                                   const Scalar radius,
                                                   IndexList* verts)
{
  xregASSERT(start_vert_index < vertices.size());
  xregASSERT(neighbors.size() == vertices.size());

  const Vertex& root_vertex = vertices[start_vert_index];

  std::unordered_set<size_type> visited_verts;
  std::queue<size_type> traversal_queue;

  traversal_queue.push(start_vert_index);

  while (!traversal_queue.empty())
  {
    const size_type cur_index = traversal_queue.front();

    traversal_queue.pop();

    // check to make sure we have not visited this vertex already
    if (visited_verts.find(cur_index) == visited_verts.end())
    {
      // mark this vertex as visited
      visited_verts.insert(cur_index);

      const Vertex& cur_vertex = vertices[cur_index];

      if ((cur_vertex - root_vertex).norm() <= radius)
      {
        // this vertex is within the radius, add it to the traversal list and
        // add its neighbors to the traversal queue for inspection

        verts->push_back(cur_index);

        const NeighborSet& cur_neighbors = neighbors[cur_index];
        for (auto nit = cur_neighbors.begin(); nit != cur_neighbors.end(); ++nit)
        {
          traversal_queue.push(*nit);
        }
      }
    }
  }
}

xreg::size_type xreg::TriMesh::number_of_bytes_used() const
{
  return (sizeof(Scalar) * vertices.size() * 3) + (sizeof(size_type) * faces.size() * 3) +
         (normals_valid ? (sizeof(Scalar) * faces.size() * 3) : 0);
}

void xreg::TriMesh::remove_duplicate_faces()
{
  TriangleList new_faces;

  remove_duplicate_faces(&new_faces);

  faces.swap(new_faces);
}

void xreg::TriMesh::remove_duplicate_faces(TriangleList* dst) const
{
  using TriangleSet = std::unordered_set<Triangle,
                                         OrderIndepArrayHash<size_type,3>,
                                         OrderIndepArrayEqualTo<size_type,3>>;

  TriangleSet tri_set;

  const size_type num_src_tris = faces.size();

  dst->clear();

  std::pair<TriangleSet::iterator, bool> set_ret;

  for (size_type i = 0; i < num_src_tris; ++i)
  {
    set_ret = tri_set.insert(faces[i]);
    if (set_ret.second)
    {
      dst->push_back(faces[i]);
    }
  }
}

void xreg::TriMesh::remove_duplicate_vertices(VertexList* dst, IndexList* inds) const
{
  using VertexIndexMap = std::unordered_map<Vertex, size_type,
                                            FixedPtHash<Vertex>,
                                            PtEuclideanNormEqualTo<Vertex>>;

  VertexIndexMap vert_map;

  const size_type num_src_verts = vertices.size();

  if (inds)
  {
    // set every value to one past the last valid index to indicate that it has
    // not been mapped yet.
    inds->assign(num_src_verts, num_src_verts);
  }

  size_type dst_vert_index = 0;
  dst->clear();

  for (size_type i = 0; i < num_src_verts; ++i)
  {
    auto map_ret = vert_map.emplace(vertices[i], dst_vert_index);

    if (inds)
    {
      // Note: map.insert does not update the mapped value when the key already
      //       exists in the map
      inds->operator[](i) = map_ret.first->second;
    }

    // If we've actually encountered a new vertex:
    if (map_ret.second)
    {
      dst->push_back(vertices[i]);
      ++dst_vert_index;
    }
  }

  xregASSERT(dst->size() == dst_vert_index);
}

void xreg::TriMesh::remove_duplicate_vertices()
{
  VertexList new_verts;
  IndexList  ind_remap;

  remove_duplicate_vertices(&new_verts, &ind_remap);

  vertices.swap(new_verts);

  update_face_indices(ind_remap);
}

void xreg::TriMesh::update_face_indices(const IndexList& vertex_map)
{
  for (auto& f : faces)
  {
    f[0] = vertex_map[f[0]];
    f[1] = vertex_map[f[1]];
    f[2] = vertex_map[f[2]];
  }
}

void xreg::TriMesh::scale_vertices(const Scalar s)
{
  ScalePts(s, vertices, &vertices);
}

void xreg::TriMesh::find_vert_inds(const VertexList& other_verts, IndexList* vert_inds,
                                   const CoordScalar tol) const
{
  const size_type num_mesh_verts  = vertices.size();
  const size_type num_other_verts = other_verts.size();

  vert_inds->assign(num_other_verts, kNPOS);

  for (size_type other_vert_ind = 0; other_vert_ind < num_other_verts; ++other_vert_ind)
  {
    for (size_type mesh_vert_ind = 0; mesh_vert_ind < num_mesh_verts; ++mesh_vert_ind)
    {
      if ((vertices[mesh_vert_ind] - other_verts[other_vert_ind]).norm() < tol)
      {
        vert_inds->operator[](other_vert_ind) = mesh_vert_ind;
        break;
      }
    }
  }
}

void xreg::TriMesh::transform(const FrameTransform& xform)
{
  ApplyTransform(xform, vertices, &vertices);

  if (normals_valid)
  {
    FrameTransform no_trans_xform = xform;
    no_trans_xform.matrix()(0,3) = 0;
    no_trans_xform.matrix()(1,3) = 0;
    no_trans_xform.matrix()(2,3) = 0;

    ApplyTransform(xform, normals, &normals);
    ApplyTransform(xform, vertex_normals, &vertex_normals);

    // TODO: actually in this case, there could be more to do
    if (std::abs(std::abs(xform.matrix().determinant()) - 1) > 1.0e-6)
    {
      // Not a rotation or reflection, need to re-normalize normal vectors
      ComputeUnitVectors(normals, &normals);
      ComputeUnitVectors(vertex_normals, &vertex_normals);
    }
  }
}

xreg::Pt3List xreg::TriMesh::face_centroids() const
{
  Pt3List cents;

  cents.reserve(faces.size());

  for (const auto& cur_face : faces)
  {
    cents.push_back((vertices[cur_face[0]] + vertices[cur_face[1]] + vertices[cur_face[2]]) / 3);
  }

  return cents;
}

void xreg::TetraMesh::extract_surface_tris(TriangleList* tris) const
{
  using TriangleUseHistogram = std::unordered_map<Triangle, size_type,
                                                  OrderIndepArrayHash<size_type,3>,
                                                  OrderIndepArrayEqualTo<size_type,3>>;

  TriangleUseHistogram tri_hist;

  std::array<Triangle,4> tmp_tris;

  const size_type num_tetra = tetrahedra.size();

  for (size_type tetra_index = 0; tetra_index < num_tetra; ++tetra_index)
  {
    std::tie(tmp_tris[0], tmp_tris[1], tmp_tris[2], tmp_tris[3]) = tetra_to_tris(tetra_index);

    for (size_type tri_index = 0; tri_index < 4; ++tri_index)
    {
      auto ins_val = tri_hist.emplace(tmp_tris[tri_index], 1);

      if (!ins_val.second)
      {
        // an entry already exists, increment it.
        ++ins_val.first->second;
      }
    }
  }

  for (auto it = tri_hist.begin(); it != tri_hist.end(); ++it)
  {
    if (it->second == 1)
    {
      tris->push_back(it->first);
    }
  }
}

void xreg::TetraMesh::extract_all_tris(TriangleList* tris) const
{
  std::array<Triangle,4> tmp_tris;

  // putting all of the triangles in set, ensures we do not have duplicate
  // entries
  std::unordered_set<Triangle, OrderIndepArrayHash<size_type,3>, OrderIndepArrayEqualTo<size_type,3>> tris_set;

  const size_type num_tetra = tetrahedra.size();

  for (size_type tetra_index = 0; tetra_index < num_tetra; ++tetra_index)
  {
    std::tie(tmp_tris[0], tmp_tris[1], tmp_tris[2], tmp_tris[3]) = tetra_to_tris(tetra_index);

    tris_set.insert(tmp_tris[0]);
    tris_set.insert(tmp_tris[1]);
    tris_set.insert(tmp_tris[2]);
    tris_set.insert(tmp_tris[3]);
  }

  tris->assign(tris_set.begin(), tris_set.end());
}

std::tuple<xreg::TetraMesh::Triangle,xreg::TetraMesh::Triangle,xreg::TetraMesh::Triangle,xreg::TetraMesh::Triangle>
xreg::TetraMesh::tetra_to_tris(size_type tetra_index) const
{
  const Tetrahedron& tetra = tetrahedra[tetra_index];

  Triangle t1;
  Triangle t2;
  Triangle t3;
  Triangle t4;

  // Empirically, it looks like it the data from Gouthami is always clockwise,
  // but we have this code just to be safe.

  // First assume a clock-wise ordering

  t1[0] = tetra[0];
  t1[1] = tetra[1];
  t1[2] = tetra[2];

  t2[0] = tetra[0];
  t2[1] = tetra[2];
  t2[2] = tetra[3];

  t3[0] = tetra[0];
  t3[1] = tetra[3];
  t3[2] = tetra[1];

  t4[0] = tetra[1];
  t4[1] = tetra[3];
  t4[2] = tetra[2];

  const Vertex& v1 = vertices[tetra[0]];
  const Vertex& v2 = vertices[tetra[1]];
  const Vertex& v3 = vertices[tetra[2]];
  const Vertex& v4 = vertices[tetra[3]];

  const Vertex tri_cent = (v1 + v2 + v3) / 3;
  const Vertex tet_cent = (v1 + v2 + v3 + v4) / 4;

  // Compute the inner product of the (what we currently think is the
  // inward facing) normal vector and the vector passing through the first
  // triangle's centroid and the centroid of the tetrahedron.
  if (((v2 - v1).cross(v3 - v1)).dot(tet_cent - tri_cent) < 0)
  {
    // We mistakingly assumed a clock-wise ordering when it is actually CCW,
    // swap the orders of each triangle

    std::swap(t1[0],t1[2]);
    std::swap(t2[0],t2[2]);
    std::swap(t3[0],t3[2]);
    std::swap(t4[0],t4[2]);
  }
  
  return std::make_tuple(t1,t2,t3,t4);
}

void xreg::ComputeMeanMesh(const TriMeshList& meshes, TriMesh* mean)
{
  const size_type num_meshes = meshes.size();

  if (num_meshes > 0)
  {
    *mean = meshes[0];

    if (num_meshes > 1)
    {
      const size_type num_verts = meshes[0].vertices.size();

#ifndef XREG_DISABLE_ASSERTS
      // sanity check
      for (size_type mesh_idx = 1; mesh_idx < num_meshes; ++mesh_idx)
      {
        xregASSERT(meshes[mesh_idx].vertices.size() == num_verts);
      }
#endif
      
      auto& mean_verts = mean->vertices;
      
      auto mean_mesh_helper_fn = [&mean_verts,&meshes,num_meshes] (const RangeType& r)
      {
        for (size_type mesh_idx = 1; mesh_idx < num_meshes; ++mesh_idx)
        {
          const auto& cur_mesh = meshes[mesh_idx];

          for (size_type i = r.begin(); i != r.end(); ++i)
          {
            mean_verts[i] += cur_mesh.vertices[i];
          }
        }

        for (size_type i = r.begin(); i != r.end(); ++i)
        {
          mean_verts[i] /= num_meshes;
        }
      };

      ParallelFor(mean_mesh_helper_fn, RangeType(0, num_verts));
    }
  }
  else
  {
    mean->vertices.clear();
    mean->faces.clear();
  }
}

namespace
{

using namespace xreg;

template <class Itr>
void SubtractMeshesByMeshHelper(const TriMesh& m0, Itr mesh_begin, Itr mesh_end)
{
  const size_type num_verts = m0.vertices.size();

#ifndef XREG_DISABLE_ASSERTS
  // sanity check
  for (Itr mesh_it = mesh_begin; mesh_it != mesh_end; ++mesh_it)
  {
    xregASSERT(num_verts == mesh_it->vertices.size());
  }
#endif
  
  auto sub_fn = [&m0,mesh_begin,mesh_end] (const RangeType& r)
  {
    const auto& m0_verts = m0.vertices;

    for (Itr mesh_it = mesh_begin; mesh_it != mesh_end; ++mesh_it)
    {
      auto& cur_verts = mesh_it->vertices;

      for (size_type i = r.begin(); i != r.end(); ++i)
      {
        cur_verts[i] -= m0_verts[i];
      }
    }
  };
  
  ParallelFor(sub_fn, RangeType(0, num_verts));
}

template <class Itr>
void AddMeshesByMeshHelper(const TriMesh& m0, Itr mesh_begin, Itr mesh_end)
{
  const size_type num_verts = m0.vertices.size();

#ifndef XREG_DISABLE_ASSERTS
  // sanity check
  for (Itr mesh_it = mesh_begin; mesh_it != mesh_end; ++mesh_it)
  {
    xregASSERT(num_verts == mesh_it->vertices.size());
  }
#endif
  
  auto add_fn = [&m0,mesh_begin,mesh_end] (const RangeType& r)
  {
    const auto& m0_verts = m0.vertices;

    for (Itr mesh_it = mesh_begin; mesh_it != mesh_end; ++mesh_it)
    {
      auto& cur_verts = mesh_it->vertices;

      for (size_type i = r.begin(); i != r.end(); ++i)
      {
        cur_verts[i] += m0_verts[i];
      }
    }
  };
  
  ParallelFor(add_fn, RangeType(0, num_verts));
}

}  // un-named

void xreg::SubtractMeshesByMesh(const TriMesh& m0, TriMeshList* dst_meshes)
{
  SubtractMeshesByMeshHelper(m0, dst_meshes->begin(), dst_meshes->end());
}

void xreg::SubtractMeshesByMesh(const TriMesh& m0, TriMesh* dst_mesh)
{
  SubtractMeshesByMeshHelper(m0, dst_mesh, dst_mesh + 1);
}

void xreg::AddMeshesByMesh(const TriMesh& m0, TriMeshList* dst_meshes)
{
  AddMeshesByMeshHelper(m0, dst_meshes->begin(), dst_meshes->end());
}

void xreg::AddMeshesByMesh(const TriMesh& m0, TriMesh* dst_mesh)
{
  AddMeshesByMeshHelper(m0, dst_mesh, dst_mesh + 1);
}

void xreg::AdjustMeshToOriginCentroid(const TriMesh& src, TriMesh* dst)
{
  *dst = src;

  if (!src.vertices.empty())
  {
    auto mean_vert = src.vertices[0];

    const size_type num_verts = src.vertices.size();

    for (size_type i = 1; i < num_verts; ++i)
    {
      mean_vert += src.vertices[i];
    }

    mean_vert /= static_cast<CoordScalar>(num_verts);

    for (size_type i = 0; i < num_verts; ++i)
    {
      dst->vertices[i] -= mean_vert;
    }
  }
}

void xreg::AdjustMeshesToCentroidsAtOrigin(const TriMeshList& src_meshes, TriMeshList* dst_meshes)
{
  const size_type num_meshes = src_meshes.size();
  dst_meshes->resize(num_meshes);

  auto cent_adjust_fn = [&src_meshes,dst_meshes] (const RangeType& r)
  {
    for (size_type i = r.begin(); i != r.end(); ++i)
    {
      AdjustMeshToOriginCentroid(src_meshes[i], &dst_meshes->operator[](i));
    }
  };

  ParallelFor(cent_adjust_fn, RangeType(0, num_meshes));
}

double xreg::ComputeMeanOfVertexDistancesTopologicalMeshes(const TriMesh& m1, const TriMesh& m2)
{
  const size_type num_verts = m1.vertices.size();

  xregASSERT(num_verts == m2.vertices.size());

  auto acc_fn = [&m1,&m2] (const RangeType& r, const double& init_val)
  {
    double ret = init_val;

    for (size_type i = r.begin(); i != r.end(); ++i)
    {
      ret += (m1.vertices[i] - m2.vertices[i]).norm();
    }

    return ret;
  };

  return ParallelReduce(0.0, acc_fn, std::plus<double>(), RangeType(0,num_verts)) / num_verts;
}

void xreg::CreateVertexMaskFromRoi(const TriMesh& src_mesh, const Pt3& mins, const Pt3& maxs, TriMesh::VertexMask* mask)
{
  const size_type num_src_verts = src_mesh.vertices.size();

  mask->assign(num_src_verts, true);

  const CoordScalar min_x = mins[0];
  const CoordScalar min_y = mins[1];
  const CoordScalar min_z = mins[2];
  const CoordScalar max_x = maxs[0];
  const CoordScalar max_y = maxs[1];
  const CoordScalar max_z = maxs[2];

  for (size_type i = 0; i < num_src_verts; ++i)
  {
    if ((src_mesh.vertices[i][0] < min_x) || (src_mesh.vertices[i][0] > max_x) ||
        (src_mesh.vertices[i][1] < min_y) || (src_mesh.vertices[i][1] > max_y) ||
        (src_mesh.vertices[i][2] < min_z) || (src_mesh.vertices[i][2] > max_z))
    {
      mask->operator[](i) = false;
    }
  }
}

void xreg::RemoveMaskedTriangles(const TriMesh::TriangleList& orig_tris,
                                 const TriMesh::VertexMask& mask, TriMesh::TriangleList* dst_tris)
{
  dst_tris->clear();

  const size_type num_orig_faces = orig_tris.size();
  for (size_type i = 0; i < num_orig_faces; ++i)
  {
    if (mask[orig_tris[i][0]] && mask[orig_tris[i][1]] && mask[orig_tris[i][2]])
    {
      dst_tris->push_back(orig_tris[i]);
    }
  }
}

void xreg::RemoveMaskedTriangles(const TriMesh::VertexMask& mask, TriMesh::TriangleList* tris)
{
  const TriMesh::TriangleList orig_faces = *tris;
  RemoveMaskedTriangles(orig_faces, mask, tris);
}

void xreg::RemoveMaskedVertices(const TriMesh& src_mesh, const TriMesh::VertexMask& mask,
                                TriMesh* dst_mesh)
{
  RemoveMaskedTriangles(src_mesh.faces, mask, &dst_mesh->faces);

  const size_type num_verts = src_mesh.vertices.size();

  std::vector<size_type> vert_map(num_verts);

  dst_mesh->vertices.clear();
  size_type cur_num_verts = 0;

  for (size_type i = 0; i < num_verts; ++i)
  {
    if (mask[i])
    {
      dst_mesh->vertices.push_back(src_mesh.vertices[i]);
      vert_map[i] = cur_num_verts;
      ++cur_num_verts;
    }
  }

  dst_mesh->update_face_indices(vert_map);
}

void xreg::CropMeshToRoi(const TriMesh& src_mesh, const Pt3& mins, const Pt3& maxs,
                         TriMesh* dst_mesh, const bool rm_verts)
{
  const size_type num_src_verts = src_mesh.vertices.size();

  TriMesh::VertexMask verts_mask;

  CreateVertexMaskFromRoi(src_mesh, mins, maxs, &verts_mask);

  if (rm_verts)
  {
    dst_mesh->vertices.clear();

    for (size_type i = 0; i < num_src_verts; ++i)
    {
      if (verts_mask[i])
      {
        dst_mesh->vertices.push_back(src_mesh.vertices[i]);
      }
    }
  }
  else
  {
    dst_mesh->vertices = src_mesh.vertices;
  }

  RemoveMaskedTriangles(src_mesh.faces, verts_mask, &dst_mesh->faces);
}

void xreg::InvertVertexMask(const TriMesh::VertexMask& src_mask, TriMesh::VertexMask* dst_mask)
{
  const size_type num_verts = src_mask.size();

  dst_mask->resize(num_verts);

  for (size_type i = 0; i < num_verts; ++i)
  {
    dst_mask->operator[](i) = !src_mask[i];
  }
}

void xreg::RemoveUnusedVertices(const TriMesh& src, TriMesh* dst,
                                TriMesh::VertexReorderMap* rev_map)
{
  const size_type orig_num_verts = src.vertices.size();
  const size_type num_tris = src.faces.size();

  const size_type kNOT_MAPPED = ~size_type(0);

  std::vector<size_type> forward_map(orig_num_verts, kNOT_MAPPED);

  size_type dst_num_verts = 0;
  for (size_type tri_index = 0; tri_index < num_tris; ++tri_index)
  {
    for (size_type i = 0; i < 3; ++i)
    {
      if (forward_map[src.faces[tri_index][i]] == kNOT_MAPPED)
      {
        // This vertex index has not yet been mapped
        forward_map[src.faces[tri_index][i]] = dst_num_verts;
        ++dst_num_verts;
      }
    }
  }

  dst->vertices.resize(dst_num_verts);
  for (size_type vert_index = 0; vert_index < orig_num_verts; ++vert_index)
  {
    if (forward_map[vert_index] != kNOT_MAPPED)
    {
      dst->vertices[forward_map[vert_index]] = src.vertices[vert_index];
    }
  }

  dst->faces = src.faces;
  for (size_type tri_index = 0; tri_index < num_tris; ++tri_index)
  {
    dst->faces[tri_index][0] = forward_map[dst->faces[tri_index][0]];
    dst->faces[tri_index][1] = forward_map[dst->faces[tri_index][1]];
    dst->faces[tri_index][2] = forward_map[dst->faces[tri_index][2]];
  }

  if (rev_map)
  {
    rev_map->clear();
    rev_map->reserve(dst_num_verts);

    for (size_type new_vert_index = 0; new_vert_index < dst_num_verts; ++new_vert_index)
    {
      for (size_type orig_vert_index = 0; orig_vert_index < orig_num_verts; ++orig_vert_index)
      {
        if (forward_map[orig_vert_index] == new_vert_index)
        {
          rev_map->push_back(orig_vert_index);
          break;
        }
      }

      xregASSERT(rev_map->size() == (new_vert_index + 1));
    }
  }
}

std::tuple<xreg::Pt3,xreg::CoordScalar>
xreg::FindClosestPtToMeshExhaustive(const Pt3& x, const TriMesh& mesh)
{
  const auto& verts = mesh.vertices;
  const auto& tris  = mesh.faces;

  const size_type num_tris = tris.size();

  xregASSERT(num_tris > 0);

  Pt3 tmp_pt1;
  Pt3 tmp_pt2;
  Pt3 cur_pt;
  CoordScalar cur_dist;
  CoordScalar tmp_dist;

  cur_pt = FindClosestPtInTri(x, verts[tris[0][0]], verts[tris[0][1]], verts[tris[0][2]]);

  tmp_pt2  = cur_pt - x;
  cur_dist = tmp_pt2.squaredNorm();

  for (size_type i = 1; i < num_tris; ++i)
  {
    tmp_pt1  = FindClosestPtInTri(x, verts[tris[i][0]], verts[tris[i][1]], verts[tris[i][2]]);
    tmp_pt2  = tmp_pt1 - x;
    tmp_dist = tmp_pt2.squaredNorm();

    if (tmp_dist < cur_dist)
    {
      cur_dist = tmp_dist;
      cur_pt   = tmp_pt1;
    }
  }

  return std::make_tuple(cur_pt, std::sqrt(cur_dist));
}

