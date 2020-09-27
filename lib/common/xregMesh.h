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

/**
 * @file
 * @brief Utilities and structures for mesh processing.
 **/

#ifndef XREGMESH_H_
#define XREGMESH_H_

#include <array>

#include <boost/container/flat_set.hpp>

#include "xregCommon.h"

namespace xreg
{

/**
 * @brief (Simple) Triangular mesh.
 *
 * Defined to be similar to Matlab's struct return from isosurface and patch.
 **/
struct TriMesh
{
  using Scalar       = CoordScalar;  ///< Scalar type, element type of each vertex
  using Vertex       = Pt3;          ///< 3D vertex type
  using VertexList   = Pt3List;      ///< Type for storing the list of vertices
  using VertexMask   = MaskVec;
  using Triangle     = std::array<size_type,3>;  ///< Triangle type, stores offsets into the VertexList
  using TriangleList = std::vector<Triangle>;    ///< Type for storing the list of triangles

  using NeighborSet           = boost::container::flat_set<size_type>;  ///< Type for storing neighboring vertex indices to a specific vertex
  using VertexNeighborSetList = std::vector<NeighborSet>;  ///< Type for storing the neighboring vertex index sets for every vertex
  using VertexDistanceList    = std::vector<size_type>;  ///< Type for storing the number of edge traversels from a boundary to each vertex
  
  using IndexList        = std::vector<size_type>;  ///< Type for storing a list of VertexList or TriangleList indices
  using ListOfIndexLists = std::vector<IndexList>;  ///< Type for storing the IndexList of triangle faces for every vertex

  /**
   * Type for storing a mapping from a new mesh's vertex list indices into
   * another mesh's vertex list indices; this is useful for copying vertices
   * from an old mesh into a new mesh with some reduction and/or reordering.
   **/
  using VertexReorderMap = std::vector<size_type>;

  /// Indicates that a vertex distance has not been computed or that it is not
  /// going to be reached (all ones in binary)
  enum { kMAX_VERT_DIST = ~size_type(0) };

  /// Indicates that an index is out of range, or used to indicate that an
  /// operation failed. Analogous to the npos field used in the STL.
  enum { kNPOS = ~size_type(0) };

  VertexList vertices;  ///< The list of vertices in this mesh
  TriangleList faces;   ///< The list of triangle faces in this mesh

  bool normals_valid = false;  ///< Indicates if normals stores valid surface normals
  VertexList normals;          ///< Defined for each triangle face and pointing to the "interior of the mesh"
  VertexList vertex_normals;   ///< Defined for each vertex and pointing to the "interior of the mesh'

  /**
   * @brief Defined for each vertex and stores the neighboring vertices
   *
   * Must be populated via a call to compute_vertex_neighbors
   **/
  VertexNeighborSetList neighbors;

  /**
   * @brief A list of triangle face vertices for each vertex.
   *
   * Must be populated via a call to populate_face_indices_for_vertices
   **/
  ListOfIndexLists vertex_face_inds;

  /**
   * @brief Compute a bounding box about this mesh.
   * @param min_x The minimum x value
   * @param max_x The maximum x value
   * @param min_y The minimum y value
   * @param max_y The maximum y value
   * @param min_z The minimum z value
   * @param max_z The maximum z value
   **/
  std::tuple<Pt3,Pt3> find_bounds() const;

  /**
   * @brief Reverse the vertex ordering at each triangle face.
   *
   * E.g. switch from counter-clockwise to clockwise, or vice-versa.
   **/
  void reverse_vertex_ordering();

  /**
   * @brief Flip the direction of surface normals at each face.
   **/
  void flip_normals();

  /**
   * @brief Compute normals vectors for each triangle face.
   *
   * This assumes a clockwise ordering of vertex indices as seen from the
   * outside of the mesh.
   * @param inward_facing (optional) Indicates that the normals should be
   *                      computed to point to the interior of the surface;
   *                      defaults to true; when false the normals point to the
   *                      exterior of the surface
   **/
  void compute_normals(const bool inward_facing = true);

  /**
   * @brief Compute normals vectors for each vertex.
   *
   * This assumes a clockwise ordering of vertex indices as seen from the
   * outside of the mesh.
   * @param inward_facing (optional) Indicates that the normals should be
   *                      computed to point to the interior of the surface;
   *                      defaults to false; when false the normals point to the
   *                      exterior of the surface
   **/
  void compute_vertex_normals(const bool inward_facing = true);
  
  /**
   * @brief Finds the triangle faces that reference a vertex.
   *
   * @param vert_index The vertex index that should be queried
   * @param face_inds The list of triangle faces that contain this vertex
   **/
  void find_faces(const size_type vert_index, IndexList* face_inds);

  /**
   * @brief For every vertex find the set of triangles that reference it.
   *
   * The result is stored in the instance variable: vertex_face_inds.
   **/
   void populate_face_indices_for_vertices();

  /**
   * @brief Populates sets of neighboring vertices (edge connected) for each
   *        vertex in the mesh.
   *
   * This stores the neighbor list in a data structure internal to this
   * object instance (the neighbors instance variable).
   **/
  void compute_vertex_neighbors();

  /**
   * @brief Populates sets of neighboring vertices (edge connected) for each
   *        vertex in the mesh.
   *
   * This stores the neighbor list in a data structure external to this
   * object instance (e.g. user provided).
   * @param neighbors The output list of sets of neighbor vertex indices
   **/
  void compute_vertex_neighbors(VertexNeighborSetList* neighbors) const;

  /**
   * @brief Given a mask, finds the vertices on the boundary of the mask.
   *
   * The boundary vertices are vertices that have been "masked out" (e.g. set
   * to false in the original vertex mask) and have at least one neighbor that
   * is not masked out (e.g. set to true in the original vertex mask). This
   * implementation searches through each triangle in the mesh.
   * @param orig_mask The vertex mask
   * @param boundary_mask_ptr The boundary mask, a vertex index will be set to
   *                          true when that vertex is on the boundary
   **/
  void find_boundary_verts(const VertexMask& orig_mask,
                           VertexMask* boundary_mask_ptr) const;

  /**
   * @brief Given a mask, finds the vertices on the boundary of the mask.
   *
   * The boundary vertices are vertices that have been "masked out" (e.g. set
   * to false in the original vertex mask) and have at least one neighbor that
   * is not masked out (e.g. set to true in the original vertex mask). This
   * implementation uses a previously computed set of neighbors for each vertex
   * instead of searching every triangle.
   * @param orig_mask The vertex mask
   * @param neighbors The set of neighbors for each vertex
   * @param boundary_mask_ptr The boundary mask, a vertex index will be set to
   *                          true when that vertex is on the boundary
   **/
  void find_boundary_verts(const VertexMask& orig_mask,
                           const VertexNeighborSetList& neighbors,
                           VertexMask* boundary_mask_ptr) const;

  /**
   * @brief Given a mask and a maximum depth/range, find all non-masked-out
   *        vertices within range of the boundary.
   *
   * This is accomplished by performing a Breadth First Search starting at the
   * boundary vertices.
   * @param verts_mask The original vertex mask
   * @param max_depth The maximum depth (in terms of edges traversed) from the
   *                  masked-out-vertex boundary
   * @param vert_dists_from_mask_ptr The distances of each non-masked-out-vertex
   *                                 from the boundary (ranging from 0 to max_depth)
   **/
  void find_verts_near_mask(const VertexMask& verts_mask,
                            const size_type max_depth,
                            VertexDistanceList* vert_dists_from_mask_ptr);

  /**
   * @brief Given a vertex and radius, find the vertices within that radius.
   *
   * This is an approximation; edges are traversed starting from a root vertex
   * and traversal along a path is terminated when a vertex outsided the
   * specified radius is encountered. The neighbors instance variable must
   * be valid prior to this call.
   * @param start_vert_index The root vertex from which traversals start and
   *                         distance is measured
   * @param radius The maximum allowable radius to visit a vertex
   * @param verts The output list of traversed vertices
   **/
  void find_verts_within_radius_local(const size_type start_vert_index,
                                      const Scalar radius,
                                      IndexList* verts);

   /**
    * @brief The number of bytes used for storing the vertices, faces, and
    *        normals.
    *
    * @return The number of bytes used
    **/
   size_type number_of_bytes_used() const;

   /**
    * @brief Removes duplicate faces.
    **/
   void remove_duplicate_faces();

   /**
    * @brief Finds duplicate faces and stores the list without duplicates in a
    *        user provided list.
    * @param dst The user provided list of triangles to populate; it will be
    *            cleared before population.
    **/
   void remove_duplicate_faces(TriangleList* dst) const;

   /**
    * @brief Finds duplicate vertices and populates a user specified list
    *        without the duplicates.
    *
    * A mapping from original vertex indices to new (non-duplicate) vertex
    * indices is populated if the user provides an IndexList.
    * @param dst The user provided list of vertices to populate; it will be
    *            cleared before population.
    * @param inds (optional) The user provided index mapping from the original
    *             vertex indices to new vertex indices in dst
    **/
   void remove_duplicate_vertices(VertexList* dst, IndexList* inds = nullptr) const;

   /**
    * @brief Finds duplicate vertices and removes them from this mesh; face
    *        indices are updated to reflect any reordering.
    **/
   void remove_duplicate_vertices();

   /**
    * @brief Updates face indices, given a remapping.
    *
    * @param vertex_map The remapping structure; vertex_map[i] = j implies that
    *                   any faces that refer to vertex i should update to refer
    *                   to index j
    **/
   void update_face_indices(const IndexList& vertex_map);

  /**
   * @brief Scales each vertex by a scale factor.
   *
   * @param s The scale factor
   **/
  void scale_vertices(const Scalar s);

  /// \brief Given a list of vertices/points, find the corresponding indices in
  ///        this mesh and populate an index mapping.
  ///
  /// If no corresponding vertex is found in this mesh, then the index value
  /// stored will be kNPOS.
  /// This performs a brute force search.
  /// \param other_verts List of vertices to search this mesh for
  /// \param face_inds List of vertex indices into this mesh
  /// \param tol (optional) Tolerance used for matching vertices (defaults to 1.0e-4)
  void find_vert_inds(const VertexList& other_verts, IndexList* vert_inds,
                      const CoordScalar tol = 1.0e-4) const;

  void transform(const FrameTransform& xform);
  
  Pt3List face_centroids() const;

};

using TriMeshList = std::vector<TriMesh>;

/**
 * @brief Tetrahedral mesh
 **/
struct TetraMesh
{
  using Scalar       = CoordScalar;
  using Vertex       = Pt3;
  using VertexList   = std::vector<Vertex>;
  using Tetrahedron  = std::array<size_type,4>;
  using TetraList    = std::vector<Tetrahedron>;
  using Triangle     = TriMesh::Triangle;
  using TriangleList = TriMesh::TriangleList;

  VertexList vertices;
  TetraList tetrahedra;

  void extract_surface_tris(TriangleList* tris) const;

  void extract_all_tris(TriangleList* tris) const;

  std::tuple<Triangle,Triangle,Triangle,Triangle> tetra_to_tris(size_type tetra_index) const;
};


/**
 * @brief Compute a mean mesh from a collection of meshes.
 * This assumes the meshes all have corresponding vertices, so that the average
 * (mean) value of the vertices are used.
 *
 * @param mesh_begin Start iterator to the collection of meshes
 * @param mesh_end End iterator to the collection of meshes
 * @param mean The mean mesh calculated
 **/
void ComputeMeanMesh(const TriMeshList& meshes, TriMesh* mean);

/**
 * @brief Subtract a mesh from each mesh in a collection.
 * This assumes the meshes all have corresponding vertices, so that the vertices
 * are just subtracted.
 *
 * @param m0 The mesh subtracted from the collection of meshes
 * @param mesh_begin Start iterator to the collection of meshes
 * @param mesh_end End iterator to the collection of meshes
 **/
void SubtractMeshesByMesh(const TriMesh& m0, TriMeshList* dst_meshes);

void SubtractMeshesByMesh(const TriMesh& m0, TriMesh* dst_mesh);

/**
 * @brief Add a mesh to each mesh in a collection.
 * This assumes the meshes all have corresponding vertices, so that the vertices
 * are just added.
 *
 * @param m0 The mesh added to the collection of meshes
 * @param mesh_begin Start iterator to the collection of meshes
 * @param mesh_end End iterator to the collection of meshes
 **/
void AddMeshesByMesh(const TriMesh& m0, TriMeshList* dst_meshes);

void AddMeshesByMesh(const TriMesh& m0, TriMesh* dst_mesh);

/**
 * @brief Moves a mesh to have centroid at the origin.
 *
 * @param src The source mesh, whose centroid is to be adjusted
 * @param dst The destination mesh, may point to the input.
 **/
void AdjustMeshToOriginCentroid(const TriMesh& src, TriMesh* dst);

/**
 * @brief Moves a collection of meshes to have centroids at the origin.
 *
 * This routine is threaded.
 * @param src_meshes The input collection of meshes
 * @param dst_meshes The output collection of meshes, may be equal to src_meshes
 **/
void AdjustMeshesToCentroidsAtOrigin(const TriMeshList& src_meshes, TriMeshList* dst_meshes);

/**
 * @brief compute the mean vertex differences between two meshes.
 *
 * This routine is threaded.
 * @param m1 Mesh 1
 * @param m2 Mesh 2
 * @return The average distance between the vertices of two meshes.
 **/
double ComputeMeanOfVertexDistancesTopologicalMeshes(const TriMesh& m1, const TriMesh& m2);

/// \brief Creates a vertex mask defined by a 3D rectangular ROI
///
/// Vertices lying inside, or on the boundary of, the ROI will have mask values set to true.
/// All other vertices will have mask values set to false.
/// \param src_mesh The mesh with vertices the mask will be defined over
/// \param mask The output, computed mask
/// \param min_x The minimum x coordinate of the rectangular ROI
/// \param max_x The maximum x coordinate of the rectangular ROI
/// \param min_y The minimum y coordinate of the rectangular ROI
/// \param max_y The maximum y coordinate of the rectangular ROI
/// \param min_z The minimum z coordinate of the rectangular ROI
/// \param max_z The maximum z coordinate of the rectangular ROI
void CreateVertexMaskFromRoi(const TriMesh& src_mesh, const Pt3& mins, const Pt3& maxs, TriMesh::VertexMask* mask);

/// \brief Copies a list of triangles, defined by vertex indices, with the exception
///        of any triangle that has at least one vertex which is masked out (the mask
///        is set to false)
///
/// \param orig_tris The source list of triangles
/// \param mask The vertex mask used to determine if a triangle should be copied to output
/// \param dst_tris The output list of triangles
void RemoveMaskedTriangles(const TriMesh::TriangleList& orig_tris,
                           const TriMesh::VertexMask& mask, TriMesh::TriangleList* dst_tris);

/// \brief Removes any triangle from a list of triangles when at least one vertex
///        is masked out (mask set to false)
///
/// This preserves relative order.
/// \param mask The vertex mask used to determine if a triangle should be kept or removed
/// \param tris The list of triangles to modify
void RemoveMaskedTriangles(const TriMesh::VertexMask& mask, TriMesh::TriangleList* tris);

/**
 * @brief Removes masked-out vertices and triangles containing masked-out vertices.
 *
 * @param src_mesh The original mesh
 * @param mask The vertex mask, indices set to false indicate masked-out
 * @param dst_mesh The output mesh
 **/
void RemoveMaskedVertices(const TriMesh& src_mesh, const TriMesh::VertexMask& mask,
                          TriMesh* dst_mesh);

/// \brief Copies the triangles from a mesh that fall on the boundary or lie inside
///        of a rectangular ROI.
///
/// Any triangle that has at least one vertex outside of the specified ROI will
/// not be copied. Optionally, any vertex that lies outside of the ROI may also
/// not be copied to the output mesh - this effects the ordering of the vertex
/// list.
/// \param src_mesh The input source mesh that will be selectively copied
/// \param dst_mesh The output mesh
/// \param min_x The minimum x coordinate of the rectangular ROI
/// \param max_x The maximum x coordinate of the rectangular ROI
/// \param min_y The minimum y coordinate of the rectangular ROI
/// \param max_y The maximum y coordinate of the rectangular ROI
/// \param min_z The minimum z coordinate of the rectangular ROI
/// \param max_z The maximum z coordinate of the rectangular ROI
/// \param rm_verts If true, then any vertex that falls outside of the ROI will
///                 not be copied. Defaults to false.
void CropMeshToRoi(const TriMesh& src_mesh, const Pt3& mins, const Pt3& maxs,
                   TriMesh* dst_mesh, const bool rm_verts = false);

/// \brief Inverts the values of a vertex mask; e.g. false -> true, true -> false
/// \param src_mask The input mask with values that will be inverted
/// \param dst_mask The output mask with inverted values of the input
void InvertVertexMask(const TriMesh::VertexMask& src_mask, TriMesh::VertexMask* dst_mask);

/**
 * @brief Removes any unused vertices from a mesh.
 *
 * @param src The input mesh
 * @param dst The output mesh (cannot be src)
 * @param rev_map (optional) Stores the reverse mapping from new vertex indices
 *                to original vertex indices; useful when batching a set of
 *                meshes with the same topology
 **/
void RemoveUnusedVertices(const TriMesh& src, TriMesh* dst,
                          TriMesh::VertexReorderMap* rev_map = nullptr);

std::tuple<Pt3,CoordScalar>
FindClosestPtToMeshExhaustive(const Pt3& x, const TriMesh& mesh);

} // xreg

#endif
