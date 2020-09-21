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

#ifndef XREGVTKMESHUTILS_H_
#define XREGVTKMESHUTILS_H_

/// \file
/// Create mesh data with VTK routines and conversion to/from VTK

#include "xregMesh.h"

// Forward declaration
class vtkPolyData;

namespace xreg
{

/**
 * @brief Converts a VTK mesh into a TriMesh.
 * @param poly_data The VTK mesh to convert; assumed to contain only triangular faces
 * @param dst_mesh The TriMesh to write into.
 **/
TriMesh ConvertVTKPolyDataToTriMesh(vtkPolyData* poly_data);

/**
 * @brief Converts a TriMesh into a VTK mesh.
 *
 * @param src_mesh The TriMesh to convert
 * @param poly_data The destination VTK mesh
 **/
void ConvertTriMeshToVTKPolyData(const TriMesh& src_mesh, vtkPolyData* poly_data);

/**
 * @brief Writes a mesh to disk using a vtkPolyDataWriter.
 *
 * The format is determined by VTK via the file extension of the given path.
 * The main intention is that this function be used to write .vtk files.
 * @param mesh The mesh to be written to disk
 * @param path The path on disk to write to
 **/
void WriteVTKMesh(const TriMesh& mesh, const std::string& path, const bool ascii = false);

// Use VTK's implementation for writing meshes in PLY format
void WritePLYMesh(const TriMesh& mesh, const std::string& path, const bool ascii = false);

/**
 * @brief Reads a mesh from disk using a vtkPolyDataReader.
 *
 * The format is determined by VTK via the file extension of the given path.
 * The main intention is that this function be used to read .vtk files.
 * @param mesh The mesh to be read from disk
 * @param path The path on disk to read from
 **/
TriMesh ReadVTKMesh(const std::string& path);

// Use VTK's implementation to read PLY meshes
TriMesh ReadPLYMesh(const std::string& path);

/// \brief Function object for creating, and refining, a surface mesh with VTK routines.
struct VTKCreateMesh
{
  using LabelList = std::vector<double>;

  /// \brief The labels that represent locations that will be used to generate
  ///        the surface
  LabelList labels = { 1.0 };  // VTK uses double for labels

  bool do_smoothing = true;

  size_type num_smooth_its = 25;

  /// \brief only pass frequencies below a threshold; this impacts the shape of the sinc
  /// convolution filter in the spatial domain
  double smooth_passband = 0.1;

  /// \brief whether or not to smooth vertices on the boundary of the mesh
  bool smooth_boundary = false;

  /// \brief whether or not to smooth sharp interior edges
  bool smooth_feature_edges = false;

  /// \brief the threshold for a feature edge
  double smooth_feature_angle = 120.0;

  /// \brief whether or not to smooth non-manifold vertices
  bool smooth_non_manifold = true;

  bool do_reduction = true;

  /// \brief Reduce/remove this percentage of the original number of triangles
  double reduction_amount = 0.25;

  bool compute_normals = false;

  /// \brief True -> do not let the VTK normals filter flip the normals
  bool no_flip_vtk_normals = false;

  /// \brief True -> leave the mesh in label image index coordinates.
  bool no_phys_coords = false;

  /// \brief Reverse the vertex ordering and flip normals of the VTK output
  bool reverse_vertex_order = false;

  /// \brief Creates, and refines, a surface mesh from a label map volume using VTK routines.
  TriMesh operator()(itk::Image<unsigned char,3>* label_img);
};

}  // xreg

#endif

