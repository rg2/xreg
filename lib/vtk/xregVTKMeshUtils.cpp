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

#include "xregVTKMeshUtils.h"

#include <vtkNew.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkPolyDataWriter.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyDataNormals.h>
#include <vtkFloatArray.h>
#include <vtkVersion.h>
#include <vtkTriangleFilter.h>
#include <vtkImageData.h>
#include <vtkDiscreteMarchingCubes.h>
#include <vtkWindowedSincPolyDataFilter.h>
#include <vtkQuadricDecimation.h>
#include <vtkPolyDataNormals.h>
#include <vtkAlgorithmOutput.h>
#include <vtkPLYWriter.h>
#include <vtkPLYReader.h>

#include <vtkVersionMacros.h>

#include "xregVTKBasicUtils.h"
#include "xregVTKITKUtils.h"
#include "xregITKBasicImageUtils.h"

xreg::TriMesh xreg::ConvertVTKPolyDataToTriMesh(vtkPolyData* poly_data)
{
  TriMesh dst_mesh;

  // First, get the array of vertices
  vtkPoints* src_pts = poly_data->GetPoints();
  const vtkIdType num_pts = src_pts->GetNumberOfPoints();

  //std::cerr << "num_pts = " << num_pts << std::endl;

  dst_mesh.vertices.resize(num_pts);

  double tmp_pt[3];

  for (vtkIdType i = 0; i < num_pts; ++i)
  {
    src_pts->GetPoint(i, tmp_pt);

    //std::cerr << "pt[" << i << "] = (" << tmp_pt[0] << ',' << tmp_pt[1]
    //                                   << ',' << tmp_pt[2] << ')' << std::endl;

    dst_mesh.vertices[i][0] = static_cast<CoordScalar>(tmp_pt[0]);
    dst_mesh.vertices[i][1] = static_cast<CoordScalar>(tmp_pt[1]);
    dst_mesh.vertices[i][2] = static_cast<CoordScalar>(tmp_pt[2]);
  }

  // Now, get the array of triangles defined by vertex indices -
  // first check by looking at the Polys to treat as triangles,
  // if there are none then use triangle strips.

  vtkSmartPointer<vtkCellArray> tri_cells;

  if (poly_data->GetNumberOfPolys() > 0)
  {
    tri_cells = poly_data->GetPolys();
  }
  else if (poly_data->GetNumberOfStrips() > 0)
  {
    vtkNew<vtkTriangleFilter> tri_filter;
    tri_filter->SetInputData(poly_data);
    tri_filter->Update();
    tri_cells = tri_filter->GetOutput()->GetPolys();
  }

  const vtkIdType num_tris = tri_cells ? tri_cells->GetNumberOfCells() : 0;

  if (num_tris > 0)
  {
    dst_mesh.faces.resize(num_tris);

    tri_cells->InitTraversal();

    vtkIdType tmp_num_pts = 0;

#if VTK_MAJOR_VERSION >= 9
    using CurIndsPtr = vtkIdType const*;
#else
    using CurIndsPtr = vtkIdType*;
#endif

    CurIndsPtr cur_inds = nullptr;

    vtkFloatArray* normals_array = vtkFloatArray::SafeDownCast(poly_data->GetCellData()->GetNormals());
    const bool have_normals = normals_array != 0;
    dst_mesh.normals_valid = have_normals;
    if (have_normals)
    {
      dst_mesh.normals.resize(num_tris);
    }

    for (vtkIdType i = 0; i < num_tris; ++i)
    {
      tri_cells->GetNextCell(tmp_num_pts, cur_inds);

      //std::cerr << "tri[" << i << "] = num_pts: " << tmp_num_pts;
      //std::cerr.flush();
      //std::cerr << ", (" << cur_inds[0] << ',' << cur_inds[1] << ',' << cur_inds[2] << ')'
      //          << std::endl;

      dst_mesh.faces[i][0] = static_cast<size_type>(cur_inds[0]);
      dst_mesh.faces[i][1] = static_cast<size_type>(cur_inds[1]);
      dst_mesh.faces[i][2] = static_cast<size_type>(cur_inds[2]);

      if (have_normals)
      {
        normals_array->GetTuple(i, tmp_pt);

        dst_mesh.normals[i][0] = static_cast<CoordScalar>(tmp_pt[0]);
        dst_mesh.normals[i][1] = static_cast<CoordScalar>(tmp_pt[1]);
        dst_mesh.normals[i][2] = static_cast<CoordScalar>(tmp_pt[2]);
      }
    }
  }
  else
  {
    // could not locate the triangles - should an exception be thrown?
    dst_mesh.faces.clear();
  }

  return dst_mesh;
}

void xreg::ConvertTriMeshToVTKPolyData(const TriMesh& src_mesh, vtkPolyData* poly_data)
{
  // First build up a list of the vertices
  vtkSmartPointer<vtkPoints> vtk_pts = vtkPoints::New(LookupVTKDataTypeID<CoordScalar>::value);

  const vtkIdType num_pts = static_cast<vtkIdType>(src_mesh.vertices.size());

  // This does an allocation
  vtk_pts->SetNumberOfPoints(num_pts);

  for (vtkIdType vertex_index = 0; vertex_index < num_pts; ++vertex_index)
  {
    vtk_pts->SetPoint(vertex_index,
                      src_mesh.vertices[vertex_index][0],
                      src_mesh.vertices[vertex_index][1],
                      src_mesh.vertices[vertex_index][2]);
  }

  // Now add these points to the polydata
  poly_data->SetPoints(vtk_pts);

  // Now create a VTK compatable list of faces
  const vtkIdType num_faces = static_cast<vtkIdType>(src_mesh.faces.size());

  vtkIdType tmp_face[3] = { 0, 0, 0 };

  vtkSmartPointer<vtkCellArray> vtk_faces = vtkCellArray::New();
  vtk_faces->Allocate(num_faces);

  for (vtkIdType face_index = 0; face_index < num_faces; ++face_index)
  {
    tmp_face[0] = static_cast<vtkIdType>(src_mesh.faces[face_index][0]);
    tmp_face[1] = static_cast<vtkIdType>(src_mesh.faces[face_index][1]);
    tmp_face[2] = static_cast<vtkIdType>(src_mesh.faces[face_index][2]);

    vtk_faces->InsertNextCell(3, tmp_face);
  }

  poly_data->SetPolys(vtk_faces);
}

void xreg::WriteVTKMesh(const TriMesh& mesh, const std::string& path, const bool ascii)
{
  vtkNew<vtkPolyData> polydata;
  vtkNew<vtkPolyDataNormals> normals;
  vtkNew<vtkPolyDataWriter> writer;

  ConvertTriMeshToVTKPolyData(mesh, polydata.GetPointer());

#if VTK_MAJOR_VERSION <= 5
  normals->SetInput(polydata.GetPointer());
#else
  normals->SetInputData(polydata.GetPointer());
#endif

  // do not split sharp edges
  normals->SplittingOff();

  // ensure that the ordering of triangles yields consistent normal directions
  normals->ConsistencyOn();

  // This ensures the normals will result in a properly visualized surface in
  // a VTK display
  normals->AutoOrientNormalsOn();

  normals->ComputeCellNormalsOn();
  normals->ComputePointNormalsOn();

  normals->Update();

  writer->SetInputConnection(normals->GetOutputPort());
  writer->SetFileName(path.c_str());
  writer->SetFileType(ascii ? VTK_ASCII : VTK_BINARY);
  writer->Write();
}

void xreg::WritePLYMesh(const TriMesh& mesh, const std::string& path, const bool ascii)
{
  vtkNew<vtkPolyData> polydata;
  vtkNew<vtkPLYWriter> writer;

  ConvertTriMeshToVTKPolyData(mesh, polydata.GetPointer());
  
  writer->SetInputData(polydata.GetPointer());
  writer->SetFileName(path.c_str());
  writer->SetFileType(ascii ? VTK_ASCII : VTK_BINARY);
  writer->Write();
}

xreg::TriMesh xreg::ReadVTKMesh(const std::string& path)
{
  vtkNew<vtkPolyDataReader> reader;
  // TODO: handle normals if available

  reader->SetFileName(path.c_str());
  reader->Update();

  return ConvertVTKPolyDataToTriMesh(reader->GetOutput());
}

xreg::TriMesh xreg::ReadPLYMesh(const std::string& path)
{
  vtkNew<vtkPLYReader> reader;

  reader->SetFileName(path.c_str());
  reader->Update();

  return ConvertVTKPolyDataToTriMesh(reader->GetOutput());
}

xreg::TriMesh xreg::VTKCreateMesh::operator()(itk::Image<unsigned char,3>* label_img)
{
  // convert from the ITK image container to VTK, so we can use the VTK routines for meshes
  // flip up/down to get correct origin index - ignore spacing and origin physical points,
  // so VTK produces a mesh in image index coordinates first - afterwards we'll convert the
  // vertex values into physical units (after flipping up/down again...)
  vtkSmartPointer<vtkImageData> vtk_img = ConvertITKImageToVTK(label_img, true, false);

  // Algorithms that may be used later
  vtkNew<vtkWindowedSincPolyDataFilter> smoother;
  vtkNew<vtkQuadricDecimation> reduce;
  vtkNew<vtkPolyDataNormals> normals;

  // Create the initial isosurface from the label image with marching cubes
  vtkNew<vtkDiscreteMarchingCubes> cubes;
  cubes->SetInputData(vtk_img.GetPointer());

  const size_type num_labels = labels.size();
  cubes->SetNumberOfContours(num_labels);
  for (size_type label_idx = 0; label_idx < num_labels; ++label_idx)
  {
    cubes->SetValue(label_idx, labels[label_idx]);
  }

  cubes->Update();

  vtkSmartPointer<vtkAlgorithmOutput> cur_output_port = cubes->GetOutputPort();
  vtkSmartPointer<vtkPolyData>        cur_output      = cubes->GetOutput();

  if (do_smoothing)
  {
    // Smooth the isosurface
    smoother->SetInputConnection(cur_output_port);

    // Smoothing parameters:

    smoother->SetNumberOfIterations(num_smooth_its);

    // only pass frequencies below 0.1; this impacts the shape of the sinc
    // convolution filter in the spatial domain
    smoother->SetPassBand(smooth_passband);

    smoother->SetBoundarySmoothing(smooth_boundary);

    smoother->SetFeatureEdgeSmoothing(smooth_feature_edges);
    smoother->SetFeatureAngle(smooth_feature_angle);

    smoother->SetNonManifoldSmoothing(smooth_non_manifold);

    // this scales the coordinates to be within [-1,1] prior to smoothing for
    // numerical stability, coordinates are re-scaled to the original coordinate
    // range upon completion
    smoother->NormalizeCoordinatesOn();

    smoother->Update();

    cur_output_port = smoother->GetOutputPort();
    cur_output      = smoother->GetOutput();
  }

  if (do_reduction)
  {
    // Decimate the smoothed isosurface
    reduce->SetInputConnection(cur_output_port);

    reduce->SetTargetReduction(reduction_amount);

    reduce->Update();

    cur_output_port = reduce->GetOutputPort();
    cur_output      = reduce->GetOutput();
  }

  // Compute face normals
  if (compute_normals)
  {
    normals->SetInputConnection(cur_output_port);

    // do not split sharp edges
    normals->SplittingOff();

    // ensure that the ordering of triangles yields consistent normal directions
    normals->ConsistencyOn();

    normals->AutoOrientNormalsOn();

    if (!no_flip_vtk_normals)
    {
      // This ensures the normals will "point inward"
      normals->FlipNormalsOn();
    }

    normals->ComputeCellNormalsOn();
    normals->ComputePointNormalsOn();

    normals->Update();

    cur_output_port = normals->GetOutputPort();
    cur_output      = normals->GetOutput();
  }

  // Convert VTK to XREG format
  auto mesh = ConvertVTKPolyDataToTriMesh(cur_output);

  // convert vertex values from image indices to physical points.
  FrameTransform vertex_xform = FrameTransform::Identity();

  // first use the flip up/down to get back to ITK indices
  vertex_xform.matrix()(1,1) = -1;
  vertex_xform.matrix()(1,3) = label_img->GetLargestPossibleRegion().GetSize()[1] + 1;

  if (!no_phys_coords)
  {
    vertex_xform = ITKImagePhysicalPointTransformsAsEigen(label_img) * vertex_xform;
  }

  mesh.transform(vertex_xform);

  if (reverse_vertex_order)
  {
    mesh.reverse_vertex_ordering();
    mesh.flip_normals();
  }

  return mesh;
}

