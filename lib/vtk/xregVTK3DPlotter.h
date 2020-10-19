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

#ifndef XREGVTK3DPLOTTER_H_
#define XREGVTK3DPLOTTER_H_

#include <vtkNew.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeAxesActor.h>
#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkScalarBarActor.h>

#include <itkImage.h>

#include <opencv2/core/core.hpp>

#include "xregCommon.h"

class vtkPolyData;

namespace xreg
{

struct CameraModel;
struct TriMesh;
struct AASlab3;

/// \brief Helper class for common plotting operations with VTK
class VTK3DPlotter
{
public:
  using RGBVec  = Pt3;
  using RGBAVec = Pt4;
  using Scalar  = CoordScalar;

  using vtkImageDataPtr = vtkSmartPointer<vtkImageData>;

  /// \brief Constructor - does some minor initialization.
  VTK3DPlotter();

  // no copying
  VTK3DPlotter(const VTK3DPlotter&) = delete;
  VTK3DPlotter& operator=(const VTK3DPlotter&) = delete;

  /// \brief Shows the window and starts the interator - blocking.
  void show();

  /// \brief Sets the title string of the window
  void set_title(const std::string& s);

  /// \brief Sets the window size (in pixels)
  void set_size(const size_type num_rows, const size_type num_cols);

  void set_full_screen(const bool fs);

  /// \brief Sets whether axes graphics should be displayed.
  void set_show_axes(const bool show_axes);

  /// \brief Plots a collection of points.
  void add_pts(const Pt3List& pts);
  
  /// \brief Plots a collection of points.
  void add_pts(const Pt3List& pts, const Scalar r, const Scalar g, const Scalar b);

  /// \brief Plots a collection of points.
  void add_pts(const Pt3List& pts, const RGBVec& rgb);

  /// \brief Plots a collection of points.
  void add_pts(const Pt3List& pts, const RGBAVec& rgba);

  /// \brief Plots a collection of points.
  void add_pts(const Pt3List& pts, const Scalar r, const Scalar g, const Scalar b, const Scalar a);

  /// \brief Plots a sphere with center and radius
  void add_sphere(const Pt3& center, const Scalar radius,
                  const Scalar r, const Scalar g, const Scalar b, const Scalar a = 1);

  /// \brief Plots a sphere with center and radius
  void add_sphere(const Pt3& center, const Scalar radius, const RGBVec& rgb);
  
  /// \brief Plots a sphere with center and radius
  void add_sphere(const Pt3& center, const Scalar radius, const RGBAVec& rgba);

  /// \brief Plots a line segment connecting two points
  void add_line(const Pt3& pt1, const Pt3& pt2,
                const Scalar r, const Scalar g, const Scalar b, const Scalar a = 1,
                const Scalar width = 1.0);

  /// \brief Plots a line segment connecting two points
  void add_line(const Pt3& pt1, const Pt3& pt2, const RGBVec& rgb, const Scalar width = 1.0);

  /// \brief Plots a line segment connecting two points
  void add_line(const Pt3& pt1, const Pt3& pt2, const RGBAVec& rgba, const Scalar width = 1.0);
  
  /// \brief Draws an arrow
  void add_arrow(const Pt3& base_pt, const Pt3& tip_pt,
                 const Scalar r, const Scalar g, const Scalar b,
                 const Scalar a = 1);

  /// \brief Draws an arrow
  void add_arrow(const Pt3& base_pt, const Pt3& tip_pt, const RGBVec& rgb);

  /// \brief Draws an arrow
  void add_arrow(const Pt3& base_pt, const Pt3& tip_pt, const RGBAVec& rgba);

  /// \brief Draws an arrow with default color.
  void add_arrow(const Pt3& base_pt, const Pt3& tip_pt);

  /// \brief Plots a plane surface.
  ///
  /// The plane is defined by an origin, o, and two points, p1, p2. The vector
  /// p1 - o defines the x-axis and extent along the x direction, likewise the
  /// vector p2 - o defines the y-axis and extent along the y direction, and
  /// o is the origin. o, p1, p2 define three corners of the plane and the
  /// fourth is inferred.
  void add_plane(const Pt3& origin_pt, const Pt3& pt1, const Pt3& pt2,
                 const Scalar r, const Scalar g, const Scalar b, const Scalar a = 1);

  /// \brief Plots a plane surface.
  void add_plane(const Pt3& origin_pt, const Pt3& pt1, const Pt3& pt2,
                 const RGBVec& rgb);

  /// \brief Plots a plane surface.
  void add_plane(const Pt3& origin_pt, const Pt3& pt1, const Pt3& pt2,
                 const RGBAVec& rgba);
  
  /// \brief Plot a circle
  void add_circle(const Pt3& center_pt, const Scalar radius,
                  const Pt3& normal_vec);

  /// \brief Plot a circle
  void add_circle(const Pt3& center_pt, const Scalar radius,
                  const Pt3& normal_vec,
                  const RGBVec& rgb);

  /// \brief Plot a circle
  void add_circle(const Pt3& center_pt, const Scalar radius,
                  const Pt3& normal_vec,
                  const RGBAVec& rgba);

  /// \brief Plot a circle
  void add_circle(const Pt3& center_pt, const Scalar radius,
                  const Pt3& normal_vec,
                  const Scalar r, const Scalar g, const Scalar b);

  /// \brief Plot a circle
  void add_circle(const Pt3& center_pt, const Scalar radius,
                  const Pt3& normal_vec,
                  const Scalar r, const Scalar g, const Scalar b,
                  const Scalar a);


  /// \brief Plots an axis-aligned slab that has been re-oriented
  void add_aa_slab(const AASlab3& slab, const FrameTransform& slab_pose);

  /// \brief Plots an axis-aligned slab that has been re-oriented
  void add_aa_slab(const AASlab3& slab, const FrameTransform& slab_pose,
                   const RGBVec& rgb);

  /// \brief Plots an axis-aligned slab that has been re-oriented
  void add_aa_slab(const AASlab3& slab, const FrameTransform& slab_pose,
                   const Scalar r, const Scalar g, const Scalar b);

  /// \brief Plots an axis-aligned slab that has been re-oriented
  void add_aa_slab(const AASlab3& slab, const FrameTransform& slab_pose,
                   const RGBAVec& rgba);
  
  /// \brief Plots an axis-aligned slab that has been re-oriented
  void add_aa_slab(const AASlab3& slab, const FrameTransform& slab_pose,
                   const Scalar r, const Scalar g, const Scalar b, const Scalar a);

  /// \brief Plots a 2D image plane with respect to the camera model "world"
  ///        coordinates.
  void add_2D_image(itk::Image<unsigned char,2>* img, const CameraModel& cam);

  /// \brief Plots a 2D image plane with respect to the camera model "world"
  ///        coordinates.
  void add_2D_image(itk::Image<unsigned short,2>* img, const CameraModel& cam);

  /// \brief Plots a 2D image plane with respect to the camera model "world"
  ///        coordinates.
  void add_2D_image(itk::Image<float,2>* img, const CameraModel& cam);

  /// \brief Plots a surface defined with a triangular surface mesh.
  void add_mesh(const TriMesh& mesh, const Scalar r, const Scalar g,
                const Scalar b, const Scalar a = 1);

  /// \brief Plots a surface defined with a triangular surface mesh.
  void add_mesh(const TriMesh& mesh, const RGBVec& rgb);
  
  /// \brief Plots a surface defined with a triangular surface mesh.
  void add_mesh(const TriMesh& mesh, const RGBAVec& rgba);

  /// \brief Plots a surface defined with a triangular surface mesh.
  ///
  /// Uses default color.
  void add_mesh(const TriMesh& mesh);

  void add_mesh_w_scalar_map(const TriMesh& mesh, const CoordScalarList& scalars,
                             const AnyParamMap& scalar_map_params = AnyParamMap());

  /// \brief Plots an existing VTK PolyData object.
  void add_polydata(vtkPolyData* polydata, const Scalar r, const Scalar g,
                    const Scalar b, const Scalar a = 1);

  /// \brief Plots an existing VTK PolyData object.
  void add_polydata(vtkPolyData* polydata, const RGBVec& rgb);

  /// \brief Plots an existing VTK PolyData object.
  void add_polydata(vtkPolyData* polydata, const RGBAVec& rgba);

  /// \brief Plots an existing VTK PolyData object.
  ///
  /// Uses default color.
  void add_polydata(vtkPolyData* polydata);

  /// \brief Retrieve the renderer object used.
  ///
  /// Useful for using other VTK routines to update the display.
  vtkRenderer* renderer();

  /// \brief Sets the flag to use, or not use, a custom camera pose
  ///
  /// \see set_cam_pose
  /// NOTE: The camera coordinate frame is changed according to the geometry
  /// defined by the xreg::CameraModel class.
  void set_use_custom_cam(const bool use_custom_cam);

  /// \brief Sets a custom camera pose
  ///
  /// NOTE: The camera coordinate frame is changed according to the geometry
  /// defined by the xreg::CameraModel class.
  void set_cam_pose(const FrameTransform& cam_pose, const bool use_custom_cam = true);

  void set_custom_az_el(const Scalar az_deg, const Scalar el_deg);
  
  void clear_custom_az_el();

  vtkCamera* vtk_cam();

  void set_interaction(const bool inter_on);

  /// \brief Render, without displaying to the user, and write to a PNG file.
  ///
  /// Similar to calling show() and manually capturing a screenshot of the
  // initial render.
  void draw_to_png(const std::string& dst_path);

  /// \brief Render, without displaying to the user, and return as a VTK Image.
  vtkImageDataPtr draw_to_vtk_img_data();

  /// \brief Render, without displaying to the user, and return as an OpenCV
  ///        mat object in BGR format.
  ///
  /// The user can optionally disable the up/down flipping (VTK places the 
  /// zero index in the lower left, compared to OpenCV convention of
  /// upper left).
  cv::Mat draw_to_ocv(const bool flip_ud = true);

  void set_bg_color(const RGBVec& bg);
  
  void set_bg_color(const Scalar& r, const Scalar& g, const Scalar& b);

  /// \brief Default background color
  static RGBVec DefaultBGColor();

  /// \brief Default mesh color RGB
  static RGBVec DefaultMeshColor();

  /// \brief Default mesh color RGB with alpha
  static RGBAVec DefaultMeshColorAlpha();

  /// \brief A reasonable color to use for bone surfaces, RGB
  static RGBVec BoneColor();

  /// \brief A reasonable color to use for bone surfaces, RGB with alpha
  static RGBAVec BoneColorAlpha();

  static std::string KeyUsageStr();

private:

  enum { kDEFAULT_WIN_NUM_ROWS = 1024 };
  enum { kDEFAULT_WIN_NUM_COLS = 1024 };

  void setup_axes();

  bool render_offscreen_ = false;

  bool use_custom_cam_ = false;

  bool inter_on_ = true;

  bool win_title_str_set_ = false;

  std::string win_title_str_;

  // These values are set to default in the ctor.
  size_type win_num_rows_;
  size_type win_num_cols_;

  bool do_full_screen_ = false;

  vtkNew<vtkRenderWindowInteractor> iren_;
  vtkNew<vtkRenderer> ren_;
  vtkNew<vtkRenderWindow> ren_win_;

  vtkNew<vtkCubeAxesActor> cube_axes_actor_;

  vtkSmartPointer<vtkScalarBarActor> scalar_bar_;

  FrameTransform cam_pose_ = FrameTransform::Identity();

  RGBVec bg_color_ = DefaultBGColor();

  bool use_custom_az_el_ = false;

  Scalar custom_az_deg_ = 0;
  Scalar custom_el_deg_ = 0;

  static unsigned long& WindowCount();
};

}  // xreg

#endif
