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

#include "xregVTK3DPlotter.h"

#include <ctime>

#include <vtkSphereSource.h>
#include <vtkLineSource.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkTextProperty.h>
#include <vtkArrowSource.h>
#include <vtkTransform.h>
#include <vtkMatrix4x4.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkCamera.h>
#include <vtkLight.h>
#include <vtkObjectFactory.h>  // vtkStandardNewMacro
#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkTextActor.h>
#include <vtkCubeAxesActor.h>
#include <vtkTextProperty.h>
#include <vtkScalarBarActor.h>
#include <vtkEllipseArcSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkPlaneSource.h>
#include <vtkTextureMapToPlane.h>
#include <vtkImageFlip.h>
#include <vtkMatrixToLinearTransform.h>
#include <vtkCubeSource.h>
#include <vtkPointData.h>
#include <vtkLookupTable.h>
#include <vtkFloatArray.h>
#include <vtkTexture.h>

#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>

#include <fmt/printf.h>

#include "xregObjWithOStream.h"
#include "xregFilesystemUtils.h"
#include "xregRigidUtils.h"
#include "xregVTKBasicUtils.h"
#include "xregVTKITKUtils.h"
#include "xregVTKMeshUtils.h"
#include "xregPerspectiveXform.h"
#include "xregAssert.h"
#include "xregSpatialPrimitives.h"
#include "xregScreenInfo.h"

namespace
{

constexpr char kQUIT_KEY           = 'q';
constexpr char kSCREEN_KEY         = 's';
constexpr char kAXES_KEY           = 'a';
constexpr char kFPS_KEY            = 'f';
constexpr char kSCALAR_LEGEND_KEY  = 'l';
constexpr char kHIDE_ALL_KEY       = 'h';
constexpr char kPRINT_CAM_INFO_KEY = 'c';

std::string GetTimestampString()
{
  std::time_t rawtime;
  char time_str[64];

  std::time(&rawtime);

  struct tm* timeinfo = std::localtime(&rawtime);

  std::strftime(time_str, 64, "%F at %H%M", timeinfo);

  return time_str;
}

std::string GetScreenShotFilePath()
{
  return xreg::GetUniqueNumericIDPath(GetTimestampString() + "_%02lu.png");
}

class ShowSceneInteractorStyleTrackballCamera
  : public vtkInteractorStyleTrackballCamera, public virtual xreg::ObjWithOStream
{
public:
  static ShowSceneInteractorStyleTrackballCamera* New();
  vtkTypeMacro(ShowSceneInteractorStyleTrackballCamera, vtkInteractorStyleTrackballCamera);

  vtkGetMacro(HideOverlays,bool);

  // vtkGetMacro fails to compile in debug mode on OSX, thus the manual implementations
  // of the getter

  //vtkGetMacro(AxesActor,vtkCubeAxesActor*);
  vtkCubeAxesActor* GetAxesActor()
  {
    return AxesActor;
  }
  vtkSetMacro(AxesActor,vtkCubeAxesActor*);

  //vtkGetMacro(FPSTextActor,vtkTextActor*);
  vtkTextActor* GetFPSTextActor()
  {
    return FPSTextActor;
  }
  vtkSetMacro(FPSTextActor,vtkTextActor*);

  //vtkGetMacro(ScalarBarActor,vtkScalarBarActor*);
  vtkScalarBarActor* GetScalarBarActor()
  {
    return ScalarBarActor;
  }

  vtkSetMacro(ScalarBarActor,vtkScalarBarActor*);

  void HideAxes()
  {
    if (AxesActor)
    {
      AxesActor->VisibilityOff();
    }
  }

  void ShowAxes()
  {
    if (AxesActor)
    {
      AxesActor->VisibilityOn();
    }
  }

  void HideFPSText()
  {
    if (FPSTextActor)
    {
      FPSTextActor->VisibilityOff();
    }
  }

  void ShowFPSText()
  {
    if (FPSTextActor)
    {
      FPSTextActor->VisibilityOn();
    }
  }

  void HideScalarBar()
  {
    if (ScalarBarActor)
    {
      ScalarBarActor->VisibilityOff();
    }
  }

  void ShowScalarBar()
  {
    if (ScalarBarActor)
    {
      ScalarBarActor->VisibilityOn();
      //this->Interactor->GetRenderWindow()->Render();
    }
  }

  void HideAllOverlays()
  {
    dout() << "hiding all overlays..." << std::endl;
    HideAxes();
    HideFPSText();
    HideScalarBar();
  }

  void ShowAllOverlays()
  {
    dout() << "showing all overlays..." << std::endl;
    ShowAxes();
    ShowFPSText();
    ShowScalarBar();
  }

  void SetHideOverlays(const bool x)
  {
    dout() << "setting HideOverlays: " << x << std::endl;

    HideOverlays = x;

    if (x)
    {
      HideAllOverlays();
    }
    else
    {
      ShowAllOverlays();
    }
  }

  void OnChar() override
  {
    vtkRenderWindowInteractor* rwi = this->Interactor;

    vtkRenderWindow* rw = rwi->GetRenderWindow();

    const char key = rwi->GetKeyCode();

    dout() << "key pressed: \'" << key << '\'' << std::endl;

    switch (key)
    {
    case kQUIT_KEY:
      rwi->ExitCallback();
      break;
    case kSCREEN_KEY:
    {
      const std::string screenshot_path = GetScreenShotFilePath();
      dout() << "saving screenshot to: " << screenshot_path << std::endl;
      // save a screenshot of the current display
      vtkNew<vtkWindowToImageFilter> win_to_img;
      win_to_img->SetInput(rw);
      win_to_img->Update();

      vtkNew<vtkPNGWriter> png_writer;
      png_writer->SetFileName(screenshot_path.c_str());
      png_writer->SetInputConnection(win_to_img->GetOutputPort());
      png_writer->Write();
    }
      break;
    case kAXES_KEY:
      if (AxesActor)
      {
        dout() << "toggling axes" << std::endl;
        AxesActor->SetVisibility(!AxesActor->GetVisibility());
        rw->Render();
      }
      break;
    case kFPS_KEY:
      if (FPSTextActor)
      {
        dout() << "toggling FPS text" << std::endl;
        FPSTextActor->SetVisibility(!FPSTextActor->GetVisibility());
        rw->Render();
      }
      break;
    case kSCALAR_LEGEND_KEY:
      if (ScalarBarActor)
      {
        dout() << "toggling Scalar Bar" << std::endl;
        ScalarBarActor->SetVisibility(!ScalarBarActor->GetVisibility());
        rw->Render();
      }
      break;
    case kHIDE_ALL_KEY:
      dout() << "toggling hiding overlays" << std::endl;
      this->SetHideOverlays(!this->GetHideOverlays());
      rw->Render();
      break;
    case kPRINT_CAM_INFO_KEY:
      {
        dout() << "printing camera info..." << std::endl;
        vtkCamera* cam = this->GetCurrentRenderer()->GetActiveCamera();      
     
        // TODO: explicitly print out az, el

        std::cout << "------------------------------------------------\n";
        cam->Print(std::cout);
        std::cout << "------------------------------------------------" << std::endl;
      }
      break;
    default:
      break;
    }

    // no need to call the super-class; does not do any event forwarding
    // this also overrides some behavior that I do not need
  }

private:
  vtkSmartPointer<vtkCubeAxesActor> AxesActor;
  vtkSmartPointer<vtkTextActor> FPSTextActor;
  vtkSmartPointer<vtkScalarBarActor> ScalarBarActor;

  bool HideOverlays;
};

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-function"
#endif

vtkStandardNewMacro(ShowSceneInteractorStyleTrackballCamera);

#ifdef __clang__
#pragma clang diagnostic pop
#endif

class FPSCallbackObj
{
public:
  FPSCallbackObj()
  {
    txt_actor_ = 0;
  }

  void fps_callback(vtkObject* caller, unsigned long, void*)
  {
    if (txt_actor_)
    {
      vtkRenderer* ren = static_cast<vtkRenderer*>(caller);
      txt_actor_->SetInput(fmt::sprintf("%.2f FPS", (1.0 / ren->GetLastRenderTimeInSeconds())).c_str());
    }
  }

  void set_txt_actor(vtkTextActor* ta)
  {
    txt_actor_ = ta;
  }

private:
  vtkTextActor* txt_actor_;
};

}  // un-named

xreg::VTK3DPlotter::VTK3DPlotter()
{
#ifdef __APPLE__
  const double scale = HiDPIScaling();
#else
  const double scale = 1.0;
#endif

  win_num_rows_ = std::lround(kDEFAULT_WIN_NUM_ROWS * scale);
  win_num_cols_ = std::lround(kDEFAULT_WIN_NUM_COLS * scale);

  ren_win_->AddRenderer(ren_.GetPointer());

  ren_->AddViewProp(cube_axes_actor_.GetPointer());
}

void xreg::VTK3DPlotter::show()
{
  if (use_custom_cam_)
  {
    // cam_pose_ is the position of the camera with respect to the "world" using the
    // camera coordinate frame definition in xregPerspectiveXform.h.
    // VTK uses the coordinate frame common in computer graphics with origin
    // on detector, y axis is view up, x axis is image right, z axis points back
    // to projection center.
    // NOTE: the projection center is the focal point, but VTK defines the focal
    //       point as the center of rotation for the camera in world coordinates.  

    // pre-multiply with the frame transformation between the two representations.
    // somehow VTK does some depth magic, we don't know the focal length, so a
    // normalized z coordindate of 1 is passed.
    
    FrameTransform vtk_cam_pose = cam_pose_ * EulerRotXYZTransXYZFrame(
                                                  3.141592653589793, 0.0, 0.0,
                                                  0.0, 0.0, 1.0);
    
    vtkNew<vtkMatrix4x4> xform_mat;
    ToVTKMat4x4(vtk_cam_pose.inverse(), xform_mat.GetPointer());

    // Transform all objects into the camera frame, this preserves good
    // behavior of lighting, unlike my attempts at moving only the camera.

    vtkActorCollection* actors = ren_->GetActors();

    actors->InitTraversal();
    
    vtkActor* cur_actor = actors->GetNextActor();

    while (cur_actor)
    {
      cur_actor->SetUserMatrix(xform_mat.GetPointer());   
      cur_actor = actors->GetNextActor();
    }
  }

  // TODO: add an "automatic re-centering" option that calls this.
  ren_->ResetCamera();
  
  // NOTE: THIS IS REQUIRED HERE, OTHERWISE SURFACES CAN BE CLIPPED OUT
  ren_->ResetCameraClippingRange();

  if (do_full_screen_)
  {
    ren_win_->FullScreenOn();
  }
  else
  {
    ren_win_->SetSize(win_num_cols_, win_num_rows_);
  }

  ren_->SetBackground(bg_color_[0], bg_color_[1], bg_color_[2]);

  setup_axes();
  
  if (use_custom_az_el_)
  {
    ren_->GetActiveCamera()->Azimuth(custom_az_deg_);
    ren_->GetActiveCamera()->Elevation(custom_el_deg_);
    ren_->GetActiveCamera()->OrthogonalizeViewUp();
  }

  if (!render_offscreen_)
  {
    // This call must happen before ren_win_->Render() (at least with VTK 7.1.1)
    // otherwise the program will crash on Linux (at least Ubuntu 16.04)
    // See http://vtk.1045678.n5.nabble.com/Display-Error-under-Ubuntu-16-04-td5740948.html
    iren_->SetRenderWindow(ren_win_.GetPointer());
  }

  ren_win_->Render();
  // Now other render window properties may be set, e.g. window title.

  if (!render_offscreen_)
  {
    FPSCallbackObj callbacks;
    
    if (!win_title_str_set_)
    {
      win_title_str_ = fmt::sprintf("Figure %03lu", WindowCount());
    }
    ren_win_->SetWindowName(win_title_str_.c_str());
    ++WindowCount();

    if (inter_on_)
    {
      const int orig_axes_vis = cube_axes_actor_->GetVisibility();

      vtkNew<ShowSceneInteractorStyleTrackballCamera> interactor_style;
      interactor_style->SetAxesActor(cube_axes_actor_.GetPointer());
      
      if (scalar_bar_)
      {
        interactor_style->SetScalarBarActor(scalar_bar_);
      }

      vtkNew<vtkTextActor> fps_txt_actor;
      callbacks.set_txt_actor(fps_txt_actor.GetPointer());
      
      ren_->AddViewProp(fps_txt_actor.GetPointer());
      ren_->AddObserver(vtkCommand::EndEvent, &callbacks, &FPSCallbackObj::fps_callback);
      
      interactor_style->SetFPSTextActor(fps_txt_actor.GetPointer());

      interactor_style->SetHideOverlays(true);
      // reset to user specified axes pref
      cube_axes_actor_->SetVisibility(orig_axes_vis);

      iren_->SetInteractorStyle(interactor_style.GetPointer());
    }
    else
    {
      iren_->Initialize();
      iren_->Disable();
    }

    iren_->Start();
  }
}

void xreg::VTK3DPlotter::set_title(const std::string& s)
{
  win_title_str_set_ = true;
  win_title_str_ = s;
}

void xreg::VTK3DPlotter::set_size(const size_type num_rows, const size_type num_cols)
{
#ifdef __APPLE__
  const double scale = HiDPIScaling();
#else
  const double scale = 1.0;
#endif

  win_num_rows_ = std::lround(scale * num_rows);
  win_num_cols_ = std::lround(scale * num_cols);
}
  
void xreg::VTK3DPlotter::set_full_screen(const bool fs)
{
  do_full_screen_ = fs;
}

void xreg::VTK3DPlotter::set_show_axes(const bool show_axes)
{
  if (show_axes)
  {
    cube_axes_actor_->VisibilityOn();
  }
  else
  {
    cube_axes_actor_->VisibilityOff();
  }
}
  
void xreg::VTK3DPlotter::add_pts(const Pt3List& pts)
{
  add_pts(pts, DefaultMeshColorAlpha());
}
  
void xreg::VTK3DPlotter::add_pts(const Pt3List& pts, const Scalar r, const Scalar g, const Scalar b)
{
  add_pts(pts, r, g, b, 1);
}

void xreg::VTK3DPlotter::add_pts(const Pt3List& pts, const RGBVec& rgb)
{
  add_pts(pts, rgb(0), rgb(1), rgb(2), 1);
}

void xreg::VTK3DPlotter::add_pts(const Pt3List& pts, const RGBAVec& rgba)
{
  add_pts(pts, rgba(0), rgba(1), rgba(2), rgba(3));
}

void xreg::VTK3DPlotter::add_pts(const Pt3List& pts, const Scalar r,
                                 const Scalar g,
                                 const Scalar b, const Scalar a)
{
  const size_type num_pts = pts.size();

  for (size_type i = 0; i < num_pts; ++i)
  {
    add_sphere(pts[i], 5, r, g, b, a);
  }
}

void xreg::VTK3DPlotter::add_sphere(const Pt3& center, const Scalar radius,
                                    const Scalar r, const Scalar g,
                                    const Scalar b, const Scalar a)
{
  vtkNew<vtkSphereSource> sphere_src;
  sphere_src->SetCenter(center(0), center(1), center(2));
  sphere_src->SetThetaResolution(20);
  sphere_src->SetPhiResolution(20);
  sphere_src->SetRadius(radius);
  sphere_src->Update();

  vtkNew<vtkPolyDataMapper> sphere_mapper;
  sphere_mapper->SetInputData(sphere_src->GetOutput());

  vtkNew<vtkActor> sphere_actor;
  sphere_actor->SetMapper(sphere_mapper.GetPointer());
  sphere_actor->GetProperty()->SetColor(r, g, b);

  sphere_actor->GetProperty()->SetOpacity(a);

  ren_->AddViewProp(sphere_actor.GetPointer());
}
  
void xreg::VTK3DPlotter::add_sphere(const Pt3& center, const Scalar radius, const RGBVec& rgb)
{
  add_sphere(center, radius, rgb[0], rgb[1], rgb[2]);
}

void xreg::VTK3DPlotter::add_sphere(const Pt3& center, const Scalar radius, const RGBAVec& rgba)
{
  add_sphere(center, radius, rgba[0], rgba[1], rgba[2], rgba[3]);
}

void xreg::VTK3DPlotter::add_line(const Pt3& pt1, const Pt3& pt2,
                                  const Scalar r, const Scalar g,
                                  const Scalar b, const Scalar a,
                                  const Scalar width)
{
  vtkNew<vtkLineSource> line_src;
  line_src->SetPoint1(pt1(0), pt1(1), pt1(2));
  line_src->SetPoint2(pt2(0), pt2(1), pt2(2));
  line_src->SetResolution(200);
  line_src->Update();

  vtkNew<vtkPolyDataMapper> line_mapper;
  line_mapper->SetInputData(line_src->GetOutput());

  vtkNew<vtkActor> line_actor;
  line_actor->SetMapper(line_mapper.GetPointer());
 
  vtkProperty* line_prop = line_actor->GetProperty();
 
  line_prop->SetColor(r, g, b);
  line_prop->SetOpacity(a);
  line_prop->SetLineWidth(width);

  ren_->AddViewProp(line_actor.GetPointer());
}
  
void xreg::VTK3DPlotter::add_line(const Pt3& pt1, const Pt3& pt2, const RGBVec& rgb, const Scalar width)
{
  add_line(pt1, pt2, rgb[0], rgb[1], rgb[2], 1.0, width);
}

void xreg::VTK3DPlotter::add_line(const Pt3& pt1, const Pt3& pt2, const RGBAVec& rgba, const Scalar width)
{
  add_line(pt1, pt2, rgba[0], rgba[1], rgba[2], rgba[3], width);
}

void xreg::VTK3DPlotter::add_arrow(const Pt3& base_pt, const Pt3& tip_pt,
                                   const Scalar r, const Scalar g,
                                   const Scalar b, const Scalar a)
{
  // build up the transform that will be needed to map the polydata from the
  // arrow source to our desired orientation

  FrameTransform xform = FrameTransform::Identity();
  Pt3 base_to_tip_orient = tip_pt - base_pt;
  
  const Scalar base_to_tip_len = base_to_tip_orient.norm();
  base_to_tip_orient /= base_to_tip_len;

  // The arrow source orients the arrow along the x-axis base to tip, so we want
  // to map that to the desired orientation
  xform.matrix().block(0,0,3,1) = base_to_tip_orient;

  // We have a degree of freedom here, choose the usual y-axis (0,1,0), unless
  // that is the desired base_to_tip_orient
  Pt3 y_axis = Pt3::Zero();
  y_axis[1] = 1;

  if ((base_to_tip_orient - y_axis).norm() < 1.0e-6)
  {
    // try the default z-axis
    y_axis[1] = 0;
    y_axis[2] = 1;
  }

  xform.matrix().block(0,1,3,1) = y_axis;

  // finish the right-handed coordinate transform:
  xform.matrix().block(0,2,3,1) = base_to_tip_orient.cross(y_axis);

  // we need to scale
  xform.matrix().block(0,0,3,3) *= base_to_tip_len;

  // now translate
  xform.matrix().block(0,3,3,1) = base_pt;

  // convert to VTK
  vtkNew<vtkMatrix4x4> vtk_xform_mat;
  ToVTKMat4x4(xform, vtk_xform_mat.GetPointer());

  vtkNew<vtkTransform> vtk_xform;
  vtk_xform->SetMatrix(vtk_xform_mat.GetPointer());

  vtkNew<vtkArrowSource> arrow_src;
  arrow_src->Update();

  vtkNew<vtkTransformPolyDataFilter> vtk_xform_filt;
  vtk_xform_filt->SetTransform(vtk_xform.GetPointer());
  vtk_xform_filt->SetInputData(arrow_src->GetOutput());
  vtk_xform_filt->Update();

  add_polydata(vtk_xform_filt->GetOutput(), r, g, b, a);
}
  
void xreg::VTK3DPlotter::add_arrow(const Pt3& base_pt, const Pt3& tip_pt, const RGBVec& rgb)
{
  add_arrow(base_pt, tip_pt, rgb[0], rgb[1], rgb[2]);
}

void xreg::VTK3DPlotter::add_arrow(const Pt3& base_pt, const Pt3& tip_pt, const RGBAVec& rgba)
{
  add_arrow(base_pt, tip_pt, rgba[0], rgba[1], rgba[2], rgba[3]);
}

void xreg::VTK3DPlotter::add_arrow(const Pt3& base_pt, const Pt3& tip_pt)
{
  add_arrow(base_pt, tip_pt, DefaultMeshColor());
}

void xreg::VTK3DPlotter::add_plane(const Pt3& origin_pt,
                                   const Pt3& pt1, const Pt3& pt2,
                                   const Scalar r, const Scalar g, const Scalar b,
                                   const Scalar a)
{
  vtkNew<vtkPlaneSource> plane_src;
  plane_src->SetOrigin(origin_pt(0), origin_pt(1), origin_pt(2));
  plane_src->SetPoint1(pt1(0), pt1(1), pt1(2));
  plane_src->SetPoint2(pt2(0), pt2(1), pt2(2));

  plane_src->SetResolution(20, 20);

  plane_src->Update();

  vtkNew<vtkPolyDataMapper> plane_mapper;

  plane_mapper->SetInputData(plane_src->GetOutput());

  plane_mapper->Update();

  vtkNew<vtkActor> plane_actor;
  plane_actor->SetMapper(plane_mapper.GetPointer());

  plane_actor->GetProperty()->SetColor(r, g, b);
  plane_actor->GetProperty()->BackfaceCullingOff();
  plane_actor->GetProperty()->FrontfaceCullingOff();

  plane_actor->GetProperty()->SetOpacity(a);

  ren_->AddViewProp(plane_actor.GetPointer());
}
  
void xreg::VTK3DPlotter::add_plane(const Pt3& origin_pt, const Pt3& pt1, const Pt3& pt2,
                                   const RGBVec& rgb)
{
  add_plane(origin_pt, pt1, pt2, rgb[0], rgb[1], rgb[2]);
}

void xreg::VTK3DPlotter::add_plane(const Pt3& origin_pt, const Pt3& pt1, const Pt3& pt2,
               const RGBAVec& rgba)
{
  add_plane(origin_pt, pt1, pt2, rgba(0), rgba(1), rgba(2), rgba(3));
}

void xreg::VTK3DPlotter::add_circle(const Pt3& center_pt, const Scalar radius,
                const Pt3& normal_vec)
{
  add_circle(center_pt, radius, normal_vec, DefaultMeshColorAlpha());
}

void xreg::VTK3DPlotter::add_circle(const Pt3& center_pt, const Scalar radius,
                const Pt3& normal_vec,
                const RGBVec& rgb)
{
  add_circle(center_pt, radius, normal_vec, rgb(0), rgb(1), rgb(2));
}

void xreg::VTK3DPlotter::add_circle(const Pt3& center_pt, const Scalar radius,
                const Pt3& normal_vec,
                const RGBAVec& rgba)
{
  add_circle(center_pt, radius, normal_vec, rgba(0), rgba(1), rgba(2), rgba(3));
}

void xreg::VTK3DPlotter::add_circle(const Pt3& center_pt, const Scalar radius,
                const Pt3& normal_vec,
                const Scalar r, const Scalar g, const Scalar b)
{
  add_circle(center_pt, radius, normal_vec, r, g, b, 1);
}

void xreg::VTK3DPlotter::add_circle(const Pt3& center_pt, const Scalar radius,
                                    const Pt3& normal_vec,
                                    const Scalar r, const Scalar g, const Scalar b,
                                    const Scalar a)
{
  vtkNew<vtkEllipseArcSource> arc;
  arc->SetCenter(center_pt[0], center_pt[1], center_pt[2]);
  arc->SetNormal(normal_vec[0], normal_vec[1], normal_vec[2]);
  arc->SetMajorRadiusVector(radius, 0, 0);
  arc->SetSegmentAngle(360);
  arc->Update();
  add_polydata(arc->GetOutput(), r, g, b, a);
}
  
void xreg::VTK3DPlotter::add_aa_slab(const AASlab3& slab, const FrameTransform& slab_pose)
{
  add_aa_slab(slab, slab_pose, DefaultMeshColorAlpha());
}

void xreg::VTK3DPlotter::add_aa_slab(const AASlab3& slab, const FrameTransform& slab_pose,
                                     const RGBVec& rgb)
{
  add_aa_slab(slab, slab_pose, rgb(0), rgb(1), rgb(2));
}

void xreg::VTK3DPlotter::add_aa_slab(const AASlab3& slab, const FrameTransform& slab_pose,
                                     const Scalar r, const Scalar g, const Scalar b)
{
  add_aa_slab(slab, slab_pose, r, g, b, 1);
}

void xreg::VTK3DPlotter::add_aa_slab(const AASlab3& slab, const FrameTransform& slab_pose,
                                     const RGBAVec& rgba)
{
  add_aa_slab(slab, slab_pose, rgba(0), rgba(1), rgba(2), rgba(3));
}

void xreg::VTK3DPlotter::add_aa_slab(const AASlab3& slab,
                                     const FrameTransform& slab_pose,
                                     const Scalar r, const Scalar g,
                                     const Scalar b, const Scalar a)
{
  vtkNew<vtkMatrix4x4> vtk_xform_mat;
  ToVTKMat4x4(slab_pose, vtk_xform_mat.GetPointer());

  vtkNew<vtkMatrixToLinearTransform> vtk_mat_to_vtk_xform;
  vtk_mat_to_vtk_xform->SetInput(vtk_xform_mat.GetPointer());
  vtk_mat_to_vtk_xform->Update();

  vtkNew<vtkCubeSource> cube_src;

  cube_src->SetBounds(slab.fll_pt(0), slab.bur_pt(0),
                      slab.fll_pt(1), slab.bur_pt(1),
                      slab.fll_pt(2), slab.bur_pt(2));

  cube_src->Update();

  vtkNew<vtkTransformPolyDataFilter> poly_data_xform_filter;
  poly_data_xform_filter->SetInputData(cube_src->GetOutput());
  poly_data_xform_filter->SetTransform(vtk_mat_to_vtk_xform.GetPointer());

  poly_data_xform_filter->Update();

  add_polydata(poly_data_xform_filter->GetOutput(), r, g, b, a);
}

namespace
{

using namespace xreg;

template <class T>
void Add2DImageHelper(itk::Image<T,2>* img, const CameraModel& cam, VTK3DPlotter& plotter)
{
  vtkNew<vtkPlaneSource> plane_src;

  // When overlaying an image as texture, we need the origin and axes of the
  // plane to correspond to the image.
  // In VTK the image origin is at the lower left, therefore we put the origin
  // of the plane at the lower left detector, the x-axis point as the lower
  // right point, and the y-axis point as the upper left.
  
  // top left pixel
  const Pt3 det_r0_c0 = cam.ind_pt_to_phys_det_pt(Pt2{0, 0}); 
  
  // lower left pixel
  const Pt3 det_rn_c0 = cam.ind_pt_to_phys_det_pt(Pt2{0, cam.num_det_rows - 1}); 
  
  // lower right pixel 
  const Pt3 det_rn_cm = cam.ind_pt_to_phys_det_pt(Pt2{cam.num_det_cols - 1, cam.num_det_rows - 1}); 

  plane_src->SetOrigin(det_rn_c0(0), det_rn_c0(1), det_rn_c0(2));
  plane_src->SetPoint1(det_rn_cm(0), det_rn_cm(1), det_rn_cm(2));
  plane_src->SetPoint2(det_r0_c0(0), det_r0_c0(1), det_r0_c0(2));

  plane_src->SetResolution(20, 20);

  plane_src->Update();

  vtkNew<vtkPolyDataMapper> plane_mapper;

  vtkNew<vtkTextureMapToPlane> tex_map_to_plane;
  tex_map_to_plane->SetInputData(plane_src->GetOutput());

  tex_map_to_plane->SetOrigin(det_rn_c0(0), det_rn_c0(1), det_rn_c0(2));
  tex_map_to_plane->SetPoint1(det_rn_cm(0), det_rn_cm(1), det_rn_cm(2));
  tex_map_to_plane->SetPoint2(det_r0_c0(0), det_r0_c0(1), det_r0_c0(2));
  tex_map_to_plane->Update();

  plane_mapper->SetInputConnection(tex_map_to_plane->GetOutputPort());

  plane_mapper->Update();

  vtkNew<vtkActor> plane_actor;
  plane_actor->SetMapper(plane_mapper.GetPointer());

  vtkNew<vtkTexture> texture;
  texture->SetInputData(ConvertITKImageToVTK(img, true, false));  // true -> flip up/down, false -> do not copy physical measurements

  plane_actor->SetTexture(texture.GetPointer());

  plotter.renderer()->AddViewProp(plane_actor.GetPointer());
}

}  // un-named

void xreg::VTK3DPlotter::add_2D_image(itk::Image<unsigned char,2>* img, const CameraModel& cam)
{
  Add2DImageHelper(img, cam, *this);
}
  
void xreg::VTK3DPlotter::add_2D_image(itk::Image<unsigned short,2>* img, const CameraModel& cam)
{
  Add2DImageHelper(img, cam, *this);
}

void xreg::VTK3DPlotter::add_2D_image(itk::Image<float,2>* img, const CameraModel& cam)
{
  Add2DImageHelper(img, cam, *this);
}

void xreg::VTK3DPlotter::add_mesh(const TriMesh& mesh,
                                  const Scalar r, const Scalar g,
                                  const Scalar b, const Scalar a)
{
  vtkNew<vtkPolyData> polydata;

  ConvertTriMeshToVTKPolyData(mesh, polydata.GetPointer());

  vtkNew<vtkPolyDataMapper> polydata_mapper;
  polydata_mapper->SetInputData(polydata.GetPointer());

  vtkNew<vtkActor> polydata_actor;
  polydata_actor->SetMapper(polydata_mapper.GetPointer());

  polydata_actor->GetProperty()->SetColor(r, g, b);

  polydata_actor->GetProperty()->SetOpacity(a);

  ren_->AddViewProp(polydata_actor.GetPointer());
}
  
void xreg::VTK3DPlotter::add_mesh(const TriMesh& mesh, const RGBVec& rgb)
{
  add_mesh(mesh, rgb[0], rgb[1], rgb[2]);
}
  
void xreg::VTK3DPlotter::add_mesh(const TriMesh& mesh, const RGBAVec& rgba)
{
  add_mesh(mesh, rgba[0], rgba[1], rgba[2], rgba[3]);
}

void xreg::VTK3DPlotter::add_mesh(const TriMesh& mesh)
{
  add_mesh(mesh, DefaultMeshColor());
}
  
void xreg::VTK3DPlotter::add_mesh_w_scalar_map(const TriMesh& mesh, const CoordScalarList& scalars,
                                               const AnyParamMap& scalar_map_params)
{
  // get param values from map
  
  bool do_scalar_override_min = false;
  bool do_scalar_override_max = false;
  
  CoordScalar scalar_override_min = 0.0;
  CoordScalar scalar_override_max = 0.0; 

  std::string scalars_title = "Scalars";

  int scalars_ticks = 4;

  {
    auto scalar_override_min_it = scalar_map_params.find("min");
    if (scalar_override_min_it != scalar_map_params.end())
    {
      do_scalar_override_min = true;
      scalar_override_min    = boost::any_cast<CoordScalar>(scalar_override_min_it->second);
    }

    auto scalar_override_max_it = scalar_map_params.find("max");
    if (scalar_override_max_it != scalar_map_params.end())
    {
      do_scalar_override_max = true;
      scalar_override_max    = boost::any_cast<CoordScalar>(scalar_override_max_it->second);
    }

    auto scalars_title_it = scalar_map_params.find("title");
    if (scalars_title_it != scalar_map_params.end())
    {
      const auto& t = scalars_title_it->second.type();

      if (t == typeid(std::string))
      {
        scalars_title = boost::any_cast<std::string>(scalars_title_it->second);
      }
      else if (t == typeid(const char*))
      {
        scalars_title = boost::any_cast<const char*>(scalars_title_it->second);
      }
      else
      {
        xregThrow("unsupported scalars title string type!");
      }
    }

    auto scalars_ticks_it = scalar_map_params.find("ticks");
    if (scalars_ticks_it != scalar_map_params.end())
    {
      scalars_ticks = boost::any_cast<int>(scalars_ticks_it->second);
    }
  }

  vtkNew<vtkPolyData> polydata;
  vtkNew<vtkFloatArray> vtk_scalars;
  vtkNew<vtkLookupTable> color_lut;

  ConvertTriMeshToVTKPolyData(mesh, polydata.GetPointer());

  vtkNew<vtkPolyDataMapper> polydata_mapper;
  polydata_mapper->SetInputData(polydata.GetPointer());

  vtkNew<vtkActor> polydata_actor;
  polydata_actor->SetMapper(polydata_mapper.GetPointer());
    
  ToVTKArray(scalars, vtk_scalars.GetPointer());
  polydata->GetPointData()->SetScalars(vtk_scalars.GetPointer());
  polydata_mapper->ScalarVisibilityOn();
  polydata_mapper->SetScalarModeToUsePointData();
  polydata_mapper->SetColorModeToMapScalars();

  // This is required to change the range displayed on the scalar bar actor
  double scalar_range[2] = { 0.0, 0.0 };
  if (!do_scalar_override_min || !do_scalar_override_max)
  {
    vtk_scalars->GetRange(scalar_range);
    scalar_range[0] = 0.0;
  }

  if (do_scalar_override_min)
  {
    scalar_range[0] = scalar_override_min;
  }
  if (do_scalar_override_max)
  {
    scalar_range[1] = scalar_override_max;
  }

  polydata_mapper->SetScalarRange(scalar_range);

  color_lut->SetTableRange(scalar_range);
  color_lut->SetSaturationRange(1.0, 1.0);
  //color_lut->SetValueRange(0.2, 1.0);
  color_lut->SetValueRange(1.0, 1.0);
  color_lut->SetHueRange(0.667, 0.0);
  //color_lut->SetRampToLinear();
  color_lut->SetRampToSCurve();
  color_lut->SetNumberOfTableValues(2048);
  color_lut->Build();

  polydata_mapper->SetLookupTable(color_lut.GetPointer());
 
  scalar_bar_ = vtkScalarBarActor::New(); 
  scalar_bar_->SetLookupTable(color_lut.GetPointer());
  scalar_bar_->SetTitle(scalars_title.c_str());
  scalar_bar_->SetNumberOfLabels(scalars_ticks);

  // results in a more aesthetically pleasing result
  polydata_mapper->InterpolateScalarsBeforeMappingOn();

  ren_->AddActor2D(scalar_bar_);

  //polydata_actor->GetProperty()->SetColor(r, g, b);
  //polydata_actor->GetProperty()->SetOpacity(a);

  ren_->AddViewProp(polydata_actor.GetPointer());
}

void xreg::VTK3DPlotter::add_polydata(vtkPolyData* polydata,
                                      const Scalar r, const Scalar g,
                                      const Scalar b, const Scalar a)
{
  vtkNew<vtkPolyDataMapper> polydata_mapper;

  polydata_mapper->SetInputData(polydata);

  polydata_mapper->Update();

  vtkNew<vtkActor> polydata_actor;
  polydata_actor->SetMapper(polydata_mapper.GetPointer());

  polydata_actor->GetProperty()->SetColor(r, g, b);

  polydata_actor->GetProperty()->SetOpacity(a);

  ren_->AddViewProp(polydata_actor.GetPointer());
}

void xreg::VTK3DPlotter::add_polydata(vtkPolyData* polydata, const RGBVec& rgb)
{
  add_polydata(polydata, rgb[0], rgb[1], rgb[2]);
}

void xreg::VTK3DPlotter::add_polydata(vtkPolyData* polydata, const RGBAVec& rgba)
{
  add_polydata(polydata, rgba(0), rgba(1), rgba(2), rgba(3));
}

void xreg::VTK3DPlotter::add_polydata(vtkPolyData* polydata)
{
  add_polydata(polydata, DefaultMeshColor());
}
  
vtkRenderer* xreg::VTK3DPlotter::renderer()
{
  return ren_.GetPointer();
}

void xreg::VTK3DPlotter::set_use_custom_cam(const bool use_custom_cam)
{
  use_custom_cam_ = use_custom_cam;
}

void xreg::VTK3DPlotter::set_cam_pose(const FrameTransform& cam_pose, const bool use_custom_cam)
{
  use_custom_cam_ = use_custom_cam;
  cam_pose_ = cam_pose;
}
  
void xreg::VTK3DPlotter::set_custom_az_el(const Scalar az_deg, const Scalar el_deg)
{
  use_custom_az_el_ = true;

  custom_az_deg_ = az_deg;
  custom_el_deg_ = el_deg;
}
  
void xreg::VTK3DPlotter::clear_custom_az_el()
{
  use_custom_az_el_ = false;
}

vtkCamera* xreg::VTK3DPlotter::vtk_cam()
{
  return ren_->GetActiveCamera();
}

void xreg::VTK3DPlotter::set_interaction(const bool inter_on)
{
  inter_on_ = inter_on;
}

void xreg::VTK3DPlotter::draw_to_png(const std::string& dst_path)
{
  vtkImageDataPtr vtk_img = draw_to_vtk_img_data();
  
  vtkNew<vtkPNGWriter> png_writer;
  png_writer->SetFileName(dst_path.c_str());
  png_writer->SetInputData(vtk_img);
  png_writer->Write();
}

xreg::VTK3DPlotter::vtkImageDataPtr
xreg::VTK3DPlotter::draw_to_vtk_img_data()
{
  render_offscreen_ = true;
  ren_win_->OffScreenRenderingOn();

  show();

  vtkNew<vtkWindowToImageFilter> win_to_img;
  win_to_img->SetInput(ren_win_.GetPointer());
  win_to_img->Update();
  
  render_offscreen_ = false;
  ren_win_->OffScreenRenderingOff();

  return win_to_img->GetOutput();
}

cv::Mat xreg::VTK3DPlotter::draw_to_ocv(const bool flip_ud)
{
  vtkImageDataPtr vtk_img = draw_to_vtk_img_data();

  if (flip_ud)
  {
    // VTK has the zero index pixel in the lower left, OpenCV convention
    // is upper left, so flip.

    vtkNew<vtkImageFlip> flipper;
    flipper->SetInputData(vtk_img);
    flipper->SetFilteredAxis(1);    // 1 -> flip y (row) direction
    flipper->FlipAboutOriginOff();  // axis cuts through image center

    flipper->Update();

    vtk_img = flipper->GetOutput();
  }

  int img_dims[3];
  vtk_img->GetDimensions(img_dims);

  xregASSERT(vtk_img->GetScalarType() == VTK_UNSIGNED_CHAR);
  xregASSERT(vtk_img->GetNumberOfScalarComponents() == 3);

  // clone to manage it's own memory, indep. of VTK
  cv::Mat ocv_img = cv::Mat(img_dims[1], img_dims[0], CV_8UC3,
                            vtk_img->GetScalarPointer()).clone();

  // change rgb to bgr
  for (int r = 0; r < img_dims[1]; ++r)
  {
    unsigned char* cur_row = &ocv_img.at<unsigned char>(r,0);

    for (int c = 0; c < img_dims[0]; ++c)
    {
      const int off = 3 * c;
      std::swap(cur_row[off], cur_row[off + 2]);
    }
  }

  return ocv_img;
}

void xreg::VTK3DPlotter::set_bg_color(const RGBVec& bg)
{
  bg_color_ = bg;
}
  
void xreg::VTK3DPlotter::set_bg_color(const Scalar& r, const Scalar& g, const Scalar& b)
{
  bg_color_[0] = r;
  bg_color_[1] = g;
  bg_color_[2] = b;
}

xreg::VTK3DPlotter::RGBVec xreg::VTK3DPlotter::DefaultBGColor()
{
  static bool need_to_init = true;
  static RGBVec rgb;

  if (need_to_init)
  {
    if (need_to_init)
    {
      need_to_init = false;

      rgb[0] = 0.7;
      rgb[1] = 0.8;
      rgb[2] = 1.0;
    }
  }

  return rgb;
}

xreg::VTK3DPlotter::RGBVec xreg::VTK3DPlotter::DefaultMeshColor()
{
  static bool need_to_init = true;
  static RGBVec rgb;

  if (need_to_init)
  {
    // need to get a lock here
    if (need_to_init)
    {
      need_to_init = false;

      rgb[0] = 0.65;
      rgb[1] = 0.65;
      rgb[2] = 0.65;
    }
  }

  return rgb;
}

xreg::VTK3DPlotter::RGBAVec
xreg::VTK3DPlotter::DefaultMeshColorAlpha()
{
  RGBAVec c;
  c.head(3) = DefaultMeshColor();
  c(3) = 1;
  return c;
}

xreg::VTK3DPlotter::RGBVec xreg::VTK3DPlotter::BoneColor()
{
  static bool need_to_init = true;
  static RGBVec rgb;

  if (need_to_init)
  {
    // TODO: make thread-safe
    need_to_init = false;

    rgb[0] = 1;
    rgb[1] = 0.9922;
    rgb[2] = 0.8980;
  }

  return rgb;
}

xreg::VTK3DPlotter::RGBAVec xreg::VTK3DPlotter::BoneColorAlpha()
{
  RGBAVec c;
  c.head(3) = BoneColor();
  c(3) = 1;
  return c;
}

std::string xreg::VTK3DPlotter::KeyUsageStr()
{
  std::stringstream ss;
  ss << "Keyboard Shortcuts:"
     << "\n  " << kQUIT_KEY << ": Quits the program"
     << "\n  " << kSCREEN_KEY << ": Saves a screenshot in the program\'s working directory with pattern YYYY-MM-DD_mesh_XXX.png (where XXX is a numeric identifier, e.g. 005)"
     << "\n  " << kAXES_KEY << ": Show/Hide axes grid"
     << "\n  " << kFPS_KEY << ": Show/Hide Frames per/second text"
     << "\n  " << kSCALAR_LEGEND_KEY << ": Show/Hide Scalar Legend Bar"
     << "\n  " << kHIDE_ALL_KEY << ": Show/Hide all overlays"
     << "\n  " << kPRINT_CAM_INFO_KEY << ": Print Camera Info";

  return ss.str();
}

void xreg::VTK3DPlotter::setup_axes()
{
  double ren_bounds[6];

  ren_->ComputeVisiblePropBounds(ren_bounds);

  cube_axes_actor_->SetBounds(ren_bounds[0], ren_bounds[1],
                              ren_bounds[2], ren_bounds[3],
                              ren_bounds[4], ren_bounds[5]);

  cube_axes_actor_->SetCamera(ren_->GetActiveCamera());

  cube_axes_actor_->GetTitleTextProperty(0)->SetColor(1.0, 0.0, 0.0);
  cube_axes_actor_->GetLabelTextProperty(0)->SetColor(1.0, 0.0, 0.0);

  cube_axes_actor_->GetTitleTextProperty(1)->SetColor(0.0, 1.0, 0.0);
  cube_axes_actor_->GetLabelTextProperty(1)->SetColor(0.0, 1.0, 0.0);

  cube_axes_actor_->GetTitleTextProperty(2)->SetColor(0.0, 0.0, 1.0);
  cube_axes_actor_->GetLabelTextProperty(2)->SetColor(0.0, 0.0, 1.0);

  cube_axes_actor_->DrawXGridlinesOn();
  cube_axes_actor_->DrawYGridlinesOn();
  cube_axes_actor_->DrawZGridlinesOn();
#if VTK_MAJOR_VERSION < 7
  cube_axes_actor_->SetGridLineLocation(VTK_GRID_LINES_FURTHEST);
#else
  cube_axes_actor_->SetGridLineLocation(vtkCubeAxesActor::VTK_GRID_LINES_FURTHEST);
#endif

  cube_axes_actor_->XAxisMinorTickVisibilityOff();
  cube_axes_actor_->YAxisMinorTickVisibilityOff();
  cube_axes_actor_->ZAxisMinorTickVisibilityOff();
}

unsigned long& xreg::VTK3DPlotter::WindowCount()
{
  static unsigned long win_count = 0;
  return win_count;
}

