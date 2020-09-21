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

#include "xregPAODrawBones.h"

#include "xregLandmarkMapUtils.h"
#include "xregAnatCoordFrames.h"
#include "xregVTKMeshUtils.h"
#include "xregHipSegUtils.h"
#include "xregRigidUtils.h"
#include "xregPointCloudUtils.h"
#include "xregAssert.h"

void xreg::DrawPAOBones::operator()()
{
  const bool is_left = side == kLEFT;

  dout() << "APP Landmarks:\n";
  PrintLandmarkMap(app_pts, dout());

  const FrameTransform app_orig_to_vol = AnteriorPelvicPlaneFromLandmarksMap(app_pts,
                                                          kAPP_ORIGIN_MEDIAN_ILIAC_CREST);
  dout() << "APP to Vol (standard origin):\n" << app_orig_to_vol.matrix() << std::endl;

  const FrameTransform app_to_vol = AnteriorPelvicPlaneFromLandmarksMap(app_pts,
                                is_left ? kAPP_ORIGIN_LEFT_FH : kAPP_ORIGIN_RIGHT_FH);

  dout() << "APP to Vol (femur origin):\n" << app_to_vol.matrix() << std::endl;

  const Pt3 femur_pt = app_to_vol.matrix().block(0,3,3,1);

  const FrameTransform vol_to_app = app_to_vol.inverse();

  //////////////////////////////////////////////////////////////////////////////
  // Determine labels of pelvis, fragment, femurs, and cut

  const bool need_to_find_pelvis_label = !pelvis_label_arg;
  if (!need_to_find_pelvis_label)
  {
    pelvis_label = *pelvis_label_arg;
  }

  const bool need_to_find_frag_label = !frag_label_arg;
  if (!need_to_find_frag_label)
  {
    frag_label = *frag_label_arg;
  }

  const bool need_to_find_femur_label = !femur_label_arg;
  if (!need_to_find_femur_label)
  {
    femur_label = *femur_label_arg;
  }

  const bool need_to_find_contra_femur_label = !contra_femur_label_arg;
  if (!need_to_find_contra_femur_label)
  {
    con_femur_label = *contra_femur_label_arg;
  }

  if (need_to_find_pelvis_label || need_to_find_frag_label)
  {
    dout() << "attempting to find some labels automatically..." << std::endl;
    
    const auto guessed_labels = GuessPelvisPAOFragCutLabels(labels.GetPointer(), true, true);

    if (need_to_find_pelvis_label)
    {
      pelvis_label = std::get<0>(guessed_labels);
      dout() << "pelvis label automatically determined to be: " << static_cast<int>(pelvis_label) << std::endl;
    }

    if (need_to_find_frag_label)
    {
      frag_label = std::get<1>(guessed_labels);
      dout() << "fragment label automatically determined to be: " << static_cast<int>(frag_label) << std::endl;
    }
  }

  if (need_to_find_femur_label)
  {
    dout() << "looking up operative femur label..." << std::endl;

    ITKIndex femur_idx;
    ITKPoint femur_pt_itk;
    femur_pt_itk[0] = femur_pt[0];
    femur_pt_itk[1] = femur_pt[1];
    femur_pt_itk[2] = femur_pt[2];

    labels->TransformPhysicalPointToIndex(femur_pt_itk, femur_idx);
    femur_label = labels->GetPixel(femur_idx);
    dout() << "determined to be " << static_cast<int>(femur_label) << std::endl;
  }

  if (need_to_find_contra_femur_label)
  {
    dout() << "looking up non-operative femur label..." << std::endl;

    const Pt3 contra_femur_pt = app_pts[is_left ? "FH-r" : "FH-l"];

    ITKIndex femur_idx;
    ITKPoint femur_pt_itk;
    femur_pt_itk[0] = contra_femur_pt[0];
    femur_pt_itk[1] = contra_femur_pt[1];
    femur_pt_itk[2] = contra_femur_pt[2];

    labels->TransformPhysicalPointToIndex(femur_pt_itk, femur_idx);
    con_femur_label = labels->GetPixel(femur_idx);
    dout() << "determined to be " << static_cast<int>(con_femur_label) << std::endl;
  }

  dout() << "Labels:\n   Pelvis: " << static_cast<int>(pelvis_label)
         << "\n     Frag: " << static_cast<int>(frag_label)
         << "\n    Femur: " << static_cast<int>(femur_label)
         << "\n  C-Femur: " << static_cast<int>(con_femur_label)
         << std::endl;
  
  const bool has_second_frag = bool(delta_sec_frag);

  const double frag_alpha = has_second_frag ? (do_frag_alpha ? 0.5 : 1.0) : 1.0;

  const bool draw_cut_planes = bool(cuts);

  if (draw_cut_planes)
  {
    cut_defs      = std::get<0>(*cuts);
    cut_defs_disp = std::get<1>(*cuts);
  
    dout() << "transforming planes to volume coordinates..." << std::endl; 
    TransformPAOCuts(app_orig_to_vol, &cut_defs, &cut_defs_disp);
  }

  VTKCreateMesh create_mesh;
  create_mesh.labels.resize(1);

  if (!pelvis_mesh_arg)
  {
    dout() << "creating pelvis mesh..." << std::endl;
    create_mesh.labels[0] = pelvis_label;
    pelvis_mesh = create_mesh(labels.GetPointer());
  }
  else
  {
    pelvis_mesh = *pelvis_mesh_arg;
  }
  
  VTK3DPlotter plotter;

  if (bg_color_arg)
  {
    plotter.set_bg_color(*bg_color_arg);
  }

  dout() << "plotting pelvis surface..." << std::endl;
  plotter.add_mesh(pelvis_mesh,
                   pelvis_color[0], pelvis_color[1], pelvis_color[2]);

  if (!no_draw_femurs)
  {
    if (!femur_mesh_arg)
    {
      dout() << "creating femur mesh..." << std::endl;
      create_mesh.labels[0] = femur_label;
      femur_mesh = create_mesh(labels.GetPointer());
    }
    else
    {
      femur_mesh = *femur_mesh_arg;
    }

    if (show_contra_femur)
    {
      if (!contra_femur_mesh_arg)
      {
        dout() << "creating contra-lateral femur mesh..." << std::endl;
        create_mesh.labels[0] = con_femur_label;
        contra_femur_mesh = create_mesh(labels.GetPointer());
      }
      else
      {
        contra_femur_mesh = *contra_femur_mesh_arg;
      }

      dout() << "plotting contra-lateral femur surface..." << std::endl;
      plotter.add_mesh(contra_femur_mesh,
                       contra_femur_color[0], contra_femur_color[1], contra_femur_color[2]);
    }
  
    dout() << "transforming femur surface..." << std::endl;
    TriMesh femur_mesh_to_disp = femur_mesh;
    //  This is a transformation from original volume coordinates to a new 
    //  volume coordinate frame
    // (This is important to keep straight for later ray casting).
    const FrameTransform delta_femur_vol = app_to_vol *
                                           (femur_pose_rel_to_frag ?
                                                      delta_Frag_and_Femur :
                                                      FrameTransform::Identity()) *
                                           delta_Femur_only * vol_to_app;
    ApplyTransform(delta_femur_vol, femur_mesh_to_disp.vertices, &femur_mesh_to_disp.vertices);
    femur_mesh_to_disp.normals_valid = false;

    dout() << "plotting femur surface..." << std::endl;
    plotter.add_mesh(femur_mesh_to_disp, femur_color[0], femur_color[1], femur_color[2]);
  }

  if (!no_draw_frag)
  {
    if (!frag_mesh_arg)
    {
      dout() << "creating fragment mesh..." << std::endl;
      create_mesh.labels[0] = frag_label;
      frag_mesh = create_mesh(labels.GetPointer());
    }
    else
    {
      frag_mesh = *frag_mesh_arg;
    }

    if (has_second_frag)
    {
      sec_frag_mesh = frag_mesh;
    
      dout() << "transforming second fragment surface..." << std::endl;
      //  This is a transformation from original volume coordinates to a new volume coordinate frame
      // (This is important to keep straight for later ray casting).
      const FrameTransform delta_frag_vol = app_to_vol * *delta_sec_frag * vol_to_app ;
      ApplyTransform(delta_frag_vol, sec_frag_mesh.vertices, &sec_frag_mesh.vertices);
      sec_frag_mesh.normals_valid = false;

      dout() << "plotting fragment surface..." << std::endl;
      plotter.add_mesh(sec_frag_mesh,
                       sec_frag_color[0], sec_frag_color[1], sec_frag_color[2],
                       frag_alpha);
    }

    dout() << "transforming fragment surface..." << std::endl;
    TriMesh frag_mesh_to_disp = frag_mesh;
    //  This is a transformation from original volume coordinates to a new volume coordinate frame
    // (This is important to keep straight for later ray casting).
    const FrameTransform delta_frag_vol = app_to_vol * delta_Frag_and_Femur * vol_to_app ;
    ApplyTransform(delta_frag_vol, frag_mesh_to_disp.vertices, &frag_mesh_to_disp.vertices);
    frag_mesh_to_disp.normals_valid = false;

    dout() << "plotting fragment surface..." << std::endl;
    plotter.add_mesh(frag_mesh_to_disp,
                     frag_color[0], frag_color[1], frag_color[2], frag_alpha);
  }
  else
  {
    dout() << "Not drawing the fragment!" << std::endl;
  }

  plotter.set_show_axes(show_axes);

  plotter.set_title(win_title);

  FrameTransform ap_pose;
  FrameTransform oblique_pose;
  FrameTransform lat_pose;
   
  // NOTE: The plotter expects the pose to be consistent with proj. axes
  // that orient the source to detector direction moving from the
  // focal point/X-ray source towards the detector

  if (use_vol_frame)
  {
    ap_pose = CreateAPViewOfLPSVol(800, femur_pt, false, true,
                                   CameraModel::kORIGIN_AT_FOCAL_PT_DET_POS_Z);
  
    const FrameTransform move_to_femur_pt = EulerRotXYZTransXYZFrame(
                                 0, 0, 0, -femur_pt(0), -femur_pt(1), -femur_pt(2));

    oblique_pose = move_to_femur_pt.inverse()
                               * EulerRotZFrame((is_left ? 1.0 : -1.0) * 45.0 * kDEG2RAD)
                               * move_to_femur_pt
                               * ap_pose;

    lat_pose = move_to_femur_pt.inverse()
                               * EulerRotZFrame((is_left ? 1.0 : -1.0) * 90.0 * kDEG2RAD)
                               * move_to_femur_pt
                               * ap_pose;
  }
  else
  {
    // This is the default branch
    CameraModel cam;
    cam.coord_frame_type = CameraModel::kORIGIN_AT_FOCAL_PT_DET_POS_Z;
    cam.setup(1200, 768, 768, 0.388, 0.388); 
    
    ap_pose = app_to_vol * CreateAPViewOfAPP(cam, 0.5, false);

    oblique_pose = app_to_vol *
          EulerRotYFrame((is_left ? 1.0 : -1.0) * 45.0 * kDEG2RAD) *
                    CreateAPViewOfAPP(cam, 0.5, false);

    lat_pose = app_to_vol *
          EulerRotYFrame((is_left ? 1.0 : -1.0) * 90.0 * kDEG2RAD) *
                    CreateAPViewOfAPP(cam, 0.5, false);
  }

  if (draw_cut_planes)
  {
    VTK3DPlotter::RGBAVec cut_planes_color;
    cut_planes_color[0] = 0;
    cut_planes_color[1] = 1;
    cut_planes_color[2] = 0;
    cut_planes_color[3] = 0.25;
    DrawPAOCutPlanes(cut_defs, cut_defs_disp, plotter,cut_planes_color);
  }
  
  if (other_pts)
  {
    for (const auto& pt : *other_pts)
    {
      plotter.add_sphere(pt, 5, 1, 0, 0);
    }
  }
  
  switch (cam_view)
  {
  case kAP:
    plotter.set_cam_pose(ap_pose);
    break;
  case kOBLIQUE:
    plotter.set_cam_pose(oblique_pose);
    break;
  case kLAT:
    plotter.set_cam_pose(lat_pose);
    break;
  case kVTK:
  default:
    // use default VTK pose
    break;
  }

  if (!png_path.empty())
  {
    xregASSERT(!render_to_ocv_mat);

    dout() << "rendering to PNG file..." << std::endl;
    plotter.draw_to_png(png_path);
  }
  else if (render_to_ocv_mat)
  {
    dout() << "render to opencv frame..." << std::endl;
    rendered_frame = plotter.draw_to_ocv();
  }
  else
  {
    dout() << "rendering to display..." << std::endl;
    plotter.show();
  }
  
  dout() << "finishing..." << std::endl; 
}
  
void xreg::DrawPAOBones::swap_cached_into_args()
{
  pelvis_label_arg       = pelvis_label;
  frag_label_arg         = frag_label;
  femur_label_arg        = femur_label;
  contra_femur_label_arg = con_femur_label;

  pelvis_mesh_arg       = pelvis_mesh;
  frag_mesh_arg         = frag_mesh;
  femur_label_arg       = femur_label;
  contra_femur_mesh_arg = contra_femur_mesh;
}

