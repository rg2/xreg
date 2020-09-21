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

#include "xregPAOCuts.h"

// For sampling random rotation axes on 2D sphere
#include <boost/random.hpp>

#include <vtkNew.h>
#include <vtkPlaneSource.h>
#include <vtkPoints.h>
#include <vtkDoubleArray.h>

#include <itkConnectedComponentImageFilter.h>
#include <itkLabelShapeKeepNObjectsImageFilter.h>

#include "xregAssert.h"
#include "xregITKBasicImageUtils.h"
#include "xregITKLabelUtils.h"
#include "xregRotUtils.h"

void xreg::DrawPAOCutPlanes(const PAOCutPlanes& cut_defs,
                            const PAOCutDispInfo& disp_info,
                            VTK3DPlotter& plotter,
                            const VTK3DPlotter::RGBAVec& color, 
                            const bool draw_normals,
                            const bool draw_implicit_planes)
{
  constexpr bool kADJUST_CUT_MIDPT = true;

  Pt3 ilium_midpt;
  Pt3 ischium_midpt;
  Pt3 pubis_midpt;
  Pt3 post_midpt;

  if (kADJUST_CUT_MIDPT)
  {
    ilium_midpt = FindClosestOnPlane(disp_info.orig_ilium_cut_plane_mid_pt,
                                     cut_defs.ilium);
    //std::cout << "Ilium: " << Norm(disp_info.orig_ilium_cut_plane_mid_pt - ilium_midpt) << std::endl;


    ischium_midpt = FindClosestOnPlane(disp_info.orig_ischium_cut_plane_mid_pt,
                                       cut_defs.ischium);
    //std::cout << "Ischium: " << Norm(disp_info.orig_ischium_cut_plane_mid_pt - ischium_midpt) << std::endl;

    pubis_midpt = FindClosestOnPlane(disp_info.orig_pubis_cut_plane_mid_pt,
                                     cut_defs.pubis);
    //std::cout << "Pubis: " << Norm(disp_info.orig_pubis_cut_plane_mid_pt - pubis_midpt) << std::endl;

    post_midpt = FindClosestOnPlane(disp_info.orig_post_cut_plane_mid_pt,
                                    cut_defs.post);
    //std::cout << "Post: " << Norm(disp_info.orig_post_cut_plane_mid_pt - post_midpt) << std::endl;
  }
  else
  {
    ilium_midpt   = disp_info.orig_ilium_cut_plane_mid_pt;
    ischium_midpt = disp_info.orig_ischium_cut_plane_mid_pt;
    pubis_midpt   = disp_info.orig_pubis_cut_plane_mid_pt;
    post_midpt    = disp_info.orig_post_cut_plane_mid_pt;
  }

  // ilium cut
  {
    vtkNew<vtkPlaneSource> cut_plane_src;
    // initially just set the size of the plane
    cut_plane_src->SetOrigin(0,0,0);
    cut_plane_src->SetPoint1(disp_info.ilium_cut_plane_width, 0, 0);
    cut_plane_src->SetPoint2(0, disp_info.ilium_cut_plane_width, 0);
    cut_plane_src->Update();

    // now reposition it
    cut_plane_src->SetCenter(ilium_midpt[0], ilium_midpt[1],
                             ilium_midpt[2]);
    cut_plane_src->SetNormal(cut_defs.ilium.normal[0],
                             cut_defs.ilium.normal[1],
                             cut_defs.ilium.normal[2]);
    cut_plane_src->Update();

    plotter.add_polydata(cut_plane_src->GetOutput(), color);
  }

  // ischium cut
  {
    vtkNew<vtkPlaneSource> cut_plane_src;
    // initially just set the size of the plane
    cut_plane_src->SetOrigin(0,0,0);
    cut_plane_src->SetPoint1(disp_info.ischium_cut_plane_width, 0, 0);
    cut_plane_src->SetPoint2(0, disp_info.ischium_cut_plane_width, 0);
    cut_plane_src->Update();

    // now reposition it
    cut_plane_src->SetCenter(ischium_midpt[0], ischium_midpt[1],
                             ischium_midpt[2]);
    cut_plane_src->SetNormal(cut_defs.ischium.normal[0],
                             cut_defs.ischium.normal[1],
                             cut_defs.ischium.normal[2]);
    cut_plane_src->Update();

    plotter.add_polydata(cut_plane_src->GetOutput(), color);
  }

  // pubis cut
  {
    vtkNew<vtkPlaneSource> cut_plane_src;
    // initially just set the size of the plane
    cut_plane_src->SetOrigin(0,0,0);
    cut_plane_src->SetPoint1(disp_info.pubis_cut_plane_width, 0, 0);
    cut_plane_src->SetPoint2(0, disp_info.pubis_cut_plane_width, 0);
    cut_plane_src->Update();

    // now reposition it
    cut_plane_src->SetCenter(pubis_midpt[0], pubis_midpt[1],
                             pubis_midpt[2]);
    cut_plane_src->SetNormal(cut_defs.pubis.normal[0],
                             cut_defs.pubis.normal[1],
                             cut_defs.pubis.normal[2]);
    cut_plane_src->Update();

    plotter.add_polydata(cut_plane_src->GetOutput(), color);
  }

  // posterior cut
  {
    vtkNew<vtkPlaneSource> cut_plane_src;
    // initially just set the size of the plane
    cut_plane_src->SetOrigin(0,0,0);
    cut_plane_src->SetPoint1(disp_info.post_cut_plane_width, 0, 0);
    cut_plane_src->SetPoint2(0, disp_info.post_cut_plane_width, 0);
    cut_plane_src->Update();

    // now reposition it
    cut_plane_src->SetCenter(post_midpt[0], post_midpt[1], post_midpt[2]);
    cut_plane_src->SetNormal(cut_defs.post.normal[0],
                             cut_defs.post.normal[1],
                             cut_defs.post.normal[2]);
    cut_plane_src->Update();

    plotter.add_polydata(cut_plane_src->GetOutput(), color);
  }

  if (draw_implicit_planes)
  {
    // midsagittal
    {
      vtkNew<vtkPlaneSource> cut_plane_src;
      // initially just set the size of the plane
      cut_plane_src->SetOrigin(0,0,0);
      cut_plane_src->SetPoint1(disp_info.midsagittal_plane_width, 0, 0);
      cut_plane_src->SetPoint2(0, disp_info.midsagittal_plane_width, 0);
      cut_plane_src->Update();

      // now reposition it
      cut_plane_src->SetCenter(disp_info.midsagittal_pt[0],
                               disp_info.midsagittal_pt[1],
                               disp_info.midsagittal_pt[2]);
      cut_plane_src->SetNormal(cut_defs.mid_sagittal.normal[0],
                               cut_defs.mid_sagittal.normal[1],
                               cut_defs.mid_sagittal.normal[2]);
      cut_plane_src->Update();

      plotter.add_polydata(cut_plane_src->GetOutput(), color);
    }

    // lateral
    {
      vtkNew<vtkPlaneSource> cut_plane_src;
      // initially just set the size of the plane
      cut_plane_src->SetOrigin(0,0,0);
      cut_plane_src->SetPoint1(disp_info.lateral_plane_width, 0, 0);
      cut_plane_src->SetPoint2(0, disp_info.lateral_plane_width, 0);
      cut_plane_src->Update();

      // now reposition it
      cut_plane_src->SetCenter(disp_info.lateral_pt[0], disp_info.lateral_pt[1],
                               disp_info.lateral_pt[2]);
      cut_plane_src->SetNormal(cut_defs.lateral.normal[0],
                               cut_defs.lateral.normal[1],
                               cut_defs.lateral.normal[2]);
      cut_plane_src->Update();

      plotter.add_polydata(cut_plane_src->GetOutput(), color);
    }
  }

  if (draw_normals)
  {
    plotter.add_arrow(ilium_midpt,
                      ilium_midpt + (15 * cut_defs.ilium.normal), 1, 0, 0);
    plotter.add_arrow(ischium_midpt,
                      ischium_midpt + (15 * cut_defs.ischium.normal), 0, 1, 0);
    plotter.add_arrow(pubis_midpt,
                      pubis_midpt + (15 * cut_defs.pubis.normal), 0, 0, 1);
    plotter.add_arrow(post_midpt,
                      post_midpt + (15 * cut_defs.post.normal), 0, 1, 1);

    if (draw_implicit_planes)
    {
      plotter.add_arrow(disp_info.midsagittal_pt,
                      disp_info.midsagittal_pt + (15 * cut_defs.mid_sagittal.normal),
                      1, 1, 1);

      plotter.add_arrow(disp_info.lateral_pt,
                        disp_info.lateral_pt + (15 * cut_defs.lateral.normal),
                        0, 0, 0);
    }
  }
}

void xreg::TransformPAOCuts(const FrameTransform& xform,
                            PAOCutPlanes* cut_defs,
                            PAOCutDispInfo* disp_info)
{
  if (cut_defs)
  {
    cut_defs->ilium   = TransformPlane(cut_defs->ilium, xform);
    cut_defs->ischium = TransformPlane(cut_defs->ischium, xform);
    cut_defs->pubis   = TransformPlane(cut_defs->pubis, xform);
    cut_defs->post    = TransformPlane(cut_defs->post, xform);

    cut_defs->mid_sagittal = TransformPlane(cut_defs->mid_sagittal, xform);
    cut_defs->lateral      = TransformPlane(cut_defs->lateral, xform);
  }

  if (disp_info)
  {
    disp_info->orig_ilium_cut_plane_mid_pt =
                                  xform * disp_info->orig_ilium_cut_plane_mid_pt;
    disp_info->orig_ischium_cut_plane_mid_pt =
                                  xform * disp_info->orig_ischium_cut_plane_mid_pt;
    disp_info->orig_pubis_cut_plane_mid_pt =
                                  xform * disp_info->orig_pubis_cut_plane_mid_pt;
    disp_info->orig_post_cut_plane_mid_pt =
                                  xform * disp_info->orig_post_cut_plane_mid_pt; 
  }
}

namespace
{

using namespace xreg;

struct ComputeCutPlaneFn
{
  Pt3 start_cut_pt;
  Pt3 end_cut_pt;

  unsigned long dim_opp_of_cut_dir;

  Pt3 cut_plane_pt;
  Pt3 cut_plane_normal;
  Pt3 cut_plane_along_cut_vec;
  Pt3 cut_plane_opp_along_cut_vec;
  
  CoordScalar cut_len;

  CoordScalar plane_width_disp;

  bool flip_normal;

  CoordScalar extend_dist_for_disp;

  int cut_vec_only_keep_dim;

  void operator()()
  {
    cut_plane_along_cut_vec = end_cut_pt - start_cut_pt;
    if (cut_vec_only_keep_dim >= 0)
    {
      for (int i = 0; i < 3; ++i)
      {
        if (i != cut_vec_only_keep_dim)
        {
          cut_plane_along_cut_vec[i] = 0;
        }
      }
    }
    
    cut_plane_along_cut_vec[dim_opp_of_cut_dir] = 0;

    cut_len = cut_plane_along_cut_vec.norm();

    cut_plane_pt = start_cut_pt + (CoordScalar(0.5) * cut_plane_along_cut_vec);

    // This is the y-axis unit vector of the cut slab with respect to the current
    // coordinate frame.
    cut_plane_along_cut_vec /= cut_len;

    // This is the x-axis unit vector of the cut slab with respect to the current
    // coordinate frame.
    cut_plane_opp_along_cut_vec = Pt3::Zero();
    cut_plane_opp_along_cut_vec[dim_opp_of_cut_dir] = 1;

    // This is the z-axis unit vector of the cut slab with respect to the current
    // coordinate frame
    cut_plane_normal = cut_plane_along_cut_vec.cross(cut_plane_opp_along_cut_vec) *
                                   (flip_normal ? CoordScalar(-1) : CoordScalar(1));

    xregASSERT(std::abs(cut_plane_normal.dot(cut_plane_along_cut_vec)) < 1.0e-4);
    xregASSERT(std::abs(cut_plane_normal.dot(cut_plane_opp_along_cut_vec)) < 1.0e-4);
    
    plane_width_disp = (2 * extend_dist_for_disp * cut_len) + cut_len;
  }
};

}  // un-named

void xreg::CreatePAOCutPlanesFromLandmarksFn::operator()()
{
  const bool is_left_side = ilium_ant_app[0] > 0;
 
  // check consistency

  xregASSERT((is_left_side && (ilium_post_app[0] > 0)) ||
             (!is_left_side && (ilium_post_app[0] < 0)));      
  
  xregASSERT((is_left_side && (ischium_post_app[0] > 0)) ||
             (!is_left_side && (ischium_post_app[0] < 0))); 
  
  xregASSERT((is_left_side && (ischium_ant_app[0] > 0)) ||
             (!is_left_side && (ischium_ant_app[0] < 0)));
  
  xregASSERT((is_left_side && (pubis_sup_app[0] > 0)) ||
             (!is_left_side && (pubis_sup_app[0] < 0))); 
  
  xregASSERT((is_left_side && (pubis_inf_app[0] > 0)) ||
             (!is_left_side && (pubis_inf_app[0] < 0)));

  //////////////////////////////////////////////////////////////////////////////
  // First compute the cutting planes

  ComputeCutPlaneFn compute_cut_plane;
  
  // Ilium/Roof cutting plane
  compute_cut_plane.start_cut_pt = ilium_ant_app;
  compute_cut_plane.end_cut_pt   = ilium_post_app;
  compute_cut_plane.dim_opp_of_cut_dir = 0;

  compute_cut_plane.flip_normal = true;
  compute_cut_plane.extend_dist_for_disp = 0.5;
  compute_cut_plane.cut_vec_only_keep_dim = -1;
  
  compute_cut_plane();
 
  const Pt3         ilium_cut_plane_pt = compute_cut_plane.cut_plane_pt;
  const CoordScalar ilium_disp_width   = compute_cut_plane.plane_width_disp;

  cut_defs.ilium.normal = compute_cut_plane.cut_plane_normal;
  const Pt3 ilium_along_cut_vec     = compute_cut_plane.cut_plane_along_cut_vec;
  const Pt3 ilium_opp_along_cut_vec = compute_cut_plane.cut_plane_opp_along_cut_vec;
  const CoordScalar ilium_cut_len   = compute_cut_plane.cut_len;

  cut_defs.ilium.scalar = ilium_cut_plane_pt.dot(cut_defs.ilium.normal);

  // Ischium cutting plane
  compute_cut_plane.start_cut_pt = ischium_ant_app;
  compute_cut_plane.end_cut_pt   = ischium_post_app;
  compute_cut_plane.dim_opp_of_cut_dir = 0;

  compute_cut_plane.flip_normal = false;
  compute_cut_plane.extend_dist_for_disp = 0.75;
  compute_cut_plane.cut_vec_only_keep_dim = -1;

  compute_cut_plane();  

  Pt3         ischium_cut_plane_pt = compute_cut_plane.cut_plane_pt;
  CoordScalar ischium_disp_width   = compute_cut_plane.plane_width_disp;
  
  cut_defs.ischium.normal = compute_cut_plane.cut_plane_normal;
  const Pt3 ischium_along_cut_vec     = compute_cut_plane.cut_plane_along_cut_vec;
  const Pt3 ischium_opp_along_cut_vec = compute_cut_plane.cut_plane_opp_along_cut_vec;
  const CoordScalar ischium_cut_len = compute_cut_plane.cut_len;

  cut_defs.ischium.scalar = ischium_cut_plane_pt.dot(cut_defs.ischium.normal);

  // Pubis cut plane
  compute_cut_plane.start_cut_pt = pubis_sup_app;
  compute_cut_plane.end_cut_pt   = pubis_inf_app;
  compute_cut_plane.dim_opp_of_cut_dir = 2;

  compute_cut_plane.flip_normal = !is_left_side;
  compute_cut_plane.extend_dist_for_disp = 1;
  compute_cut_plane.cut_vec_only_keep_dim = 1;

  compute_cut_plane();  
  
  Pt3  pubis_cut_plane_pt = compute_cut_plane.cut_plane_pt;
  CoordScalar pubis_disp_width   = compute_cut_plane.plane_width_disp;
  
  cut_defs.pubis.normal = compute_cut_plane.cut_plane_normal;
  const Pt3 pubis_along_cut_vec     = compute_cut_plane.cut_plane_along_cut_vec;
  const Pt3 pubis_opp_along_cut_vec = compute_cut_plane.cut_plane_opp_along_cut_vec;
  const CoordScalar pubis_cut_len = compute_cut_plane.cut_len;
  
  cut_defs.pubis.scalar = pubis_cut_plane_pt.dot(cut_defs.pubis.normal);

  // Posterior cut/break:
  
  // The posterior cut/break is defined by the landmarks of the other cuts
  compute_cut_plane.start_cut_pt = ischium_post_app;
  compute_cut_plane.end_cut_pt   = ilium_post_app;
  compute_cut_plane.dim_opp_of_cut_dir = 0;

  compute_cut_plane.flip_normal = false;
  compute_cut_plane.extend_dist_for_disp = 0.5;
  compute_cut_plane.cut_vec_only_keep_dim = -1;

  compute_cut_plane();  

  Pt3  post_cut_plane_pt = compute_cut_plane.cut_plane_pt;
  CoordScalar post_disp_width   = compute_cut_plane.plane_width_disp;

  cut_defs.post.normal = compute_cut_plane.cut_plane_normal;
  const Pt3 post_along_cut_vec     = compute_cut_plane.cut_plane_along_cut_vec;
  const Pt3 post_opp_along_cut_vec = compute_cut_plane.cut_plane_opp_along_cut_vec;
  const CoordScalar post_cut_len = compute_cut_plane.cut_len;

  cut_defs.post.scalar = post_cut_plane_pt.dot(cut_defs.post.normal);

  // mid-sagittal bounding plane
  // if the fragment is on the left, than we want the normal to point towards
  // the right and to the left when the fragment is on the right.
  cut_defs.mid_sagittal.normal(0) = is_left_side ? -1 : 1;
  cut_defs.mid_sagittal.normal(1) = 0;
  cut_defs.mid_sagittal.normal(2) = 0;

  cut_defs.mid_sagittal.scalar = 0;

  // lateral bounding plane - assume label map is LPS, so just take image
  // width as the extreme
  const Pt3 img_phys_size = ITKImagePhysicalExtentsAsEigen(src_labels);

  // if the fragment is on the left, than we want the normal to point towards
  // the left and to the right when the fragment is on the right.
  cut_defs.lateral.normal(0) = is_left_side ? 1 : -1;
  cut_defs.lateral.normal(1) = 0;
  cut_defs.lateral.normal(2) = 0;

  // The origin is around the mid-sagittal plane, so the image width (Left/Right)
  // should be outside the body (positive on left, negative on right)
  cut_defs.lateral.scalar = (is_left_side ? 1 : -1) * img_phys_size[0]
                                                    * cut_defs.lateral.normal(0);

  disp_info.ilium_cut_plane_width   = ilium_disp_width;
  disp_info.ischium_cut_plane_width = ischium_disp_width;
  disp_info.pubis_cut_plane_width   = pubis_disp_width;
  disp_info.post_cut_plane_width    = post_disp_width;

  disp_info.orig_ilium_cut_plane_mid_pt   = ilium_cut_plane_pt;
  disp_info.orig_ischium_cut_plane_mid_pt = ischium_cut_plane_pt;
  disp_info.orig_pubis_cut_plane_mid_pt   = pubis_cut_plane_pt;
  disp_info.orig_post_cut_plane_mid_pt    = post_cut_plane_pt;

  disp_info.midsagittal_pt(0) = 0;
  disp_info.midsagittal_pt(1) = 0;
  disp_info.midsagittal_pt(2) = 0;

  disp_info.lateral_pt(0) = (is_left_side ? 1 : -1) * img_phys_size[0];
  disp_info.lateral_pt(1) = 0;
  disp_info.lateral_pt(2) = 0;

  disp_info.midsagittal_plane_width = std::max(img_phys_size[1], img_phys_size[2]);
  disp_info.lateral_plane_width = disp_info.midsagittal_plane_width;

  //VTK3DPlotter plotter;
  //DrawPAOCutPlanes(cut_defs, disp_info, plotter, true, true);
  //plotter.show();

  // create cutting plane slabs
  if (create_slabs)
  {
    // Ilium/Roof Slab
    const CoordScalar ilium_chisel_width = ilium_cut_chisel_width_scale * chisel_width;
    const CoordScalar ilium_max_cut_len = ilium_cut_len * ilium_cut_max_len_scale;

    const Pt3 ilium_slab_move_from_mid_pt =
                            (cut_defs.ilium.normal * chisel_thickness * 0.5) + 
                            (ilium_along_cut_vec * ilium_max_cut_len * 0.5) +
                            (ilium_opp_along_cut_vec * ilium_chisel_width * 0.5);

    const Pt3 ilium_slab_fll_wrt_app = ilium_cut_plane_pt - 
                                                    ilium_slab_move_from_mid_pt; 

    cut_slabs.ilium_cut_to_world.matrix().setIdentity();
    cut_slabs.ilium_cut_to_world.matrix().block(0,0,3,1) = ilium_opp_along_cut_vec;
    cut_slabs.ilium_cut_to_world.matrix().block(0,1,3,1) = ilium_along_cut_vec;
    cut_slabs.ilium_cut_to_world.matrix().block(0,2,3,1) = cut_defs.ilium.normal;
    cut_slabs.ilium_cut_to_world.matrix().block(0,3,3,1) = ilium_slab_fll_wrt_app;
    xregASSERT(std::abs(cut_slabs.ilium_cut_to_world.matrix().determinant() - 1.0) < 1.0e-4);

    cut_slabs.ilium_cut.fll_pt = Pt3::Zero();
    cut_slabs.ilium_cut.bur_pt(0) = ilium_chisel_width;
    cut_slabs.ilium_cut.bur_pt(1) = ilium_max_cut_len;
    cut_slabs.ilium_cut.bur_pt(2) = chisel_thickness;
  
    // Ischium
    const CoordScalar ischium_chisel_width = ischium_cut_chisel_width_scale * chisel_width;
    const CoordScalar ischium_max_cut_len = ischium_cut_len * ischium_cut_max_len_scale;

    const Pt3 ischium_slab_move_from_mid_pt =
                            (cut_defs.ischium.normal * chisel_thickness * 0.5) - 
                            (ischium_along_cut_vec * ischium_max_cut_len * 0.5) +
                            (ischium_opp_along_cut_vec * ischium_chisel_width * 0.5);

    const Pt3 ischium_slab_fll_wrt_app = ischium_cut_plane_pt - 
                                                    ischium_slab_move_from_mid_pt; 

    cut_slabs.ischium_cut_to_world.matrix().setIdentity();
    cut_slabs.ischium_cut_to_world.matrix().block(0,0,3,1) = ischium_opp_along_cut_vec;
    cut_slabs.ischium_cut_to_world.matrix().block(0,1,3,1) = -ischium_along_cut_vec;
    cut_slabs.ischium_cut_to_world.matrix().block(0,2,3,1) = cut_defs.ischium.normal;
    cut_slabs.ischium_cut_to_world.matrix().block(0,3,3,1) = ischium_slab_fll_wrt_app;

    xregASSERT(std::abs(cut_slabs.ischium_cut_to_world.matrix().determinant() - 1.0) < 1.0e-4);

    cut_slabs.ischium_cut.fll_pt = Pt3::Zero();
    cut_slabs.ischium_cut.bur_pt(0) = ischium_chisel_width;
    cut_slabs.ischium_cut.bur_pt(1) = ischium_max_cut_len;
    cut_slabs.ischium_cut.bur_pt(2) = chisel_thickness;

    // Pubis
    const CoordScalar pubis_chisel_width = pubis_cut_chisel_width_scale * chisel_width;
    const CoordScalar pubis_max_cut_len = pubis_cut_len * pubis_cut_max_len_scale;

    const Pt3 pubis_slab_move_from_mid_pt =
                            (cut_defs.pubis.normal * chisel_thickness * 0.5) - 
                            (pubis_along_cut_vec * pubis_max_cut_len * 0.5) +
                            (pubis_opp_along_cut_vec * pubis_chisel_width * 0.5);

    const Pt3 pubis_slab_fll_wrt_app = pubis_cut_plane_pt - 
                                                    pubis_slab_move_from_mid_pt; 

    cut_slabs.pubis_cut_to_world.matrix().setIdentity();
    cut_slabs.pubis_cut_to_world.matrix().block(0,0,3,1) = (is_left_side ? 1 : -1) * pubis_opp_along_cut_vec;
    cut_slabs.pubis_cut_to_world.matrix().block(0,1,3,1) = -pubis_along_cut_vec;
    cut_slabs.pubis_cut_to_world.matrix().block(0,2,3,1) = cut_defs.pubis.normal;
    cut_slabs.pubis_cut_to_world.matrix().block(0,3,3,1) = pubis_slab_fll_wrt_app;

    xregASSERT(std::abs(cut_slabs.pubis_cut_to_world.matrix().determinant() - 1.0) < 1.0e-4);

    cut_slabs.pubis_cut.fll_pt = Pt3::Zero();
    cut_slabs.pubis_cut.bur_pt(0) = pubis_chisel_width;
    cut_slabs.pubis_cut.bur_pt(1) = pubis_max_cut_len;
    cut_slabs.pubis_cut.bur_pt(2) = chisel_thickness;
    
    // Posterior
    const CoordScalar post_chisel_width = post_cut_chisel_width_scale * chisel_width;
    const CoordScalar post_max_cut_len = post_cut_len * post_cut_max_len_scale;

    const Pt3 post_slab_move_from_mid_pt =
                            (cut_defs.post.normal * chisel_thickness * 0.5) - 
                            (post_along_cut_vec * post_max_cut_len * 0.5) +
                            (post_opp_along_cut_vec * post_chisel_width * 0.5);

    const Pt3 post_slab_fll_wrt_app = post_cut_plane_pt - 
                                                    post_slab_move_from_mid_pt; 

    cut_slabs.post_cut_to_world.matrix().setIdentity();
    cut_slabs.post_cut_to_world.matrix().block(0,0,3,1) = post_opp_along_cut_vec;
    cut_slabs.post_cut_to_world.matrix().block(0,1,3,1) = -post_along_cut_vec;
    cut_slabs.post_cut_to_world.matrix().block(0,2,3,1) = cut_defs.post.normal;
    cut_slabs.post_cut_to_world.matrix().block(0,3,3,1) = post_slab_fll_wrt_app;

    xregASSERT(std::abs(cut_slabs.post_cut_to_world.matrix().determinant() - 1.0) < 1.0e-4);

    cut_slabs.post_cut.fll_pt = Pt3::Zero();
    cut_slabs.post_cut.bur_pt(0) = post_chisel_width;
    cut_slabs.post_cut.bur_pt(1) = post_max_cut_len;
    cut_slabs.post_cut.bur_pt(2) = chisel_thickness;
  }
}

xreg::PAOCheckInsideCutPlanesFn::PAOCheckInsideCutPlanesFn(
                                            const PAOCutPlanes& cut_defs)
{
  //////////////////////////////////////////////////////////////////////////////
  // Build up the implicit function that defines the convex region of the fragment

  vtkNew<vtkPoints> plane_pts;
  plane_pts->SetNumberOfPoints(6);

  vtkNew<vtkDoubleArray> plane_normals;
  plane_normals->SetNumberOfComponents(3);
  plane_normals->SetNumberOfTuples(6);

  auto arbitrary_pt_on_plane = [] (const Plane3& pl)
  {
    constexpr CoordScalar kTOL = static_cast<CoordScalar>(1.0e-6);

    const auto& n = pl.normal;
    const auto& s = pl.scalar;

    Pt3 pt = Pt3::Zero();

    // s is non-zero
    if (std::abs(n[0]) > kTOL)
    {
      pt[0] = s / n[0];
    }
    else if (std::abs(n[1]) > kTOL)
    {
      pt[1] = s / n[1];
    }
    else if (std::abs(n[2]) > kTOL)
    {
      pt[2] = s / n[2];
    }
    // else normal vec is zero

    return pt;
  };

  // mid-sagittal bounding plane
  const Pt3 midsagittal_pt = arbitrary_pt_on_plane(cut_defs.mid_sagittal);

  plane_pts->SetPoint(0, midsagittal_pt(0), midsagittal_pt(1), midsagittal_pt(2));
  plane_normals->SetTuple3(0, cut_defs.mid_sagittal.normal(0),
                           cut_defs.mid_sagittal.normal(1),
                           cut_defs.mid_sagittal.normal(2));

  // lateral bounding plane
  const Pt3 lateral_pt = arbitrary_pt_on_plane(cut_defs.lateral);
  plane_pts->SetPoint(1, lateral_pt(0), lateral_pt(1), lateral_pt(2));
  plane_normals->SetTuple3(1, cut_defs.lateral.normal(0),
                           cut_defs.lateral.normal(1),
                           cut_defs.lateral.normal(2));

  const Pt3 ilium_cut_plane_pt   = arbitrary_pt_on_plane(cut_defs.ilium);
  const Pt3 ischium_cut_plane_pt = arbitrary_pt_on_plane(cut_defs.ischium);
  const Pt3 pubis_cut_plane_pt   = arbitrary_pt_on_plane(cut_defs.pubis);
  const Pt3 post_cut_plane_pt    = arbitrary_pt_on_plane(cut_defs.post);

  plane_pts->SetPoint(2, ilium_cut_plane_pt[0], ilium_cut_plane_pt[1],
                      ilium_cut_plane_pt[2]);
  plane_normals->SetTuple3(2, cut_defs.ilium.normal[0],
                           cut_defs.ilium.normal[1], cut_defs.ilium.normal[2]);

  plane_pts->SetPoint(3, ischium_cut_plane_pt[0], ischium_cut_plane_pt[1],
                      ischium_cut_plane_pt[2]);
  plane_normals->SetTuple3(3, cut_defs.ischium.normal[0],
                           cut_defs.ischium.normal[1],
                           cut_defs.ischium.normal[2]);

  plane_pts->SetPoint(4, pubis_cut_plane_pt[0], pubis_cut_plane_pt[1],
                      pubis_cut_plane_pt[2]);
  plane_normals->SetTuple3(4, cut_defs.pubis.normal[0],
                           cut_defs.pubis.normal[1],
                           cut_defs.pubis.normal[2]);

  plane_pts->SetPoint(5, post_cut_plane_pt[0], post_cut_plane_pt[1],
                      post_cut_plane_pt[2]);
  plane_normals->SetTuple3(5, cut_defs.post.normal[0], cut_defs.post.normal[1],
                           cut_defs.post.normal[2]);

  vtk_planes->SetPoints(plane_pts.GetPointer());
  vtk_planes->SetNormals(plane_normals.GetPointer());
}

bool xreg::PAOCheckInsideCutPlanesFn::inside(const Pt3& x)
{
  return vtk_planes->FunctionValue(x[0], x[1], x[2]) < 1.0e-6;
}

bool xreg::PAOCheckInsideCutPlanesFn::inside(const Pt3& x,
                                             const CoordScalar w)
{
  const CoordScalar d = static_cast<CoordScalar>(
                                 vtk_planes->FunctionValue(x[0], x[1], x[2]));

  return (d < 1.0e-6) && ((-w - d) > 1.0e-6);
}

xreg::CoordScalar
xreg::PAOCheckInsideCutPlanesFn::signed_dist(const Pt3& x)
{
  return static_cast<CoordScalar>(vtk_planes->FunctionValue(x[0], x[1], x[2]));
}

void xreg::CreatePAOFragLabelMapFromCutPlanesFn::operator()()
{
  using LabelImageIndex    = LabelVol::IndexType;
  using LabelImageIterator = itk::ImageRegionIteratorWithIndex<LabelVol>;

  if (create_labels_w_cuts)
  {
    pot_cut_labels = MakeITK3DVol<LabelType>(
                                      src_labels->GetLargestPossibleRegion(), 0);
  }

  PAOCheckInsideCutPlanesFn check_cuts_convex_hull(cut_defs);

  //////////////////////////////////////////////////////////////////////////////
  // Update the pelvis labels that are contained in the fragment region

  FrameTransform label_inds_to_label_phys = ITKImagePhysicalPointTransformsAsEigen(src_labels);

  const FrameTransform label_ind_to_app = app_to_vol.inverse()
                                            * label_inds_to_label_phys;

  Pt3 cur_pt_app;
  Pt3 cur_ind;

  LabelVolPtr init_frag_labels = ITKImageDeepCopy(src_labels);

  LabelImageIterator label_it(init_frag_labels,
                              init_frag_labels->GetLargestPossibleRegion());

  frag_inserted = false;

  for (label_it.GoToBegin(); !label_it.IsAtEnd(); ++label_it)
  {
    LabelType& cur_label = label_it.Value();

    if (cur_label == pelvis_label)
    {
      const LabelImageIndex& itk_idx = label_it.GetIndex();

      cur_ind[0] = itk_idx[0];
      cur_ind[1] = itk_idx[1];
      cur_ind[2] = itk_idx[2];

      cur_pt_app = label_ind_to_app * cur_ind;

      const CoordScalar d = check_cuts_convex_hull.signed_dist(cur_pt_app);

      if (d < 1.0e-6)
      {
        cur_label = frag_label;
        frag_inserted = true;
      
        if (create_labels_w_cuts && ((d + cut_width) > 1.0e-6))
        {
          // mark as potential cut
          pot_cut_labels->SetPixel(label_it.GetIndex(), 1);
        }
      }
    }
  }

  if (frag_inserted)
  {
    // check against bad region mask
    // Doing this before checking connected components, allows a little more tolerance on this mask,
    // so that it just needs to break up the invalid regions
    if (invalid_frag_locations)
    {
      xregASSERT(ImagesHaveSameCoords(init_frag_labels.GetPointer(), invalid_frag_locations.GetPointer()));

      LabelImageIterator invalid_it(invalid_frag_locations,
                                    invalid_frag_locations->GetLargestPossibleRegion());

      for (invalid_it.GoToBegin(); !invalid_it.IsAtEnd(); ++invalid_it)
      {
        if (invalid_it.Value())
        {
          const auto cur_idx = invalid_it.GetIndex();

          if (init_frag_labels->GetPixel(cur_idx) == frag_label)
          {
            init_frag_labels->SetPixel(cur_idx, pelvis_label);
          }
        }
      }
    }

    // disabling this path, as I think it is better to reject the cases were the pubis
    // cut plane is in the invalid region of the pubis  
#if 0
    // any fragment voxels that are adjacent to pelvis voxels are potential cut voxels
    // mainly need to do this if the invalid mask removes all cut voxels, we've seen
    // this in very rare cases with the pubis cut.
    // TODO: also, this only needs to be done if the cut width at least a voxel large
    LabelImageIterator label_it(init_frag_labels,
                                init_frag_labels->GetLargestPossibleRegion());

    const auto label_img_size = init_frag_labels->GetLargestPossibleRegion().GetSize();

    for (label_it.GoToBegin(); !label_it.IsAtEnd(); ++label_it)
    {
      if (label_it.Value() == frag_label)
      {
        const auto cur_idx = label_it.GetIndex();
        
        LabelImageIndex neighbor_idx;      

        bool set_as_pot_frag = false;

        for (int k : { -1, 0, 1 })
        {
          neighbor_idx[2] = cur_idx[2] + k;

          if ((neighbor_idx[2] >= 0) && (neighbor_idx[2] < label_img_size[2]))
          {
            for (int j : { -1, 0, 1 })
            {
              neighbor_idx[1] = cur_idx[1] + j;

              if ((neighbor_idx[1] >= 0) && (neighbor_idx[1] < label_img_size[1]))
              {
                for (int i : { -1, 0, 1 })
                {
                  // do not check the current voxel which we know is a fragment
                  if (k || j || i)
                  {
                    neighbor_idx[0] = cur_idx[0] + i;

                    if ((neighbor_idx[0] >= 0) && (neighbor_idx[0] < label_img_size[0]))
                    {
                      if (init_frag_labels->GetPixel(neighbor_idx) == pelvis_label)
                      {
                        set_as_pot_frag = true;
                      }
                    }
                  }  // end if (k || j || i)
                  
                  if (set_as_pot_frag)
                  {
                    break;
                  }
                }  // end for i
              }
                  
              if (set_as_pot_frag)
              {
                break;
              }
            }  // end for j
          }
              
          if (set_as_pot_frag)
          {
            break;
          }
        }  // end for k

        if (set_as_pot_frag)
        {
          // set to potential cut
          pot_cut_labels->SetPixel(label_it.GetIndex(), 1);
        }
      }  // end if frag_label
    }
#endif

    if (check_connected)
    {
      // choose the largest connected component that has the fragment label - sometimes
      // the planes are setup in such a way that parts of the inferior pubis get
      // set with the fragment label
      using FindConnCompsFilter        = itk::ConnectedComponentImageFilter<LabelVol,LabelVol>;
      using KeepLargestConnCompsFilter = itk::LabelShapeKeepNObjectsImageFilter<LabelVol>;

      // first remove all other labels besides the fragment
      LabelVolPtr only_frag_labels = ApplyMaskToITKImage(
                                                init_frag_labels.GetPointer(),
                                                init_frag_labels.GetPointer(),
                                                frag_label, LabelType(0));

      auto cc_filter = FindConnCompsFilter::New();
      cc_filter->SetInput(only_frag_labels);

      auto largest_cc_filter = KeepLargestConnCompsFilter::New();
      largest_cc_filter->SetInput(cc_filter->GetOutput());
      largest_cc_filter->SetBackgroundValue(0);
      largest_cc_filter->SetNumberOfObjects(1);
      largest_cc_filter->SetAttribute(
                       KeepLargestConnCompsFilter::LabelObjectType::NUMBER_OF_PIXELS);

      largest_cc_filter->Update();

      LabelVolPtr largest_cc_img = largest_cc_filter->GetOutput();

      // Now copy the original label map, and update with the largest connected
      // component mask
      frag_labels_no_cut = ITKImageDeepCopy(src_labels);

      LabelImageIterator label_it(largest_cc_img,
                                  largest_cc_img->GetLargestPossibleRegion());

      for (label_it.GoToBegin(); !label_it.IsAtEnd(); ++label_it)
      {
        if (label_it.Value())
        {
          // non-zero indicates it was part of the largest connected component
          frag_labels_no_cut->SetPixel(label_it.GetIndex(), frag_label);
        }
      }
    }
  }
  else
  {
    frag_labels_no_cut = init_frag_labels;
  }
 
  if (create_labels_w_cuts)
  {
    frag_labels_w_cut = ITKImageDeepCopy(frag_labels_no_cut.GetPointer());
        
    if (save_cut_pts)
    {
      cut_pts_wrt_vol.clear();
    }

    Pt3 cur_pt_vol;

    LabelImageIterator pot_cut_it(pot_cut_labels,
                                  pot_cut_labels->GetLargestPossibleRegion());

    for (pot_cut_it.GoToBegin(); !pot_cut_it.IsAtEnd(); ++pot_cut_it)
    {
      const LabelImageIndex& itk_idx = pot_cut_it.GetIndex();
      
      // If this is a potential cut, and it has not been changed from fragment
      // then mark it as cut, otherwise leave as pelvis.
      if (pot_cut_it.Value() &&
          (frag_labels_w_cut->GetPixel(itk_idx) == frag_label))
      {
        frag_labels_w_cut->SetPixel(itk_idx, cut_label);

        if (save_cut_pts)
        {
          cur_ind[0] = itk_idx[0];
          cur_ind[1] = itk_idx[1];
          cur_ind[2] = itk_idx[2];
          
          cur_pt_vol = label_inds_to_label_phys * cur_ind;

          cut_pts_wrt_vol.push_back(cur_pt_vol);
        }
      }
    }
  
    // double check to see if there are any fragment labels left.
    // This is a corner case where the cut is tangent to the bone and inserting
    // cut labels removes all of the fragment labels
    frag_inserted = false;

    LabelImageIterator label_it(frag_labels_w_cut,
                                frag_labels_w_cut->GetLargestPossibleRegion());

    for (label_it.GoToBegin(); !label_it.IsAtEnd(); ++label_it)
    {
      if (label_it.Value() == frag_label)
      {
        frag_inserted = true;
        break;
      }
    }
  }
}

itk::Image<unsigned char,3>::Pointer
xreg::CreatePAOFragmentLabelMapUsingLandmarks(const Pt3& ilium_ant_app,
                                              const Pt3& ilium_post_app,
                                              const Pt3& ischium_post_app,
                                              const Pt3& ischium_ant_app,
                                              const Pt3& pubis_sup_app,
                                              const Pt3& pubis_inf_app,
                                              const itk::Image<unsigned char,3>* src_labels,
                                              const unsigned char pelvis_label,
                                              const unsigned char frag_label,
                                              const FrameTransform& app_to_vol_phys,
                                              PAOCutPlanes* cut_defs_arg,
                                              PAOCutDispInfo* disp_info)
{
  CreatePAOCutPlanesFromLandmarksFn create_cuts = { ilium_ant_app,    ilium_post_app,
                                                    ischium_post_app, ischium_ant_app,
                                                    pubis_sup_app,    pubis_inf_app,
                                                    app_to_vol_phys,
                                                    src_labels };

  create_cuts.create_slabs = false;

  create_cuts();

  if (cut_defs_arg)
  {
    *cut_defs_arg = create_cuts.cut_defs;
  }

  if (disp_info)
  {
    *disp_info = create_cuts.disp_info;
  }

  CreatePAOFragLabelMapFromCutPlanesFn create_frag;
  create_frag.cut_defs = create_cuts.cut_defs;
  create_frag.src_labels = src_labels;
  create_frag.pelvis_label = pelvis_label;
  create_frag.frag_label = frag_label;
  create_frag.app_to_vol = app_to_vol_phys;
  create_frag.check_connected = true;

  create_frag();

  return create_frag.frag_labels_no_cut;
}

xreg::PAOSampleRandomCutPlaneAdjusts::PAOSampleRandomCutPlaneAdjusts()
{
  std::random_device rand_dev;
  rng_eng.seed(rand_dev());

  num_samples = 0;
}

void xreg::PAOSampleRandomCutPlaneAdjustsRotNormalTransMidPt::operator()()
{
  using Sphere2DUniDist = boost::random::uniform_on_sphere<CoordScalar>;

  auto& rng = this->rng_eng;

  NormalDist std_normal_dist(0,1);
  auto std_normal_rng = [&rng,&std_normal_dist] () { return std_normal_dist(rng); };

  UniDist uni_about_one_dist(-1,1);
  auto uni_about_one_rng = [&rng,&uni_about_one_dist] () { return uni_about_one_dist(rng); };

  Sphere2DUniDist sphere_2d_uni_dist(3);

  const Pt3* src_normals[4] = { &this->src_cut_defs.ilium.normal,
                                &this->src_cut_defs.ischium.normal,
                                &this->src_cut_defs.pubis.normal,
                                &this->src_cut_defs.post.normal }; 

  const CoordScalar* src_scalars[4] = { &this->src_cut_defs.ilium.scalar,
                                        &this->src_cut_defs.ischium.scalar,
                                        &this->src_cut_defs.pubis.scalar,
                                        &this->src_cut_defs.post.scalar };

  const Pt3* src_mid_pts[4] = { &this->src_disp_info.orig_ilium_cut_plane_mid_pt,
                                &this->src_disp_info.orig_ischium_cut_plane_mid_pt,
                                &this->src_disp_info.orig_pubis_cut_plane_mid_pt,
                                &this->src_disp_info.orig_post_cut_plane_mid_pt };

  const CoordScalar mean_rot_ang[4] = { ilium_sample_params.mean_rot_ang_deg   * kDEG2RAD,
                                        ischium_sample_params.mean_rot_ang_deg * kDEG2RAD,
                                        pubis_sample_params.mean_rot_ang_deg   * kDEG2RAD,
                                        post_sample_params.mean_rot_ang_deg    * kDEG2RAD };
 
  const CoordScalar std_dev_rot_ang[4] = { ilium_sample_params.std_dev_rot_ang_deg   * kDEG2RAD,
                                           ischium_sample_params.std_dev_rot_ang_deg * kDEG2RAD,
                                           pubis_sample_params.std_dev_rot_ang_deg   * kDEG2RAD,
                                           post_sample_params.std_dev_rot_ang_deg    * kDEG2RAD };
  
  const CoordScalar mean_trans[4] = { ilium_sample_params.mean_trans,
                                      ischium_sample_params.mean_trans,
                                      pubis_sample_params.mean_trans,
                                      post_sample_params.mean_trans };
  const CoordScalar std_dev_trans[4] = { ilium_sample_params.std_dev_trans,
                                         ischium_sample_params.std_dev_trans,
                                         pubis_sample_params.std_dev_trans,
                                         post_sample_params.std_dev_trans };

  const char* cut_names[4] = { "Ilium", "Ischium", "Pubis", "Post" };
 
  this->dst_cut_defs.clear();
  this->dst_cut_defs.reserve(this->num_samples);

  this->dst_cut_disp_infos.clear();
  this->dst_cut_disp_infos.reserve(this->num_samples);

  PAOCutPlanes   tmp_defs = this->src_cut_defs;
  PAOCutDispInfo tmp_disp = this->src_disp_info;

  Pt3* dst_normals[4] = { &tmp_defs.ilium.normal,
                          &tmp_defs.ischium.normal,
                          &tmp_defs.pubis.normal,
                          &tmp_defs.post.normal }; 

  CoordScalar* dst_scalars[4] = { &tmp_defs.ilium.scalar,
                                  &tmp_defs.ischium.scalar,
                                  &tmp_defs.pubis.scalar,
                                  &tmp_defs.post.scalar };

  Pt3* dst_mid_pts[4] = { &tmp_disp.orig_ilium_cut_plane_mid_pt,
                          &tmp_disp.orig_ischium_cut_plane_mid_pt,
                          &tmp_disp.orig_pubis_cut_plane_mid_pt,
                          &tmp_disp.orig_post_cut_plane_mid_pt };

  for (size_type sample_idx = 0; sample_idx < this->num_samples; ++sample_idx)
  {
    for (size_type cut_idx = 0; cut_idx < 4; ++cut_idx)
    {
      Pt3&         n = *dst_normals[cut_idx];
      CoordScalar& s = *dst_scalars[cut_idx];
      Pt3&         p = *dst_mid_pts[cut_idx];
     
      // initialize with original data
      n = *src_normals[cut_idx];
      s = *src_scalars[cut_idx];
      p = *src_mid_pts[cut_idx];

      CoordScalar rot_ang_rad = 0;
      CoordScalar trans_mag   = 0;

      if (use_normal)
      {
        rot_ang_rad = (std_dev_rot_ang[cut_idx] * std_normal_rng()) + mean_rot_ang[cut_idx];
        trans_mag   = (std_dev_trans[cut_idx] * std_normal_rng()) + mean_trans[cut_idx];
      }
      else
      {
        rot_ang_rad = (std_dev_rot_ang[cut_idx] * uni_about_one_rng()) + mean_rot_ang[cut_idx];
        trans_mag   = (std_dev_trans[cut_idx] * uni_about_one_rng()) + mean_trans[cut_idx];
      }

      auto rand_rot_axis = sphere_2d_uni_dist(this->rng_eng);

      Pt3 scaled_rot_axis;
      scaled_rot_axis[0] = rand_rot_axis[0];
      scaled_rot_axis[1] = rand_rot_axis[1];
      scaled_rot_axis[2] = rand_rot_axis[2];

      scaled_rot_axis *= rot_ang_rad;
      
      this->dout() << cut_names[cut_idx] << " deltas (sample #" << sample_idx << "):\n"
                   << "  rot angle (deg): " << (rot_ang_rad / kDEG2RAD) << '\n'
                   << "           trans.: " << trans_mag << std::endl;
      
      n = ExpSO3(scaled_rot_axis) * n;

      p += n * trans_mag;

      s = n.dot(p);
    }

    this->dst_cut_defs.push_back(tmp_defs);
    this->dst_cut_disp_infos.push_back(tmp_disp);
  }
}

void xreg::PAOSampleRandomCutPlaneAdjustsSpherical::operator()()
{
  NormalDist std_normal_dist(0,1);
 
  auto& rng = this->rng_eng;

  auto std_normal_rng = [&rng,&std_normal_dist] () { return std_normal_dist(rng); };

  const Plane3* src_planes[4] = { &this->src_cut_defs.ilium,
                                  &this->src_cut_defs.ischium,
                                  &this->src_cut_defs.pubis,
                                  &this->src_cut_defs.post };

  const Pt3* src_mid_pts[4] = { &this->src_disp_info.orig_ilium_cut_plane_mid_pt,
                                &this->src_disp_info.orig_ischium_cut_plane_mid_pt,
                                &this->src_disp_info.orig_pubis_cut_plane_mid_pt,
                                &this->src_disp_info.orig_post_cut_plane_mid_pt };

  const CoordScalar mean_rot_theta[4] = { ilium_sample_params.mean_theta_deg   * kDEG2RAD,
                                          ischium_sample_params.mean_theta_deg * kDEG2RAD,
                                          pubis_sample_params.mean_theta_deg   * kDEG2RAD,
                                          post_sample_params.mean_theta_deg    * kDEG2RAD };
 
  const CoordScalar std_dev_rot_theta[4] = {
                                 ilium_sample_params.std_dev_theta_deg   * kDEG2RAD,
                                 ischium_sample_params.std_dev_theta_deg * kDEG2RAD,
                                 pubis_sample_params.std_dev_theta_deg   * kDEG2RAD,
                                 post_sample_params.std_dev_theta_deg    * kDEG2RAD };
  
  const CoordScalar mean_rot_phi[4] = { ilium_sample_params.mean_phi_deg   * kDEG2RAD,
                                        ischium_sample_params.mean_phi_deg * kDEG2RAD,
                                        pubis_sample_params.mean_phi_deg   * kDEG2RAD,
                                        post_sample_params.mean_phi_deg    * kDEG2RAD };
 
  const CoordScalar std_dev_rot_phi[4] = { ilium_sample_params.std_dev_phi_deg   * kDEG2RAD,
                                           ischium_sample_params.std_dev_phi_deg * kDEG2RAD,
                                           pubis_sample_params.std_dev_phi_deg   * kDEG2RAD,
                                           post_sample_params.std_dev_phi_deg    * kDEG2RAD };

  const CoordScalar mean_r[4] = { ilium_sample_params.mean_r,
                                  ischium_sample_params.mean_r,
                                  pubis_sample_params.mean_r,
                                  post_sample_params.mean_r };
  const CoordScalar std_dev_r[4] = { ilium_sample_params.std_dev_r,
                                     ischium_sample_params.std_dev_r,
                                     pubis_sample_params.std_dev_r,
                                     post_sample_params.std_dev_r };

  const char* cut_names[4] = { "Ilium", "Ischium", "Pubis", "Post" };
 
  this->dst_cut_defs.clear();
  this->dst_cut_defs.reserve(this->num_samples);

  this->dst_cut_disp_infos.clear();
  this->dst_cut_disp_infos.reserve(this->num_samples);

  PAOCutPlanes   tmp_defs = this->src_cut_defs;
  PAOCutDispInfo tmp_disp = this->src_disp_info;

  Plane3* dst_planes[4] = { &tmp_defs.ilium,
                            &tmp_defs.ischium,
                            &tmp_defs.pubis,
                            &tmp_defs.post };

  Pt3* dst_mid_pts[4] = { &tmp_disp.orig_ilium_cut_plane_mid_pt,
                          &tmp_disp.orig_ischium_cut_plane_mid_pt,
                          &tmp_disp.orig_pubis_cut_plane_mid_pt,
                          &tmp_disp.orig_post_cut_plane_mid_pt };

  for (size_type sample_idx = 0; sample_idx < this->num_samples; ++sample_idx)
  {
    for (size_type cut_idx = 0; cut_idx < 4; ++cut_idx)
    {
      auto& dst_plane = *dst_planes[cut_idx];

      Pt3& p = *dst_mid_pts[cut_idx];
     
      // initialize with original data
      p = *src_mid_pts[cut_idx];

      dst_plane = TranslatePlane(*src_planes[cut_idx], -p);

      CoordScalar theta_rad = 0;
      CoordScalar phi_rad   = 0;
      CoordScalar r         = 0;
      std::tie(theta_rad,phi_rad,r) = PlaneNormalScalarToSpherical(dst_plane);

      const CoordScalar delta_theta = (std_dev_rot_theta[cut_idx] * std_normal_rng()) +
                                                              mean_rot_theta[cut_idx];
      const CoordScalar delta_phi = (std_dev_rot_phi[cut_idx] * std_normal_rng()) +
                                                              mean_rot_phi[cut_idx];
      const CoordScalar delta_r = (std_dev_r[cut_idx] * std_normal_rng()) +
                                                              mean_r[cut_idx];
      
      this->dout() << cut_names[cut_idx] << " deltas:\n"
                   << "  delta theta: " << (delta_theta / kDEG2RAD) << '\n'
                   << "    delta phi: " << (delta_phi / kDEG2RAD) << '\n'
                   << "      delta r: " << delta_r << std::endl;

      theta_rad += delta_theta;
      phi_rad   += delta_phi;
      r         += delta_r;

      dst_plane = PlaneSphericalToNormalScalar(theta_rad, phi_rad, r);
      dst_plane = TranslatePlane(dst_plane, p);

      const Plane3 tmp_plane = PlaneSphericalToNormalScalar(delta_theta, delta_phi, delta_r);
  
      // This has some issues (updated point not lying on updated plane)
      // with non-zero delta-r, but does not appear to cause
      // any inaccurate display issues    
      p += tmp_plane.normal * tmp_plane.scalar;
    }

    this->dst_cut_defs.push_back(tmp_defs);
    this->dst_cut_disp_infos.push_back(tmp_disp);
  }
}

