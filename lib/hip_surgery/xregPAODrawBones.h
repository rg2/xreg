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

#ifndef XREGPAODRAWBONES_H_
#define XREGPAODRAWBONES_H_

#include <boost/optional.hpp>

#include "xregVTK3DPlotter.h"
#include "xregPAOCuts.h"
#include "xregObjWithOStream.h"
#include "xregMesh.h"

namespace xreg
{

struct DrawPAOBones : ObjWithOStream
{
  using LabelType     = unsigned char;
  using LabelImage    = itk::Image<LabelType,3>;
  using LabelImagePtr = LabelImage::Pointer;
  using ITKPoint      = LabelImage::PointType;
  using ITKIndex      = LabelImage::IndexType;

  LabelImagePtr labels;

  LandMap3 app_pts; 

  enum Side
  {
    kLEFT,
    kRIGHT
  };

  Side side;

  std::string png_path;

  bool render_to_ocv_mat = false;

  boost::optional<unsigned char> pelvis_label_arg;

  boost::optional<unsigned char> frag_label_arg;

  boost::optional<unsigned char> femur_label_arg;

  boost::optional<unsigned char> contra_femur_label_arg;

  FrameTransform delta_Frag_and_Femur = FrameTransform::Identity();
  
  FrameTransform delta_Femur_only = FrameTransform::Identity();

  bool femur_pose_rel_to_frag = true;

  bool show_contra_femur = true;

  boost::optional<FrameTransform> delta_sec_frag;

  bool do_frag_alpha = false;

  bool show_axes = false;

  std::string win_title = "PAO";

  enum CamView
  {
    kAP,
    kOBLIQUE,
    kLAT,
    kVTK
  };

  CamView cam_view = kAP;

  bool use_vol_frame = false;

  boost::optional<std::tuple<PAOCutPlanes,PAOCutDispInfo>> cuts;
  
  boost::optional<Pt3List> other_pts;

  bool no_draw_frag = false;

  bool no_draw_femurs = false;

  boost::optional<TriMesh> pelvis_mesh_arg;
  boost::optional<TriMesh> frag_mesh_arg;
  boost::optional<TriMesh> femur_mesh_arg;
  boost::optional<TriMesh> contra_femur_mesh_arg;
  
  Pt3 pelvis_color = { 1.0, 0.9922, 0.890 };
  
  Pt3 frag_color = { 0.0, 1.0, 1.0 };

  Pt3 sec_frag_color = { 1.0, 0.0, 0.0 };

  Pt3 femur_color = { 0.0, 1.0, 0.0 };

  Pt3 contra_femur_color = { 1.0, 1.0, 0.0 };

  boost::optional<Pt3> bg_color_arg;

  // Start computed values, that may be useful to a user
  LabelType pelvis_label    = 0;
  LabelType frag_label      = 0;
  LabelType femur_label     = 0;
  LabelType con_femur_label = 0;

  PAOCutPlanes   cut_defs;
  PAOCutDispInfo cut_defs_disp;
  
  TriMesh pelvis_mesh;
  TriMesh frag_mesh;
  TriMesh femur_mesh;
  TriMesh contra_femur_mesh;
  TriMesh sec_frag_mesh;

  cv::Mat rendered_frame;

  // call this to run
  void operator()();

  void swap_cached_into_args();
};

}  // xreg

#endif

