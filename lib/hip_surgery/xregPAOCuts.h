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

#ifndef XREGPAOCUTS_H_
#define XREGPAOCUTS_H_

#include <random>

#include <vtkPlanes.h>

#include "xregSpatialPrimitives.h"
#include "xregVTK3DPlotter.h"
#include "xregObjWithOStream.h"

namespace xreg
{

/// \brief Defines PAO cutting planes.
///
/// All coordinates are with respect to the APP.
/// The normal vectors point outside (away from) the fragment region.
struct PAOCutPlanes
{
  Plane3 ilium;
  Plane3 ischium;
  Plane3 pubis;
  Plane3 post;

  // This are just bounding planes used to create a convex hull, not actually cuts!
  Plane3 mid_sagittal;
  Plane3 lateral;
};

/// \brief Container for drawing PAO cuts - useful for debug/presentation
struct PAOCutDispInfo
{
  CoordScalar ilium_cut_plane_width;
  CoordScalar ischium_cut_plane_width;
  CoordScalar pubis_cut_plane_width;
  CoordScalar post_cut_plane_width;

  Pt3 orig_ilium_cut_plane_mid_pt;
  Pt3 orig_ischium_cut_plane_mid_pt;
  Pt3 orig_pubis_cut_plane_mid_pt;
  Pt3 orig_post_cut_plane_mid_pt;

  // These never change
  Pt3 midsagittal_pt;
  Pt3 lateral_pt;

  CoordScalar midsagittal_plane_width;
  CoordScalar lateral_plane_width;
};

/// \brief Visualize the cutting planes
void DrawPAOCutPlanes(const PAOCutPlanes& cut_defs,
                      const PAOCutDispInfo& disp_info,
                      VTK3DPlotter& plotter,
                      const VTK3DPlotter::RGBAVec& color = VTK3DPlotter::DefaultMeshColorAlpha(), 
                      const bool draw_normals = false,
                      const bool draw_implicit_planes = false);

/// \brief Apply a rigid transform to PAO cutting planes and, optionally,
///        the display information.
void TransformPAOCuts(const FrameTransform& xform,
                      PAOCutPlanes* cut_defs,
                      PAOCutDispInfo* disp_info = nullptr);

/// \brief Representation of the PAO cuts as slabs with rigid positions with
///        respect to the original volume.
///
/// Each frame transformation maps points in the local slab coordinate frame to
/// some other frame.
/// slab front lower left corner is origin. back upper right corner
/// is (length x, length y, length z).
struct PAOCutSlabs
{
  AASlab3 ilium_cut;
  AASlab3 ischium_cut;
  AASlab3 pubis_cut;
  AASlab3 post_cut;

  // These are here, as the initial transformations are computed from the
  // process of initially creating the cuts.
  FrameTransform ilium_cut_to_world;
  FrameTransform ischium_cut_to_world;
  FrameTransform pubis_cut_to_world;
  FrameTransform post_cut_to_world;
};

/// \brief Given a collection of anatomical landmarks, create the appropriate PAO cutting planes.
///
/// The points should be specified in APP coordinates with default origin
/// (at the median of the left and right pubis points), so that it is easy to
/// automatically determine which side of the patient the fragment lies in.
/// See AnteriorPelvicPlaneFromLandmarks for the detailed definition of the APP.
struct CreatePAOCutPlanesFromLandmarksFn
{
  using LabelScalar = unsigned char;
  using LabelImage  = itk::Image<LabelScalar,3>;

  Pt3 ilium_ant_app;
  Pt3 ilium_post_app;
  Pt3 ischium_post_app;
  Pt3 ischium_ant_app;
  Pt3 pubis_sup_app;
  Pt3 pubis_inf_app;

  FrameTransform app_to_vol_phys;

  const LabelImage* src_labels;
                     
  PAOCutPlanes cut_defs;

  PAOCutDispInfo disp_info;

  bool create_slabs;

  CoordScalar chisel_width;
  CoordScalar chisel_thickness;

  // The slab z dimensions will be the distance between cut landmarks, scaled by
  // these (so each should be greater than 1 to allow longer cuts).
  CoordScalar ilium_cut_max_len_scale;
  CoordScalar ischium_cut_max_len_scale;
  CoordScalar pubis_cut_max_len_scale;
  CoordScalar post_cut_max_len_scale;

  // Scale factors along the chisel width dimension; this is used to account for
  // cuts (such as ilium) that a single chisel width cannot account for.
  CoordScalar ilium_cut_chisel_width_scale;
  CoordScalar ischium_cut_chisel_width_scale;
  CoordScalar pubis_cut_chisel_width_scale;
  CoordScalar post_cut_chisel_width_scale;

  PAOCutSlabs cut_slabs; 

  void operator()();
};

/// \brief Helper to compute whether points lie within the convex hull defined
///        defined by the PAO cutting planes.
///
/// All calculations are done in the cutting planes space, which is typically the
/// APP with traditional origin (e.g. NOT at the femoral head).
struct PAOCheckInsideCutPlanesFn
{
  /// \brief Sets up the VTK data structure for computing point in convex hull
  ///        computations. 
  explicit PAOCheckInsideCutPlanesFn(const PAOCutPlanes& cut_defs);

  /// \brief Returns true if the input point, x, lies within the convex hull
  ///        defined by the cutting planes.
  bool inside(const Pt3& x);

  /// \brief Returns true if the input point, x, lies within the convex hull
  ///        defined by the cutting planes and no further than a distance of w.
  ///
  /// This is useful for checking for cut voxels.
  bool inside(const Pt3& x, const CoordScalar w);

  /// \brief Returns the signed distance of the input point, x, to the closest
  ///        bounding plane.
  ///
  /// Negative indicates insided the convex hull, positive outside, zero on the
  /// boundary. The absolute value is the unsigned distance.
  CoordScalar signed_dist(const Pt3& x);

  vtkNew<vtkPlanes> vtk_planes;
};

/// \brief Given pre-determined cutting planes, create a label map replacing
///        the appropriate pelvis voxels with fragment voxels.
///
/// May also insert cut voxels
struct CreatePAOFragLabelMapFromCutPlanesFn
{
  using LabelType   = unsigned char;
  using LabelVol    = itk::Image<LabelType,3>;
  using LabelVolPtr = LabelVol::Pointer;

  PAOCutPlanes cut_defs;

  const LabelVol* src_labels;

  LabelType pelvis_label = 0;
  LabelType frag_label   = 0;
  LabelType cut_label    = 0;

  FrameTransform app_to_vol = FrameTransform::Identity();

  bool frag_inserted;

  bool check_connected = true;

  LabelVolPtr invalid_frag_locations;

  LabelVolPtr frag_labels_no_cut;

  bool create_labels_w_cuts = false;
  CoordScalar cut_width = CoordScalar(1);
  LabelVolPtr frag_labels_w_cut;

  bool save_cut_pts = false;
  Pt3List cut_pts_wrt_vol;

  // temporary storage for keeping track of locations that
  // may be cuts.
  LabelVolPtr pot_cut_labels;

  void operator()();
};

/// \brief Given a collection of anatomical landmarks, and an existing label
///        map, containing at least the pelvis, create a label map with the
///        fragment set as new label.
///
/// The points should be specified in APP coordinates with default origin
/// (at the median of the left and right pubis points), so that it is easy to
/// automatically determine which side of the patient the fragment lies in.
/// See AnteriorPelvicPlaneFromLandmarks for the detailed definition of the APP.
itk::Image<unsigned char,3>::Pointer
CreatePAOFragmentLabelMapUsingLandmarks(const Pt3& ilium_ant_app,
                                        const Pt3& ilium_post_app,
                                        const Pt3& ischium_post_app,
                                        const Pt3& ischium_ant_app,
                                        const Pt3& pubis_sup_app,
                                        const Pt3& pubis_inf_app,
                                        const itk::Image<unsigned char,3>* src_labels,
                                        const unsigned char pelvis_label,
                                        const unsigned char frag_label,
                                        const FrameTransform& app_to_vol_phys,
                                        PAOCutPlanes* cut_defs_arg = nullptr,
                                        PAOCutDispInfo* disp_info = nullptr);

struct PAOSampleRandomCutPlaneAdjusts : public ObjWithOStream
{
  using CutPlaneDefsList = std::vector<PAOCutPlanes>;
  using CutDispInfoList  = std::vector<PAOCutDispInfo>;
  
  using RNGEngine  = std::mt19937;
  using NormalDist = std::normal_distribution<CoordScalar>;
  using UniDist    = std::uniform_real_distribution<CoordScalar>;

  PAOSampleRandomCutPlaneAdjusts();

  virtual ~PAOSampleRandomCutPlaneAdjusts() = default;

  virtual void operator()() = 0;
  
  RNGEngine rng_eng;

  PAOCutPlanes src_cut_defs;

  PAOCutDispInfo src_disp_info;

  size_type num_samples;

  CutPlaneDefsList dst_cut_defs;

  CutDispInfoList  dst_cut_disp_infos;
};

/// \brief Given an existing set of cuts, apply random transformations to them.
///
/// This will apply a random rotation to normal vectors, move the cut mid-points
/// (e.g. the entire plane) along this new normal by a random amount.
/// When a uniform distribution is used, the mean is treated as the interval
/// center point and the end points are computed as center +/- 1 std. dev.
struct PAOSampleRandomCutPlaneAdjustsRotNormalTransMidPt
          : public PAOSampleRandomCutPlaneAdjusts
{
  bool use_normal = false;

  struct CutSampleParams
  {
    // see note above about how these are used in uniform dists
    CoordScalar mean_rot_ang_deg;
    CoordScalar std_dev_rot_ang_deg;
    CoordScalar mean_trans;
    CoordScalar std_dev_trans;
  };

  CutSampleParams ilium_sample_params   = { 0, 15, 0, 10.0 };
  CutSampleParams ischium_sample_params = { 0, 15, 0,  7.5 };
  CutSampleParams pubis_sample_params   = { 0, 30, 0, 10.0 };
  CutSampleParams post_sample_params    = { 0, 15, 0,  7.5 };

  void operator()() override;
};

/// \brief Given an existing set of cuts, apply random transformations to them.
///
/// This transforms each plane into spherical coordinates and adds random noise
/// to the spherical params and then transforms back into standard normal/scalar.
struct PAOSampleRandomCutPlaneAdjustsSpherical
          : public PAOSampleRandomCutPlaneAdjusts
{
  struct CutSampleParams
  {
    CoordScalar mean_theta_deg;
    CoordScalar std_dev_theta_deg;
    CoordScalar mean_phi_deg;
    CoordScalar std_dev_phi_deg;
    CoordScalar mean_r;
    CoordScalar std_dev_r;
  };

  CutSampleParams ilium_sample_params   = { 0, 12.5, 0, 7.5, 0, 2.0 };
  CutSampleParams ischium_sample_params = { 0, 12.5, 0, 5.0, 0, 1.0 };
  CutSampleParams pubis_sample_params   = { 0,  7.5, 0, 5.0, 0, 2.0 };
  CutSampleParams post_sample_params    = { 0, 12.5, 0, 5.0, 0, 2.0 };

  void operator()() override;
};

}  // xreg

#endif

