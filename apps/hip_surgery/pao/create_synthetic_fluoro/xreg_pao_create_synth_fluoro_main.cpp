/*
 * MIT License
 *
 * Copyright (c) 2020-2022 Robert Grupp
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

#include <fmt/format.h>

// xreg
#include "xregProgOptUtils.h"
#include "xregLandmarkFiles.h"
#include "xregITKIOUtils.h"
#include "xregLandmarkMapUtils.h"
#include "xregAnatCoordFrames.h"
#include "xregStringUtils.h"
#include "xregCIOSFusionDICOM.h"
#include "xregSampleUtils.h"
#include "xregSampleUniformUnitVecs.h"
#include "xregRotUtils.h"
#include "xregRigidUtils.h"
#include "xregH5ProjDataIO.h"
#include "xregRayCastProgOpts.h"
#include "xregRayCastInterface.h"
#include "xregHUToLinAtt.h"
#include "xregImageAddPoissonNoise.h"

int main(int argc, char* argv[])
{
  using namespace xreg;
  
  constexpr int kEXIT_VAL_SUCCESS = 0;
  constexpr int kEXIT_VAL_BAD_USE = 1;

  // First, set up the program options

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Given an input CT volume of the hip and landmarks for identifying the APP, "
              "this tool creates simulated fluoroscopy collections. Each collection begins "
              "with an approximate AP pose and two additional views collected at orbital "
              "rotations in the opposite direction. A volume which has been modified to "
              "represent an adjusted, and temporarily fixed, PAO fragment is the intended "
              "use case for this application. All views are randomly sampled. Poisson noise "
              "is also incorporated.");
  
  po.set_arg_usage("<Input CT vol.> <APP Landmarks> <side> <num collects> "
                   "<output proj. data prefix> <output pose prefix>");
  po.set_min_num_pos_args(6);

  po.add("lands-ras", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "lands-ras",
         "Read landmarks in RAS coordinates instead of LPS.")
    << false;

  po.add("lands", 'l', ProgOpts::kSTORE_STRING, "lands",
         "Path to a landmarks file containing 3D landmarks which should be projected into 2D and "
         "saved in the projection data.")
    << "";

  po.add("lr-off", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "lr-off",
         "Absolute value of the left/right (medial) offset (in mm) applied to bring more of the pelvis "
         "into the field of view.")
    << 25.0;

  po.add("is-off", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "is-off",
         "Value of the inferior/superior offset (in mm) applied to bring more of the pelvis "
         "into the field of view.")
    << 35.0;

  po.add("mean-orbit-rot-1", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "mean-orbit-rot-1",
         "Mean orbital rotation angle applied from the first view in order to obtain the second view. Degrees.")
    << -10.0;
  
  po.add("std-orbit-rot-1", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "std-orbit-rot-1",
         "Standard deviation of the orbital rotation angle applied from the first view "
         "in order to obtain the second view. Degrees.")
    << 3.0;

  po.add("mean-orbit-rot-2", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "mean-orbit-rot-2",
         "Mean orbital rotation angle applied from the first view in order to obtain the third view. Degrees.")
    << 15.0;
  
  po.add("std-orbit-rot-2", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "std-orbit-rot-2",
         "Standard deviation of the orbital rotation angle applied from the first view"
         "in order to obtain the third view. Degrees.")
    << 3.0;

  po.add("orbit-rot-perturb", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "orbit-rot-perturb",
         "Controls the amount of non-orbital rotation applied to views 2 and 3. The rotation axis is "
         "sampled uniformly and the angle is sampled uniformly from U(-X,+X), where X is the value "
         "specified here. Degrees.")
    << 2.0;

  po.add("orbit-trans-perturb", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "orbit-trans-perturb",
         "Controls the amount of translation perturbation applied to the orbital rotations of views 2 and 3. "
         "The direction axis is sampled uniformly and the magnitude is sampled uniformly from U(-Y,+Y), "
         "where Y is the value specified here (in mm).")
    << 2.0;

  po.add("ap-src-det-ratio", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "ap-src-det-ratio",
         "The approximate positioning parameter of the ipsilateral femoral head along the source-to-detector "
         "direction for the first (approximately) AP view. This should lie in [0,1], with 0 at the source "
         "and 1 at the detector.")
    << 0.8;

  po.add("vol-rot", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "vol-rot",
         "Controls the amount of rotation applied to the initial pose of the volume with respect to the "
         "three views. The rotation axis is sampled uniformly and the angle is uniformly sampled from "
         "U(-Z,+Z), where Z is the value specified here. Degrees. This is applied in the APP frame.")
    << 10.0;

  po.add("vol-trans", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "vol-trans",
         "Controls the amount of translation applied to the initial pose of the volume with respect to the "
         "three views. The direction is sampled uniformly and the magnitude is uniformly sampled from "
         "U(0,W), where W is the value specified here (in mm). This is applied in the APP frame.")
    << 10.0;

  po.add("num-photons", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32, "num-photons",
         "Number of photons to use when adding Poisson noise.")
    << ProgOpts::uint32(2000);

  po.add_backend_flags();

  try
  {
    po.parse(argc, argv);
  }
  catch (const ProgOpts::Exception& e)
  {
    std::cerr << "Error parsing command line arguments: " << e.what() << std::endl;
    po.print_usage(std::cerr);
    return kEXIT_VAL_BAD_USE;
  }

  if (po.help_set())
  {
    po.print_usage(std::cout);
    po.print_help(std::cout);
    return kEXIT_VAL_SUCCESS;
  }

  const bool verbose = po.get("verbose");
  std::ostream& vout = po.vout();
  
  const std::string src_intens_path         = po.pos_args()[0];
  const std::string app_fcsv_path           = po.pos_args()[1];
  const std::string side_str                = po.pos_args()[2];
  const size_type   num_collects            = StringCast<size_type>(po.pos_args()[3]);
  const std::string output_proj_data_prefix = po.pos_args()[4];
  const std::string output_pose_prefix      = po.pos_args()[5];

  const bool is_left = side_str == "left";

  if (!is_left && (side_str != "right"))
  {
    std::cerr << "ERROR: Invalid side string: " << side_str << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  vout << "Ipsilateral side is the " << side_str << " side." << std::endl;

  const bool lands_ras = po.get("lands-ras");

  const std::string fcsv_to_proj = po.get("lands");

  const double lr_off = po.get("lr-off");
  const double is_off = po.get("is-off");

  const double mean_orbit_rot_1_deg = po.get("mean-orbit-rot-1");
  const double std_orbit_rot_1_deg  = po.get("std-orbit-rot-1");
  const double mean_orbit_rot_2_deg = po.get("mean-orbit-rot-2");
  const double std_orbit_rot_2_deg  = po.get("std-orbit-rot-2");

  const double orbit_rot_small_perturb_deg = po.get("orbit-rot-perturb");
  const double orbit_trans_small_perturb = po.get("orbit-trans-perturb");

  const double ap_src_det_ratio = po.get("ap-src-det-ratio");

  const double vol_rot_deg = po.get("vol-rot");
  const double vol_trans   = po.get("vol-trans");

  const size_type num_photons = po.get("num-photons").as_uint32();

  //////////////////////////////////////////////////////////////////////////////
  // Read in input intensity volume
  
  vout << "reading in source intensity volume..." << std::endl;
  auto src_intens = ReadITKImageFromDisk<RayCaster::Vol>(src_intens_path);

  vout << "converting HU --> Lin. Att." << std::endl;
  src_intens = HUToLinAtt(src_intens.GetPointer(), -130);

  //////////////////////////////////////////////////////////////////////////////
  // Get the landmarks

  vout << "reading APP landmarks..." << std::endl;
  const LandMap3 app_pts = ReadLandmarksFileNamePtMap(app_fcsv_path, !lands_ras);

  vout << "APP Landmarks:\n";
  PrintLandmarkMap(app_pts, vout);
  
  const Pt3 femur_pt = app_pts.find(fmt::format("FH-{}", side_str[0]))->second;
  
  FrameTransform app_to_vol = AnteriorPelvicPlaneFromLandmarksMap(app_pts,
                                      is_left ? kAPP_ORIGIN_LEFT_FH : kAPP_ORIGIN_RIGHT_FH);

  // shift the origin medial to get more of the pelvis in the FOV (x) 
  // shift the origin down a little so we can fit more of the pelvis in the FOV (y)
  app_to_vol = app_to_vol * EulerRotXYZTransXYZFrame(0, 0, 0, (is_left ? -1 : 1) * lr_off, is_off, 0);
  
  vout << "APP to Vol:\n" << app_to_vol.matrix() << std::endl;

  const FrameTransform vol_to_app = app_to_vol.inverse();

  LandMap3 lands_to_proj;

  if (!fcsv_to_proj.empty())
  {
    vout << "Reading landmarks to project..." << std::endl;

    lands_to_proj = ReadLandmarksFileNamePtMap(fcsv_to_proj, !lands_ras);

    vout << "Landmarks to project:\n";
    PrintLandmarkMap(lands_to_proj, vout);
  }

  const bool need_to_proj_lands = !lands_to_proj.empty();

  const auto default_cam = NaiveCamModelFromCIOSFusion(MakeNaiveCIOSFusionMetaDR(), true);

  vout << "setting up ray caster..." << std::endl;
  auto rc = LineIntRayCasterFromProgOpts(po);

  rc->set_camera_models(RayCaster::CameraModelList(3,default_cam));
  rc->set_num_projs(3);
  rc->set_volume(src_intens);

  vout << "  allocating resources..." << std::endl;
  rc->allocate_resources();

  std::mt19937 rng_eng;
  SeedRNGEngWithRandDev(&rng_eng);

  //std::uniform_real_distribution<CoordScalar> intersect_pt_dist(0.65,0.8);

  // for sampling random rotation axes and random translation directions
  UniformOnUnitSphereDist unit_vec_dist(3);

  // for sampling random rotation angles (in APP w/ origin at FH)
  std::uniform_real_distribution<CoordScalar> rot_ang_dist(-vol_rot_deg * kDEG2RAD,
                                                            vol_rot_deg * kDEG2RAD);

  // for sampling random translation magnitudes (in APP)
  std::uniform_real_distribution<CoordScalar> trans_mag_dist(0,vol_trans);

  // for sampling small perturbations of change in C-Arm view, so we do not have pure
  // orbital rotation
  std::uniform_real_distribution<CoordScalar> carm_small_rot_ang_dist(
                                                -orbit_rot_small_perturb_deg * kDEG2RAD,
                                                 orbit_rot_small_perturb_deg * kDEG2RAD);
  
  std::uniform_real_distribution<CoordScalar> carm_small_trans_mag_dist(
                                                -orbit_trans_small_perturb, orbit_trans_small_perturb);

  for (size_type collect_idx = 0; collect_idx < num_collects; ++collect_idx)
  {
    vout << "Collection: " << collect_idx << std::endl;
   
    vout << "  sampling collection geoms..." << std::endl;

    std::vector<CameraModel> cams(3);

    cams[0] = default_cam;

    // these are in degrees
    std::array<CoordScalar,2> mean_rot_angs    = { static_cast<CoordScalar>(mean_orbit_rot_1_deg),
                                                   static_cast<CoordScalar>(mean_orbit_rot_2_deg) };
    std::array<CoordScalar,2> std_dev_rot_angs = { static_cast<CoordScalar>(std_orbit_rot_1_deg),
                                                   static_cast<CoordScalar>(std_orbit_rot_2_deg) };
    
    for (size_type cam_idx = 1; cam_idx < 3; ++cam_idx)
    {
      vout << "    view " << (cam_idx + 1) << std::endl;

      const auto& ref_cam = cams[0];

      const CoordScalar cur_rot_ang_rad = std::normal_distribution<CoordScalar>(
          mean_rot_angs[cam_idx-1], std_dev_rot_angs[cam_idx-1])(rng_eng) * kDEG2RAD;
      
      FrameTransform carm_rand_perturb = FrameTransform::Identity();
      {
        Pt3 so3 = unit_vec_dist(rng_eng);
        so3 *= carm_small_rot_ang_dist(rng_eng);

        Pt3 trans = unit_vec_dist(rng_eng);
        trans *= carm_small_trans_mag_dist(rng_eng);

        carm_rand_perturb.matrix().block(0,0,3,3) = ExpSO3(so3);
        carm_rand_perturb.matrix().block(0,3,3,1) = trans;
      }

      const FrameTransform new_extrins = ref_cam.extrins
                                            * EulerRotXFrame(cur_rot_ang_rad)
                                            * carm_rand_perturb;

      auto& cur_cam = cams[cam_idx];

      cur_cam.coord_frame_type = ref_cam.coord_frame_type;

      cur_cam.setup(ref_cam.intrins,
                    new_extrins.matrix(),
                    ref_cam.num_det_rows, ref_cam.num_det_cols,
                    ref_cam.det_row_spacing, ref_cam.det_col_spacing);
    }

    vout << "  sampling ground truth pose of volume..." << std::endl;
      
    const FrameTransform ap_view_cam_wrt_app = CreateAPViewOfAPP(default_cam,
                                                  static_cast<CoordScalar>(ap_src_det_ratio), true, is_left);

    // add some noise in the APP coordinate frame
    Pt3 so3 = unit_vec_dist(rng_eng);
    so3 *= rot_ang_dist(rng_eng);

    FrameTransform delta_app = FrameTransform::Identity();
    
    delta_app.matrix().block(0,0,3,3) = ExpSO3(so3);

    Pt3 trans = unit_vec_dist(rng_eng);
    trans *= trans_mag_dist(rng_eng);

    delta_app.matrix().block(0,3,3,1) = trans;

    const FrameTransform gt_cam_wrt_vol = app_to_vol * delta_app * ap_view_cam_wrt_app;

    vout << "  writing ground truth pose to disk..." << std::endl;
    WriteITKAffineTransform(fmt::format("{}_{:03d}.h5", output_pose_prefix, collect_idx), gt_cam_wrt_vol);

    vout << "  updating ray caster params..." << std::endl;
    rc->set_camera_models(cams);
    rc->distribute_xform_among_cam_models(gt_cam_wrt_vol);

    vout << "  ray casting..." << std::endl;
    rc->compute();

    vout << "  adding poisson noise..." << std::endl;
    ProjDataU16List proj_data(3);

    for (size_type proj_idx = 0; proj_idx < 3; ++proj_idx)
    {
      vout << "    view " << proj_idx << "..." << std::endl;
      
      auto& dst_pd = proj_data[proj_idx];
      
      dst_pd.cam = cams[proj_idx];
      dst_pd.img = SamplePoissonProjFromAttProj(rc->proj(proj_idx).GetPointer(), num_photons);
      
      dst_pd.rot_to_pat_up = is_left ? ProjDataRotToPatUp::kZERO : ProjDataRotToPatUp::kONE_EIGHTY;

      if (need_to_proj_lands)
      {
        vout << "      projecting landmarks..." << std::endl;

        const FrameTransform gt_vol_wrt_cam = gt_cam_wrt_vol.inverse();

        auto& lands_2d = dst_pd.landmarks;
      
        for (const auto& land3 : lands_to_proj)
        {
          lands_2d.emplace(land3.first,
                           dst_pd.cam.phys_pt_to_ind_pt(gt_vol_wrt_cam * land3.second).head(2));
        }
      }
    }

    vout << "  writing proj. data to disk..." << std::endl;
    WriteProjDataH5ToDisk(proj_data, fmt::format("{}_{:03d}.h5", output_proj_data_prefix, collect_idx));
  }
 
  vout << "exiting..." << std::endl;
  
  return kEXIT_VAL_SUCCESS;
}
