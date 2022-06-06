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
#include "xregFilesystemUtils.h"
#include "xregStringUtils.h"
#include "xregITKIOUtils.h"
#include "xregHipSegUtils.h"
#include "xregLandmarkMapUtils.h"
#include "xregAnatCoordFrames.h"
#include "xregPAOCuts.h"
#include "xregPointCloudUtils.h"
#include "xregVTKMeshUtils.h"
#include "xregTimer.h"
#include "xregPAOIO.h"

using namespace xreg;

int main(int argc, char* argv[])
{
  constexpr int kEXIT_VAL_SUCCESS       = 0;
  constexpr int kEXIT_VAL_BAD_USE       = 1;
  constexpr int kEXIT_VAL_BAD_CUTS_FILE = 2;

  // First, set up the program options

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Creates a PAO fragment from an existing segmentation and landmarks. "
              "The fragment output is an updated segmentation - e.g. Pelvis, Femur(s), Fragment, and Cut. "
              "The landmarks are assumed to be in the coordinate frame of the segmentation volume. "
              "To compute the APP, left and right Anterior Superior Iliac Spines (\"ASIS-{l,r}\") and "
              "left and right Inferior Pubis Symphisis (\"IPS-{l,r}\") are required. "
              "To compute the cutting planes the "
              "Anterior Ilium (\"AIl-{l,r}\"), "
              "Posterior Ilium (\"PIl-{l,r}\"), "
              "Posterior Ischium (\"PIs-{l,r}\"), "
              "Anterior Ischium (\"AIs-{l,r}\"), "
              "Superior Pubis (\"SP-{l,r}\"), "
              "Inferior Pubis (\"IP-{l,r}\") "
              "are required. "
              "The side string must be either \"left\" or \"right.\" "
              "The fragment segmentations with, and without, cuts are saved to disk, unless a \"-\" is passed in place of a path.");
  
  po.set_arg_usage("<Input Segmentation> <APP Landmarks> "
                   "<Cut Landmarks OR Cut Planes H5> <side> "
                   "<Output Fragment Segmentation without Cuts> "
                   "<Output Fragment Segmentation with Cuts> [<Cuts File>]");
  po.set_min_num_pos_args(6);

  po.add("lands-ras", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "lands-ras",
         "Read landmarks in RAS coordinates instead of LPS.")
    << false;

  po.add("cut-width", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "cut-width",
         "The width, in physical units of the volume, of the cut. A value of zero, "
         "indicates that there is no gap created by the chisel.")
    << 1.0;

  po.add("pelvis-label", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32, "pelvis-label",
         "The value used to indicate the pelvis in the input segmentation; if not "
         "provided, then the smallest positive value in the segmentation is used.");

  po.add("frag-label", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32, "frag-label",
         "The value used to indicate the fragment in the output segmentation; if not "
         "provided, then the N+1 is used, where N is the largest positive value in the input segmentation.");

  po.add("cut-label", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32, "cut-label",
         "The value used to indicate the cut in the output segmentation; if not "
         "provided, then the N+2 is used, where N is the largest positive value in the input segmentation.");

  po.add("invalid-mask", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "invalid-mask",
         "Labeling of pelvis voxels that are not allowed to be a fragment, providing this "
         "ignores any potential fragment voxels that are masked out (non-zero value).")
    << "";

  po.add("warn-no-cut-vox", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "warn-no-cut-vox",
         "Print out warnings when any fragment has a specific cut that is not assigned any cut voxels; "
         "this probably indicates a problem/bad fragment.")
    << false;

  po.add("add-noise-lands", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "add-noise-lands",
         "Adds isotropic Gaussian noise to each cut landmark; this is useful for differentiating "
         "between the plan and actual cuts.")
    << false;

  po.add("noise-mean-lands", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "noise-mean-lands",
         "Mean of the isotropic Gaussian noise to be added to the landmark points.")
    << 0.0;

  po.add("noise-std-lands", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "noise-std-lands",
         "Standard deviation of the isotropic Gaussian noise to be added to the landmark points.")
    << 5.0;

  po.add("add-noise-planes", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "add-noise-planes",
         "Apply a random rotation to each plane's normal vector and add random noise to each plane's "
         "scalar component; this is useful for differentiating between the plan and actual cuts.")
    << false;

  po.add("use-spherical-plane-noise", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "use-spherical-plane-noise",
         "When adding noise to the cutting planes, decompose each plane into spherical coordinates "
         "and add noise to those parameters, otherwise each plane normal vector is rotated (randomly) "
         "and then each cut mid-point is translated by a random amount in the new normal direction.")
    << false;

  po.add("use-normal-dist-plane-noise", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE,
         "use-normal-dist-plane-noise",
         "When applying random rotation to the normal and translation (non-spherical method), use "
         "normally distributed rotation angles and translation magnitudes; otherwise uniform sampling "
         "is used.")
    << false;

  // -------------------------------------------------------------------------------------------
  // Ilium plane sampling params
  po.add("ilium-mean-rot-ang", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "ilium-mean-rot-ang",
         "When randomly rotating the plane normals, this specifies the mean rotation angle in degrees."
         " Ilium cut plane.")
    << 0.0;

  po.add("ilium-stddev-rot-ang", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "ilium-stddev-rot-ang",
         "When randomly rotating the plane normals, this specifies either the standard deviation of the "
         "rotation angle normal distribution, or the extent in each direction for the rotation angle "
         "uniform distribution. Units are in degrees."
         " Ilium cut plane.")
    << 12.5;

  po.add("ilium-mean-trans", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "ilium-mean-trans",
         "When randomly rotating the plane normals, this specifies the mean translation along the rotated normal."
         " Ilium cut plane.")
    << 0.0;

  po.add("ilium-stddev-trans", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "ilium-stddev-trans",
         "When randomly rotating the plane normals, this specifies either the standard deviation of the "
         "translation normal distribution, or the extent in each direction for the translation "
         "uniform distribution."
         " Ilium cut plane.")
    << 7.5;

  // -------------------------------------------------------------------------------------------
  // Ischium plane sampling params
  po.add("ischium-mean-rot-ang", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "ischium-mean-rot-ang",
         "When randomly rotating the plane normals, this specifies the mean rotation angle in degrees."
         " Ischium cut plane.")
    << 0.0;

  po.add("ischium-stddev-rot-ang", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "ischium-stddev-rot-ang",
         "When randomly rotating the plane normals, this specifies either the standard deviation of the "
         "rotation angle normal distribution, or the extent in each direction for the rotation angle "
         "uniform distribution. Units are in degrees."
         " Ischium cut plane.")
    << 15.0;

  po.add("ischium-mean-trans", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "ischium-mean-trans",
         "When randomly rotating the plane normals, this specifies the mean translation along the rotated normal."
         " Ischium cut plane.")
    << 0.0;

  po.add("ischium-stddev-trans", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "ischium-stddev-trans",
         "When randomly rotating the plane normals, this specifies either the standard deviation of the "
         "translation normal distribution, or the extent in each direction for the translation "
         "uniform distribution."
         " Ischium cut plane.")
    << 7.5;

  // -------------------------------------------------------------------------------------------
  // Pubis plane sampling params
  po.add("pubis-mean-rot-ang", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "pubis-mean-rot-ang",
         "When randomly rotating the plane normals, this specifies the mean rotation angle in degrees."
         " Pubis cut plane.")
    << 0.0;

  po.add("pubis-stddev-rot-ang", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "pubis-stddev-rot-ang",
         "When randomly rotating the plane normals, this specifies either the standard deviation of the "
         "rotation angle normal distribution, or the extent in each direction for the rotation angle "
         "uniform distribution. Units are in degrees."
         " Pubis cut plane.")
    << 30.0;

  po.add("pubis-mean-trans", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "pubis-mean-trans",
         "When randomly rotating the plane normals, this specifies the mean translation along the rotated normal."
         " Pubis cut plane.")
    << 0.0;

  po.add("pubis-stddev-trans", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "pubis-stddev-trans",
         "When randomly rotating the plane normals, this specifies either the standard deviation of the "
         "translation normal distribution, or the extent in each direction for the translation "
         "uniform distribution."
         " Pubis cut plane.")
    << 10.0;

  // -------------------------------------------------------------------------------------------
  // Posterior plane sampling params
  po.add("post-mean-rot-ang", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "post-mean-rot-ang",
         "When randomly rotating the plane normals, this specifies the mean rotation angle in degrees."
         " Posterior cut plane.")
    << 0.0;

  po.add("post-stddev-rot-ang", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "post-stddev-rot-ang",
         "When randomly rotating the plane normals, this specifies either the standard deviation of the "
         "rotation angle normal distribution, or the extent in each direction for the rotation angle "
         "uniform distribution. Units are in degrees."
         " Posterior cut plane.")
    << 15.0;

  po.add("post-mean-trans", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "post-mean-trans",
         "When randomly rotating the plane normals, this specifies the mean translation along the rotated normal."
         " Posterior cut plane.")
    << 0.0;

  po.add("post-stddev-trans", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "post-stddev-trans",
         "When randomly rotating the plane normals, this specifies either the standard deviation of the "
         "translation normal distribution, or the extent in each direction for the translation "
         "uniform distribution."
         " Posterior cut plane.")
    << 7.5;

#if 0
  po.add("noise-mean-normals", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "noise-mean-normals",
         "Mean of the rotation angle (in degrees) to be applied to the plane normal vectors. "
         "The rotation axis is uniformly sampled from the sphere.")
    << 0.0;

  po.add("noise-std-normals", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "noise-std-normals",
         "Standard deviation of the rotation angle (in degrees) to be applied to the plane normal vectors.")
    << 2.0;

  po.add("noise-mean-scalars", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "noise-mean-scalars",
         "Mean of the noise added to each plane's scalar parameter.")
    << 0.0;

  po.add("noise-std-scalars", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "noise-std-scalars",
         "Standard deviation of the noise added to each plane's scalar parameter.")
    << 2.0;
#endif

  po.add("batch", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32, "batch",
         "Batch compute random plane perturbations, in this case the output "
         "file paths are actually prefixes that will create files ending in "
         "_%03lu.nii.gz and _%03lu.h5. A value of zero indicates no batch "
         "will be computed.")
    << ProgOpts::uint32(0);

  po.add("no-connected", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE,
         "no-connected",
         "Do not check for the largest connected component of the fragment "
         "label map.")
    << false;

  po.add("debug-display", 'd', ProgOpts::kSTORE_TRUE, "debug-display",
         "Display interactive view of pelvis mesh and planned cuts. For debugging, etc.")
    << false;

  po.add("debug-show-slabs", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE,
         "debug-show-slabs",
         "Instead of cutting planes in the debug display, show the cutting slabs. "
         "e.g. the 3D extent of the cuts.")
    << false;

  po.add("time", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "time",
         "Always print the runtime to calculate the segmentation volume.")
    << false;
  
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

  const double cut_width = po.get("cut-width");

  const bool   add_noise_lands     = po.get("add-noise-lands");
  const double noise_mean_lands    = po.get("noise-mean-lands");
  const double noise_std_dev_lands = po.get("noise-std-lands");

  const bool add_noise_planes = po.get("add-noise-planes");
  const bool use_spherical_for_add_noise_planes = po.get("use-spherical-plane-noise");
  const bool use_normal_noise = po.get("use-normal-dist-plane-noise");

  const double ilium_normal_rot_mean_deg   = po.get("ilium-mean-rot-ang");
  const double ilium_normal_rot_stddev_deg = po.get("ilium-stddev-rot-ang");
  const double ilium_trans_mean            = po.get("ilium-mean-trans");
  const double ilium_trans_stddev          = po.get("ilium-stddev-trans"); 

  const double ischium_normal_rot_mean_deg   = po.get("ischium-mean-rot-ang");
  const double ischium_normal_rot_stddev_deg = po.get("ischium-stddev-rot-ang");
  const double ischium_trans_mean            = po.get("ischium-mean-trans");
  const double ischium_trans_stddev          = po.get("ischium-stddev-trans"); 

  const double pubis_normal_rot_mean_deg   = po.get("pubis-mean-rot-ang");
  const double pubis_normal_rot_stddev_deg = po.get("pubis-stddev-rot-ang");
  const double pubis_trans_mean            = po.get("pubis-mean-trans");
  const double pubis_trans_stddev          = po.get("pubis-stddev-trans"); 

  const double post_normal_rot_mean_deg   = po.get("post-mean-rot-ang");
  const double post_normal_rot_stddev_deg = po.get("post-stddev-rot-ang");
  const double post_trans_mean            = po.get("post-mean-trans");
  const double post_trans_stddev          = po.get("post-stddev-trans"); 

#if 0
  const double noise_mean_normals = po.get("noise-mean-normals").as_double() * kDEG2RAD;
  const double noise_std_normals  = po.get("noise-std-normals").as_double() * kDEG2RAD;
  const double noise_mean_scalars = po.get("noise-mean-scalars");
  const double noise_std_scalars  = po.get("noise-std-scalars");
#endif

  const std::string src_label_path = po.pos_args()[0];

  const std::string app_fcsv_path = po.pos_args()[1];
  const std::string cut_fcsv_path = po.pos_args()[2];
  const bool input_is_cut_landmarks = IsSupportedLandmarksFileNamePtMap(cut_fcsv_path);

  const std::string side_str = po.pos_args()[3];

  const std::string dst_frag_no_cuts_path = po.pos_args()[4];
  const std::string dst_frag_w_cuts_path  = po.pos_args()[5];

  const bool save_frag_no_cuts = dst_frag_no_cuts_path != "-";
  const bool save_frag_w_cuts  = dst_frag_w_cuts_path  != "-";

  const std::string cuts_file_path = (po.pos_args().size() > 6) ? po.pos_args()[6] : std::string();

  const std::string invalid_mask_path = po.get("invalid-mask");

  if (!save_frag_no_cuts && !save_frag_w_cuts)
  {
    std::cerr << "WARNING: NO FRAGMENT WILL BE SAVED!!!" << std::endl;
  }

  const bool warn_no_cut_vox = po.get("warn-no-cut-vox");

  const bool is_left = side_str == "left";

  if (!is_left && (side_str != "right"))
  {
    std::cerr << "ERROR: Invalid side string: " << side_str << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  vout << "Will create fragment on " << side_str << " side." << std::endl;

  const bool lands_ras = po.get("lands-ras");

  const bool debug_disp = po.get("debug-display");
  const bool debug_show_slabs = po.get("debug-show-slabs");

  const bool frag_use_largest_conn_comp = !po.get("no-connected").as_bool();

  const unsigned long num_batches = po.get("batch").as_uint32();
  const bool do_batch = num_batches > 0;

  if (do_batch && !add_noise_planes)
  {
    std::cerr << "Noise must be added to planes when batch processing "
                 "is enabled!" << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  const bool do_timing = po.get("time");

  std::ostream& time_out = do_timing ? std::cout : vout;

  using LabelType     = unsigned char;
  using LabelImage    = itk::Image<unsigned char,3>;
  using LabelImagePtr = LabelImage::Pointer;

  using RNGEngine  = std::mt19937;
  using NormalDist = std::normal_distribution<CoordScalar>;

  std::random_device rand_dev;
  RNGEngine rng_eng(rand_dev());

  //////////////////////////////////////////////////////////////////////////////
  // Read in input label map

  vout << "reading in source labels..." << std::endl;
  LabelImagePtr src_labels = ReadITKImageFromDisk<LabelImage>(src_label_path);

  LabelImagePtr invalid_mask;
  if (!invalid_mask_path.empty())
  {
    vout << "reading in invalid fragment voxels mask..." << std::endl;
    invalid_mask = ReadITKImageFromDisk<LabelImage>(invalid_mask_path);
  }

  //////////////////////////////////////////////////////////////////////////////
  // Determine labels of pelvis, fragment, and cut

  LabelType pelvis_label = 0;
  LabelType frag_label   = 0;
  LabelType cut_label    = 0;

  const bool need_to_find_pelvis_label = !po.has("pelvis-label");
  if (!need_to_find_pelvis_label)
  {
    pelvis_label = static_cast<unsigned char>(po.get("pelvis-label").as_uint32());
  }

  const bool need_to_find_frag_label = !po.has("frag-label");
  if (!need_to_find_frag_label)
  {
    frag_label = static_cast<unsigned char>(po.get("frag-label").as_uint32());
  }

  const bool need_to_find_cut_label = !po.has("cut-label");
  if (!need_to_find_cut_label)
  {
    cut_label = static_cast<unsigned char>(po.get("cut-label").as_uint32());
  }

  if (need_to_find_pelvis_label || need_to_find_frag_label || need_to_find_cut_label)
  {
    vout << "attempting to find some labels automatically..." << std::endl;
    const auto guessed_labels = GuessPelvisPAOFragCutLabels(src_labels.GetPointer(), false, false);

    if (need_to_find_pelvis_label)
    {
      pelvis_label = std::get<0>(guessed_labels);
      vout << "pelvis label automatically determined to be: " << static_cast<int>(pelvis_label) << std::endl;
    }

    if (need_to_find_frag_label)
    {
      frag_label = std::get<1>(guessed_labels);
      vout << "fragment label automatically determined to be: " << static_cast<int>(frag_label) << std::endl;
    }

    if (need_to_find_cut_label)
    {
      cut_label = std::get<2>(guessed_labels);
      vout << "cut label automatically determined to be: " << static_cast<int>(cut_label) << std::endl;
    }
  }

  vout << "Labels:\n  Pelvis: " << static_cast<int>(pelvis_label)
       << "\n    Frag: " << static_cast<int>(frag_label)
       << "\n     Cut: " << static_cast<int>(cut_label)
       << std::endl;

  //////////////////////////////////////////////////////////////////////////////
  // Get the landmarks

  const LandMap3 app_pts = ReadLandmarksFileNamePtMap(app_fcsv_path, !lands_ras);

  LandMap3 cut_pts;

  if (input_is_cut_landmarks)
  {
    cut_pts = ReadLandmarksFileNamePtMap(cut_fcsv_path, !lands_ras);
  }

  vout << "APP Landmarks:\n";
  PrintLandmarkMap(app_pts, vout);
  
  const FrameTransform app_to_vol = AnteriorPelvicPlaneFromLandmarksMap(app_pts);

  vout << "APP to Vol:\n" << app_to_vol.matrix() << std::endl;

  const FrameTransform vol_to_app = app_to_vol.inverse();
  
  PAOCutDispInfo disp;
  PAOCutPlanes   cut_defs;
    
  Timer tmr;

  CreatePAOCutPlanesFromLandmarksFn create_cuts;

  bool slabs_valid = false;

  if (input_is_cut_landmarks)
  {
    vout << "Cut Landmarks" << (add_noise_lands ? " (before noise) " : "") << ":\n";
    PrintLandmarkMap(cut_pts, vout);

    if (add_noise_lands)
    {
      vout << "adding noise to cut landmarks..." << std::endl;

      NormalDist iso_noise_dist(noise_mean_lands, noise_std_dev_lands);

      for (auto& cut_pt : cut_pts)
      {
        cut_pt.second[0] += iso_noise_dist(rng_eng);
        cut_pt.second[1] += iso_noise_dist(rng_eng);
        cut_pt.second[2] += iso_noise_dist(rng_eng);
      }

      vout << "Cut Landmarks (after noise):\n";
      PrintLandmarkMap(cut_pts, vout);
    }

    const std::string side_postfix = is_left ? "-l" : "-r";

    const Pt3 ilium_ant_app    = vol_to_app *
                                      cut_pts[fmt::format("AIl{}", side_postfix)];
    const Pt3 ilium_post_app   = vol_to_app *
                                      cut_pts[fmt::format("PIl{}", side_postfix)];
    const Pt3 ischium_ant_app  = vol_to_app *
                                      cut_pts[fmt::format("AIs{}", side_postfix)];
    const Pt3 ischium_post_app = vol_to_app *
                                      cut_pts[fmt::format("PIs{}", side_postfix)];
    const Pt3 pubis_sup_app    = vol_to_app *
                                      cut_pts[fmt::format("SP{}", side_postfix)];
    const Pt3 pubis_inf_app    = vol_to_app *
                                      cut_pts[fmt::format("IP{}", side_postfix)];

    //////////////////////////////////////////////////////////////////////////////
    // compute the fragment

    vout << "computing fragment..." << std::endl;
    
    {
      CreatePAOCutPlanesFromLandmarksFn create_cuts_tmp = { ilium_ant_app, ilium_post_app,
                                                            ischium_post_app, ischium_ant_app,
                                                            pubis_sup_app, pubis_inf_app,
                                                            app_to_vol,
                                                            src_labels.GetPointer()
                                                          };
    
      // ugly, I know
      create_cuts = create_cuts_tmp;
    }

    create_cuts.create_slabs = true;
    create_cuts.chisel_width = 15;
    create_cuts.chisel_thickness = cut_width;

    create_cuts.ilium_cut_max_len_scale   = 2;
    create_cuts.ischium_cut_max_len_scale = 3;
    create_cuts.pubis_cut_max_len_scale   = 6;
    create_cuts.post_cut_max_len_scale    = 2;

    create_cuts.ilium_cut_chisel_width_scale   = 8;
    create_cuts.ischium_cut_chisel_width_scale = 6;
    create_cuts.pubis_cut_chisel_width_scale   = 4;
    create_cuts.post_cut_chisel_width_scale    = 8;

    tmr.start();

    create_cuts();
   
    disp     = create_cuts.disp_info;
    cut_defs = create_cuts.cut_defs;

    tmr.stop();
    
    slabs_valid = true;
  }
  else
  {
    if (add_noise_lands)
    {
      std::cerr << "WARNING: cannot add noise to cut landmarks, since input is "
                   "cutting planes file." << std::endl;
    }

    vout << "reading existing cutting plane data..." << std::endl;
    const auto pao_defs = ReadPAOCutPlanesFile(cut_fcsv_path);
    
    cut_defs = std::get<0>(pao_defs);
    
    // In general, the display structure is optional, however for this
    // application, we require it.
    auto& tmp_disp = std::get<1>(pao_defs);
    
    if (!tmp_disp)
    {
      std::cerr << "ERROR: existing cutting plane file does not contain display info!" << std::endl;
      return kEXIT_VAL_BAD_CUTS_FILE;
    }

    disp = *tmp_disp;

    auto& tmp_slab = std::get<2>(pao_defs);

    if (tmp_slab)
    {
      create_cuts.cut_slabs = *tmp_slab;
      
      slabs_valid = true;
    }
  }
  
  const unsigned long num_fragments = do_batch ? num_batches : 1;
  
  // Randomly adjust cutting planes, if specified
  std::unique_ptr<PAOSampleRandomCutPlaneAdjusts> rand_cuts;
  if (use_spherical_for_add_noise_planes)
  {
    rand_cuts.reset(new PAOSampleRandomCutPlaneAdjustsSpherical);
  }
  else
  {
    rand_cuts.reset(new PAOSampleRandomCutPlaneAdjustsRotNormalTransMidPt);
    auto* c = static_cast<PAOSampleRandomCutPlaneAdjustsRotNormalTransMidPt*>(rand_cuts.get());
  
    c->use_normal = use_normal_noise;
    
    c->ilium_sample_params.mean_rot_ang_deg    = ilium_normal_rot_mean_deg;
    c->ilium_sample_params.std_dev_rot_ang_deg = ilium_normal_rot_stddev_deg;
    c->ilium_sample_params.mean_trans          = ilium_trans_mean;
    c->ilium_sample_params.std_dev_trans       = ilium_trans_stddev;
    
    c->ischium_sample_params.mean_rot_ang_deg    = ischium_normal_rot_mean_deg;
    c->ischium_sample_params.std_dev_rot_ang_deg = ischium_normal_rot_stddev_deg;
    c->ischium_sample_params.mean_trans          = ischium_trans_mean;
    c->ischium_sample_params.std_dev_trans       = ischium_trans_stddev;

    c->pubis_sample_params.mean_rot_ang_deg    = pubis_normal_rot_mean_deg;
    c->pubis_sample_params.std_dev_rot_ang_deg = pubis_normal_rot_stddev_deg;
    c->pubis_sample_params.mean_trans          = pubis_trans_mean;
    c->pubis_sample_params.std_dev_trans       = pubis_trans_stddev;

    c->post_sample_params.mean_rot_ang_deg    = post_normal_rot_mean_deg;
    c->post_sample_params.std_dev_rot_ang_deg = post_normal_rot_stddev_deg;
    c->post_sample_params.mean_trans          = post_trans_mean;
    c->post_sample_params.std_dev_trans       = post_trans_stddev;
  }

  if (add_noise_planes)
  {
    vout << "randomly adjusting plane parameters..." << std::endl;
  
    rand_cuts->set_debug_output_stream(vout, verbose);
    
    rand_cuts->num_samples   = num_fragments;
    rand_cuts->src_cut_defs  = cut_defs;
    rand_cuts->src_disp_info = disp;
    // The sampling parameters are set to reasonable defaults

    (*rand_cuts)();
  }

  for (unsigned long frag_idx = 0; frag_idx < num_fragments; ++frag_idx)
  {
    vout << "frag idx: " << frag_idx << std::endl;

    if (add_noise_planes)
    {
      cut_defs = rand_cuts->dst_cut_defs[frag_idx];
      disp     = rand_cuts->dst_cut_disp_infos[frag_idx];
    }

    if (debug_disp)
    {
      VTKCreateMesh create_mesh;
      create_mesh.labels.assign(1, pelvis_label);

      vout << "creating pelvis mesh..." << std::endl;
      TriMesh pelvis_mesh = create_mesh(src_labels.GetPointer());
     
      vout << "transforming pelvis mesh to APP..." << std::endl;
      pelvis_mesh.transform(app_to_vol.inverse());

      VTK3DPlotter plotter;
   
      //VTK3DPlotter::RGBAVec bone_color = VTK3DPlotter::BoneColorAlpha();
      //bone_color[3] = 0.25; 
      const VTK3DPlotter::RGBVec bone_color = VTK3DPlotter::BoneColor();
      
      plotter.add_mesh(pelvis_mesh, bone_color);
    
      VTK3DPlotter::RGBAVec cuts_color;
      cuts_color(0) = 0;
      cuts_color(1) = 1;
      cuts_color(2) = 0;
      cuts_color(3) = 0.5;

      if (!debug_show_slabs)
      {
        DrawPAOCutPlanes(cut_defs, disp, plotter, cuts_color,
                         false,  // draw plane normals
                         false    // draw helper boundary planes
                        );
      }
      else
      {
        if (slabs_valid)
        {
          constexpr bool kSAMPLE_SLABS = false;

          if (!kSAMPLE_SLABS)
          {
            plotter.add_aa_slab(create_cuts.cut_slabs.ilium_cut,
                                create_cuts.cut_slabs.ilium_cut_to_world,
                                cuts_color);
            
            plotter.add_aa_slab(create_cuts.cut_slabs.ischium_cut,
                                create_cuts.cut_slabs.ischium_cut_to_world,
                                cuts_color);
            
            plotter.add_aa_slab(create_cuts.cut_slabs.pubis_cut,
                                create_cuts.cut_slabs.pubis_cut_to_world,
                                cuts_color);

            plotter.add_aa_slab(create_cuts.cut_slabs.post_cut,
                                create_cuts.cut_slabs.post_cut_to_world,
                                cuts_color); 
          }
          else
          {
            constexpr CoordScalar sampling_dist = 3;

            // ilium cut
            Pt3List pts = SampleAASlab3PtsIso(create_cuts.cut_slabs.ilium_cut, sampling_dist);
            ApplyTransform(create_cuts.cut_slabs.ilium_cut_to_world, pts, &pts);
            plotter.add_pts(pts, cuts_color);
            
            // ischium cut
            pts = SampleAASlab3PtsIso(create_cuts.cut_slabs.ischium_cut, sampling_dist);
            ApplyTransform(create_cuts.cut_slabs.ischium_cut_to_world, pts, &pts);
            plotter.add_pts(pts, cuts_color);
           
            // pubis cut
            pts = SampleAASlab3PtsIso(create_cuts.cut_slabs.pubis_cut, sampling_dist);
            ApplyTransform(create_cuts.cut_slabs.pubis_cut_to_world, pts, &pts);
            plotter.add_pts(pts, cuts_color);
         
            // posterior cut
            pts = SampleAASlab3PtsIso(create_cuts.cut_slabs.post_cut, sampling_dist);
            ApplyTransform(create_cuts.cut_slabs.post_cut_to_world, pts, &pts);
            plotter.add_pts(pts, cuts_color);
          }
        }
        else
        {
          std::cerr << "WARNING: cannot plot slabs, because cuts read directly from file without slab info!" << std::endl;
        }
      }

      plotter.show();
    }

    // Now create the actual label map
    CreatePAOFragLabelMapFromCutPlanesFn create_frag_label_fn;
    create_frag_label_fn.cut_defs = cut_defs;
    create_frag_label_fn.src_labels = src_labels;
    create_frag_label_fn.pelvis_label = pelvis_label;
    create_frag_label_fn.frag_label   = frag_label;
    create_frag_label_fn.cut_label    = cut_label;
    create_frag_label_fn.app_to_vol = app_to_vol;
    create_frag_label_fn.check_connected = frag_use_largest_conn_comp;
    create_frag_label_fn.create_labels_w_cuts = cut_width > 1.0e-6;
    create_frag_label_fn.cut_width = cut_width;
    create_frag_label_fn.invalid_frag_locations = invalid_mask;

    vout << "creating fragment labels..." << std::endl;
    
    tmr.start();
    
    create_frag_label_fn();
    
    tmr.stop();
      
    time_out << fmt::format("time: {:.3f}", tmr.elapsed_seconds()) << std::endl;

    LabelImagePtr no_cut_labels = create_frag_label_fn.frag_labels_no_cut;

    //////////////////////////////////////////////////////////////////////////////
    // Save the fragment/segmentation

    if (save_frag_no_cuts)
    {
      vout << "writing fragment with no cuts..." << std::endl;
      WriteITKImageToDisk(no_cut_labels.GetPointer(),
        do_batch ? fmt::format("{}_{:03d}.nii.gz", dst_frag_no_cuts_path, frag_idx)
                 : dst_frag_no_cuts_path);
    }

    LabelImagePtr w_cut_labels = create_frag_label_fn.frag_labels_w_cut;

    if (save_frag_w_cuts && (cut_width <= 1.0e-6))
    {
      // user wants to save cuts, but specified a zero width... strange.
      std::cerr << fmt::format("WARNING: Cut Width of {:.4f}, implies no cuts to apply!", cut_width) << std::endl;
      w_cut_labels = no_cut_labels;
    }

    if (save_frag_w_cuts)
    {
      vout << "writing fragment with cuts..." << std::endl;
      WriteITKImageToDisk(w_cut_labels.GetPointer(),
          do_batch ? fmt::format("{}_{:03d}.nii.gz", dst_frag_w_cuts_path, frag_idx)
                   : dst_frag_w_cuts_path);
    }

    if (!cuts_file_path.empty())
    {
      // Debugging
      //PrintPAOCutPlanes(cut_defs, vout);

      vout << "writing cuts to file..." << std::endl;
      
      if (add_noise_planes)
      {
        std::cerr << "WARNING: random pose adjustments applied to plane parameters, "
                 "therefore slabs will not be written to file!"
                  << std::endl;
      }
     
      WritePAOCutPlanesFile(cut_defs,
          do_batch ? fmt::format("{}_{:03d}.h5", cuts_file_path, frag_idx)
                   : cuts_file_path,
                           &disp,
           (!add_noise_planes && slabs_valid) ? &create_cuts.cut_slabs : nullptr);
    }
    
    if (warn_no_cut_vox)
    {
      Pt3List ilium_cut_pts;
      Pt3List ischium_cut_pts;
      Pt3List pubis_cut_pts;
      Pt3List post_cut_pts;

      PAOExtract3DCutPointsForEachCut(w_cut_labels.GetPointer(), cut_label,
                                      cut_defs,
                                      vol_to_app,
                                      &ilium_cut_pts,
                                      &ischium_cut_pts,
                                      &pubis_cut_pts,
                                      &post_cut_pts,
                                      nullptr,
                                      nullptr,
                                      nullptr,
                                      nullptr);

      const bool no_il_pts = ilium_cut_pts.empty();
      const bool no_is_pts = ischium_cut_pts.empty();
      const bool no_pu_pts = pubis_cut_pts.empty();
      const bool no_pt_pts = post_cut_pts.empty();
      
      if (no_il_pts || no_is_pts || no_pu_pts || no_pt_pts)
      {
        vout << "WARNING: some cuts were not assigned any cut voxels... fragment " << frag_idx << std::endl;
      
        if (no_il_pts)
        {
          vout << "WARNING: ilium cut was not assigned any cut voxels..." << std::endl;
        }
      
        if (no_is_pts)
        {
          vout << "WARNING: ischium cut was not assigned any cut voxels..." << std::endl;
        }
      
        if (no_pu_pts)
        {
          vout << "WARNING: pubis cut was not assigned any cut voxels..." << std::endl;
        }
      
        if (no_pt_pts)
        {
          vout << "WARNING: posterior cut was not assigned any cut voxels..." << std::endl;
        }
      }
    }

  }  // for number of fragments

  vout << "exiting..." << std::endl;

  return kEXIT_VAL_SUCCESS;
}
