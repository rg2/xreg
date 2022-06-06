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
#include "xregHipSegUtils.h"
#include "xregLandmarkMapUtils.h"
#include "xregAnatCoordFrames.h"
#include "xregPAOVolAfterRepo.h"
#include "xregSampleUtils.h"

int main(int argc, char* argv[])
{
  using namespace xreg;
  
  constexpr int kEXIT_VAL_SUCCESS       = 0;
  constexpr int kEXIT_VAL_BAD_USE       = 1;
  constexpr int kEXIT_VAL_BAD_CUTS_FILE = 2;

  // First, set up the program options

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Creates a new CT volume by piecewise rigidly warping the ipsilateral PAO fragment and femur. "
              "Warped fragment and femur bone structures will overwrite existing intensity values. "
              "Locations corresponding to cuts are given an HU value corresponding to air. "
              "Other locations which are no longer occupied after transforming the fragment or femur are given "
              "random values drawn from N(mu,sigma). Sigma is set using the \"replace-val-std\" flag. "
              "Mu is drawn from U(l,u) and l and u are set using \"replace-val-mean-lower\" and "
              "\"replace-val-mean-upper,\" respectively. The default behavior is to sample random HU "
              "values corresponding to muscle tissue.");
  
  po.set_arg_usage("<Orig. CT vol.> <Input PAO Segmentation> <APP Landmarks> "
                   "<side> <frag+femur tranform> <femur only transform> "
                   "<Output CT vol.>");
  po.set_min_num_pos_args(7);

  po.add("lands-ras", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "lands-ras",
         "Read landmarks in RAS coordinates instead of LPS.")
    << false;

  po.add("pelvis-label", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32, "pelvis-label",
         "The value used to indicate the pelvis in the input segmentation; if not "
         "provided, then the smallest positive value in the segmentation is used.");

  po.add("frag-label", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32, "frag-label",
         "The value used to indicate the fragment in the output segmentation; if not "
         "provided, then the second largest positive value in the segmentation is used.");

  po.add("cut-label", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32, "cut-label",
         "The value used to indicate the cut in the output segmentation; if not "
         "provided, then the largest positive value in the input segmentation is used.");
  
  po.add("femur-label", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32, "femur-label",
         "The value used to indicate the ipsilateral femur in the input segmentation; if not "
         "provided, then the value in the segmentation located at the femoral head landmark is used.");

  po.add("replace-val-mean-lower", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "replace-val-mean-lower",
         "Lower bound of the uniform distribution used to sample the mean of the tissue replacement distribution.")
    << 35.0;

  po.add("replace-val-mean-upper", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "replace-val-mean-upper",
         "Upper bound of the uniform distribution used to sample the mean of the tissue replacement distribution.")
    << 55.0;

  po.add("replace-val-std", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "replace-val-std",
         "Standard deviation of the tissue replacement distribution. A value <= 0 indicates that "
         "a constant value will be used.")
    << 20.0;

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

  const std::string src_intens_path  = po.pos_args()[0];
  const std::string src_label_path   = po.pos_args()[1];
  const std::string app_fcsv_path    = po.pos_args()[2];
  const std::string side_str         = po.pos_args()[3];
  const std::string frag_xform_path  = po.pos_args()[4];
  const std::string femur_xform_path = po.pos_args()[5];
  const std::string dst_intens_path  = po.pos_args()[6];

  const bool is_left = side_str == "left";

  if (!is_left && (side_str != "right"))
  {
    std::cerr << "ERROR: Invalid side string: " << side_str << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  vout << "Warp fragment/femur on " << side_str << " side." << std::endl;

  const bool lands_ras = po.get("lands-ras");

  const double replace_val_mean_lower = po.get("replace-val-mean-lower");
  const double replace_val_mean_upper = po.get("replace-val-mean-upper");
  const double replace_val_std_dev    = po.get("replace-val-std");

  const bool replace_val_mean_const = std::abs(replace_val_mean_upper - replace_val_mean_lower) < 1.0e-8;

  if (!replace_val_mean_const && ((replace_val_mean_upper - replace_val_mean_lower) < -1.0e-8))
  {
    std::cerr << "ERROR: empty range passed for replacement value mean distribution!" << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  //////////////////////////////////////////////////////////////////////////////
  // Read in input intensity volume
  
  vout << "reading in source intensity volume..." << std::endl;
  auto src_intens = ReadITKImageFromDisk<UpdateVolAfterRepos::Vol>(src_intens_path);

  //////////////////////////////////////////////////////////////////////////////
  // Read in input label map

  vout << "reading in source labels/segmentation..." << std::endl;
  auto cuts_seg = ReadITKImageFromDisk<UpdateVolAfterRepos::LabelVol>(src_label_path);
  
  //////////////////////////////////////////////////////////////////////////////
  // Get the landmarks

  vout << "reading APP landmarks..." << std::endl;
  const LandMap3 app_pts = ReadLandmarksFileNamePtMap(app_fcsv_path, !lands_ras);

  vout << "APP Landmarks:\n";
  PrintLandmarkMap(app_pts, vout);
  
  const FrameTransform app_to_vol = AnteriorPelvicPlaneFromLandmarksMap(app_pts,
                                      is_left ? kAPP_ORIGIN_LEFT_FH : kAPP_ORIGIN_RIGHT_FH);

  vout << "APP to Vol:\n" << app_to_vol.matrix() << std::endl;

  const FrameTransform vol_to_app = app_to_vol.inverse();

  //////////////////////////////////////////////////////////////////////////////
  // Determine labels of pelvis, fragment, and cut

  using VolScalar   = UpdateVolAfterRepos::VolScalar;
  using LabelScalar = UpdateVolAfterRepos::LabelScalar;
  
  LabelScalar pelvis_label = 0;
  LabelScalar frag_label   = 0;
  LabelScalar cut_label    = 0;
  LabelScalar femur_label  = 0;

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

  const bool need_to_find_femur_label = !po.has("femur-label");
  if (!need_to_find_femur_label)
  {
    cut_label = static_cast<unsigned char>(po.get("femur-label").as_uint32());
  }

  if (need_to_find_pelvis_label || need_to_find_frag_label ||
      need_to_find_cut_label || need_to_find_femur_label)
  {
    vout << "attempting to find pelvis/femur/frag/cut labels automatically..." << std::endl;
    const auto guessed_labels = GuessPelvisFemurPAOFragLabels(cuts_seg.GetPointer(),
                                                   app_pts.find(fmt::format("FH-{}", side_str[0]))->second,
                                                              true, true);

    if (need_to_find_pelvis_label)
    {
      pelvis_label = std::get<0>(guessed_labels);
      vout << "pelvis label automatically determined to be: " << static_cast<int>(pelvis_label) << std::endl;
    }

    if (need_to_find_femur_label)
    {
      femur_label = std::get<1>(guessed_labels);
      vout << "femur label automatically determined to be: " << static_cast<int>(femur_label) << std::endl;
    }

    if (need_to_find_frag_label)
    {
      frag_label = std::get<2>(guessed_labels);
      vout << "fragment label automatically determined to be: " << static_cast<int>(frag_label) << std::endl;
    }

    if (need_to_find_cut_label)
    {
      cut_label = std::get<3>(guessed_labels);
      vout << "cut label automatically determined to be: " << static_cast<int>(cut_label) << std::endl;
    }
  }

  vout << "Labels:\n  Pelvis: " << static_cast<int>(pelvis_label)
       << "\n    Frag: " << static_cast<int>(frag_label)
       << "\n     Cut: " << static_cast<int>(cut_label)
       << "\n   Femur: " << static_cast<int>(femur_label)
       << std::endl;

  vout << "Reading fragment transformation from disk..." << std::endl;
  const FrameTransform frag_xform = ReadITKAffineTransformFromFile(frag_xform_path);
  
  vout << "Reading femur transformation from disk..." << std::endl;
  const FrameTransform femur_xform = ReadITKAffineTransformFromFile(femur_xform_path);

  std::mt19937 rng_eng;
  SeedRNGEngWithRandDev(&rng_eng);

  const double replace_val = replace_val_mean_const
                               ? replace_val_mean_upper
                               : std::uniform_real_distribution<double>(
                                   replace_val_mean_lower,replace_val_mean_upper)(rng_eng);
  
  vout << fmt::format("tissue replacement values will be drawn from N({:.2f},{:.2f})",
                      replace_val, replace_val_std_dev)
       << std::endl;

  vout << "setting up warper..." << std::endl;

  UpdateVolAfterRepos update_vol;
  update_vol.src_vol = src_intens;
  update_vol.labels_of_air = { cut_label };
  update_vol.labels = cuts_seg;
  update_vol.labels_dilate_rad = 0;
  update_vol.labels_of_repo_objs = { frag_label, femur_label };
  update_vol.repo_objs_xforms = { app_to_vol * frag_xform * vol_to_app,
                                  app_to_vol * frag_xform * femur_xform * vol_to_app };

  update_vol.default_val = replace_val;
  update_vol.add_rand_to_default_val_std_dev = replace_val_std_dev;

  vout << "warping..." << std::endl;
  update_vol();

  vout << "writing warped image to disk..." << std::endl;
  WriteITKImageToDisk(update_vol.dst_vol.GetPointer(), dst_intens_path);
  
  vout << "exiting..." << std::endl;
  
  return kEXIT_VAL_SUCCESS;
}
