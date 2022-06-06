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
#include "xregPAOVolAfterRepo.h"
#include "xregMetalObjSampling.h"

int main(int argc, char* argv[])
{
  using namespace xreg;
  
  constexpr int kEXIT_VAL_SUCCESS = 0;
  constexpr int kEXIT_VAL_BAD_USE = 1;

  // First, set up the program options

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Inserts screws and/or K-wires into a volume. "
              "The shapes and poses of the objects are randomly sampled and then "
              "inserted into an input volume. The default behavior is to up-sample "
              "the input volume in order to avoid aliasing effects and then "
              "downsample back to the original resolution. "
              "WARNING: This program may use a MASSIVE amount of memory when upsampling.");
  
  po.set_arg_usage("<Input CT vol.> <Input PAO Segmentation> <APP Landmarks> "
                   "<side> <frag+femur tranform> <insertion surf. labels> "
                   "<Output CT vol.> [<Output definitions of inserted objects>]");
  po.set_min_num_pos_args(7);

  po.add("lands-ras", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "lands-ras",
         "Read landmarks in RAS coordinates instead of LPS.")
    << false;

#if 0
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
#endif
  
  po.add("p-wire", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "p-wire",
         "The probability of inserting a k-wire; probability of screw is 1 minus this.")
    << 1.0;

  po.add("p-two", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "p-two",
         "The probability of inserting two objects; probability of three is 1 minus this.")
    << 1.0;

  po.add("super-sample", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "super-sample",
         "Up-sampling (super-sampling) factor used for inserting objects into volume. "
         "This drastically affects the amount of runtime memory required. "
         "A value of 1 indicates no up-sampling will be used.")
    << 4.0;

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

  const std::string src_intens_path   = po.pos_args()[0];
  const std::string src_label_path    = po.pos_args()[1];
  const std::string app_fcsv_path     = po.pos_args()[2];
  const std::string side_str          = po.pos_args()[3];
  const std::string frag_xform_path   = po.pos_args()[4];
  const std::string insert_label_path = po.pos_args()[5];
  const std::string dst_intens_path   = po.pos_args()[6];
  const std::string obj_defs_path     = (po.pos_args().size() > 7) ? po.pos_args()[7] : std::string();

  const bool is_left = side_str == "left";

  if (!is_left && (side_str != "right"))
  {
    std::cerr << "ERROR: Invalid side string: " << side_str << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  vout << "Insert objects on " << side_str << " side." << std::endl;

  const bool lands_ras = po.get("lands-ras");
  
  const double p_kwire = po.get("p-wire");
  const double p_two   = po.get("p-two");

  const double super_sample_factor = po.get("super-sample");

  //////////////////////////////////////////////////////////////////////////////
  // Read in input intensity volume
  
  vout << "reading in source intensity volume..." << std::endl;
  auto src_intens = ReadITKImageFromDisk<AddPAOScrewKWireToVol::Vol>(src_intens_path);

  //////////////////////////////////////////////////////////////////////////////
  // Read in input label map

  vout << "reading in source labels/segmentation..." << std::endl;
  auto cuts_seg = ReadITKImageFromDisk<PAOSampleScrewWireInsertionPts::LabelVol>(src_label_path);
  
  //////////////////////////////////////////////////////////////////////////////
  // Read in insertion label map

  vout << "reading object insertion labels/segmentation..." << std::endl;
  auto insert_labels = ReadITKImageFromDisk<PAOSampleScrewWireInsertionPts::LabelVol>(insert_label_path);
  
  //////////////////////////////////////////////////////////////////////////////
  // Get the landmarks

  vout << "reading APP landmarks..." << std::endl;
  const LandMap3 app_pts = ReadLandmarksFileNamePtMap(app_fcsv_path, !lands_ras);

  vout << "APP Landmarks:\n";
  PrintLandmarkMap(app_pts, vout);
  
  const Pt3 femur_pt = app_pts.find(fmt::format("FH-{}", side_str[0]))->second;
  
  const FrameTransform app_to_vol = AnteriorPelvicPlaneFromLandmarksMap(app_pts,
                                      is_left ? kAPP_ORIGIN_LEFT_FH : kAPP_ORIGIN_RIGHT_FH);

  vout << "APP to Vol:\n" << app_to_vol.matrix() << std::endl;

  vout << "Reading fragment transformation from disk..." << std::endl;
  const FrameTransform frag_xform = ReadITKAffineTransformFromFile(frag_xform_path);
  
  vout << "Setting up object insertion start/end sampler..." << std::endl;
  PAOSampleScrewWireInsertionPts sample_insertion_pts;
  sample_insertion_pts.set_debug_output_stream(vout, verbose);
  sample_insertion_pts.prob_two_objs = p_two;
  sample_insertion_pts.cut_seg = cuts_seg;
  sample_insertion_pts.insert_labels = insert_labels;
  sample_insertion_pts.app_to_vol = app_to_vol;  
  sample_insertion_pts.femur_pt_wrt_vol = femur_pt;

  vout << "  initializing object insertion start/end sampler..." << std::endl;
  sample_insertion_pts.init();

  vout << "    sampling screw start/stops..." << std::endl;
  sample_insertion_pts.run(frag_xform);
      
  const size_type num_objs = sample_insertion_pts.obj_ends.size();

  vout << "    inserting object intensities: "
       << num_objs << " objects..."
       << std::endl;

  vout << "setting up screw/kwire volume insertion..." << std::endl;

  AddPAOScrewKWireToVol add_objs;
  add_objs.set_debug_output_stream(vout, verbose);
  add_objs.prob_screw = 1.0 - p_kwire;
  add_objs.orig_vol = src_intens;
  add_objs.obj_start_pts = sample_insertion_pts.obj_starts;
  add_objs.obj_end_pts = sample_insertion_pts.obj_ends;
  add_objs.super_sample_factor = super_sample_factor;

  vout << "  performing insertion..." << std::endl;
  add_objs();

  vout << "writing updated volume to disk..." << std::endl;
  WriteITKImageToDisk(add_objs.obj_vol.GetPointer(), dst_intens_path);
 
  vout << "exiting..." << std::endl;
  
  return kEXIT_VAL_SUCCESS;
}
