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

#include <itkBinaryMorphologicalOpeningImageFilter.h>
#include <itkBinaryBallStructuringElement.h>

#include "xregProgOptUtils.h"
#include "xregLandmarkFiles.h"
#include "xregAnatCoordFrames.h"
#include "xregLandmarkMapUtils.h"
#include "xregITKIOUtils.h"
#include "xregHipSegUtils.h"
#include "xregCSVUtils.h"
#include "xregLabelWarping.h"
#include "xregStringUtils.h"
#include "xregRigidUtils.h"

int main(int argc, char* argv[])
{
  using namespace xreg;

  constexpr int kEXIT_VAL_SUCCESS = 0;
  constexpr int kEXIT_VAL_BAD_USE = 1;

  // First, set up the program options

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Given a PAO fragment label map, sample valid movements in the APP "
              "coordinate frame. The APP will be computed to have origin at the "
              "fragment side center of femural head. The landmarks are assumed to "
              "be in the coordinate frame of the segmentation volume. To compute "
              "the APP, left and right Anterior Superior Iliac Spines "
              "(\"ASIS-{l,r}\") and left and right Inferior Pubis Symphisis "
              "(\"IPS-{l,r}\") are required. The side string must be either "
              "\"left\" or \"right.\" The output transforms will be written to "
              "disk in the following format: "
              "<Prefix>_<frag|femur>_<transform sample index>.h5. "
              "The femur transform is meant to be applied to the femur only first, "
              "followed by the fragment + femur transform to both objects.");
  
  po.set_arg_usage("<Input Segmentation> <APP Landmarks> <side> "
                   "<Number of Transforms> <Output Transform Prefix>");
  po.set_min_num_pos_args(5);

  po.add("lands-ras", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "lands-ras",
         "Read landmarks in RAS coordinates instead of LPS.")
    << false;

  po.add("no-medialize", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-medialize",
         "Do not tend to medialize the fragment. Medializtion effectively sets the translation "
         "mean in the X direction to always point towards the interior of the patient.")
    << false;

  po.add("frag-rot-mean-x", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "frag-rot-mean-x",
         "The mean rotation angle, in degrees, to be applied to the fragment/femur about the X axis in the APP (R->L).")
    << 10.0;

  po.add("frag-rot-std-x", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "frag-rot-std-x",
         "The rotation angle standard deviation, in degrees, of the rotation applied to the fragment/femur about the X axis in the APP (R->L).")
    << 5.0;

  po.add("frag-rot-mean-y", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "frag-rot-mean-y",
         "The mean rotation angle, in degrees, to be applied to the fragment/femur about the Y axis in the APP (I->S).")
    << 0.0;

  po.add("frag-rot-std-y", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "frag-rot-std-y",
         "The rotation angle standard deviation, in degrees, of the rotation applied to the fragment/femur about the Y axis in the APP (I->S).")
    << 5.0;

  po.add("frag-rot-mean-z", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "frag-rot-mean-z",
         "The mean rotation angle, in degrees, to be applied to the fragment/femur about the Z axis in the APP (P->A).")
    << 10.0;

  po.add("frag-rot-std-z", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "frag-rot-std-z",
         "The rotation angle standard deviation, in degrees, of the rotation applied to the fragment/femur about the Z axis in the APP (P->A).")
    << 5.0;
  
  po.add("femur-rot-mean-x", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "femur-rot-mean-x",
         "The mean rotation angle about the APP X axis (R->L), in degrees, to be applied to the femur after moving the fragment.")
    << 0.0;

  po.add("femur-rot-std-x", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "femur-rot-std-x",
         "The rotation angle standard deviation about the APP X axis (R->L), in degrees, of the rotation applied to the femur after moving the fragment.")
    << 10.0;
  
  po.add("femur-rot-mean-y", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "femur-rot-mean-y",
         "The mean rotation angle about the APP Y axis (I->S), in degrees, to be applied to the femur after moving the fragment.")
    << 0.0;

  po.add("femur-rot-std-y", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "femur-rot-std-y",
         "The rotation angle standard deviation about the APP Y axis (I->S), in degrees, of the rotation applied to the femur after moving the fragment.")
    << 5.0;
  
  po.add("femur-rot-mean-z", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "femur-rot-mean-z",
         "The mean rotation angle about the APP Z axis (P->A), in degrees, to be applied to the femur after moving the fragment.")
    << 0.0;

  po.add("femur-rot-std-z", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "femur-rot-std-z",
         "The rotation angle standard deviation about the APP Z axis (P->A), in degrees, of the rotation applied to the femur after moving the fragment.")
    << 1.0;

  po.add("trans-mean-x", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "trans-mean-x",
         "The mean amount of translation to be applied to the fragment/femur in the APP X-Axis (Right->Right), this "
         "will be negated according to the side argument.")
    << 2.5;

  po.add("trans-std-x", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "trans-std-x",
         "The standard deviation of translation in the APP X-Axis (Left->Right).")
    << 5.0;

  po.add("trans-mean-y", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "trans-mean-y",
         "The mean amount of translation to be applied to the fragment/femur in the APP Y-Axis (Inferior->Superior).")
    << -2.0;

  po.add("trans-std-y", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "trans-std-y",
         "The standard deviation of translation in the APP Y-Axis (Inferior->Superior).")
    << 2.0;

  po.add("trans-mean-z", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "trans-mean-z",
         "The mean amount of translation to be applied to the fragment/femur in the APP Z-Axis (Posterior->Anterior).")
    << 2.0;

  po.add("trans-std-z", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "trans-std-z",
         "The standard deviation of translation in the APP Z-Axis (Posterior->Anterior).")
    << 3.0;

  po.add("no-rot", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-rot",
         "Do not sample any rotation (overrides the rotation sampling settings).")
    << false;

  po.add("no-trans", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-trans",
         "Do not sample any translation (overrides the translation sampling settings).")
    << false;

  po.add("frag-rot-for-femur", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "frag-rot-for-femur",
         "Use the fragment\'s rotation and translation on the femur, otherwise a factor is included in the femur "
         "transform to cancel out the fragment\'s rotational component; the translation component is retained.")
    << false;

  po.add("pelvis-label", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32, "pelvis-label",
         "The value used to indicate the pelvis in the input segmentation; if not "
         "provided, then the smallest positive value in the segmentation is used.");

  po.add("frag-label", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32, "frag-label",
         "The value used to indicate the fragment in the output segmentation; if not "
         "provided, then the N+1 is used, where N is the largest positive value in the input segmentation.");

  po.add("cut-label", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32, "cut-label",
         "The value used to indicate the cut in the output segmentation; if not "
         "provided, then the N+2 is used, where N is the largest positive value in the input segmentation.");

  po.add("femur-label", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32, "femur-label",
         "The label of the the operative side femur. If not provided - it will be "
         "estimated by looking at the label of the femural head on the operative side.");

  po.add("contra-femur-label", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32, "contra-femur-label",
         "The label of the the non-operative side (contra-lateral) femur. "
         "If not provided, the contra-lateral femur will not be used in this process.");

  po.add("no-collision-check", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE,
         "no-collision-check",
         "Indicates that no collision checking will be performed when sampling "
         "repositioning.")
    << false;

  po.add("no-param-adjust", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE,
         "no-param-adjust",
         "Do not adjust parameters to ensure realistic movements. When this is "
         "not set, mean rotations about X (L-R) are always made positive (rotate "
         "fragment \"forward\"), mean rotations about ")
    << false;

  po.add("morph-open-size", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_INT32, "morph-open-size",
         "The radius of a morphological ball kernel to be applied to the input cuts label map for "
         "the pelvis, fragment, and ipsilateral femur labels. When a value of 0 or smaller is supplied,"
         " the morphology is not applied.")
    << ProgOpts::int32(0);

  po.add("debug-write-open-labels", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "debug-write-open-labels",
         "Path to write out the label map after performing the opening operation - used for debug.")
    << "";

  po.add("uniform-sampling", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "uniform-sampling",
         "Use uniform distributions for sampling values instead of normal distributions. The sample "
         "interval will be mean +/- 1 standard deviation.")
    << false;

  po.add("logn-mix-sampling", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "logn-mix-sampling",
         "Use log-normal for sampling the fragment X,Y,Z and femur X rotational components, and "
         "normal distributions for the remainder.")
    << false;

  po.add("frag-params-csv", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "frag-params-csv",
         "Path to CSV file with sampled parameters for the fragment+femur transform; "
         "a row represents a single reposition, and rotation X, Y, Z are columns 1-3, and "
         "translation X, Y, Z are columns 4-6.")
    << "";

  po.add("femur-params-csv", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "femur-params-csv",
         "Path to CSV file with sampled parameters for the femur transform; "
         "a row represents a single reposition, and rotation X, Y, Z are columns 1-3, and "
         "translation X, Y, Z are columns 4-6.")
    << "";

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

  std::ostream& vout = po.vout();

  const std::string src_label_path = po.pos_args()[0];

  const std::string app_fcsv_path = po.pos_args()[1];

  const std::string side_str = po.pos_args()[2];

  const unsigned long num_xforms = StringCast<unsigned long>(po.pos_args()[3]);

  const std::string xforms_prefix = po.pos_args()[4];

  const bool is_left = side_str == "left";

  if (!is_left && (side_str != "right"))
  {
    std::cerr << "ERROR: Invalid side string: " << side_str << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  vout << "Fragment specified on the " << side_str << " side." << std::endl;

  const bool lands_ras = po.get("lands-ras");

  const bool no_trans = po.get("no-trans");
  const bool no_rot   = po.get("no-rot");

  const bool use_frag_rot_for_femur = po.get("frag-rot-for-femur");

  const bool no_medialize = po.get("no-medialize");

  const bool no_param_adjust = po.get("no-param-adjust");

  const bool no_check_collisions = po.get("no-collision-check");

  const std::string frag_params_csv_path  = po.get("frag-params-csv");
  const std::string femur_params_csv_path = po.get("femur-params-csv");

  using LabelType  = SampleValidLabelWarpsFn::LabelScalar;
  using LabelImage = SampleValidLabelWarpsFn::LabelImage;
  using ITKPoint   = LabelImage::PointType;
  using ITKIndex   = LabelImage::IndexType;
  using LabelSet   = SampleValidLabelWarpsFn::LabelSet;

  // Check sampling parameter validity.
  const bool uniform_sampling  = po.get("uniform-sampling");
  const bool logn_mix_sampling = po.get("logn-mix-sampling");

  if ((int(uniform_sampling) + int(logn_mix_sampling)) > 1)
  {
    std::cerr << "ERROR: select one, or neither, of uniform or log-normal "
                 "       mix of sampling." << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  //////////////////////////////////////////////////////////////////////////////
  // Get the landmarks

  const auto app_pts = ReadLandmarksFileNamePtMap(app_fcsv_path, !lands_ras);

  vout << "APP Landmarks:\n";
  PrintLandmarkMap(app_pts, vout);

  const FrameTransform app_to_vol = AnteriorPelvicPlaneFromLandmarksMap(app_pts,
                                      is_left ? kAPP_ORIGIN_LEFT_FH : kAPP_ORIGIN_RIGHT_FH);

  vout << "APP to Vol (femur origin):\n" << app_to_vol.matrix() << std::endl;

  const Pt3 femur_pt = app_to_vol.matrix().block(0,3,3,1);

  //////////////////////////////////////////////////////////////////////////////
  // Read in input label map

  vout << "reading in source labels..." << std::endl;
  auto src_labels = ReadITKImageFromDisk<LabelImage>(src_label_path);

  //////////////////////////////////////////////////////////////////////////////
  // Determine labels of pelvis, fragment, femurs, and cut

  LabelType pelvis_label    = 0;
  LabelType frag_label      = 0;
  LabelType cut_label       = 0;
  LabelType femur_label     = 0;
  LabelType con_femur_label = 0;

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
    femur_label = static_cast<unsigned char>(po.get("femur-label").as_uint32());
  }

  const bool use_con_femur_label = po.has("contra-femur-label");
  if (use_con_femur_label)
  {
    con_femur_label = static_cast<unsigned char>(po.get("contra-femur-label").as_uint32());
  }
  else
  {
    vout << "will not use contra-lateral femur" << std::endl;
  }

  if (need_to_find_pelvis_label || need_to_find_frag_label || need_to_find_cut_label)
  {
    vout << "attempting to find some labels automatically..." << std::endl;

    const auto guessed_labels = GuessPelvisPAOFragCutLabels(src_labels.GetPointer(),
                                                            true, true);

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

  if (need_to_find_femur_label)
  {
    vout << "looking up femur label..." << std::endl;

    ITKIndex femur_idx;
    ITKPoint femur_pt_itk;
    femur_pt_itk[0] = femur_pt[0];
    femur_pt_itk[1] = femur_pt[1];
    femur_pt_itk[2] = femur_pt[2];

    src_labels->TransformPhysicalPointToIndex(femur_pt_itk, femur_idx);
    femur_label = src_labels->GetPixel(femur_idx);
    vout << "determined to be " << static_cast<int>(femur_label) << std::endl;
  }

  vout << "Labels:\n   Pelvis: " << static_cast<int>(pelvis_label)
       << "\n     Frag: " << static_cast<int>(frag_label)
       << "\n      Cut: " << static_cast<int>(cut_label)
       << "\n    Femur: " << static_cast<int>(femur_label)
       << "\n  C-Femur: " << (use_con_femur_label ? fmt::format("{}", static_cast<int>(con_femur_label)) : std::string("N/A"))
       << std::endl;

  const int morph_open_len = po.get("morph-open-size").as_int32();
  if (morph_open_len > 0)
  {
    vout << "performing opening morphology on the input labels (kernel radius: "
         << morph_open_len << ") ..." << std::endl;
    
    using BallKernel = itk::BinaryBallStructuringElement<LabelType,3>;
    using MorphOpenFilter = itk::BinaryMorphologicalOpeningImageFilter<LabelImage,LabelImage,BallKernel>;

    BallKernel ball_kernel;
    ball_kernel.SetRadius(morph_open_len);
    ball_kernel.CreateStructuringElement();

    {
      MorphOpenFilter::Pointer open_filter = MorphOpenFilter::New();
      open_filter->SetInput(src_labels);
      open_filter->SetKernel(ball_kernel);
      open_filter->SetBackgroundValue(0);
      open_filter->SetForegroundValue(pelvis_label);

      open_filter->Update();
      src_labels = open_filter->GetOutput();
    }

    {
      MorphOpenFilter::Pointer open_filter = MorphOpenFilter::New();
      open_filter->SetInput(src_labels);
      open_filter->SetKernel(ball_kernel);
      open_filter->SetBackgroundValue(0);
      open_filter->SetForegroundValue(frag_label);

      open_filter->Update();
      src_labels = open_filter->GetOutput();
    }
    
    {
      MorphOpenFilter::Pointer open_filter = MorphOpenFilter::New();
      open_filter->SetInput(src_labels);
      open_filter->SetKernel(ball_kernel);
      open_filter->SetBackgroundValue(0);
      open_filter->SetForegroundValue(femur_label);

      open_filter->Update();
      src_labels = open_filter->GetOutput();
    }

    // This should not really need to be done
    //if (use_con_femur_label)
    //{
    //  MorphOpenFilter::Pointer open_filter = MorphOpenFilter::New();
    //  open_filter->SetInput(src_labels);
    //  open_filter->SetKernel(ball_kernel);
    //  open_filter->SetBackgroundValue(0);
    //  open_filter->SetForegroundValue(con_femur_label);
    //
    //  open_filter->Update();
    //  src_labels = open_filter->GetOutput();
    //}
    
    const std::string debug_morph_path = po.get("debug-write-open-labels");
    if (!debug_morph_path.empty())
    {
      vout << "writing opened label map to disk..." << std::endl;
      WriteITKImageToDisk(src_labels.GetPointer(), debug_morph_path);
    }
  }
  
  //////////////////////////////////////////////////////////////////////////////
  // Get the sampling parameters

  Pt3 frag_rot_means_rad;
  frag_rot_means_rad(0) = po.get("frag-rot-mean-x").as_double() * kDEG2RAD;
  frag_rot_means_rad(1) = po.get("frag-rot-mean-y").as_double() * kDEG2RAD;
  frag_rot_means_rad(2) = po.get("frag-rot-mean-z").as_double() * kDEG2RAD;
 
  if (!no_param_adjust)
  { 
    vout << "adjusting mean rotations of fragment for current size (ensuring signs)..." << std::endl;
    frag_rot_means_rad(0) = std::abs(frag_rot_means_rad(0));
    frag_rot_means_rad(1) = (is_left ?  1 : -1) * std::abs(frag_rot_means_rad(1));
    frag_rot_means_rad(2) = (is_left ? -1 :  1) * std::abs(frag_rot_means_rad(2));
  }

  Pt3 frag_rot_std_devs_rad;
  frag_rot_std_devs_rad(0) = po.get("frag-rot-std-x").as_double() * kDEG2RAD;
  frag_rot_std_devs_rad(1) = po.get("frag-rot-std-y").as_double() * kDEG2RAD;
  frag_rot_std_devs_rad(2) = po.get("frag-rot-std-z").as_double() * kDEG2RAD;
  
  Pt3 femur_rot_means_rad    = Pt3::Zero();
  Pt3 femur_rot_std_devs_rad = Pt3::Zero();

  femur_rot_means_rad(0)    = po.get("femur-rot-mean-x").as_double() * kDEG2RAD;
  femur_rot_std_devs_rad(0) = po.get("femur-rot-std-x").as_double() * kDEG2RAD;
  femur_rot_means_rad(1)    = po.get("femur-rot-mean-y").as_double() * kDEG2RAD;
  femur_rot_std_devs_rad(1) = po.get("femur-rot-std-y").as_double() * kDEG2RAD;
  femur_rot_means_rad(2)    = po.get("femur-rot-mean-z").as_double() * kDEG2RAD;
  femur_rot_std_devs_rad(2) = po.get("femur-rot-std-z").as_double() * kDEG2RAD;
  
  if (!no_param_adjust)
  { 
    vout << "adjusting mean rotations of femur for current size (ensuring signs)..." << std::endl;
    femur_rot_means_rad(0) = -1 * std::abs(femur_rot_means_rad(0));
  }

  if (no_rot)
  {
    frag_rot_means_rad.setZero();
    frag_rot_std_devs_rad.setZero();
  
    femur_rot_means_rad.setZero();
    femur_rot_std_devs_rad.setZero();

    vout << "NO ROTATION FLAG PASSED!" << std::endl;
  }

  vout << "Frag/Femur Rotation mean angles (deg):\n" << (frag_rot_means_rad / kDEG2RAD) << std::endl;
  vout << "Frag/Femur Rotation angle std. devs. (deg)\n: " << (frag_rot_std_devs_rad / kDEG2RAD) << std::endl;

  vout << "Femur Only Rotation mean angle (deg):\n" << (femur_rot_means_rad / kDEG2RAD) << std::endl;
  vout << "Femur Only Rotation angle std. dev. (deg):\n" << (femur_rot_std_devs_rad / kDEG2RAD) << std::endl;

  Pt3 trans_means;
  Pt3 trans_stds;

  trans_means[0] = po.get("trans-mean-x").as_double();
  trans_means[1] = po.get("trans-mean-y").as_double();
  trans_means[2] = po.get("trans-mean-z").as_double();

  if (!no_medialize)
  {
    // medialize the fragment - e.g. tend to move it medially, not outwards

    if (is_left)
    {
      // we want the fragment to move to the right
      if (trans_means[0] > 0)
      {
        trans_means[0] *= -1;
      }
    }
    else
    {
      // we want the fragment to move to the left
      if (trans_means[0] < 0)
      {
        trans_means[0] *= -1;
      }
    }
  }

  vout << "Translation Mean:\n" << trans_means << std::endl;
  trans_stds[0] = po.get("trans-std-x").as_double();
  trans_stds[1] = po.get("trans-std-y").as_double();
  trans_stds[2] = po.get("trans-std-z").as_double();

  if (no_trans)
  {
    vout << "NO TRANSLATION FLAG PASSED!" << std::endl;
    trans_stds.setZero();
    trans_means.setZero();
  }

  vout << "Translation Std. Devs.:\n" << trans_stds << std::endl;
  
  // sampling object - will be reused with different parameters for
  // fragment and femur.
  SampleValidLabelWarpsFn sample_warps;
  
  sample_warps.inter_to_vol_xform = app_to_vol;
  sample_warps.num_xforms = num_xforms;
  sample_warps.check_for_collision = !no_check_collisions;

  // set up sampling params for frag+femur
  if (uniform_sampling)
  {
    auto* sample_params = new SampleValidLabelWarpsFn::SampleRepoParamsAllUniform;

    sample_params->rot_lower(0) = frag_rot_means_rad(0) - frag_rot_std_devs_rad(0);
    sample_params->rot_lower(1) = frag_rot_means_rad(1) - frag_rot_std_devs_rad(1);
    sample_params->rot_lower(2) = frag_rot_means_rad(2) - frag_rot_std_devs_rad(2);

    sample_params->rot_upper(0) = frag_rot_means_rad(0) + frag_rot_std_devs_rad(0);
    sample_params->rot_upper(1) = frag_rot_means_rad(1) + frag_rot_std_devs_rad(1);
    sample_params->rot_upper(2) = frag_rot_means_rad(2) + frag_rot_std_devs_rad(2);

    sample_params->trans_lower(0) = trans_means(0) - trans_stds(0);
    sample_params->trans_lower(1) = trans_means(1) - trans_stds(1);
    sample_params->trans_lower(2) = trans_means(2) - trans_stds(2);
  
    sample_params->trans_upper(0) = trans_means(0) + trans_stds(0);
    sample_params->trans_upper(1) = trans_means(1) + trans_stds(1);
    sample_params->trans_upper(2) = trans_means(2) + trans_stds(2);

    sample_warps.param_sampler.reset(sample_params);
  }
  else if (logn_mix_sampling)
  {
    // use mix of lognormal and normal
    
    auto* sample_params = new SampleValidLabelWarpsFn::SampleRepoParamsLogNormalMixFrag;

    sample_params->rot_m = frag_rot_means_rad;
    sample_params->rot_s = frag_rot_std_devs_rad;

    if (sample_params->rot_m(0) < 0.0)
    {
      // log-normal must always sample positive
      sample_params->rot_m(0) *= -1.0;
      sample_params->negate_rot_x = true;
    }
    else
    {
      sample_params->negate_rot_x = false;
    }

    if (sample_params->rot_m(1) < 0.0)
    {
      // log-normal must always sample positive
      sample_params->rot_m(1) *= -1.0;
      sample_params->negate_rot_y = true;
    }
    else
    {
      sample_params->negate_rot_y = false;
    }

    if (sample_params->rot_m(2) < 0.0)
    {
      // log-normal must always sample positive
      sample_params->rot_m(2) *= -1.0;
      sample_params->negate_rot_z = true;
    }
    else
    {
      sample_params->negate_rot_z = false;
    }

    sample_params->trans_mean    = trans_means;
    sample_params->trans_std_dev = trans_stds;

    sample_warps.param_sampler.reset(sample_params);
  }
  else
  {
    // use default of normal sampling

    auto* sample_params = new SampleValidLabelWarpsFn::SampleRepoParamsAllNormal;
    
    sample_params->rot_mean    = frag_rot_means_rad;
    sample_params->rot_std_dev = frag_rot_std_devs_rad;

    sample_params->trans_mean    = trans_means;
    sample_params->trans_std_dev = trans_stds;

    sample_warps.param_sampler.reset(sample_params);
  }

  //////////////////////////////////////////////////////////////////////////////
  // Sample movement of the fragment - this is a transformation from original
  // volume coordinates to a new volume coordinate frame
  // (This is important to keep straight for later ray casting).
  FrameTransformList delta_frag_xforms;

  sample_warps.mov_labels.insert(frag_label);
  sample_warps.mov_labels.insert(femur_label);

  sample_warps.fixed_labels.insert(pelvis_label);
  if (use_con_femur_label)
  {
    sample_warps.fixed_labels.insert(con_femur_label);
  }

  vout << "sampling frag + femur..." << std::endl;
 
  sample_warps.labels = src_labels.GetPointer();

  sample_warps();

  delta_frag_xforms = sample_warps.delta_inter_xforms;

  vout << "Frag+Femur Rejection Prob.: "
       << fmt::format("{:.8f}", sample_warps.rejection_prob())
       << " " << sample_warps.num_xforms << " / "
       << sample_warps.num_samples << " samples accepted."
       << std::endl;

  if (!frag_params_csv_path.empty())
  {
    vout << "writing frag params CSV..." << std::endl;
    WriteCSVFile(frag_params_csv_path, sample_warps.xform_vals);
  }

  vout << "Writing Sampled Frag+Femur Transforms:\n";
  for (unsigned long xform_idx = 0; xform_idx < num_xforms; ++xform_idx)
  {
    CoordScalar rot_ang   = 0;
    CoordScalar trans_mag = 0;
    std::tie(rot_ang,trans_mag) = ComputeRotAngTransMag(delta_frag_xforms[xform_idx]);

    vout << "Transform " << xform_idx << ": rotation: " << (rot_ang / kDEG2RAD)
         << " (deg), translation: " << trans_mag << std::endl;

    WriteITKAffineTransform(fmt::format("{}_frag_{}.h5", xforms_prefix, xform_idx),
                            delta_frag_xforms[xform_idx]);
  }

  //////////////////////////////////////////////////////////////////////////////
  // Sample movement (rotation) of the femur... we are cheating a little here,
  // in that we're not updating the label map to reflect the new position of the
  // fragment, but it shouldn't have moved all that much, and the femur will most
  // likely not be colliding with the pelvis in either case for small rotations
  // Once again, this is a transformation from original volume coordinates to a
  // new volume coordinate frame (This is important to keep straight for later ray casting).

  FrameTransformList delta_femur_xforms;

  sample_warps.mov_labels.clear();
  sample_warps.mov_labels.insert(femur_label);

  sample_warps.fixed_labels.clear();
  sample_warps.fixed_labels.insert(frag_label);
  sample_warps.fixed_labels.insert(pelvis_label);
  if (use_con_femur_label)
  {
    sample_warps.fixed_labels.insert(con_femur_label);
  }

  // set up sampling params for femur
  if (uniform_sampling)
  {
    auto* sample_params = new SampleValidLabelWarpsFn::SampleRepoParamsAllUniform;

    sample_params->rot_lower(0) = femur_rot_means_rad(0) - femur_rot_std_devs_rad(0);
    sample_params->rot_lower(1) = femur_rot_means_rad(1) - femur_rot_std_devs_rad(1);
    sample_params->rot_lower(2) = femur_rot_means_rad(2) - femur_rot_std_devs_rad(2);

    sample_params->rot_upper(0) = femur_rot_means_rad(0) + femur_rot_std_devs_rad(0);
    sample_params->rot_upper(1) = femur_rot_means_rad(1) + femur_rot_std_devs_rad(1);
    sample_params->rot_upper(2) = femur_rot_means_rad(2) + femur_rot_std_devs_rad(2);

    sample_params->trans_lower.setZero();
    sample_params->trans_upper.setZero();

    sample_warps.param_sampler.reset(sample_params);
  }
  else if (logn_mix_sampling)
  {
    // use mix of lognormal and normal
    
    auto* sample_params = new SampleValidLabelWarpsFn::SampleRepoParamsLogNormalMixFemur;

    sample_params->rot_m = femur_rot_means_rad;
    sample_params->rot_s = femur_rot_std_devs_rad;

    if (sample_params->rot_m(0) < 0.0)
    {
      // log-normal must always sample positive
      sample_params->rot_m(0) *= -1.0;
      sample_params->negate_rot_x = true;
    }
    else
    {
      sample_params->negate_rot_x = false;
    }
  }
  else
  {
    // use default of normal sampling

    auto* sample_params = new SampleValidLabelWarpsFn::SampleRepoParamsAllNormal;
    
    sample_params->rot_mean    = femur_rot_means_rad;
    sample_params->rot_std_dev = femur_rot_std_devs_rad;

    sample_params->trans_mean.setZero();
    sample_params->trans_std_dev.setZero();

    sample_warps.param_sampler.reset(sample_params);
  }

  vout << "sampling femur..." << std::endl;
  
  sample_warps();

  delta_femur_xforms = sample_warps.delta_inter_xforms;

  vout << "Femur Rejection Prob.: "
       << fmt::format("{:.8f}", sample_warps.rejection_prob())
       << " " << sample_warps.num_xforms << " / "
       << sample_warps.num_samples << " samples accepted."
       << std::endl;
  
  if (!femur_params_csv_path.empty())
  {
    vout << "writing femur params CSV..." << std::endl;
    WriteCSVFile(femur_params_csv_path, sample_warps.xform_vals);
  }

  vout << "Writing Sampled Femur Transforms:\n";
  for (unsigned long xform_idx = 0; xform_idx < num_xforms; ++xform_idx)
  {
    if (!use_frag_rot_for_femur)
    {
      // undo any rotation from the fragment
      FrameTransform tmp_femur_xform = delta_femur_xforms[xform_idx];
      
      FrameTransform frag_trans = delta_frag_xforms[xform_idx];
      frag_trans.matrix().block(0,0,3,3).setIdentity();
      
      delta_femur_xforms[xform_idx] = delta_frag_xforms[xform_idx].inverse() * frag_trans * tmp_femur_xform;
    }

    CoordScalar rot_ang   = 0;
    CoordScalar trans_mag = 0;
    std::tie(rot_ang,trans_mag) = ComputeRotAngTransMag(delta_femur_xforms[xform_idx]);

    vout << "Transform " << xform_idx << ": rotation: " << (rot_ang / kDEG2RAD) << " (deg), translation: " << trans_mag << std::endl;

    WriteITKAffineTransform(fmt::format("{}_femur_{}.h5", xforms_prefix, xform_idx),
                            delta_femur_xforms[xform_idx]);
  }

  vout << "exiting..." << std::endl;

  return kEXIT_VAL_SUCCESS;
}
