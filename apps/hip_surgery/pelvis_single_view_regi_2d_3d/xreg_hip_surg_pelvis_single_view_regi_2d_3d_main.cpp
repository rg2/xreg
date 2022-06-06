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

#include "xregProgOptUtils.h"
#include "xregLandmarkFiles.h"
#include "xregITKIOUtils.h"
#include "xregITKLabelUtils.h"
#include "xregLandmarkMapUtils.h"
#include "xregAnatCoordFrames.h"
#include "xregH5ProjDataIO.h"
#include "xregRayCastProgOpts.h"
#include "xregRayCastInterface.h"
#include "xregImgSimMetric2DPatchCommon.h"
#include "xregImgSimMetric2DGradImgParamInterface.h"
#include "xregImgSimMetric2DProgOpts.h"
#include "xregHUToLinAtt.h"
#include "xregProjPreProc.h"
#include "xregPnPUtils.h"
#include "xregMultiObjMultiLevel2D3DRegi.h"
#include "xregMultiObjMultiLevel2D3DRegiDebug.h"
#include "xregSE3OptVars.h"
#include "xregIntensity2D3DRegiCMAES.h"
#include "xregIntensity2D3DRegiBOBYQA.h"
#include "xregRegi2D3DPenaltyFnSE3EulerDecomp.h"
#include "xregNormDist.h"

int main(int argc, char* argv[])
{
  using namespace xreg;
  
  constexpr int kEXIT_VAL_SUCCESS = 0;
  constexpr int kEXIT_VAL_BAD_USE = 1;

  // First, set up the program options

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Register a pelvis model to a single fluoroscopic view. "
              "The registration first solves a PnP problem using anatomical "
              "landmarks from the 3D volume and the 2D image. The PnP pose "
              "estimate is then used to initialize an intensity-based "
              "registration. "
              "This program may serve as a general template for performing "
              "single-object 2D/3D registration to a single view.");
  
  po.set_arg_usage("<Input CT vol.> <3D Landmarks> <Proj. Data File> "
                   "<output pose> [<output debug data file>]");
  po.set_min_num_pos_args(4);

  po.add("proj-idx", 'p', ProgOpts::kSTORE_UINT32, "proj-idx",
         "Index of the projection to register (the projection data file may store several projections)")
    << ProgOpts::uint32(0);

  po.add("vol-seg", 's', ProgOpts::kSTORE_STRING, "vol-seg",
         "Path to label map/segmentation of the pelvis. When no label map is provided "
         "the entire volume is used and treated as a single object.")
    << "";

  po.add("pelvis-label", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32, "pelvis-label",
         "Label of the pelvis in the label map.")
    << ProgOpts::uint32(1);

  po.add("lands-ras", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "lands-ras",
         "Read landmarks in RAS coordinates instead of LPS.")
    << false;
  
  po.add("no-log-remap", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-log-remap",
         "Do NOT perform log remapping of the projection intensities during pre-processing.")
    << false;

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
  
  const std::string ct_path        = po.pos_args()[0];
  const std::string fcsv_3d_path   = po.pos_args()[1];
  const std::string proj_data_path = po.pos_args()[2];
  const std::string dst_pose_path  = po.pos_args()[3];

  const bool use_identity_for_init_cam_to_vol = fcsv_3d_path == "-";

  const bool save_debug = po.pos_args().size() > 4;

  const std::string dst_debug_path = save_debug ? po.pos_args()[4] : std::string();

  const bool lands_ras = po.get("lands-ras");

  const size_type proj_idx = po.get("proj-idx").as_uint32();

  const std::string seg_path = po.get("vol-seg");

  const bool use_seg = !seg_path.empty();

  const unsigned char pelvis_label = static_cast<unsigned char>(po.get("pelvis-label").as_uint32());

  const bool no_log_remap = po.get("no-log-remap");

  //////////////////////////////////////////////////////////////////////////////
  // Read in input intensity volume
  
  vout << "reading in source intensity volume..." << std::endl;
  auto ct_intens = ReadITKImageFromDisk<RayCaster::Vol>(ct_path);

  vout << "converting HU --> Lin. Att." << std::endl;
  ct_intens = HUToLinAtt(ct_intens.GetPointer());

  if (use_seg)
  {
    vout << "reading label map..." << std::endl;
    auto ct_labels = ReadITKImageFromDisk<itk::Image<unsigned char,3>>(seg_path);
    
    vout << "cropping intensity volume tightly around pelvis label ("
         << static_cast<int>(pelvis_label) << ")..." << std::endl;

    ct_intens = MakeVolListFromVolAndLabels(ct_intens.GetPointer(), ct_labels.GetPointer(),
                                            { pelvis_label }, 0)[0];
  }

  //////////////////////////////////////////////////////////////////////////////
  // Get the landmarks

  LandMap3 lands_3d;

  if (!use_identity_for_init_cam_to_vol)
  {
    vout << "reading 3D landmarks..." << std::endl;
    lands_3d = ReadLandmarksFileNamePtMap(fcsv_3d_path, !lands_ras);

    vout << "3D Landmarks:\n";
    PrintLandmarkMap(lands_3d, vout);
  }

  ProjPreProc proj_preproc;
  proj_preproc.params.no_log_remap = no_log_remap;

  {
    vout << "reading projection data..." << std::endl;
    DeferredProjReader proj_reader(proj_data_path);
    
    const auto proj_metas = proj_reader.proj_data_F32();

    xregASSERT(proj_idx < proj_metas.size());

    proj_preproc.input_projs = { proj_metas[proj_idx] };

    proj_preproc.input_projs[0].img = proj_reader.read_proj_F32(proj_idx);
  }

  proj_preproc.set_debug_output_stream(vout, verbose);
  
  vout << "preprocessing projection..." << std::endl;
  proj_preproc();

  vout << "2D Landmarks:\n";
  PrintLandmarkMap(proj_preproc.output_projs[0].landmarks, vout);
  
  FrameTransform init_cam_to_vol = FrameTransform::Identity();

  if (!use_identity_for_init_cam_to_vol)
  {
    vout << "solving PnP problem for initial regi estimate..." << std::endl;
    init_cam_to_vol = PnPPOSITAndReprojCMAES(proj_preproc.output_projs[0].cam,
                                              lands_3d, proj_preproc.output_projs[0].landmarks);
  }
  else
  {
    vout << "using identity for initial regi estimate" << std::endl;
  }

  vout << "setting up multi-level regi object.." << std::endl;

  MultiLevelMultiObjRegi ml_mo_regi;
  ml_mo_regi.set_debug_output_stream(vout, verbose);

  // calling this with save_debug==true will allocate the debug object
  ml_mo_regi.set_save_debug_info(save_debug);

  ml_mo_regi.vol_names = { "Pelvis" };
  
  ml_mo_regi.vols = { ct_intens };

  ml_mo_regi.fixed_proj_data = proj_preproc.output_projs;

  // setup the camera reference frame which we optimize in
  auto cam_align_ref = std::make_shared<MultiLevelMultiObjRegi::CamAlignRefFrameWithCurPose>();
  cam_align_ref->vol_idx = 0;
  cam_align_ref->center_of_rot_wrt_vol = ITKVol3DCenterAsPhysPt(ct_intens.GetPointer());
  cam_align_ref->cam_extrins = ml_mo_regi.fixed_proj_data[0].cam.extrins;

  ml_mo_regi.ref_frames = { cam_align_ref };

  // se(3) lie algebra vector space for optimization
  auto se3_vars = std::make_shared<SE3OptVarsLieAlg>();

  ml_mo_regi.init_cam_to_vols = { init_cam_to_vol };

  ml_mo_regi.levels.resize(2);

  using UseCurEstForInit = MultiLevelMultiObjRegi::Level::SingleRegi::InitPosePrevPoseEst;

  // Level 1
  {
    vout << "  setting regi level 1..." << std::endl;
    
    auto& lvl = ml_mo_regi.levels[0];

    lvl.fixed_imgs_to_use = { 0 };

    lvl.ds_factor = 0.125;
    
    vout << "    setting up ray caster..." << std::endl;
    lvl.ray_caster = LineIntRayCasterFromProgOpts(po);
    
    vout << "    setting up sim metric..." << std::endl;
    auto sm = PatchGradNCCSimMetricFromProgOpts(po);
    {
      auto* grad_sm = dynamic_cast<ImgSimMetric2DGradImgParamInterface*>(sm.get());

      grad_sm->set_smooth_img_before_sobel_kernel_radius(5);
    }
   
    {
      auto* patch_sm = dynamic_cast<ImgSimMetric2DPatchCommon*>(sm.get());
      xregASSERT(patch_sm);

      patch_sm->set_patch_radius(std::lround(lvl.ds_factor * 41));
      patch_sm->set_patch_stride(1);
    }

    lvl.sim_metrics = { sm };

    lvl.regis.resize(1);

    auto& regi = lvl.regis[0];

    regi.mov_vols    = { 0 };  // pelvis vol pose is optimized over
    regi.ref_frames  = { 0 };  // use ref frame that is camera aligned
    regi.static_vols = { };    // No other objects exist

    // use the current estimate of the pelvis as the initialization at this phase
    auto init_guess_fn = std::make_shared<UseCurEstForInit>();
    init_guess_fn->vol_idx = 0;
    regi.init_mov_vol_poses = { init_guess_fn };

    auto cmaes_regi = std::make_shared<Intensity2D3DRegiCMAES>();
    cmaes_regi->set_opt_vars(se3_vars);
    cmaes_regi->set_opt_x_tol(0.01);
    cmaes_regi->set_opt_obj_fn_tol(0.01);
    cmaes_regi->set_pop_size(100);
    cmaes_regi->set_sigma({ 15 * kDEG2RAD, 15 * kDEG2RAD, 30 * kDEG2RAD, 50, 50, 100 });
    
    auto pen_fn = std::make_shared<Regi2D3DPenaltyFnSE3EulerDecomp>();
    pen_fn->rot_x_pdf   = std::make_shared<NormalDist1D>(0, 15 * kDEG2RAD);
    pen_fn->rot_y_pdf   = std::make_shared<NormalDist1D>(0, 15 * kDEG2RAD);
    pen_fn->rot_z_pdf   = std::make_shared<NormalDist1D>(0, 10 * kDEG2RAD);
    pen_fn->trans_x_pdf = std::make_shared<NormalDist1D>(0, 30);
    pen_fn->trans_y_pdf = std::make_shared<NormalDist1D>(0, 30);
    pen_fn->trans_z_pdf = std::make_shared<NormalDist1D>(0, 150);

    cmaes_regi->set_penalty_fn(pen_fn);
    cmaes_regi->set_img_sim_penalty_coefs(0.9, 0.1);

    regi.regi = cmaes_regi;
  }
  
  // Level 2
  {
    vout << "  setting regi level 2..." << std::endl;
    
    auto& lvl = ml_mo_regi.levels[1];
    
    lvl.fixed_imgs_to_use = { 0 };

    lvl.ds_factor = 0.25;
    
    vout << "    setting up ray caster..." << std::endl;
    lvl.ray_caster = LineIntRayCasterFromProgOpts(po);
    
    vout << "    setting up sim metric..." << std::endl;
    auto sm = PatchGradNCCSimMetricFromProgOpts(po);

    {
      auto* grad_sm = dynamic_cast<ImgSimMetric2DGradImgParamInterface*>(sm.get());

      grad_sm->set_smooth_img_before_sobel_kernel_radius(5);
    }
   
    {
      auto* patch_sm = dynamic_cast<ImgSimMetric2DPatchCommon*>(sm.get());
      xregASSERT(patch_sm);

      patch_sm->set_patch_radius(std::lround(lvl.ds_factor * 41));
      patch_sm->set_patch_stride(1);
    }

    lvl.sim_metrics = { sm };

    lvl.regis.resize(1);

    auto& regi = lvl.regis[0];

    regi.mov_vols    = { 0 };  // pelvis vol pose is optimized over
    regi.ref_frames  = { 0 };  // use ref frame that is camera aligned
    regi.static_vols = { };    // No other objects exist

    // use the current estimate of the pelvis as the initialization at this phase
    auto init_guess_fn = std::make_shared<UseCurEstForInit>();
    init_guess_fn->vol_idx = 0;
    regi.init_mov_vol_poses = { init_guess_fn };

    auto bobyqa_regi = std::make_shared<Intensity2D3DRegiBOBYQA>();
    bobyqa_regi->set_opt_vars(se3_vars);
    bobyqa_regi->set_opt_x_tol(0.0001);
    bobyqa_regi->set_opt_obj_fn_tol(0.0001);
    bobyqa_regi->set_bounds({ 2.5 * kDEG2RAD, 2.5 * kDEG2RAD, 2.5 * kDEG2RAD,
                              5, 5, 10 });
    
    lvl.regis[0].regi = bobyqa_regi;
  }

  if (save_debug)
  {
    vout << "  setting regi debug info..." << std::endl;

    DebugRegiResultsMultiLevel::VolPathInfo debug_vol_path;
    debug_vol_path.vol_path = ct_path;
    
    if (use_seg)
    {
      debug_vol_path.label_vol_path = seg_path;
      debug_vol_path.labels_used    = { pelvis_label };
    }
    
    ml_mo_regi.debug_info->vols = { debug_vol_path };

    DebugRegiResultsMultiLevel::ProjDataPathInfo debug_proj_path;
    debug_proj_path.path = proj_data_path;
    debug_proj_path.projs_used = { proj_idx };

    ml_mo_regi.debug_info->fixed_projs = debug_proj_path;
    
    ml_mo_regi.debug_info->proj_pre_proc_info = proj_preproc.params;

    ml_mo_regi.debug_info->regi_names = { { "CMA-ES" }, { "BOBYQA" } };
  }

  vout << "running regi..." << std::endl;
  ml_mo_regi.run();

  vout << "writing pose to disk..." << std::endl;
  WriteITKAffineTransform(dst_pose_path, ml_mo_regi.cur_cam_to_vols[0]);

  if (save_debug)
  {
    vout << "writing debug info to disk..." << std::endl;
    WriteMultiLevel2D3DRegiDebugToDisk(*ml_mo_regi.debug_info, dst_debug_path);
  }

  vout << "exiting..." << std::endl;
  
  return kEXIT_VAL_SUCCESS;
}
