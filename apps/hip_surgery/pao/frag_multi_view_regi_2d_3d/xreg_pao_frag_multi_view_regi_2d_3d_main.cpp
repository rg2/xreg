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
#include "xregRegi2D3DPenaltyFnSE3Mag.h"
#include "xregFoldNormDist.h"
#include "xregHipSegUtils.h"

int main(int argc, char* argv[])
{
  using namespace xreg;
  
  constexpr int kEXIT_VAL_SUCCESS = 0;
  constexpr int kEXIT_VAL_BAD_USE = 1;

  // First, set up the program options

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Register pelvis, femur, and fragment models to a collection "
              "of fluoroscopic views. The relative poses of the views are "
              "assumed to be known and encoded in the projection data. The "
              "registration first solves a PnP problem using anatomical landmarks "
              "from the 3D volume and a single 2D view. The PnP pose "
              "estimate is then used to initialize an intensity-based, "
              "multiple-view, multiple-object registration. "
              "This program may serve as a general template for performing "
              "multi-object 2D/3D registration to a multiple views.");
  
  po.set_arg_usage("<Input CT vol.> <Input CT seg.> <side string> "
                   "<3D APP Landmarks> <3D Regi. Landmarks> <Proj. Data File> "
                   "<output pelvis pose> <output femur pose> <output frag. pose> "
                   "<output relative femur pose APP> <output relative frag. pose APP> "
                   "[<output debug data file>]");
  po.set_min_num_pos_args(11);

  po.add("pnp-proj-idx", 'p', ProgOpts::kSTORE_UINT32, "pnp-proj-idx",
         "Index of the projection used for solving the PnP problem.")
    << ProgOpts::uint32(0);

  po.add("lands-ras", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "lands-ras",
         "Read landmarks in RAS coordinates instead of LPS.")
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
  
  const std::string ct_path                 = po.pos_args()[0];
  const std::string seg_path                = po.pos_args()[1];
  const std::string side_str                = ToLowerCase(po.pos_args()[2]);
  const std::string app_lands_path          = po.pos_args()[3];
  const std::string regi_lands_path         = po.pos_args()[4];
  const std::string proj_data_path          = po.pos_args()[5];
  const std::string dst_pelvis_pose_path    = po.pos_args()[6];
  const std::string dst_femur_pose_path     = po.pos_args()[7];
  const std::string dst_frag_pose_path      = po.pos_args()[8];
  const std::string dst_rel_femur_pose_path = po.pos_args()[9];
  const std::string dst_rel_frag_pose_path  = po.pos_args()[10];

  const bool save_debug = po.pos_args().size() > 11;

  const std::string dst_debug_path = save_debug ? po.pos_args()[11] : std::string();

  const bool lands_ras = po.get("lands-ras");

  const size_type pnp_proj_idx = po.get("pnp-proj-idx").as_uint32();

  if ((side_str != "left") && (side_str != "right"))
  {
    std::cerr << "ERROR: side string must be either \"left\" or \"right\" - \""
              << side_str << "\" was passed." << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  const bool is_left = side_str == "left";
  
  //////////////////////////////////////////////////////////////////////////////
  // Get the landmarks

  vout << "reading 3D landmarks for regi..." << std::endl;
  const LandMap3 regi_lands_3d = ReadLandmarksFileNamePtMap(regi_lands_path, !lands_ras);

  vout << "reading landmarks for establishing APP..." << std::endl;
  const LandMap3 app_lands_3d = ReadLandmarksFileNamePtMap(app_lands_path, !lands_ras);

  vout << "3D Landmarks For Registration:\n";
  PrintLandmarkMap(regi_lands_3d, vout);
 
  vout << "3D Landmarks For APP:\n";
  PrintLandmarkMap(app_lands_3d, vout);
  
  const Pt3 femur_pt = app_lands_3d.find(fmt::format("FH-{}", is_left ? 'l' : 'r'))->second;

  //////////////////////////////////////////////////////////////////////////////
  // Read in input intensity volume
  
  vout << "reading in source intensity volume..." << std::endl;
  auto ct_intens = ReadITKImageFromDisk<RayCaster::Vol>(ct_path);

  vout << "converting HU --> Lin. Att." << std::endl;
  ct_intens = HUToLinAtt(ct_intens.GetPointer());

  vout << "reading label map..." << std::endl;
  auto ct_labels = ReadITKImageFromDisk<itk::Image<unsigned char,3>>(seg_path);
 
  unsigned char pelvis_label = 0;
  unsigned char femur_label  = 0;
  unsigned char frag_label   = 0;

  vout << "determing pelvis/femur/frag labels..." << std::endl;
  std::tie(pelvis_label,femur_label,frag_label,std::ignore) =
                      GuessPelvisFemurPAOFragLabels(ct_labels.GetPointer(),
                                                    femur_pt, true, true);

  vout << "cropping intensity volume tightly around labels:"
       << "\n  Pelvis: " << static_cast<int>(pelvis_label)
       << "\n   Femur: " << static_cast<int>(femur_label)
       << "\n    Frag: " << static_cast<int>(frag_label)
       << std::endl;

  auto ct_vols = MakeVolListFromVolAndLabels(ct_intens.GetPointer(), ct_labels.GetPointer(),
                                             { pelvis_label, femur_label, frag_label }, 0);

  ProjPreProc proj_preproc;
    
  vout << "reading projection data..." << std::endl;
  proj_preproc.input_projs = ReadProjDataH5F32FromDisk(proj_data_path);

  const size_type num_projs = proj_preproc.input_projs.size();

  xregASSERT(pnp_proj_idx < num_projs);

  proj_preproc.set_debug_output_stream(vout, verbose);
  
  vout << "preprocessing projection..." << std::endl;
  proj_preproc();

  vout << "2D Landmarks:\n";
  PrintLandmarkMap(proj_preproc.output_projs[pnp_proj_idx].landmarks, vout);

  vout << "solving PnP problem for initial pelvis regi estimate..." << std::endl;
  const FrameTransform init_cam_to_vol = PnPPOSITAndReprojCMAES(
                                            proj_preproc.output_projs[pnp_proj_idx].cam,
                                            regi_lands_3d, proj_preproc.output_projs[pnp_proj_idx].landmarks);

  vout << "setting up multi-level regi object.." << std::endl;

  MultiLevelMultiObjRegi ml_mo_regi;
  ml_mo_regi.set_debug_output_stream(vout, verbose);

  // calling this with save_debug==true will allocate the debug object
  ml_mo_regi.set_save_debug_info(save_debug);

  ml_mo_regi.vol_names = { "Pelvis", "Femur", "Frag." };
  
  ml_mo_regi.vols = ct_vols;

  ml_mo_regi.fixed_proj_data = proj_preproc.output_projs;

  // setup the camera reference frame which we optimize in
  const FrameTransform app_to_vol = AnteriorPelvicPlaneFromLandmarksMap(app_lands_3d,
                                                is_left ? kAPP_ORIGIN_LEFT_FH : kAPP_ORIGIN_RIGHT_FH);

  ml_mo_regi.ref_frames = { MultiLevelMultiObjRegi::MakeStaticRefFrame(app_to_vol.inverse(), true) };

  // se(3) lie algebra vector space for optimization
  auto se3_vars = std::make_shared<SE3OptVarsLieAlg>();

  ml_mo_regi.init_cam_to_vols = { init_cam_to_vol, init_cam_to_vol, init_cam_to_vol };

  ml_mo_regi.levels.resize(2);

  using UseCurEstForInit = MultiLevelMultiObjRegi::Level::SingleRegi::InitPosePrevPoseEst;

  // Level 1
  {
    vout << "  setting regi level 1..." << std::endl;
    
    auto& lvl = ml_mo_regi.levels[0];

    // use all views
    lvl.fixed_imgs_to_use.resize(num_projs);
    std::iota(lvl.fixed_imgs_to_use.begin(), lvl.fixed_imgs_to_use.end(), 0);

    lvl.ds_factor = 0.125;
    
    vout << "    setting up ray caster..." << std::endl;
    lvl.ray_caster = LineIntRayCasterFromProgOpts(po);
    
    vout << "    setting up sim metrics..." << std::endl;
    lvl.sim_metrics.reserve(num_projs);
    
    for (size_type proj_idx = 0; proj_idx < num_projs; ++proj_idx)
    {
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

      lvl.sim_metrics.push_back(sm);
    }

    lvl.regis.resize(3);

    // 0 --> regi for pelvis
    // 1 --> regi for femur
    // 2 --> regi for frag

    // setup pelvis regi
    {
      auto& regi = lvl.regis[0];

      regi.mov_vols    = { 0 };  // pelvis vol pose is optimized over
      regi.ref_frames  = { 0 };  // use APP ref frame with origin at femoral head center
      regi.static_vols = { };    // No other objects have been registered yet 

      // use the current estimate of the pelvis as the initialization at this phase
      auto init_guess_fn = std::make_shared<UseCurEstForInit>();
      init_guess_fn->vol_idx = 0;
      regi.init_mov_vol_poses = { init_guess_fn };

      auto cmaes_regi = std::make_shared<Intensity2D3DRegiCMAES>();
      cmaes_regi->set_opt_vars(se3_vars);
      cmaes_regi->set_opt_x_tol(0.01);
      cmaes_regi->set_opt_obj_fn_tol(0.01);
      cmaes_regi->set_pop_size(100);
      cmaes_regi->set_sigma({ 0.3, 0.3, 0.3, 5, 5, 5 });
     
      auto pen_fn = std::make_shared<Regi2D3DPenaltyFnSE3Mag>();
      pen_fn->rot_pdfs_per_obj   = { std::make_shared<FoldNormDist>(10 * kDEG2RAD, 10 * kDEG2RAD) };
      pen_fn->trans_pdfs_per_obj = { std::make_shared<FoldNormDist>(50, 50) };

      cmaes_regi->set_penalty_fn(pen_fn);
      cmaes_regi->set_img_sim_penalty_coefs(0.9, 0.1);

      regi.regi = cmaes_regi;
    }
    
    // setup femur regi
    {
      auto& regi = lvl.regis[1];

      regi.mov_vols    = { 1 };  // femur vol pose is optimized over
      regi.ref_frames  = { 0 };  // use APP ref frame with origin at femoral head center
      regi.static_vols = { 0 };  // The pelvis has been approximately registered,
                                 // keep it projected in the background using its current pose est.

      // use the current estimate of the pelvis pose to initialize femur regi.
      auto init_guess_fn = std::make_shared<UseCurEstForInit>();
      init_guess_fn->vol_idx = 0;
      regi.init_mov_vol_poses = { init_guess_fn };

      // use the current pose estimate of the pelvis for its static bg pose
      regi.static_vol_poses = { init_guess_fn };

      auto cmaes_regi = std::make_shared<Intensity2D3DRegiCMAES>();
      cmaes_regi->set_opt_vars(se3_vars);
      cmaes_regi->set_opt_x_tol(0.01);
      cmaes_regi->set_opt_obj_fn_tol(0.01);
      cmaes_regi->set_pop_size(100);
      cmaes_regi->set_sigma({ 0.3, 0.3, 0.3, 5, 5, 5 });
     
      auto pen_fn = std::make_shared<Regi2D3DPenaltyFnSE3Mag>();
      pen_fn->rot_pdfs_per_obj   = { std::make_shared<FoldNormDist>(45 * kDEG2RAD, 45 * kDEG2RAD) };
      pen_fn->trans_pdfs_per_obj = { std::make_shared<FoldNormDist>(10, 10) };

      cmaes_regi->set_penalty_fn(pen_fn);
      cmaes_regi->set_img_sim_penalty_coefs(0.9, 0.1);

      regi.regi = cmaes_regi;
    }
    
    // setup frag regi
    {
      auto& regi = lvl.regis[2];

      regi.mov_vols    = { 2 };    // frag vol pose is optimized over
      regi.ref_frames  = { 0 };    // use APP ref frame with origin at femoral head center
      regi.static_vols = { 0,1 };  // The pelvis and femur have been approximately registered,
                                   // keep them projected in the background using its current pose est.

      // use the current estimate of the pelvis pose to initialize fragment regi.
      auto init_guess_fn = std::make_shared<UseCurEstForInit>();
      init_guess_fn->vol_idx = 0;
      regi.init_mov_vol_poses = { init_guess_fn };

      // use the current pose estimates of the pelvis and femur for their static bg poses
      auto femur_static_pose_fn = std::make_shared<UseCurEstForInit>();
      femur_static_pose_fn->vol_idx = 1;

      regi.static_vol_poses = { init_guess_fn, femur_static_pose_fn };

      auto cmaes_regi = std::make_shared<Intensity2D3DRegiCMAES>();
      cmaes_regi->set_opt_vars(se3_vars);
      cmaes_regi->set_opt_x_tol(0.01);
      cmaes_regi->set_opt_obj_fn_tol(0.01);
      cmaes_regi->set_pop_size(100);
      cmaes_regi->set_sigma({ 0.3, 0.3, 0.3, 5, 5, 5 });
     
      auto pen_fn = std::make_shared<Regi2D3DPenaltyFnSE3Mag>();
      pen_fn->rot_pdfs_per_obj   = { std::make_shared<FoldNormDist>(25 * kDEG2RAD, 25 * kDEG2RAD) };
      pen_fn->trans_pdfs_per_obj = { std::make_shared<FoldNormDist>(10, 10) };

      cmaes_regi->set_penalty_fn(pen_fn);
      cmaes_regi->set_img_sim_penalty_coefs(0.9, 0.1);

      regi.regi = cmaes_regi;
    }
  }
  
  // Level 2
  {
    vout << "  setting regi level 2..." << std::endl;
    
    auto& lvl = ml_mo_regi.levels[1];
    
    // use all views
    lvl.fixed_imgs_to_use.resize(num_projs);
    std::iota(lvl.fixed_imgs_to_use.begin(), lvl.fixed_imgs_to_use.end(), 0);

    lvl.ds_factor = 0.25;
    
    vout << "    setting up ray caster..." << std::endl;
    lvl.ray_caster = LineIntRayCasterFromProgOpts(po);
    
    vout << "    setting up sim metrics..." << std::endl;
    lvl.sim_metrics.reserve(num_projs);
    
    for (size_type proj_idx = 0; proj_idx < num_projs; ++proj_idx)
    {
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

      lvl.sim_metrics.push_back(sm);
    }

    lvl.regis.resize(4);

    // pelvis regi
    {
      auto& regi = lvl.regis[0];

      regi.mov_vols    = { 0 };    // pelvis vol pose is optimized over
      regi.ref_frames  = { 0 };    // use APP ref frame with origin at femoral head center
      regi.static_vols = { 1,2 };  // Keep the femur and fragment fixed in the bg at their
                                   // current pose estimates while refining the pelvis pose

      // use the current estimate of the pelvis as the initialization at this phase
      auto init_guess_fn = std::make_shared<UseCurEstForInit>();
      init_guess_fn->vol_idx = 0;
      regi.init_mov_vol_poses = { init_guess_fn };
      
      auto femur_static_pose_fn = std::make_shared<UseCurEstForInit>();
      femur_static_pose_fn->vol_idx = 1;
      
      auto frag_static_pose_fn = std::make_shared<UseCurEstForInit>();
      frag_static_pose_fn->vol_idx = 2;

      regi.static_vol_poses = { femur_static_pose_fn, frag_static_pose_fn };

      auto bobyqa_regi = std::make_shared<Intensity2D3DRegiBOBYQA>();
      bobyqa_regi->set_opt_vars(se3_vars);
      bobyqa_regi->set_opt_x_tol(0.001);
      bobyqa_regi->set_opt_obj_fn_tol(0.001);
      bobyqa_regi->set_bounds({ 5 * kDEG2RAD, 5 * kDEG2RAD, 5 * kDEG2RAD, 30, 30, 30 });
      
      regi.regi = bobyqa_regi;
    }
    
    // femur regi
    {
      auto& regi = lvl.regis[1];

      regi.mov_vols    = { 1 };    // femur vol pose is optimized over
      regi.ref_frames  = { 0 };    // use APP ref frame with origin at femoral head center
      regi.static_vols = { 0,2 };  // Keep the pelvis and fragment fixed in the bg at their
                                   // current pose estimates while refining the femur pose

      // use the current estimate of the femur as the initialization at this phase
      auto init_guess_fn = std::make_shared<UseCurEstForInit>();
      init_guess_fn->vol_idx = 1;
      regi.init_mov_vol_poses = { init_guess_fn };
      
      auto pelvis_static_pose_fn = std::make_shared<UseCurEstForInit>();
      pelvis_static_pose_fn->vol_idx = 0;
      
      auto frag_static_pose_fn = std::make_shared<UseCurEstForInit>();
      frag_static_pose_fn->vol_idx = 2;

      regi.static_vol_poses = { pelvis_static_pose_fn, frag_static_pose_fn };

      auto bobyqa_regi = std::make_shared<Intensity2D3DRegiBOBYQA>();
      bobyqa_regi->set_opt_vars(se3_vars);
      bobyqa_regi->set_opt_x_tol(0.001);
      bobyqa_regi->set_opt_obj_fn_tol(0.001);
      bobyqa_regi->set_bounds({ 10 * kDEG2RAD, 10 * kDEG2RAD, 10 * kDEG2RAD, 30, 30, 30 });
      
      regi.regi = bobyqa_regi;
    }
    
    // frag regi
    {
      auto& regi = lvl.regis[2];

      regi.mov_vols    = { 2 };    // frag vol pose is optimized over
      regi.ref_frames  = { 0 };    // use APP ref frame with origin at femoral head center
      regi.static_vols = { 0,1 };  // Keep the pelvis and femur fixed in the bg at their
                                   // current pose estimates while refining the frag pose

      // use the current estimate of the fragment as the initialization at this phase
      auto init_guess_fn = std::make_shared<UseCurEstForInit>();
      init_guess_fn->vol_idx = 2;
      regi.init_mov_vol_poses = { init_guess_fn };
      
      auto pelvis_static_pose_fn = std::make_shared<UseCurEstForInit>();
      pelvis_static_pose_fn->vol_idx = 0;
      
      auto femur_static_pose_fn = std::make_shared<UseCurEstForInit>();
      femur_static_pose_fn->vol_idx = 1;

      regi.static_vol_poses = { pelvis_static_pose_fn, femur_static_pose_fn };

      auto bobyqa_regi = std::make_shared<Intensity2D3DRegiBOBYQA>();
      bobyqa_regi->set_opt_vars(se3_vars);
      bobyqa_regi->set_opt_x_tol(0.001);
      bobyqa_regi->set_opt_obj_fn_tol(0.001);
      bobyqa_regi->set_bounds({ 10 * kDEG2RAD, 10 * kDEG2RAD, 10 * kDEG2RAD, 30, 30, 30 });
      
      regi.regi = bobyqa_regi;
    }
    
    // all objects simultaneous regi
    {
      auto& regi = lvl.regis[3];

      regi.mov_vols    = { 0, 1, 2 };  // pelvis, femur, and frag vol poses are optimized over
      regi.ref_frames  = { 0, 0, 0 };  // use APP ref frame with origin at femoral head center
      regi.static_vols = {  };         // No static objects as they are all being optimized

      // use the current estimates of the objects as the initialization at this phase
      auto pelvis_init_guess_fn = std::make_shared<UseCurEstForInit>();
      pelvis_init_guess_fn->vol_idx = 0;
      
      auto femur_init_guess_fn = std::make_shared<UseCurEstForInit>();
      femur_init_guess_fn->vol_idx = 1;
      
      auto frag_init_guess_fn = std::make_shared<UseCurEstForInit>();
      frag_init_guess_fn->vol_idx = 2;
      
      regi.init_mov_vol_poses = { pelvis_init_guess_fn, femur_init_guess_fn, frag_init_guess_fn };
      
      auto bobyqa_regi = std::make_shared<Intensity2D3DRegiBOBYQA>();
      bobyqa_regi->set_opt_vars(se3_vars);
      bobyqa_regi->set_opt_x_tol(0.001);
      bobyqa_regi->set_opt_obj_fn_tol(0.001);
      bobyqa_regi->set_bounds({ 2.5 * kDEG2RAD, 2.5 * kDEG2RAD, 2.5 * kDEG2RAD, 10, 10, 10,
                                2.5 * kDEG2RAD, 2.5 * kDEG2RAD, 2.5 * kDEG2RAD, 10, 10, 10,
                                2.5 * kDEG2RAD, 2.5 * kDEG2RAD, 2.5 * kDEG2RAD, 10, 10, 10 });
      
      regi.regi = bobyqa_regi;
    }
  }

  if (save_debug)
  {
    vout << "  setting regi debug info..." << std::endl;

    DebugRegiResultsMultiLevel::VolPathInfo debug_vol_path;
    debug_vol_path.vol_path = ct_path;
    
    debug_vol_path.label_vol_path = seg_path;
    debug_vol_path.labels_used    = { pelvis_label, femur_label, frag_label };
    
    ml_mo_regi.debug_info->vols = { debug_vol_path };

    DebugRegiResultsMultiLevel::ProjDataPathInfo debug_proj_path;
    debug_proj_path.path = proj_data_path;

    ml_mo_regi.debug_info->fixed_projs = debug_proj_path;
    
    ml_mo_regi.debug_info->proj_pre_proc_info = proj_preproc.params;

    ml_mo_regi.debug_info->regi_names = { { "Pelvis1", "Femur1", "Frag1" },
                                          { "Pelvis2", "Femur2", "Frag2", "All" } };
  }

  vout << "running regi..." << std::endl;
  ml_mo_regi.run();

  vout << "writing pelvis regi. pose to disk..." << std::endl;
  WriteITKAffineTransform(dst_pelvis_pose_path, ml_mo_regi.cur_cam_to_vols[0]);

  vout << "writing femur regi. pose to disk..." << std::endl;
  WriteITKAffineTransform(dst_femur_pose_path, ml_mo_regi.cur_cam_to_vols[1]);

  vout << "writing frag. regi. pose to disk..." << std::endl;
  WriteITKAffineTransform(dst_frag_pose_path, ml_mo_regi.cur_cam_to_vols[2]);

  vout << "writing relative pose of femur wrt APP..." << std::endl;
  WriteITKAffineTransform(dst_rel_femur_pose_path,
                          app_to_vol.inverse() * ml_mo_regi.cur_cam_to_vols[0] *
                          ml_mo_regi.cur_cam_to_vols[1].inverse() * app_to_vol);

  vout << "writing relative pose of fragment wrt APP..." << std::endl;
  WriteITKAffineTransform(dst_rel_frag_pose_path,
                          app_to_vol.inverse() * ml_mo_regi.cur_cam_to_vols[0] *
                          ml_mo_regi.cur_cam_to_vols[2].inverse() * app_to_vol);

  if (save_debug)
  {
    vout << "writing debug info to disk..." << std::endl;
    WriteMultiLevel2D3DRegiDebugToDisk(*ml_mo_regi.debug_info, dst_debug_path);
  }

  vout << "exiting..." << std::endl;
  
  return kEXIT_VAL_SUCCESS;
}
