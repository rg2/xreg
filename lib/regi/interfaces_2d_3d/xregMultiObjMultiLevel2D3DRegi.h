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

#ifndef XREGMULTIOBJMULTILEVEL2D3DREGI_H_
#define XREGMULTIOBJMULTILEVEL2D3DREGI_H_

#include "xregProjData.h"
#include "xregRayCastInterface.h"
#include "xregImgSimMetric2D.h"
#include "xregIntensity2D3DRegi.h"

namespace xreg
{

// Forward declarations
struct DebugRegiResultsMultiLevel;

// General rule: this struct assumes the user creates the instance for the
// appropriate objects: ray casters, sim metrics, regi, vol list, and sets 
// all of the appropriate params, running the process will do the resource
// allocation, etc.

struct MultiLevelMultiObjRegi : ObjWithOStream
{
  using Mask2DList = std::vector<ImgSimMetric2D::ImageMaskPtr>;

  using IndexList = Intensity2D3DRegi::IndexList;
  
  struct RefFrameInfo
  {
    FrameTransform ref_frame;
    bool maps_vol_to_ref;
    
    Intensity2D3DRegi::DynRefFrameFn dyn_ref_frame_fn;
  };

  struct RefFrame
  {
    virtual bool standard() const { return true; }

    virtual RefFrameInfo get(const MultiLevelMultiObjRegi* multi_level_multi_obj_regi) const = 0;
  };

  struct StaticRefFrame : RefFrame
  {
    RefFrameInfo info;

    RefFrameInfo get(const MultiLevelMultiObjRegi* multi_level_multi_obj_regi) const override;
  };

  struct CamAlignRefFrameWithCurPose : RefFrame
  {
    size_type vol_idx;

    FrameTransform cam_extrins;
   
    Pt3 center_of_rot_wrt_vol;

    RefFrameInfo get(const MultiLevelMultiObjRegi* multi_level_multi_obj_regi) const override;

    FrameTransform compute_inter(const FrameTransform& cam_to_vol) const;
  };

  struct DynRefFrame : RefFrame
  {
    bool standard() const override { return false; }
  };

  struct DynVolRefFrame : DynRefFrame
  {
    size_type dyn_vol_idx = ~static_cast<size_type>(0);
    
    FrameTransform inter_to_vol = FrameTransform::Identity();
    
    RefFrameInfo get(const MultiLevelMultiObjRegi*) const override;
  };

  struct Level
  {
    /// This must be set prior to registration
    double ds_factor;

    /// This must be constructed prior to registration
    Intensity2D3DRegi::RayCasterPtr ray_caster;

    /// This must be resized and each element constructed prior to registration
    Intensity2D3DRegi::SimMetricList sim_metrics;  // one metric per view

    /// This must be set prior to registration
    IndexList fixed_imgs_to_use;

    struct SingleRegi
    {
      struct InitPose
      {
        virtual FrameTransform get(const MultiLevelMultiObjRegi* multi_level_multi_obj_regi) const = 0;
      };

      struct InitPoseId : InitPose
      {
        FrameTransform get(const MultiLevelMultiObjRegi* multi_level_multi_obj_regi) const;
      };

      struct InitPosePrevPoseEst : InitPose
      {
        size_type vol_idx;
        
        FrameTransform get(const MultiLevelMultiObjRegi* multi_level_multi_obj_regi) const;
      };

      struct InitPosePrevPoseEstTransOnly : InitPose
      {
        size_type vol_idx;
        
        FrameTransform get(const MultiLevelMultiObjRegi* multi_level_multi_obj_regi) const;
      };

      struct InitPosePrevPoseEstRotOnly : InitPose
      {
        size_type vol_idx;
        
        FrameTransform get(const MultiLevelMultiObjRegi* multi_level_multi_obj_regi) const;
      };

      using InitPoseList = std::vector<std::shared_ptr<InitPose>>;

      /// This must be set prior to registration
      IndexList mov_vols;

      /// This must be set prior to registration
      IndexList static_vols;

      /// This must be set prior to registration
      IndexList ref_frames;
      static const size_type kNO_REF_FRAME_USED = ~size_type(0);

      /// This must be set prior to registration
      InitPoseList init_mov_vol_poses;

      /// This must be set prior to registration
      InitPoseList static_vol_poses;

      /// This must be constructed prior to registration, optimization parameters
      /// must also be set
      std::shared_ptr<Intensity2D3DRegi> regi;

      std::vector<std::function<void()>> fns_to_call_right_before_regi_run;
    };

    std::vector<SingleRegi> regis;
  };

  /// This must be set prior to registration
  RayCaster::VolList vols;

  StrList vol_names;

  /// This must be set prior to registration
  ProjDataF32List fixed_proj_data;

  /// This must be set prior to registration if masks are to be used.
  /// if empty no masks are used, and if an individual mask object is
  /// null, than no mask is used for the corresponding projection.
  /// If a mask pixel value is zero, then it will not be "used" in the
  /// similarity metric computation.
  Mask2DList masks_2d;

  /// This must be set prior to registration
  FrameTransformList init_cam_to_vols;

  FrameTransformList cur_cam_to_vols;

  /// This must be set prior to registration
  std::vector<std::shared_ptr<RefFrame>> ref_frames;

  /// This must be set prior to registration
  StrList output_regi_cam_to_vol_paths;

  // TODO: GPU info? I don't think so, it will be specified in the ctors to the ray casters, sim metrics, which are done outside of this logic.

  std::vector<Level> levels;

  // This will be allocated by calling set_save_debug_info()
  std::shared_ptr<DebugRegiResultsMultiLevel> debug_info;

  // (Advanced Feature)
  // Whether or not certain objects (ray casters, sim metrics, 2D/3D regi objects) should
  // be deallocated when they are no longer necessary. This behavior is useful when a
  // registration strategy use nearly all of the GPU resources and cannot execute unless
  // all unrelated GPU resources are released. Disabling this behavior is useful when the
  // related objects may be reused between calls of run() and may potentially result in
  // quicker subsequent runs.
  bool dealloc_resources = true;

  // (Advanced Feature)
  // Whether or not the user provided ray tracer object needs its resources allocated.
  // This should nearly always be set to true. However, it is useful to set to false when
  // an existing ray tracer may be resused in order to save time during resource allocation.
  // The volumes and camera models should not change between successive calls to run() when
  // this flag is set to false.
  bool ray_caster_needs_resources_alloc = true;

  // (Advanced Feature)
  // Whether or not the user provided sim metric objects need their resources allocated.
  // This should nearly always be set to true. However, it is useful to set to false when
  // existing sim metrics may be resused in order to save time during resource allocation.
  // Fixed images and masks should not change between successive calls to run() when
  // this flag is set to false.
  bool sim_metrics_need_resources_alloc = true;

  /// This must be set prior to registration if debug info is to be saved
  /// The debug_info member is valid after this
  void set_save_debug_info(const bool save_debug_info);

  void run();

  std::shared_ptr<StaticRefFrame>
  static MakeStaticRefFrame(const FrameTransform& ref_frame, const bool maps_vol_to_ref);
  
  std::shared_ptr<StaticRefFrame>
  static MakeIdRefFrame();
};

}  // xreg

#endif

