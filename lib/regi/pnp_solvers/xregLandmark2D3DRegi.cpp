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

#include "xregLandmark2D3DRegi.h"

#include "xregExceptionUtils.h"
#include "xregLandmarkMapUtils.h"
#include "xregPointCloudUtils.h"

void xreg::Landmark2D3DRegi::set_cam(const CameraModel& cam)
{
  cams_.assign(1, cam);
}

void xreg::Landmark2D3DRegi::set_cams(const CamModelList& cams)
{
  cams_ = cams;
}

void xreg::Landmark2D3DRegi::set_inds_2d(const Pt2List& inds_2d)
{
  inds_2d_.assign(1, inds_2d);
}

void xreg::Landmark2D3DRegi::set_inds_2d(const ListOfPt2Lists& inds_2d)
{
  inds_2d_ = inds_2d;
}
  
const xreg::Landmark2D3DRegi::ListOfPt2Lists&
xreg::Landmark2D3DRegi::inds_2d() const
{
  return inds_2d_;
}

void xreg::Landmark2D3DRegi::set_world_pts_3d(const Pt3List& pts_3d)
{
  world_pts_3d_ = pts_3d;
}
  
const xreg::Pt3List& xreg::Landmark2D3DRegi::world_pts_3d() const
{
  return world_pts_3d_;
}
 
void xreg::Landmark2D3DRegi::set_inds_2d_and_world_pts_3d(const LandMap2& inds_2d, const LandMap3& pts_3d)
{
  set_inds_2d_and_world_pts_3d(std::vector<LandMap2>{ inds_2d }, pts_3d);
}

namespace
{

template <class tMap>
std::vector<std::string> ExtractKeys(const tMap& m)
{
  std::vector<std::string> keys;
  keys.reserve(m.size());

  for (const auto& kv : m)
  {
    keys.push_back(kv.first);
  }

  std::sort(keys.begin(), keys.end());

  return keys;
}

}  // un-named

void xreg::Landmark2D3DRegi::set_inds_2d_and_world_pts_3d(const std::vector<LandMap2>& inds_2d,
                                                          const LandMap3& pts_3d)
{
  auto str_set_inter = [] (const std::vector<std::string>& s1,
                           const std::vector<std::string>& s2)
  {
    std::vector<std::string> s_inter;
    s_inter.reserve(std::max(s1.size(),s2.size()));
    
    std::set_intersection(s1.begin(), s1.end(),
                          s2.begin(), s2.end(),
                          std::back_inserter(s_inter));

    return s_inter;
  };

  auto common_lands = ExtractKeys(pts_3d);

  for (const auto& cur_inds_2d : inds_2d)
  {
    common_lands = str_set_inter(common_lands, ExtractKeys(cur_inds_2d));
  }
  
  const size_type num_lands = common_lands.size();

  if (num_lands == 0)
  {
    xregThrow("No corresponding landmarks found between 2D and 3D!");
  }

  const size_type num_views = inds_2d.size();

  ListOfPt2Lists inds_2d_corr(num_views, Pt2List(num_lands));

  Pt3List pts_3d_corr(num_lands);

  for (size_type land_idx = 0; land_idx < num_lands; ++land_idx)
  {
    const auto& land_name = common_lands[land_idx];
    
    pts_3d_corr[land_idx] = pts_3d.find(land_name)->second;
  
    for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
    {
      inds_2d_corr[view_idx][land_idx] = inds_2d[view_idx].find(land_name)->second;
    }
  }

  set_inds_2d(inds_2d_corr);
  set_world_pts_3d(pts_3d_corr);
}

void xreg::Landmark2D3DRegi::set_init_cam_to_world(const FrameTransform& init_cam_to_world)
{
  init_cam_to_world_ = init_cam_to_world;
}

const xreg::FrameTransform& xreg::Landmark2D3DRegi::regi_cam_to_world() const
{
  return regi_cam_to_world_;
}

void xreg::Landmark2D3DRegi::set_ref_frame(const FrameTransform& ref_frame, const bool maps_world_to_ref)
{
  ref_frame_ = ref_frame;
  ref_frame_maps_world_to_ref_ = maps_world_to_ref;
  ref_frame_inv_ = ref_frame.inverse();
  
  use_ref_frame_as_pts_3d_centroid_ = false;
}
  
int xreg::Landmark2D3DRegi::num_pts_required()
{
  throw UnsupportedOperationException();
}
  
void xreg::Landmark2D3DRegi::set_use_ref_frame_as_pts_3d_centroid(
                               const bool use_ref_frame_as_pts_3d_centroid)
{
  use_ref_frame_as_pts_3d_centroid_ = use_ref_frame_as_pts_3d_centroid;
}

void xreg::Landmark2D3DRegi::run()
{
  if (uses_ref_frame() && use_ref_frame_as_pts_3d_centroid_)
  {
    FrameTransform ref_frame = FrameTransform::Identity(); 
    ref_frame.matrix().block(0,3,3,1) = -ComputeCentroid(world_pts_3d_);
    set_ref_frame(ref_frame, true);
    
    // turn this flag back on so the reference frame may be recomputed if the points are changed
    use_ref_frame_as_pts_3d_centroid_ = true;
  }

  run_impl();
}

