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

#ifndef XREGREGI2D3DPENALTYFNLANDREPROJ_H_
#define XREGREGI2D3DPENALTYFNLANDREPROJ_H_

#include "xregRegi2D3DPenaltyFn.h"
#include "xregHDF5ReadWriteInterface.h"

namespace xreg
{

class Regi2D3DPenaltyFnLandReproj : public Regi2D3DPenaltyFn
{
public:
  size_type vol_idx = 0;

  Pt3List lands_3d;
  Pt2List lands_2d;
  // Assumes a single view!

  size_type num_lands = 0;

  // We will allow digitization error to have standard deviation of 100 pixels in
  // both the row and column directions.
  // This is about 20 mm on a DR shot with CIOS Fusion (0.194 mm/pixel)
  CoordScalar std_dev = 100;

  bool use_outlier_det = false;

  CoordScalar outlier_thresh_num_stds = 1;

  void set_lands(const LandMap3& lands_3d_map, const LandMap2& lands_2d_map);

  std::vector<std::vector<CoordScalar>> tmp_reproj_dists;

  std::vector<size_type> tmp_num_inliers;

  void setup() override;

  void compute(const ListOfFrameTransformLists& cams_wrt_objs,
               const size_type num_projs,
               const CamList& cams, const CamAssocList& cam_assocs,
               const std::vector<bool>& intermediate_frames_wrt_vol,
               const FrameTransformList& intermediate_frames,
               const FrameTransformList& regi_xform_guesses,
               const ListOfFrameTransformLists* xforms_from_opt) override;
  
  struct LandDebugInfo : DebugInfo
  {
    size_type vol_idx;

    Pt3List lands_3d;
    Pt2List lands_2d;

    CoordScalar std_dev;

    bool use_outlier_det;

    CoordScalar outlier_thresh_num_stds;
    
    void write(H5::Group* h5) override;
    
    void read(const H5::Group& h5) override;
  };

  std::shared_ptr<DebugInfo> debug_info() override;

private:
  std::shared_ptr<LandDebugInfo> debug_info_;

};

}  // xreg

#endif

