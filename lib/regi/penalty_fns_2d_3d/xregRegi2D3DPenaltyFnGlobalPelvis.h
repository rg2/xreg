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

#ifndef XREGREGI2D3DPENALTYFNGLOBALPELVIS_H_
#define XREGREGI2D3DPENALTYFNGLOBALPELVIS_H_

#include "xregRegi2D3DPenaltyFn.h"

namespace xreg
{

class Regi2D3DPenaltyFnGlobalPelvis : public Regi2D3DPenaltyFn
{
public:
  size_type vol_idx = 0;

  bool pat_is_up = true;

  Pt3 left_fh_wrt_vol;
  Pt3 right_fh_wrt_vol;

  Pt3 left_asis_wrt_vol;
  Pt3 right_asis_wrt_vol;

  Pt3 left_iof_wrt_vol;
  Pt3 right_iof_wrt_vol;

  // Assumes a single view!
  
  void compute(const ListOfFrameTransformLists& cams_wrt_objs,
               const size_type num_projs,
               const CamList& cams, const CamAssocList& cam_assocs,
               const std::vector<bool>& intermediate_frames_wrt_vol,
               const FrameTransformList& intermediate_frames,
               const FrameTransformList& regi_xform_guesses,
               const ListOfFrameTransformLists* xforms_from_opt) override;
};

}  // xreg


#endif

