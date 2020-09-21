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

#ifndef XREGREGI2D3DPENALTYFNSE3EULERDECOMP_H_
#define XREGREGI2D3DPENALTYFNSE3EULERDECOMP_H_

#include "xregRegi2D3DPenaltyFn.h"

namespace xreg
{

// forward declarations:
class Dist;

// This will only work for a single object, several instances should be created for
// each object
class Regi2D3DPenaltyFnSE3EulerDecomp : public Regi2D3DPenaltyFn
{
public:
  using DistPtr = std::shared_ptr<Dist>;

  DistPtr rot_x_pdf;
  DistPtr rot_y_pdf;
  DistPtr rot_z_pdf;

  DistPtr trans_x_pdf;
  DistPtr trans_y_pdf;
  DistPtr trans_z_pdf;

  size_type obj_idx = 0;

  bool use_xforms_from_opt = false;

  void compute(const ListOfFrameTransformLists& cams_wrt_objs,
               const size_type num_projs,
               const CamList& cams,
               const CamAssocList& cam_assocs,
               const std::vector<bool>& intermediate_frames_wrt_vol,
               const FrameTransformList& intermediate_frames,
               const FrameTransformList& regi_xform_guesses,
               const ListOfFrameTransformLists* xforms_from_opt) override;
};

}  // xreg

#endif

