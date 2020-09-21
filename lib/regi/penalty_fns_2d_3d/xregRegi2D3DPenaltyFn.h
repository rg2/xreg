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

#ifndef XREGREGI2D3DPENALTYFN_H_
#define XREGREGI2D3DPENALTYFN_H_

#include "xregCommon.h"

namespace xreg
{

struct H5ReadWriteInterface;
struct CameraModel;

class Regi2D3DPenaltyFn
{
public:
  using CamList      = std::vector<CameraModel>;
  using CamAssocList = std::vector<size_type>;

  using ListOfFrameTransformLists = std::vector<FrameTransformList>;

  using DebugInfo = H5ReadWriteInterface;

  Regi2D3DPenaltyFn() = default;

  ~Regi2D3DPenaltyFn() = default;

  //Regi2D3DPenaltyFn(const Regi2D3DPenaltyFn&) = delete;
  //Regi2D3DPenaltyFn& operator=(const Regi2D3DPenaltyFn&) = delete;

  virtual void setup();

  virtual void compute(const ListOfFrameTransformLists& cams_wrt_objs,
                       const size_type num_projs,
                       const CamList& cams,
                       const CamAssocList& cam_assocs,
                       const std::vector<bool>& intermediate_frames_wrt_vol,
                       const FrameTransformList& intermediate_frames,
                       const FrameTransformList& regi_xform_guesses,
                       const ListOfFrameTransformLists* xforms_from_opt) = 0;
  
  const CoordScalarList& reg_vals() const;

  bool compute_probs() const;

  void set_compute_probs(const bool compute_probs);
  
  const CoordScalarList& log_probs() const;

  void set_save_debug_info(const bool save);

  virtual std::shared_ptr<DebugInfo> debug_info();

protected:
  
  CoordScalarList reg_vals_;

  bool compute_probs_ = false;
  
  CoordScalarList log_probs_;

  bool save_debug_info_ = false;
};

}  // xreg

#endif

