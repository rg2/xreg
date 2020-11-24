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

#ifndef XREGREGI2D3DPENALTYFNCOMBO_H_
#define XREGREGI2D3DPENALTYFNCOMBO_H_

#include "xregRegi2D3DPenaltyFn.h"
#include "xregHDF5ReadWriteInterface.h"

namespace xreg
{

class Regi2D3DPenaltyFnCombo : public Regi2D3DPenaltyFn
{
public:
  using PenaltyFnPtr  = std::shared_ptr<Regi2D3DPenaltyFn>;
  using PenaltyFnList = std::vector<PenaltyFnPtr>;
  
  PenaltyFnList pen_fns;
  
  void setup() override;

  void compute(const ListOfFrameTransformLists& cams_wrt_objs,
               const size_type num_projs,
               const CamList& cams,
               const CamAssocList& cam_assocs,
               const std::vector<bool>& intermediate_frames_wrt_vol,
               const FrameTransformList& intermediate_frames,
               const FrameTransformList& regi_xform_guesses,
               const ListOfFrameTransformLists* xforms_from_opt) override;

  struct ComboDebugInfo : DebugInfo
  {
    std::vector<std::shared_ptr<DebugInfo>> pen_fns_debug_infos;
    
    void write(H5::Group* h5) override;

    void read(const H5::Group& h5) override;
  };
  
  std::shared_ptr<DebugInfo> debug_info() override;

private:

  std::shared_ptr<ComboDebugInfo> debug_info_;
};

}  // xreg

#endif

