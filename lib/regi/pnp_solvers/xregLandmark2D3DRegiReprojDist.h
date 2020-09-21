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

#ifndef XREGLANDMARK2D3DREGIREPROJDIST_H_
#define XREGLANDMARK2D3DREGIREPROJDIST_H_

#include "xregLandmark2D3DRegi.h"

namespace xreg
{

// Forward declarations:
class SE3OptVars;

class Landmark2D3DRegiReprojDist : public Landmark2D3DRegi
{
public:
  using SE3OptVarsPtr = std::shared_ptr<SE3OptVars>;

  using RegFn = std::function<CoordScalar(const FrameTransform&)>;

  Landmark2D3DRegiReprojDist() = default;

  void set_se3_vars(SE3OptVarsPtr se3_vars);

  void set_reg_fn(const RegFn& reg_fn, const CoordScalar reg_coeff);
  
  bool uses_ref_frame() const override;

protected:
  
  FrameTransform cam_to_world(const PtN& x) const;
  FrameTransform world_to_cam(const PtN& x) const;

  CoordScalar reproj_cost(const FrameTransform& cur_world_to_cam) const;

  SE3OptVarsPtr se3_vars_;
 
  CoordScalar reg_coeff_ = 0.1;

  RegFn reg_fn_;
};

}  // xreg

#endif

