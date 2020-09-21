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

#ifndef XREGPOSIT_H_
#define XREGPOSIT_H_

#include "xregLandmark2D3DRegi.h"

namespace xreg
{

class POSIT : public Landmark2D3DRegi
{
public:
  POSIT() = default;

  void set_ref_pt_idx(const int ref_pt_idx);

  void set_prefer_points_between_origin_and_det(const bool p);
  
  int num_pts_required() override;
  
  bool uses_ref_frame() const override;

protected:
  void run_impl() override;

private:

  FrameTransform run_for_single_ref_pt(const size_type ref_pt_idx) const;

  int ref_pt_idx_ = -1;

  bool prefer_points_between_origin_and_det_ = false;

};

}  // xreg

#endif

