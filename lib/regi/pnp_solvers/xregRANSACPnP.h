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

#ifndef XREGRANSACPNP_H
#define XREGRANSACPNP_H

#include "xregLandmark2D3DRegi.h"

namespace xreg
{

class RANSACPnP : public Landmark2D3DRegi
{
public:

  RANSACPnP() = default;

  void set_pnp_prop(std::shared_ptr<Landmark2D3DRegi> pnp_prop);

  void set_pnp(std::shared_ptr<Landmark2D3DRegi> pnp);

  void set_num_proposals(const int num_proposals);

  void set_inlier_reproj_thresh_pixels(const CoordScalar thresh);

  void set_min_num_inliers(const size_type min_num_inliers);

  CoordScalar best_mean_reproj() const;

  size_type best_num_inliers() const;
  
  bool uses_ref_frame() const override;

protected:
  void run_impl() override;

private:
  // This uses a reduced set of points to generate a proposed solution to the PnP problem.
  // The proposed solution is used to compute inliners, which are passed to the next
  // PnP solver specified by pnp_. This may be something that requires a small number
  // of points to compute a solution and not rely on an initial estimate, like POSIT or
  // the C-arm P3P solver.
  std::shared_ptr<Landmark2D3DRegi> pnp_prop_;

  // This uses the inlier set to refine the PnP estimate. This is usually something
  // that can operate with a larger set of points, such as a minimization of
  // re-projection distances.
  std::shared_ptr<Landmark2D3DRegi> pnp_;

  // <= 0 --> do all possible
  int num_proposals_ = 50;

  CoordScalar inlier_reproj_thresh_pixels_ = 40;

  size_type min_num_inliers_ = 4;

  CoordScalar cur_best_mean_reproj_ = 0;
    
  size_type cur_best_num_inliers_ = 0;
};

}  // xreg

#endif

