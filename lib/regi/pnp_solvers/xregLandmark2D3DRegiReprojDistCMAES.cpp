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

#include "xregLandmark2D3DRegiReprojDistCMAES.h"

#include <cmaes_interface.h>

#include "xregAssert.h"
#include "xregSE3OptVars.h"
#include "xregTBBUtils.h"
  
void xreg::Landmark2D3DRegiReprojDistCMAES::set_pop_size(const size_type p)
{
  pop_size_set_ = true;
  pop_size_ = p;
}

xreg::size_type xreg::Landmark2D3DRegiReprojDistCMAES::pop_size() const
{
  return pop_size_;
}

void xreg::Landmark2D3DRegiReprojDistCMAES::set_sigma(const PtN& sigma)
{
  sigma_set_ = true;
  sigma_ = sigma;
}

const xreg::PtN& xreg::Landmark2D3DRegiReprojDistCMAES::sigma() const
{
  return sigma_;
}

void xreg::Landmark2D3DRegiReprojDistCMAES::run_impl()
{
  xregASSERT(se3_vars_.get());

  const size_type dim = se3_vars_->num_params();

  if (!pop_size_set_)
  {
    pop_size_ = static_cast<size_type>(4 + std::floor(3 * std::log(dim)));
  }

  if (!sigma_set_)
  {
    sigma_.resize(dim,1);
    sigma_.setOnes();
  }

  cmaes_t evo;
  std::memset(&evo, 0, sizeof(evo));

  std::vector<double> zeros(dim, 0);

  PtN_d sigma_dbl = sigma_.cast<double>();

  double* obj_fn_vals = cmaes_init(&evo, static_cast<int>(dim), zeros.data(),
                                   &sigma_dbl[0], 0, static_cast<int>(pop_size_), "non");

  evo.sp.stopTolFun = 1.0e-3;
  evo.sp.stopTolX   = 1.0e-6;

  while (!cmaes_TestForTermination(&evo))
  {
    double* const* pop = cmaes_SamplePopulation(&evo);
 
    auto obj_fn_helper = [&] (const RangeType& r)
    {
      for (size_type p = r.begin(); p < r.end(); ++p)
      {
        obj_fn_vals[p] = this->reproj_cost(this->world_to_cam(
            Eigen::Map<PtN_d>(const_cast<double*>(pop[p]), dim, 1).cast<CoordScalar>()));
      }
    };

    ParallelFor(obj_fn_helper, RangeType(0, pop_size_));
    
    cmaes_UpdateDistribution(&evo, obj_fn_vals);
  }

  regi_cam_to_world_ = cam_to_world(Eigen::Map<PtN_d>(
                            const_cast<double*>(cmaes_GetPtr(&evo, "xmean")), dim, 1).cast<CoordScalar>());

  cmaes_exit(&evo);
}

std::shared_ptr<xreg::Landmark2D3DRegiReprojDistCMAES>
xreg::MakeLand2D3DRegiReprojCMAESStandardParams()
{
  auto regi = std::make_shared<Landmark2D3DRegiReprojDistCMAES>();

  regi->set_se3_vars(std::make_shared<SE3OptVarsLieAlg>());
  
  regi->set_pop_size(50);

  Pt6 sigma;
  sigma(0) = 0.3;
  sigma(1) = 0.3;
  sigma(2) = 0.3;
  sigma(3) = 20;
  sigma(4) = 20;
  sigma(5) = 20;
  regi->set_sigma(sigma);

  regi->set_use_ref_frame_as_pts_3d_centroid(true);

  return regi;
}

