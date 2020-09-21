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

#ifndef XREGINTENSITY2D3DREGIDIFFEVO_H_
#define XREGINTENSITY2D3DREGIDIFFEVO_H_

#include "xregDiffEvo.h"
#include "xregIntensity2D3DRegi.h"

namespace xreg
{

class Intensity2D3DRegiDiffEvo : public Intensity2D3DRegi
{
public:

  Intensity2D3DRegiDiffEvo();

  size_type max_num_projs_per_view_per_iter() const override;

  void set_evo_rate(const CoordScalar F);

  void set_cross_over_prob(const CoordScalar cr);

  void set_pop_size(const size_type ps);

  void set_max_num_iters(const size_type max_iters);

  void set_box_contraint(const ScalarList& bounds);

  void run() override;
  
  void set_print_status_inc(const double inc);

protected:
  
  // we'll do this at the start of run()
  void init_opt() override;
  
  void debug_write_comb_sim_score() override;

  void debug_write_opt_pose_vars() override;

private:

  DifferentialEvolution diff_evo_;
  
  // Interval to print the percentage of iterations completed.
  // 0.1 --> print in approximately 10% intervals
  // 1.1 --> do not print (e.g. print in 110% intervals)
  double print_status_inc_ = 0.1;
};

}  // xreg

#endif

