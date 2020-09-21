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

#ifndef XREGDIFFEVO_H_
#define XREGDIFFEVO_H_

#include "xregCommon.h"

namespace xreg
{

struct DifferentialEvolution
{
  using NonIterCallbackFn = std::function<void(DifferentialEvolution*)>;
  using IterCallbackFn    = std::function<void(DifferentialEvolution*,const size_type)>;

  using CostFn = std::function<PtN(const PtNList& pts_to_eval)>;

  PtN init_guess;

  PtN box_constraints;

  size_type pop_size = 100;

  // when true the evolution rate (or F) is randomly
  // chosen from U[F,1] for each mutation vector
  bool dither = true;

  // the F parameter in the literature
  CoordScalar evo_rate = 0.5;

  // Cr parameter in the literature
  // Choose smaller values (0.2) when you think the problem is separable, e.g. by
  // moving along coordinate axes, or larger (0.9) if not
  CoordScalar cross_over_prob = 0.2;

  size_type max_num_its = 100;

  CostFn cost_fn;
  
  // Outputs/State Variables
  
  PtNList cur_pop;
  PtN     cur_pop_fn_vals;

  size_type   best_pop_idx;
  CoordScalar best_fn_val;

  PtNList cand_pop;
  PtN     cand_pop_fn_vals;

  // Callbacks
  
  NonIterCallbackFn after_init_callback;
  NonIterCallbackFn after_final_iter_callback;

  IterCallbackFn begin_of_iter_callback;
  IterCallbackFn end_of_iter_callback;

  PtN best_param() const;

  void run();
};

}  // xreg

#endif

