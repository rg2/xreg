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

#include "xregDiffEvo.h"

#include <random>

#include "xregAssert.h"
#include "xregSampleUtils.h"

xreg::PtN xreg::DifferentialEvolution::best_param() const
{
  xregASSERT(best_pop_idx < cur_pop.size());
  return cur_pop[best_pop_idx];
}

void xreg::DifferentialEvolution::run()
{
  xregASSERT(evo_rate < 1);

  xregASSERT(bool(cost_fn));

  const size_type dim = init_guess.size();
  xregASSERT(dim > 0);

  xregASSERT(box_constraints.size() == dim);

  xregASSERT(pop_size > 0);

  std::mt19937 rng_eng;
  SeedRNGEngWithRandDev(&rng_eng);

  std::uniform_int_distribution<size_type> pop_idx_uni_dist(0, pop_size - 1);
  std::uniform_int_distribution<size_type> dim_idx_uni_dist(0, dim - 1);
  
  std::uniform_real_distribution<CoordScalar> uni_dist_01(0,1);

  std::uniform_real_distribution<CoordScalar> dither_dist(evo_rate, 1);

  cand_pop.assign(pop_size, PtN(dim));

  cur_pop.assign(pop_size, PtN(dim));

  // uniform initialization
  cur_pop[0] = init_guess;

  for (size_type d = 0; d < dim; ++d)
  {
    const auto abs_box = std::abs(box_constraints[d]);

    std::uniform_real_distribution<CoordScalar> box_dist(-abs_box, abs_box);
  
    for (size_type i = 1; i < pop_size; ++i)
    {
      cur_pop[i][d] = box_dist(rng_eng);
    }
  }

  // get initial objective function values at the initial population
  cur_pop_fn_vals = cost_fn(cur_pop);
  
  // find the current best estimate
  best_pop_idx = 0;
  best_fn_val  = cur_pop_fn_vals[0];

  for (size_type i = 1; i < pop_size; ++i)
  {
    if (cur_pop_fn_vals[i] < best_fn_val)
    {
      best_fn_val  = cur_pop_fn_vals[i];
      best_pop_idx = i;
    }
  }

  if (after_init_callback)
  {
    after_init_callback(this);
  }

  for (size_type iter = 0; iter < max_num_its; ++iter)
  {
    if (begin_of_iter_callback)
    {
      begin_of_iter_callback(this, iter);
    }

    // for each population element
    for (size_type i = 0; i < pop_size; ++i)
    {
      const PtN& x = cur_pop[i];

      // get random agents from the population
      size_type r0 = 0;
      size_type r1 = 0;
      size_type r2 = 0;
      
      // make sure the agents are unique from the current population element and
      // each other
      do
      {
        r0 = pop_idx_uni_dist(rng_eng);
      }
      while (r0 == i);
      
      do
      {
        r1 = pop_idx_uni_dist(rng_eng);
      }
      while ((r1 == i) || (r1 == r0));
      
      do
      {
        r2 = pop_idx_uni_dist(rng_eng);
      }
      while ((r2 == i) || (r2 == r0) || (r2 == r1));
      
      const PtN& x_r0 = cur_pop[r0];
      const PtN& x_r1 = cur_pop[r1];
      const PtN& x_r2 = cur_pop[r2];

      PtN& y = cand_pop[i];
    
      const size_type cross_over_dim = dim_idx_uni_dist(rng_eng);

      const CoordScalar F = dither ? dither_dist(rng_eng) : evo_rate;

      for (size_type d = 0; d < dim; ++d)
      {
        if ((d == cross_over_dim) || (uni_dist_01(rng_eng) < cross_over_prob))
        {
          y[d] = x_r0[d] + (F * (x_r1[d] - x_r2[d]));
        }
        else
        {
          y[d] = x[d];
        }
      }
    }

    // compute the objective function values at all of the candidates/mutants
    cand_pop_fn_vals = cost_fn(cand_pop);
  
    // select values that are better and also update the best value found
    for (size_type i = 0; i < pop_size; ++i)
    {
      const CoordScalar& cand_val = cand_pop_fn_vals[i];

      if (cand_val < cur_pop_fn_vals[i])
      {
        cur_pop_fn_vals[i] = cand_val;

        cur_pop[i] = cand_pop[i];
        
        if (cand_val < best_fn_val)
        {
          best_fn_val  = cand_val;
          best_pop_idx = i;
        }
      }
    }
    
    if (end_of_iter_callback)
    {
      end_of_iter_callback(this, iter);
    }
  }

  if (after_final_iter_callback)
  {
    after_final_iter_callback(this);
  }
}
