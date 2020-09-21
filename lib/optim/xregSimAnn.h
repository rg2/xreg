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

#ifndef XREGSIMANN_H_
#define XREGSIMANN_H_

#include <random>

#include "xregCommon.h"

namespace xreg
{

CoordScalar SimAnnLinearTempDecay(const CoordScalar init_temp,
                                  const CoordScalar /*cur_temp*/,
                                  const size_type cur_it,
                                  const size_type max_num_its);

struct SimulatedAnnealing
{
  using ComputeEnergyFn = std::function<CoordScalar(const PtN&)>;
  using ProposalFn      = std::function<PtN(const PtN&)>;
  using TempUpdateFn    = std::function<CoordScalar(const CoordScalar,  // init temp
                                                    const CoordScalar,  // cur temp
                                                    const size_type,    // cur it
                                                    const size_type)>;  // max its
  
  using BeginIterFn = std::function<void(SimulatedAnnealing*,  // this opt. object
                                         const size_type,      // iter
                                         const PtN&,           // cur_pt
                                         const CoordScalar,    // cur_energy
                                         const CoordScalar,    // cur_temp
                                         const PtN&,           // best_energy_pt
                                         const CoordScalar)>;  // best_energy_val
  
  using EndIterFn = BeginIterFn;

  // Inputs:

  CoordScalar init_temp = 100;

  size_type max_num_its = 10000000;

  PtN init_guess;
  
  ComputeEnergyFn energy_fn;

  ProposalFn prop_fn;
  
  TempUpdateFn temp_update_fn = &SimAnnLinearTempDecay;

  BeginIterFn begin_of_iter_fn;

  EndIterFn end_of_iter_fn;

  // Outputs:
  
  PtN         best_energy_pt;
  CoordScalar best_energy_val;
    
  PtN         cur_pt;
  CoordScalar cur_energy;

  void run();
};

struct MultivarUniformPropSimulatedAnnealing
{
  PtN lower_bounds;
  PtN upper_bounds;

  std::mt19937 rng_eng;
  
  MultivarUniformPropSimulatedAnnealing();

  PtN sample(const PtN& cur_pt);

  SimulatedAnnealing::ProposalFn get_prop_fn_callback();
};

struct MultivarNormNoCovPropSimulatedAnnealing
{
  PtN sigma;
    
  std::mt19937 rng_eng;
  
  std::normal_distribution<CoordScalar> std_norm_rng =
                                          std::normal_distribution<CoordScalar>(0, 1); 
  
  MultivarNormNoCovPropSimulatedAnnealing();

  PtN sample(const PtN& cur_pt);
  
  SimulatedAnnealing::ProposalFn get_prop_fn_callback();
};

struct AdaptMultivarNormNoCovPropSimulatedAnnealing
{
  size_type iter_of_last_update;
  
  size_type update_freq = 10000;  // max num iters to go before updating

  // disable with 0 to use per dim num samples
  size_type const_num_samples_all_dims = 201;
  
  // disable with <= 0 to use per dim spacings
  CoordScalar const_sample_spacings_all_dims = 0.1;

  std::vector<size_type> num_samples_per_dim;

  PtN sample_spacings_per_dim;

  MultivarNormNoCovPropSimulatedAnnealing prop_helper;
  
  void run(SimulatedAnnealing* opt,
           const size_type     iter,
           const PtN&          cur_pt,
           const CoordScalar   cur_energy,
           const CoordScalar   cur_temp,
           const PtN&          best_energy_pt,
           const CoordScalar   best_energy_val);

  SimulatedAnnealing::BeginIterFn get_begin_iter_fn_callback();
};

}  // xreg

#endif

