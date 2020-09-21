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

#include "xregSimAnn.h"

#include "xregSampleUtils.h"
#include "xregNormDistFit.h"
#include "xregAssert.h"

xreg::CoordScalar xreg::SimAnnLinearTempDecay(const CoordScalar init_temp,
                                              const CoordScalar /*cur_temp*/,
                                              const size_type cur_it,
                                              const size_type max_num_its)
{
  return std::max(CoordScalar(1.0e-6),
            1 - (static_cast<CoordScalar>(cur_it) / static_cast<CoordScalar>(max_num_its)));
}
  
void xreg::SimulatedAnnealing::run()
{
  best_energy_pt  = init_guess;
  best_energy_val = energy_fn(init_guess);

  cur_pt     = init_guess;
  cur_energy = best_energy_val;

  PtN next_pt;

  CoordScalar next_energy;

  CoordScalar cur_temp = init_temp;

  std::mt19937 uni_rng_eng;
  std::uniform_real_distribution<CoordScalar> uni_rng_01(0, 1);

  SeedRNGEngWithRandDev(&uni_rng_eng);

  for (size_type iter = 0; iter < max_num_its; ++iter)
  {
    if (begin_of_iter_fn)
    {
      begin_of_iter_fn(this, iter, cur_pt, cur_energy, cur_temp, best_energy_pt, best_energy_val);
    }

    // get the next proposed point to visit and compute its energy
    next_pt     = prop_fn(cur_pt);
    next_energy = energy_fn(next_pt);
  
    // Move to the proposed point with some propability proportional to the
    // difference in energy compared to the current point
    if (uni_rng_01(uni_rng_eng) < std::exp((next_energy - cur_energy) / -cur_temp))
    {
      cur_pt     = next_pt;
      cur_energy = next_energy;

      // check to see if this is the best
      if (cur_energy < best_energy_val)
      {
        best_energy_val = cur_energy;
        best_energy_pt  = cur_pt;
      }
    }
    // else stay at the current point
    
    // update the temperature
    cur_temp = temp_update_fn(init_temp, cur_temp, iter, max_num_its);
  
    if (end_of_iter_fn)
    {
      end_of_iter_fn(this, iter, cur_pt, cur_energy, cur_temp, best_energy_pt, best_energy_val);
    }
  }
}

xreg::MultivarUniformPropSimulatedAnnealing::MultivarUniformPropSimulatedAnnealing()
{
  SeedRNGEngWithRandDev(&rng_eng);
}

xreg::PtN xreg::MultivarUniformPropSimulatedAnnealing::sample(const PtN& cur_pt)
{
  PtN new_pt = cur_pt;

  const size_type dim = lower_bounds.size();

  for (size_type d = 0; d < dim; ++d)
  {
    std::uniform_real_distribution<CoordScalar> uni_rng(lower_bounds(d), upper_bounds(d));

    new_pt(d) += uni_rng(rng_eng);
  }

  return new_pt;
}
  
xreg::SimulatedAnnealing::ProposalFn xreg::MultivarUniformPropSimulatedAnnealing::get_prop_fn_callback()
{
  return [this] (const PtN& cur_pt)
  {
    return this->sample(cur_pt);
  };
}
  
xreg::MultivarNormNoCovPropSimulatedAnnealing::MultivarNormNoCovPropSimulatedAnnealing()
{
  SeedRNGEngWithRandDev(&rng_eng);
}

xreg::PtN xreg::MultivarNormNoCovPropSimulatedAnnealing::sample(const PtN& cur_pt)
{
  PtN new_pt = cur_pt;

  const size_type dim = sigma.size();

  for (size_type d = 0; d < dim; ++d)
  {
    new_pt(d) += sigma(d) * std_norm_rng(rng_eng);
  }

  return new_pt;
}

xreg::SimulatedAnnealing::ProposalFn xreg::MultivarNormNoCovPropSimulatedAnnealing::get_prop_fn_callback()
{
  return [this] (const PtN& cur_pt)
  {
    return this->sample(cur_pt);
  };
}
  
void xreg::AdaptMultivarNormNoCovPropSimulatedAnnealing::run(SimulatedAnnealing* opt,
                                                             const size_type     iter,
                                                             const PtN&          cur_pt,
                                                             const CoordScalar   cur_energy,
                                                             const CoordScalar   cur_temp,
                                                             const PtN&          best_energy_pt,
                                                             const CoordScalar   best_energy_val)
{
  if ((iter == 0) || ((iter - iter_of_last_update) >= update_freq))
  {
    const size_type dim = cur_pt.size();

    if (iter == 0)
    {
      opt->prop_fn = prop_helper.get_prop_fn_callback();

      prop_helper.sigma.resize(dim);
    }

    auto build_off_axis_samples = [] (const size_type num_samples,
                                      const CoordScalar spacing)
    {
      xregASSERT((num_samples % 2) == 1);  // should be odd

      const size_type half_num_samples = num_samples / 2;

      PtN samples(num_samples);

      samples(0) = 0;
      
      CoordScalar spacing_off = spacing;
      
      size_type idx_off = 1;

      for (size_type i = 0; i < half_num_samples;
           ++i, spacing_off += spacing, idx_off += 2)
      {
        samples(idx_off)     =  spacing_off;
        samples(idx_off + 1) = -spacing_off;
      }

      return samples;
    };

    const bool use_const_num_samples_all_dims = const_num_samples_all_dims > 0;
    
    const bool use_const_sample_spacing_all_dims = const_sample_spacings_all_dims > 0;

    const bool precompute_samples = use_const_num_samples_all_dims &&
                                    use_const_sample_spacing_all_dims;

    PtN off_axis_samples;

    if (precompute_samples)
    {
      off_axis_samples = build_off_axis_samples(const_num_samples_all_dims,
                                                const_sample_spacings_all_dims);
    }

    PtN tmp_sample_pt;

    PtN tmp_energies;
    
    for (size_type d = 0; d < dim; ++d)
    {
      const size_type num_samples = use_const_num_samples_all_dims ?
                                        const_num_samples_all_dims :
                                          num_samples_per_dim[d];
        
      const CoordScalar spacing = use_const_sample_spacing_all_dims ?
                                      const_sample_spacings_all_dims :
                                        sample_spacings_per_dim[d];

      tmp_energies.resize(num_samples);

      tmp_energies(0) = cur_energy / -cur_temp;
      
      if (!precompute_samples)
      {
        off_axis_samples = build_off_axis_samples(num_samples, spacing);
      }

      tmp_sample_pt = cur_pt;
      
      for (size_type i = 1; i < num_samples; ++i)
      {
        tmp_sample_pt(d) = cur_pt(d) + off_axis_samples(i);
        
        tmp_energies(i) = opt->energy_fn(tmp_sample_pt) / -cur_temp;
      }

      try
      {        
        std::tie(std::ignore, prop_helper.sigma(d)) = FindGaussianMeanStdFromLogPts(
                                                    off_axis_samples, tmp_energies);
      }
      catch (...)
      {
        prop_helper.sigma(d) = num_samples * spacing;
      }
    }
    
    iter_of_last_update = iter;

    //std::cout << "sigma:\n" << prop_helper.sigma << std::endl;
  } 
}

xreg::SimulatedAnnealing::BeginIterFn
xreg::AdaptMultivarNormNoCovPropSimulatedAnnealing::get_begin_iter_fn_callback()
{
  return [this] (SimulatedAnnealing* opt,
                 const size_type     iter,
                 const PtN&          cur_pt,
                 const CoordScalar   cur_energy,
                 const CoordScalar   cur_temp,
                 const PtN&          best_energy_pt,
                 const CoordScalar   best_energy_val)
  {
    this->run(opt, iter, cur_pt, cur_energy, cur_temp, best_energy_pt, best_energy_val);
  };
}

