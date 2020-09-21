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

#include "xregPSO.h"

#include <algorithm>
#include <random>

#include "xregAssert.h"
#include "xregSampleUtils.h"

void xreg::ParticleSwarmOpt::set_num_particles(const size_type num_parts)
{
  num_particles_ = num_parts;
}

xreg::size_type xreg::ParticleSwarmOpt::num_particles() const
{
  return num_particles_;
}

void xreg::ParticleSwarmOpt::set_init_guess(const Vec& x)
{
  init_guess_ = x;
}

void xreg::ParticleSwarmOpt::set_upper_bounds(const Vec& ub)
{
  upper_bounds_ = ub;
}

void xreg::ParticleSwarmOpt::set_lower_bounds(const Vec& lb)
{
  lower_bounds_ = lb;
}

void xreg::ParticleSwarmOpt::set_box_about_zero(const Vec& b)
{
  upper_bounds_ = b.array().abs();

  lower_bounds_ = upper_bounds_ * -1;
}

std::tuple<xreg::ParticleSwarmOpt::Vec,xreg::ParticleSwarmOpt::Scalar>
xreg::ParticleSwarmOpt::compute()
{
  // https://en.wikipedia.org/wiki/Particle_swarm_optimization
  
  const size_type dim = upper_bounds_.size();

  const bool has_init_guess = init_guess_.size() > 0;

  xregASSERT(!has_init_guess || (dim == init_guess_.size())); 
  xregASSERT(dim == lower_bounds_.size());  

  xregASSERT(num_particles_ > 0);

  std::mt19937 rng_eng;
  SeedRNGEngWithRandDev(&rng_eng);

  using UniDist = std::uniform_real_distribution<Scalar>;
  std::vector<UniDist> uni_dists(dim);

  auto rand_vec = [&]()
  {
    Vec v(dim);

    for (size_type i = 0; i < dim; ++i)
    {
      v(i) = uni_dists[i](rng_eng);
    }

    return v;
  };

  best_val_ = std::numeric_limits<Scalar>::max();

  auto update_best_param_and_val = [&]()
  {
    size_type best_idx = ~size_type(0);

    for (size_type i = 0; i < num_particles_; ++i)
    {
      if (particles_best_obj_fn_vals_[i] < best_val_)
      {
        best_val_ = particles_best_obj_fn_vals_[i];
        best_idx  = i;
      }
    }

    if (best_idx != ~size_type(0))
    {
      // we updated the best value, get the parameters
      // we only do this once, at the end of the loop, in case the
      // search dimension is very large and this copy is expensive
      best_param_ = particles_best_obj_fn_params_[best_idx];
    }
  };

  auto clamp_particle = [&](Vec& x)
  {
    for (size_type d = 0; d < dim; ++d)
    {
      x[d] = std::min(std::max(x[d], lower_bounds_[d]), upper_bounds_[d]);
    }
  };

  before_init_vals();

  // initialize uniform distributions to be uniform in param bounds
  for (size_type i = 0; i < dim; ++i)
  {
    uni_dists[i] = UniDist(lower_bounds_[i], upper_bounds_[i]);
  }

  particles_cur_obj_fn_vals_.resize(num_particles_);
  particles_cur_params_.resize(num_particles_);
  particles_cur_velocities_.resize(num_particles_);

  if (has_init_guess)
  {
    particles_cur_params_[0] = init_guess_; 
  }

  // create random initial particles
  for (size_type i = (has_init_guess ? 1 : 0); i < num_particles_; ++i)
  {
    particles_cur_params_[i] = rand_vec();
  }

  // the current params are must be the best for each particle at the start
  particles_best_obj_fn_params_ = particles_cur_params_;

  // compute objective function values at the current params
  compute_obj_fns();

  // the best objective function values for each particle must be equal to the
  // only values we have so far
  particles_best_obj_fn_vals_ = particles_cur_obj_fn_vals_;

  // update the global best
  update_best_param_and_val();

  if (percent_of_range_for_init_velocities_ > 1.0e-6)
  {
    // compute random initial velocities
    for (size_type i = 0; i < dim; ++i)
    {
      const Scalar v = std::abs(upper_bounds_[i] - lower_bounds_[i])
                          * percent_of_range_for_init_velocities_;

      uni_dists[i] = UniDist(-v, v);
    }

    for (size_type i = 0; i < num_particles_; ++i)
    {
      particles_cur_velocities_[i] = rand_vec();
    }
  }
  else
  {
    // use zero initial velocity
    for (size_type i = 0; i < num_particles_; ++i)
    {
      particles_cur_velocities_[i].setConstant(dim, 0);
    }
  }

  // only need a single uniform distribution from now on
  uni_dists.assign(1, UniDist(0,1));

  after_init_vals();

  for (size_type it = 0; it < max_num_its_; ++it)
  {
    start_iter();

    for (size_type i = 0; i < num_particles_; ++i)
    {
      // update the velocity
      Vec& x = particles_cur_params_[i];
      Vec& v = particles_cur_velocities_[i];

      for (size_type d = 0; d < dim; ++d)
      {
        const Scalar r_p = uni_dists[0](rng_eng);
        const Scalar r_g = uni_dists[0](rng_eng);
      
        v[d] = (v[d] * omega_) +
               (phi_p_ * r_p * (particles_best_obj_fn_params_[i][d] - x[d])) +
               (phi_g_ * r_g * (best_param_[d] - x[d])); 
      }

      // update the position
      x += v;
      clamp_particle(x);  // respect bounds
    }

    // compute objective function values at the current params
    compute_obj_fns();

    // find best values at each particle
    for (size_type i = 0; i < num_particles_; ++i)
    {
      if (particles_cur_obj_fn_vals_[i] < particles_best_obj_fn_vals_[i])
      {
        // new value is better than the previous best for this particle
        particles_best_obj_fn_vals_[i]   = particles_cur_obj_fn_vals_[i];
        particles_best_obj_fn_params_[i] = particles_cur_params_[i];
      }
    }

    // update the global best
    update_best_param_and_val();

    end_iter();
  }
  
  finish();

  return std::make_tuple(best_param_, best_val_);
}

void xreg::ParticleSwarmOpt::set_omega(const Scalar w)
{
  omega_ = w;
}

void xreg::ParticleSwarmOpt::set_phi_p(const Scalar p)
{
  phi_p_ = p;
}

void xreg::ParticleSwarmOpt::set_phi_g(const Scalar p)
{
  phi_g_ = p;
}
  
xreg::ParticleSwarmOpt::Scalar xreg::ParticleSwarmOpt::serial_obj_fn(const Vec& x)
{
  throw ObjFnNotImplementedException();
}
  
void xreg::ParticleSwarmOpt::all_obj_fn(const VecList& params, ScalarList& obj_fn_vals)
{
  throw ObjFnNotImplementedException();
}

void xreg::ParticleSwarmOpt::before_init_vals()
{ }

void xreg::ParticleSwarmOpt::after_init_vals()
{ }

void xreg::ParticleSwarmOpt::start_iter()
{ }

void xreg::ParticleSwarmOpt::end_iter()
{ }

void xreg::ParticleSwarmOpt::finish()
{ }
  
void xreg::ParticleSwarmOpt::compute_obj_fns()
{
  if (obj_fn_type_ == kALL_OBJ_FN)
  {
    all_obj_fn(particles_cur_params_, particles_cur_obj_fn_vals_);
  }
  else
  {
    for (size_type i = 0; i < num_particles_; ++i)
    {
      particles_cur_obj_fn_vals_[i] = serial_obj_fn(particles_cur_params_[i]);
    }
  }
}
  
void xreg::ParticleSwarmOpt::set_max_num_its(const size_type m)
{
  max_num_its_ = m;
}

xreg::size_type xreg::ParticleSwarmOpt::max_num_its() const
{
  return max_num_its_;
}

