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

#ifndef XREGPSO_H_
#define XREGPSO_H_

#include "xregCommon.h"

namespace xreg
{

class ParticleSwarmOpt
{
public:
  using Scalar    = CoordScalar;
  using Vec       = PtN;

  struct ObjFnNotImplementedException { };

  ParticleSwarmOpt() { }

  virtual ~ParticleSwarmOpt() { }

  // No copying
  ParticleSwarmOpt(const ParticleSwarmOpt&) = delete;
  ParticleSwarmOpt& operator=(const ParticleSwarmOpt&) = delete;

  void set_num_particles(const size_type num_parts);

  size_type num_particles() const;

  void set_init_guess(const Vec& x);

  void set_upper_bounds(const Vec& ub);

  void set_lower_bounds(const Vec& lb);

  void set_box_about_zero(const Vec& b);

  std::tuple<Vec,Scalar> compute();

  void set_omega(const Scalar w);

  void set_phi_p(const Scalar p);

  void set_phi_g(const Scalar p);

  void set_max_num_its(const size_type m);

  size_type max_num_its() const;

protected:
  
  using ScalarList = std::vector<Scalar>;
  using VecList    = std::vector<Vec>;

  // One of these needs to be implemented by the sub-class
  
  virtual Scalar serial_obj_fn(const Vec& x);
  
  virtual void all_obj_fn(const VecList& params, ScalarList& obj_fn_vals);

  // Indicate which objective function method should be used
  enum ObjFnType
  {
    kSERIAL_OBJ_FN,
    kALL_OBJ_FN
  };

  // this should be set by the sub-class
  ObjFnType obj_fn_type_;

  // Callback methods

  virtual void before_init_vals();

  virtual void after_init_vals();

  virtual void start_iter();

  virtual void end_iter();

  virtual void finish();
  
  // shared data with sub-class 
  Vec init_guess_;
  
  Vec    best_param_;
  Scalar best_val_;

  ScalarList particles_best_obj_fn_vals_;
  VecList    particles_best_obj_fn_params_;
  ScalarList particles_cur_obj_fn_vals_;
  VecList    particles_cur_params_;
  VecList    particles_cur_velocities_;
  
  Vec upper_bounds_;
  Vec lower_bounds_;

  size_type num_particles_ = 50;

  // Parameters are values that work well for general
  // problems, source:
  // "Particle Swarm Optimization" - James Kennedy 2011
  // Encyclopedia of machine learning 

  /// inertia
  Scalar omega_ = 0.7298;

  /// weighting to the local best
  Scalar phi_p_ = 1.4961;

  /// weighting to the global best
  Scalar phi_g_ = 1.4961;

  Scalar percent_of_range_for_init_velocities_ = 0.3333;

  size_type max_num_its_ = 100;

private:
  void compute_obj_fns();
};

}  // xreg

#endif

