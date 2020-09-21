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

#ifndef XREGSAMPLEUNIFORMUNITVECS_H_
#define XREGSAMPLEUNIFORMUNITVECS_H_

#include <random>

#include "xregCommon.h"

namespace xreg
{

struct UniformOnUnitSphereDist
{
  using ScalarStdNormDist = std::normal_distribution<CoordScalar>;

  ScalarStdNormDist std_norm_dist;

  size_type dim = 0;

  explicit UniformOnUnitSphereDist(const size_type dim_arg);
  
  // If you need another generator add another overload here
  // and call the templated helper in the .cpp

  PtN operator()(std::mt19937& g);
};

}  // xreg

#endif

