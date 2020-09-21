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

#include "xregSampleUniformUnitVecs.h"

#include "xregAssert.h"

xreg::UniformOnUnitSphereDist::UniformOnUnitSphereDist(const size_type dim_arg)
{
  xregASSERT(dim_arg > 0);
  
  std_norm_dist.param(ScalarStdNormDist::param_type(0,1));

  dim = dim_arg;
}

namespace  // un-named
{

using namespace xreg;

template <class tGenerator, class tDist>
PtN UniformOnUnitSphereDistHelper(tGenerator& g, tDist& std_norm_dist, const size_type dim)
{
  PtN x(dim);

  for (size_type i = 0; i < dim; ++i)
  {
    x(i) = std_norm_dist(g);
  }

  x.normalize();

  return x;
}

}  // unamed
  
xreg::PtN xreg::UniformOnUnitSphereDist::operator()(std::mt19937& g)
{
  return UniformOnUnitSphereDistHelper(g, std_norm_dist, dim);
}

