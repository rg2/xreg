/*
 * MIT License
 *
 * Copyright (c) 2020-2022 Robert Grupp
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

#ifndef XREGDISTINTERFACE_H_
#define XREGDISTINTERFACE_H_

#include <random>

#include "xregCommon.h"

namespace xreg
{

class Dist
{
public:
  using Scalar = CoordScalar;

  struct UnsupportedOperation { };

  virtual Scalar density(const PtN& x) const = 0;

  virtual Scalar log_density(const PtN& x) const = 0;

  virtual Scalar norm_const() const;

  virtual Scalar log_norm_const() const;

  virtual bool normalized() const = 0;

  virtual size_type dim() const = 0;

  virtual PtN densities(const PtNList& pts) const;

  virtual PtN log_densities(const PtNList& pts) const;

  virtual PtN draw_sample(std::mt19937& g) const;

  // The i,j entry of the output matrix stores the ith component of the jth sample.
  // e.g. The columns of the output matrix are the samples.
  virtual MatMxN draw_samples(const size_type num_samples, std::mt19937& g) const;
};

}  // un-named

#endif

