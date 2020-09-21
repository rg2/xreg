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

#ifndef XREGNULLDIST_H_
#define XREGNULLDIST_H_

#include "xregDistInterface.h"

namespace xreg
{

// used to apply no regularization to the translation component
// when we are optimizing, and only changing, the rotation component
class NullDist final : public Dist
{
public:
  explicit NullDist(const size_type dim);
  
  Scalar operator()(const Scalar x) const;

  Scalar log_density(const Scalar x) const;

  Scalar density(const PtN& x) const override;

  Scalar log_density(const PtN& x) const override;

  Scalar norm_const() const override;

  Scalar log_norm_const() const override;

  bool normalized() const override;

  size_type dim() const override;

private:
  size_type dim_ = 0;
};

}  // xreg

#endif

