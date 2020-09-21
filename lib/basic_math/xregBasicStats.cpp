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

#include "xregBasicStats.h"

xreg::CoordScalar xreg::SampleMean(const CoordScalarList& x)
{
  CoordScalar sum = 0;

  for (const auto& s : x)
  {
    sum += s;
  }
  
  return sum / x.size();
}

xreg::CoordScalar xreg::SampleStdDev(const CoordScalarList& x)
{
  return (x.size() > 1) ? SampleStdDev(x, SampleMean(x)) : CoordScalar(0);
}

xreg::CoordScalar xreg::SampleStdDev(const CoordScalarList& x, const CoordScalar sample_mean)
{
  CoordScalar sum = 0;
  
  const size_type N = x.size();

  if (N > 1)
  {
    CoordScalar tmp = 0;

    for (size_type i = 0; i < N; ++i)
    {
      tmp = x[i] - sample_mean;

      sum += tmp * tmp;
    }

    sum = std::sqrt(sum / (N - 1));
  }
  
  return sum;
}

std::tuple<xreg::CoordScalar,xreg::CoordScalar> xreg::SampleMeanAndStdDev(const CoordScalarList& x)
{
  const CoordScalar sample_mean = SampleMean(x);

  return std::make_tuple(sample_mean, SampleStdDev(x, sample_mean));
}

