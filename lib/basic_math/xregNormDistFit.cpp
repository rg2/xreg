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

#include "xregNormDistFit.h"

#include "xregPolyFit.h"
#include "xregExceptionUtils.h"

std::tuple<xreg::CoordScalar,xreg::CoordScalar>
xreg::FindGaussianMeanStdFromLogPts(const PtN& x, const PtN& log_y)
{
  const auto parabola_coeffs = SolveVandermondeMat(x, log_y, 2);
 
  if (std::abs(parabola_coeffs(2)) > 1.0e-8)
  {
    const CoordScalar two_a = 2 * parabola_coeffs(2);

    const CoordScalar sigma = std::sqrt(std::abs(1 / two_a));
    
    const CoordScalar mu = -parabola_coeffs(1) / two_a;
    
    return std::make_tuple(mu,sigma);
  }
  else
  {
    xregThrow("No curvature!");
  }
}

