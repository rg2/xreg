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

#include "xregPolyFindZeros.h"

#include <cmath>

std::vector<double>
xreg::QuadraticFnRealRoots(const double a, const double b, const double c)
{
  std::vector<double> roots;

  const double discrim = (b * b) - (4 * a * c);
  
  if (discrim > 1.0e-8)
  {
    // positive discriminant - two real roots
    const double sqrt_discrim = std::sqrt(discrim);
    const double minus_two_a  = -2 * a;
    
    roots = { (b + sqrt_discrim) / minus_two_a,
              (b - sqrt_discrim) / minus_two_a };
  }
  else if (discrim > -1.0e-8)
  {
    // zero discriminant - one real root
    roots = { -b / (2 * a) };
  }
  // else negative discriminant - two imaginary roots
  
  return roots;
}

