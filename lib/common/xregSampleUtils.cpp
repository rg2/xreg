/*
 * MIT License
 *
 * Copyright (c) 2020, 2021 Robert Grupp
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

#include "xregSampleUtils.h"

#include <vector>
#include <random>
#include <algorithm>

#include "xregAssert.h"

void xreg::SeedRNGEngWithRandDev(std::mt19937* rng)
{
  std::random_device rd;

  std::vector<std::random_device::result_type> init_states(std::mt19937::state_size);
  std::generate(init_states.begin(), init_states.end(), [&rd] () { return rd(); });

  std::seed_seq init_seed_seq(init_states.begin(), init_states.end());
  
  rng->seed(init_seed_seq);
}

xreg::size_type xreg::BinCoeff(const size_type n, const size_type k)
{
  xregASSERT(k <= n);

  // https://en.wikipedia.org/wiki/Binomial_coefficient#Multiplicative_formula
  
  const size_type n_plus_1 = n + 1;

  const size_type limit = std::min(k, n - k);
  
  size_type numer = 1;
  size_type denom = 1;
  
  for (size_type i = 1; i <= limit; ++i)
  {
    numer *= n_plus_1 - i;

    denom *= i;
  }
  
  return numer / denom;
}

std::vector<std::vector<xreg::size_type>>
xreg::BruteForce3Combos(const size_type num_elem)
{
  xregASSERT(num_elem >= 3);
  
  std::vector<std::vector<size_type>> combos;

  std::vector<size_type> tmp_arr(3);

  for (size_type i = 0; i < (num_elem - 2); ++i)
  {
    tmp_arr[0] = i;

    for (size_type j = (i + 1); j < (num_elem - 1); ++j)
    {
      tmp_arr[1] = j;

      for (size_type k = (j + 1); k < num_elem; ++k)
      {
        tmp_arr[2] = k;

        combos.push_back(tmp_arr);
      }
    }
  }

  xregASSERT(combos.size() == BinCoeff(num_elem, 3));

  return combos;
}

std::vector<std::vector<xreg::size_type>>
xreg::BruteForce4Combos(const size_type num_elem)
{
  xregASSERT(num_elem >= 4);

  std::vector<std::vector<size_type>> combos;

  std::vector<size_type> tmp_arr(4);

  for (size_type i = 0; i < (num_elem - 3); ++i)
  {
    tmp_arr[0] = i;

    for (size_type j = (i + 1); j < (num_elem - 2); ++j)
    {
      tmp_arr[1] = j;

      for (size_type k = (j + 1); k < (num_elem - 1); ++k)
      {
        tmp_arr[2] = k;
        
        for (size_type l = (k + 1); l < num_elem; ++l)
        {
          tmp_arr[3] = l;

          combos.push_back(tmp_arr);
        }
      }
    }
  }
  
  xregASSERT(combos.size() == BinCoeff(num_elem, 4));

  return combos;
}
 
