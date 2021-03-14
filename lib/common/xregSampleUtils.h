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

#ifndef XREGSAMPLEUTILS_H_
#define XREGSAMPLEUTILS_H_

#include <random>

#include "xregCommon.h"

namespace xreg
{

// helper function to initialize mt rng engine with a random device, this is
// code that I keep having to re-write everywhere.
void SeedRNGEngWithRandDev(std::mt19937* rng);

// Compute binomial coefficient - e.g. n choose k.
size_type BinCoeff(const size_type n, const size_type k);

// Sample some combinations of elements
std::vector<std::vector<size_type>>
SampleCombos(const size_type num_elem, const size_type combo_len,
             const size_type num_combos, std::mt19937& rng);

// Return an exhaustive list of combinations of 3 elements from a collection of
// a specified list. Each combination is represented by a list of 3 indices.
std::vector<std::vector<size_type>>
BruteForce3Combos(const size_type num_elem);

// Return an exhaustive list of combinations of 4 elements from a collection of
// a specified list. Each combination is represented by a list of 4 indices.
std::vector<std::vector<size_type>>
BruteForce4Combos(const size_type num_elem);

}  // xreg

#endif

