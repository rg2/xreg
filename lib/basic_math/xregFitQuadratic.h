/*
 * MIT License
 *
 * Copyright (c) 2022 Robert Grupp
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

 #ifndef XREGFITQUADRADIC_H_
 #define XREGFITQUADRADIC_H_

#include "xregCommon.h"

namespace xreg
{

// Fits a quadradic form, h(x) = 0.5 * x^T H x, using a set of observations
// (x_1, f(x_1), ... (x_N, f(x_N)).
//
// NOTE: if you know that your data really fits the form: 0.5 * x^T H x + C,
//       then f(0) = C should be calculated first (by the user) and f(x_i) - C
//       should be passed to this function when solving for H.
//
// params is a MxN matrix of observation parameters where the
// ith column holds the ith M-D parameter vector.
//
// fn_vals is N-D vector where the ith element holds the observed
// function value for the ith parameter vector.
//
// A MxM matrix, H, is returned. No constraint is placed on H being
// symmetric.
MatMxN FitQuadradicForm(const MatMxN& params, const PtN& fn_vals);

// Same interface as FitQuadradicForm, but with the quadratic form
// limited to having a symmetric matrix, H.
MatMxN FitQuadradicFormSymmetric(const MatMxN& params, const PtN& fn_vals);

}  // xreg

 #endif

