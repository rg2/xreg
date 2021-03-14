/*
 * MIT License
 *
 * Copyright (c) 2020,2021 Robert Grupp
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

#ifndef XREGFITCIRCLE_H_
#define XREGFITCIRCLE_H_

#include <tuple>

#include "xregCommon.h"

namespace xreg
{

/// \brief Fits a 2D circle to three points.
///
/// From the approach described here: http://mathworld.wolfram.com/Circle.html
std::tuple<Pt2,CoordScalar>
FitCircle2D(const Pt2& x,
            const Pt2& y,
            const Pt2& z);

/// \brief Fits a 2D circle to a collection of points.
///
/// First computes a fit with the first, middle, and last points, then
/// solves a non-linear least-squares optimization to refine the
/// center point and radius.
/// Requires at least three points.
std::tuple<Pt2,CoordScalar> FitCircle2D(const Pt2List& pts);

/// \brief Fits a 2D circle to a collection of points using a RANSAC
///        strategy to detect outliers.
///
/// The FitCircle2D() call with three points is used to generate candidate
/// solutions and create the consensus sets. The call to FitCircle2D() on a
/// collection of points is called using the conensus to produce the final
/// solution.
std::tuple<Pt2,CoordScalar>
FitCircle2DRansac(const Pt2List& pts, const int num_proposals = -1,
                  const CoordScalar inlier_thresh = 0.01f);

}  // xreg

#endif

