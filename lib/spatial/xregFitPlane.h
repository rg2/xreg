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

#ifndef XREGFITPLANE_H_
#define XREGFITPLANE_H_

#include <tuple>

#include "xregSpatialPrimitives.h"

namespace xreg
{

/// \brief Fits a plane to a collection of 3D points.
///
/// A tuple is returned with the plane and reference mean point.
std::tuple<Plane3,Pt3> FitPlaneToPoints(const Pt3List& pts);

Plane3 FitPlaneToPoints(const Pt3& x1, const Pt3& x2, const Pt3& x3,
                        const bool checked = false);

// returns (plane, closest point to plane, list of points used to estimate the plane)
std::tuple<Plane3,Pt3,Pt3List>
FitPlaneToPointsRANSAC(const Pt3List& pts,
                       const double consensus_ratio_min_thresh = 0.5,
                       const size_type max_num_its = 50);

// returns (plane, refernce point on plane)
std::tuple<Plane3,Pt3>
FitPlaneToPointsMAP(const Pt3List& pts, const Plane3& p0);

}  // xreg

#endif

