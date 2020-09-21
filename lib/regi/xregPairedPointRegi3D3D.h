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

#ifndef XREGPAIREDPOINTREGI3D3D_H_
#define XREGPAIREDPOINTREGI3D3D_H_

#include "xregCommon.h"

namespace xreg
{

/**
 * @brief Computes "optimal" rigid/similarity transformation between two point clouds
 * with correspondence.
 *
 * This is an implementation of Horn's Quaternion Method.
 * This routine is threaded.
 * @param pts1 Point cloud 1
 * @param pts2 Point cloud 2
 * @param xform The computed transformation from the pts1 into pts2
 * @param scale (optional) Scale factor to force, if less than,
 *              or equal to, zero then the scale factor is computed automatically
 **/
FrameTransform PairedPointRegi3D3D(const Pt3List& pts1, const Pt3List& pts2, const CoordScalar scale = CoordScalar(1));

FrameTransform PairedPointRegi3D3D(const LandMap3& pts1, const LandMap3& pts2, const CoordScalar scale = CoordScalar(1));

}  // xreg

#endif

