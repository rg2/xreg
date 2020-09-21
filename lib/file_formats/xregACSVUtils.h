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

#ifndef XREGACSVUTILS_H_
#define XREGACSVUTILS_H_

#include <tuple>

#include "xregCommon.h"

namespace xreg
{

/// \brief Parses an ACSV file saved in Slicer and returns a rectangular ROI.
///
/// The ROI is defined by the center point (tuple index 0) and half length
/// (tuple index 1) in each dimension.
/// The values are in physical coordinates.
/// The default behavior is to negate the center x and y values in order
/// to convert RAS to LPS.
std::tuple<Pt3,Pt3>
ReadROIFromACSV(const std::string& acsv_path, const bool ras2lps = true);

}  // xreg

#endif

