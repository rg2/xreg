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

#ifndef XREGINTENSITY2D3DREGIDIRECT_H_
#define XREGINTENSITY2D3DREGIDIRECT_H_

#include "xregIntensity2D3DRegiNLOptInterface.h"

namespace xreg
{

/// \brief 2D/3D Intensity-Based Registration object using the DIRECT optimization method.
///
/// This is a contrained, global, optimization algorithm.
class Intensity2D3DRegiDIRECT : public Intensity2D3DRegiNLOptInterface
{
public:
  Intensity2D3DRegiDIRECT();
};

/// \brief 2D/3D Intensity-Based Registration object using the local-biased DIRECT optimization method.
///
/// This is a contrained, global, but local-biased, optimization algorithm.
class Intensity2D3DRegiDIRECTL : public Intensity2D3DRegiNLOptInterface
{
public:
  Intensity2D3DRegiDIRECTL();
};

/// \brief 2D/3D Intensity-Based Registration object using the local-biased, randomized, DIRECT optimization method.
///
/// This is a contrained, global, but local-biased, optimization algorithm.
class Intensity2D3DRegiDIRECTRandL : public Intensity2D3DRegiNLOptInterface
{
public:
  Intensity2D3DRegiDIRECTRandL();
};

}  // xreg

#endif
