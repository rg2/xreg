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

#ifndef XREGINTENSITY2D3DREGIBOBYQA_H_
#define XREGINTENSITY2D3DREGIBOBYQA_H_

#include "xregIntensity2D3DRegiNLOptInterface.h"

namespace xreg
{

/// \brief 2D/3D Intensity-Based Registration object using the BOBYQA optimization method.
///
/// This is a contrained optimization algorithm and requires a search space of
/// at least two dimensions.
class Intensity2D3DRegiBOBYQA : public Intensity2D3DRegiNLOptInterface
{
public:
  Intensity2D3DRegiBOBYQA();

protected:
  void set_nl_opt_max_num_fn_evals_for_max_iters(nlopt::opt* opt_obj) override;
};

}  // xreg

#endif
