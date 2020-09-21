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

#include "xregIntensity2D3DRegiBOBYQA.h"

#include "xregSE3OptVars.h"

xreg::Intensity2D3DRegiBOBYQA::Intensity2D3DRegiBOBYQA()
  : Intensity2D3DRegiNLOptInterface(nlopt::LN_BOBYQA, true, false)
{ }

void xreg::Intensity2D3DRegiBOBYQA::set_nl_opt_max_num_fn_evals_for_max_iters(nlopt::opt* opt_obj)
{
  // 1 for initial pose
  // 2 * num vols * pose dim for +/- offsets in each dim
  // final 1 for the next step 
  opt_obj->set_maxeval(1 + (2 * this->num_vols() * this->opt_vars_->num_params()) + this->max_num_iters_);
}
