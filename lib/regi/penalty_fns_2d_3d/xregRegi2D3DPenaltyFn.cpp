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

#include "xregRegi2D3DPenaltyFn.h"

void xreg::Regi2D3DPenaltyFn::setup()
{ }

const xreg::CoordScalarList& xreg::Regi2D3DPenaltyFn::reg_vals() const
{
  return reg_vals_;
}

bool xreg::Regi2D3DPenaltyFn::compute_probs() const
{
  return compute_probs_;
}

void xreg::Regi2D3DPenaltyFn::set_compute_probs(const bool compute_probs)
{
  compute_probs_ = compute_probs;
}

const xreg::CoordScalarList& xreg::Regi2D3DPenaltyFn::log_probs() const
{
  return log_probs_;
}

void xreg::Regi2D3DPenaltyFn::set_save_debug_info(const bool save)
{
  save_debug_info_ = save;
}

std::shared_ptr<xreg::Regi2D3DPenaltyFn::DebugInfo> xreg::Regi2D3DPenaltyFn::debug_info()
{
  return nullptr;
}

