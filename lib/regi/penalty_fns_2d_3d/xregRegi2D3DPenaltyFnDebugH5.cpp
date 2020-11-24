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

#include "xregRegi2D3DPenaltyFnDebugH5.h"

#include "xregHDF5.h"
#include "xregRegi2D3DPenaltyFnCombo.h"
#include "xregRegi2D3DPenaltyFnLandReproj.h"

std::shared_ptr<xreg::H5ReadWriteInterface>
xreg::MakePenFnDebugObjH5(const H5::Group& h5)
{
  std::shared_ptr<H5ReadWriteInterface> debug_info;

  if (ObjectInGroupH5("pen-fn-name", h5))
  {
    const std::string pen_fn_name = ReadStringH5("pen-fn-name", h5);

    if (pen_fn_name == "combo")
    {
      debug_info = std::make_shared<Regi2D3DPenaltyFnCombo::ComboDebugInfo>();
    }
    else if (pen_fn_name == "lands-reproj")
    {
      debug_info = std::make_shared<Regi2D3DPenaltyFnLandReproj::LandDebugInfo>();
    }
    else
    {
      xregThrow("unsupported penalty function debug info!!");
    }
  }

  return debug_info;
}
