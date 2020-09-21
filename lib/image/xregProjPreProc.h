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

#ifndef XREGPROJPREPROC_H_
#define XREGPROJPREPROC_H_

#include "xregProjData.h"
#include "xregObjWithOStream.h"

namespace xreg
{

struct ProjPreProcParams
{
  int crop_width = 50;

  bool auto_mask = false;
  
  bool invert_mask = false;
  
  double auto_mask_thresh = 0.04;
  double auto_mask_level  = 0.35;

  bool allow_iterative_thresh = true;

  bool no_log_remap = false;
};

struct ProjPreProc : ObjWithOStream
{
  ProjPreProcParams params;

  ProjDataF32List input_projs;

  ProjDataF32List output_projs;

  ProjDataU8List output_mask_projs;

  void operator()();
};

}  // xreg

#endif

