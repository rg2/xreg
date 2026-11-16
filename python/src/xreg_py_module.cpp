/*
 * MIT License
 *
 * Copyright (c) 2026 Robert Grupp
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

#include "xreg_py_common.h"

#include "xregAssert.h"
#include "xregVersionInfo.h"

NB_MODULE(_xreg, m)
{
  using namespace xreg;
  using namespace xreg::py;

  m.doc() = "Python bindings for xReg - 2D/3D registration and related utilities.";

  // Failed xreg assertions raise a Python exception (RuntimeError) instead of
  // terminating the interpreter with SIGABRT.
  SetAssertFailThrows(true);

  m.attr("__version__") = ProjVerStrAndGitSHA1IfAvail();

  // double precision versions of kDEG2RAD and kRAD2DEG
  m.attr("DEG2RAD") = 3.141592653589793 / 180.0;
  m.attr("RAD2DEG") = 180.0 / 3.141592653589793;

  // NOTE: the order matters, types must be bound before they are used in the
  //       signatures of other bindings.
  BindImage(m);
  BindProjData(m);
  BindIO(m);
  BindBackend(m);
  BindRayCast(m);
  BindSimMetrics(m);
  BindOpt(m);
  BindRegi(m);
  BindHip(m);
}
