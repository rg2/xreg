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

#include "xregPAOIO.h"

#include "xregH5PAOIO.h"
#include "xregExceptionUtils.h"
#include "xregFilesystemUtils.h"
#include "xregStringUtils.h"

std::tuple<xreg::PAOCutPlanes,boost::optional<xreg::PAOCutDispInfo>,boost::optional<xreg::PAOCutSlabs>>
xreg::ReadPAOCutPlanesFile(const std::string& path)
{
  const std::string file_ext = ToLowerCase(Path(path).file_extension());

  if (file_ext == ".h5")
  {
    return ReadPAOCutPlanesH5File(path);
  }
  else
  {
    xregThrow("Unsupported PAO Cuts File Extension: %s", path.c_str());
  }
}

void xreg::WritePAOCutPlanesFile(const PAOCutPlanes& cut_defs,
                                 const std::string& path,
                                 const PAOCutDispInfo* cut_disp,
                                 const PAOCutSlabs* cut_slabs)
{
  const std::string file_ext = ToLowerCase(Path(path).file_extension());

  if (file_ext == ".h5")
  {
    WritePAOCutPlanesH5File(cut_defs, path, cut_disp, cut_slabs, true);
  }
  else
  {
    xregThrow("Unsupported PAO Cuts File Extension: %s", path.c_str());
  }
}

