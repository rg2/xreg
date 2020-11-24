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

#ifndef XREGH5PAOIO_H_
#define XREGH5PAOIO_H_

#include <boost/optional.hpp>

#include "xregPAOCuts.h"

// Forward Declarations
namespace H5
{

class Group;

}  // H5

namespace xreg
{

/// \brief Read cut planes from HDF5 format.
///
/// The display information is optional, however if a valid pointer
/// is provided, then exceptions will be thrown if the appropriate
/// entries are not found.
std::tuple<PAOCutPlanes,boost::optional<PAOCutDispInfo>,boost::optional<PAOCutSlabs>>
ReadPAOCutPlanesH5(const H5::Group& h5);

/// \brief Read cut planes from a file in HDF5 format.
///
/// The display information is optional, however if a valid pointer
/// is provided, then exceptions will be thrown if the appropriate
/// entries are not found.
std::tuple<PAOCutPlanes,boost::optional<PAOCutDispInfo>,boost::optional<PAOCutSlabs>>
ReadPAOCutPlanesH5File(const std::string& path);

/// \brief Write cut planes to HDF5.
///
/// The display information is optional.
void WritePAOCutPlanesH5(const PAOCutPlanes& cut_defs,
                         H5::Group* h5,
                         const PAOCutDispInfo* cut_disp = nullptr,
                         const PAOCutSlabs* cut_slabs = nullptr,
                         const bool compress = true);

/// \brief Write cut planes to a file in HDF5 format.
///
/// The display information is optional.
void WritePAOCutPlanesH5File(const PAOCutPlanes& cut_defs,
                             const std::string& path,
                             const PAOCutDispInfo* cut_disp = nullptr,
                             const PAOCutSlabs* cut_slabs = nullptr,
                             const bool compress = true);

}  // xreg


#endif

