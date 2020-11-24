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

#ifndef XREGH5SE3OPTVARSIO_H_
#define XREGH5SE3OPTVARSIO_H_

#include <memory>

// Forward Declarations
namespace H5
{

class Group;

}  // H5

namespace xreg
{

// Forward Declarations
class SE3OptVars;
class SE3OptVarsEuler;
class SE3OptVarsLieAlg;
class SE3OptVarsTransOnly;
class SE3OptVarsTransXOnly;
class SE3OptVarsTransYOnly;
class SE3OptVarsTransZOnly;
class SO3OptVarsLieAlg;
class SO3OptVarsEuler;
class SO3OptVarsOnlyX;
class SO3OptVarsOnlyY;
class SO3OptVarsOnlyZ;

std::shared_ptr<SE3OptVars> ReadSE3OptVarsH5(const H5::Group& h5);

void WriteSE3OptVarsH5(const SE3OptVars& opt_vars, H5::Group* h5);

void WriteSE3OptVarsH5(const SE3OptVarsEuler& opt_vars, H5::Group* h5);

void WriteSE3OptVarsH5(const SE3OptVarsLieAlg& opt_vars, H5::Group* h5);

void WriteSE3OptVarsH5(const SE3OptVarsTransOnly& opt_vars, H5::Group* h5);

void WriteSE3OptVarsH5(const SE3OptVarsTransXOnly& opt_vars, H5::Group* h5);

void WriteSE3OptVarsH5(const SE3OptVarsTransYOnly& opt_vars, H5::Group* h5);

void WriteSE3OptVarsH5(const SE3OptVarsTransZOnly& opt_vars, H5::Group* h5);

void WriteSE3OptVarsH5(const SO3OptVarsLieAlg& opt_vars, H5::Group* h5);

void WriteSE3OptVarsH5(const SO3OptVarsEuler& opt_vars, H5::Group* h5);

void WriteSE3OptVarsH5(const SO3OptVarsOnlyX& opt_vars, H5::Group* h5);

void WriteSE3OptVarsH5(const SO3OptVarsOnlyY& opt_vars, H5::Group* h5);

void WriteSE3OptVarsH5(const SO3OptVarsOnlyZ& opt_vars, H5::Group* h5);

}  // xreg

#endif

