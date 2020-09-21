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

#include "xregVersionInfo.h"

#include <fmt/format.h>

extern const char* xregGIT_SHA1;

extern bool xregHAS_GIT_SHA1;

extern const char* xregPROJ_VERSION;

std::string xreg::ProjVerStr()
{
  return xregPROJ_VERSION;
}

bool xreg::HasGitSHA1()
{
  return xregHAS_GIT_SHA1;
}

std::string xreg::GitSHA1()
{
  return xregGIT_SHA1;
}

std::string xreg::ProjVerStrAndGitSHA1IfAvail()
{
  if (HasGitSHA1())
  {
    return fmt::format("{} ({})", ProjVerStr(), GitSHA1());
  }
  else
  {
    return ProjVerStr();
  }
}

