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

#include "xregH5SE3OptVarsIO.h"

#include "xregSE3OptVars.h"
#include "xregHDF5.h"

std::shared_ptr<xreg::SE3OptVars> xreg::ReadSE3OptVarsH5(const H5::Group& h5)
{
  std::shared_ptr<SE3OptVars> se3_vars;

  const std::string param_str = ReadStringH5("parameterization", h5);

  if (param_str == "se3-euler")
  {
    se3_vars = std::make_shared<SE3OptVarsEuler>(
                            ReadSingleScalarH5ULong("rot-x-order", h5),
                            ReadSingleScalarH5ULong("rot-y-order", h5),
                            ReadSingleScalarH5ULong("rot-z-order", h5),
                            ReadSingleScalarH5ULong("trans-x-order", h5),
                            ReadSingleScalarH5ULong("trans-y-order", h5),
                            ReadSingleScalarH5ULong("trans-z-order", h5));
  }
  else if (param_str == "se3-lie-alg")
  {
    se3_vars = std::make_shared<SE3OptVarsLieAlg>();
  }
  else if (param_str == "translation-only")
  {
    se3_vars = std::make_shared<SE3OptVarsTransOnly>(
                            ReadSingleScalarH5Bool("trans-x", h5),
                            ReadSingleScalarH5Bool("trans-y", h5),
                            ReadSingleScalarH5Bool("trans-z", h5));
  }
  else if (param_str == "trans-x-only")
  {
    se3_vars = std::make_shared<SE3OptVarsTransXOnly>();
  }
  else if (param_str == "trans-y-only")
  {
    se3_vars = std::make_shared<SE3OptVarsTransYOnly>();
  }
  else if (param_str == "trans-z-only")
  {
    se3_vars = std::make_shared<SE3OptVarsTransZOnly>();
  }
  else if (param_str == "so3-lie-alg")
  {
    se3_vars = std::make_shared<SO3OptVarsLieAlg>();
  }
  else if (param_str == "rot-only-euler")
  {
    se3_vars = std::make_shared<SO3OptVarsEuler>(
                            ReadSingleScalarH5ULong("rot-x-order", h5),
                            ReadSingleScalarH5ULong("rot-y-order", h5),
                            ReadSingleScalarH5ULong("rot-z-order", h5));
  }
  else if (param_str == "rot-x-only")
  {
    se3_vars = std::make_shared<SO3OptVarsOnlyX>();
  }
  else if (param_str == "rot-y-only")
  {
    se3_vars = std::make_shared<SO3OptVarsOnlyY>();
  }
  else if (param_str == "rot-z-only")
  {
    se3_vars = std::make_shared<SO3OptVarsOnlyZ>();
  }
  else
  {
    xregThrow("unsupported SE3 parameterization: %s", param_str.c_str());
  }

  return se3_vars;
}

void xreg::WriteSE3OptVarsH5(const SE3OptVars& opt_vars, H5::Group* h5)
{
  WriteSingleScalarH5("num-params", opt_vars.num_params(), h5);

  const auto* ov = &opt_vars;
  
  const auto* ov_e = dynamic_cast<const SE3OptVarsEuler*>(ov);
  if (ov_e)
  {
    WriteSE3OptVarsH5(*ov_e, h5);
  }
  else
  {
    const auto* ov_la = dynamic_cast<const SE3OptVarsLieAlg*>(ov);
    if (ov_la)
    {
      WriteSE3OptVarsH5(*ov_la, h5);
    }
    else
    {
      const auto* ov_tox = dynamic_cast<const SE3OptVarsTransXOnly*>(ov);
      if (ov_tox)
      {
        WriteSE3OptVarsH5(*ov_tox, h5);
      }
      else
      {
        const auto* ov_toy = dynamic_cast<const SE3OptVarsTransYOnly*>(ov);
        if (ov_toy)
        {
          WriteSE3OptVarsH5(*ov_toy, h5);
        }
        else
        {
          const auto* ov_toz = dynamic_cast<const SE3OptVarsTransZOnly*>(ov);
          if (ov_toz)
          {
            WriteSE3OptVarsH5(*ov_toz, h5);
          }
          else
          {
            const auto* ov_to = dynamic_cast<const SE3OptVarsTransOnly*>(ov);
            if (ov_to)
            {
              WriteSE3OptVarsH5(*ov_to, h5);
            }
            else
            {
              const auto* ov_rote = dynamic_cast<const SO3OptVarsEuler*>(ov);
              if (ov_rote)
              {
                WriteSE3OptVarsH5(*ov_rote, h5);
              }
              else
              {
                const auto* ov_rotx = dynamic_cast<const SO3OptVarsOnlyX*>(ov);
                if (ov_rotx)
                {
                  WriteSE3OptVarsH5(*ov_rotx, h5);
                }
                else
                {
                  const auto* ov_roty = dynamic_cast<const SO3OptVarsOnlyY*>(ov);
                  if (ov_roty)
                  {
                    WriteSE3OptVarsH5(*ov_roty, h5);
                  }
                  else
                  {
                    const auto* ov_rotz = dynamic_cast<const SO3OptVarsOnlyZ*>(ov);
                    if (ov_rotz)
                    {
                      WriteSE3OptVarsH5(*ov_rotz, h5);
                    }
                    else
                    {
                      const auto* ov_so3_la = dynamic_cast<const SO3OptVarsLieAlg*>(ov);
                      if (ov_so3_la)
                      {
                        WriteSE3OptVarsH5(*ov_so3_la, h5);
                      }
                      else
                      {
                        xregThrow("Unsupported SE3 Parameterization");
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}

void xreg::WriteSE3OptVarsH5(const SE3OptVarsEuler& opt_vars, H5::Group* h5)
{
  WriteStringH5("parameterization", "se3-euler", h5, false);
  WriteSingleScalarH5("rot-x-order", opt_vars.rot_x_order(), h5);
  WriteSingleScalarH5("rot-y-order", opt_vars.rot_y_order(), h5);
  WriteSingleScalarH5("rot-z-order", opt_vars.rot_z_order(), h5);
  WriteSingleScalarH5("trans-x-order", opt_vars.trans_x_order(), h5);
  WriteSingleScalarH5("trans-y-order", opt_vars.trans_y_order(), h5);
  WriteSingleScalarH5("trans-z-order", opt_vars.trans_z_order(), h5);
}

void xreg::WriteSE3OptVarsH5(const SE3OptVarsLieAlg& opt_vars, H5::Group* h5)
{
  WriteStringH5("parameterization", "se3-lie-alg", h5, false);
}

void xreg::WriteSE3OptVarsH5(const SE3OptVarsTransOnly& opt_vars, H5::Group* h5)
{
  WriteStringH5("parameterization", "translation-only", h5, false);
  WriteSingleScalarH5("trans-x", opt_vars.use_x(), h5);
  WriteSingleScalarH5("trans-y", opt_vars.use_y(), h5);
  WriteSingleScalarH5("trans-z", opt_vars.use_z(), h5);
}

void xreg::WriteSE3OptVarsH5(const SE3OptVarsTransXOnly& opt_vars, H5::Group* h5)
{
  WriteStringH5("parameterization", "trans-x-only", h5, false);
}

void xreg::WriteSE3OptVarsH5(const SE3OptVarsTransYOnly& opt_vars, H5::Group* h5)
{
  WriteStringH5("parameterization", "trans-y-only", h5, false);
}

void xreg::WriteSE3OptVarsH5(const SE3OptVarsTransZOnly& opt_vars, H5::Group* h5)
{
  WriteStringH5("parameterization", "trans-z-only", h5, false);
}

void xreg::WriteSE3OptVarsH5(const SO3OptVarsLieAlg& opt_vars, H5::Group* h5)
{
  WriteStringH5("parameterization", "so3-lie-alg", h5, false);
}

void xreg::WriteSE3OptVarsH5(const SO3OptVarsEuler& opt_vars, H5::Group* h5)
{
  WriteStringH5("parameterization", "rot-only-euler", h5, false);
  WriteSingleScalarH5("rot-x-order", opt_vars.rot_x_order(), h5);
  WriteSingleScalarH5("rot-y-order", opt_vars.rot_y_order(), h5);
  WriteSingleScalarH5("rot-z-order", opt_vars.rot_z_order(), h5);
}

void xreg::WriteSE3OptVarsH5(const SO3OptVarsOnlyX& opt_vars, H5::Group* h5)
{
  WriteStringH5("parameterization", "rot-x-only", h5, false);
}

void xreg::WriteSE3OptVarsH5(const SO3OptVarsOnlyY& opt_vars, H5::Group* h5)
{
  WriteStringH5("parameterization", "rot-y-only", h5, false);
}

void xreg::WriteSE3OptVarsH5(const SO3OptVarsOnlyZ& opt_vars, H5::Group* h5)
{
  WriteStringH5("parameterization", "rot-z-only", h5, false);
}

