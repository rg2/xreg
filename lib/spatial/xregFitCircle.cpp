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

#include "xregFitCircle.h"

#include "xregCMAESInterface.h"

std::tuple<xreg::Pt2,xreg::CoordScalar>
xreg::FitCircle2D(const Pt2& x,
                  const Pt2& y,
                  const Pt2& z)
{
  Mat3x3 A;
  A(0,0) = x(0);
  A(0,1) = x(1);
  A(0,2) = 1;
  A(1,0) = y(0);
  A(1,1) = y(1);
  A(1,2) = 1;
  A(2,0) = z(0);
  A(2,1) = z(1);
  A(2,2) = 1;
  
  const CoordScalar a = A.determinant();

  const CoordScalar x_norm_sq = x.squaredNorm();
  const CoordScalar y_norm_sq = y.squaredNorm();
  const CoordScalar z_norm_sq = z.squaredNorm();

  Mat3x3 D;
  D(0,0) = x_norm_sq;
  D(0,1) = x(1);
  D(0,2) = 1;
  D(1,0) = y_norm_sq;
  D(1,1) = y(1);
  D(1,2) = 1;
  D(2,0) = z_norm_sq;
  D(2,1) = z(1);
  D(2,2) = 1;

  const CoordScalar d = -D.determinant();

  Mat3x3 E = D;
  E(0,1) = x(0);
  E(1,1) = y(0);
  E(2,1) = z(0);

  const CoordScalar e = E.determinant();

  Mat3x3 F = E;
  F(0,2) = x(1);
  F(1,2) = y(1);
  F(2,2) = z(1);

  const CoordScalar f = -F.determinant();

  Pt2 c;
  c(0) = d / (-2 * a);
  c(1) = e / (-2 * a);

  return std::make_tuple(c,
      std::sqrt((((d * d) + (e * e)) / (4 * a * a)) - (f / a)));
}

namespace
{

using namespace xreg;

class CMAESFitCircle2D : public CmaesOptimizer
{
public:
  CMAESFitCircle2D(const Pt2List& pts)
    : CmaesOptimizer(3), pts_(pts), num_pts_(pts.size())
  {
    obj_fn_exec_type_ = CmaesOptimizer::kPARALLEL_OBJ_FN_EVAL;
  }

protected:
  double obj_fn(const Pt& x)
  {
    const Pt2 c = x.head(2);
    const CoordScalar r = x(2);

    double f = 0;
   
    if (r > 1.0e-6)
    {
      const double r_sq = r * r;
  
      for (size_type i = 0; i < num_pts_; ++i)
      {
        const double a = (c - pts_[i]).squaredNorm() - r_sq;
        f += a * a;
      }
    }
    else
    {
      f = std::numeric_limits<double>::max();
    }

    return f;
  }

private:
  const Pt2List& pts_;
  const size_type num_pts_;
};

}  // un-named

std::tuple<xreg::Pt2,xreg::CoordScalar> xreg::FitCircle2D(const Pt2List& pts)
{
  const size_type num_pts = pts.size();
  xregASSERT(num_pts >= 3);

  Pt2 init_center;
  CoordScalar init_radius;
  std::tie(init_center, init_radius) = FitCircle2D(pts[0], pts[num_pts / 2],
                                                   pts[num_pts - 1]);

  // this is somewhat arbitrary depending on the data
  Pt3 s;
  s(0) = 10;
  s(1) = 10;
  s(2) = 10;

  Pt3 init_guess;
  init_guess.head(2) = init_center;
  init_guess(2) = init_radius;

  CMAESFitCircle2D opt(pts);
  opt.set_sigma(s);
  opt.set_init_guess(init_guess);
  opt.set_pop_size(500);
  opt.run();

  const Pt3 end_x = opt.sol();

  return std::make_tuple(Pt2(end_x.head(2)), end_x(2));
}

