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

#include "xregITKRemapUtils.h"

std::vector<itk::RGBPixel<unsigned char>> xreg::GenericAnatomyLUT()
{
  using PixelType = itk::RGBPixel<unsigned char>;

  std::vector<PixelType> lut(256);

  // Since ITK has some weird comma overloaded initializer that has compiler warnings
  auto make_rgb = [](const unsigned char r, const unsigned char g, const unsigned char b)
  {
    PixelType p;
    p[0] = r;
    p[1] = g;
    p[2] = b;

    return p;
  };

  // LUT values taken from:
  // https://www.slicer.org/wiki/Documentation/4.1/SlicerApplication/LookupTables/GenericAnatomyColors

  // ITK seems to overload the comma operator in a strange.
  lut[  0] = make_rgb(  0,   0,   0);
  lut[  1] = make_rgb(128, 174, 128);
  lut[  2] = make_rgb(241, 214, 145);
  lut[  3] = make_rgb(177, 122, 101);
  lut[  4] = make_rgb(111, 184, 210);
  lut[  5] = make_rgb(216, 101,  79);
  lut[  6] = make_rgb(221, 130, 101);
  lut[  7] = make_rgb(144, 238, 144);
  lut[  8] = make_rgb(192, 104,  88);
  lut[  9] = make_rgb(220, 245,  20);
  lut[ 10] = make_rgb( 78,  63,   0);
  lut[ 11] = make_rgb(255, 250, 220);
  lut[ 12] = make_rgb(230, 220,  70);
  lut[ 13] = make_rgb(200, 200, 235);
  lut[ 14] = make_rgb(250, 250, 210);
  lut[ 15] = make_rgb(244, 214,  49);
  lut[ 16] = make_rgb(  0, 151, 206);
  lut[ 17] = make_rgb(216, 101,  79);
  lut[ 18] = make_rgb(183, 156, 220);
  lut[ 19] = make_rgb(183, 214, 211);
  lut[ 20] = make_rgb(152, 189, 207);
  // TODO: Add the rest

  return lut;
}

