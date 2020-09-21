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

#include "xregTimer.h"

void xreg::Timer::start()
{
  if (need_to_start_tmr_)
  {
    need_to_start_tmr_ = false;

    start_time_ = clock_type::now();
  }
}

void xreg::Timer::stop()
{
  if (!need_to_start_tmr_)
  {
    elapsed_secs_ += elapsed_seconds_since_start();
    need_to_start_tmr_ = true;
  }
}

void xreg::Timer::reset()
{
  elapsed_secs_ = 0;
  need_to_start_tmr_ = true;
}

double xreg::Timer::elapsed_seconds() const
{
  return elapsed_secs_;
}

double xreg::Timer::elapsed_seconds_since_start() const
{
  return std::chrono::duration<double>(clock_type::now() - start_time_).count();
}
