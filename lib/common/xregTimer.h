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

#ifndef XREGTIMER_H_
#define XREGTIMER_H_

#include <chrono>

namespace xreg
{

/// \brief A timer class mimicking the features of the CISST stopwatch.
class Timer
{
public:
  /// \brief Starts/resumes the timer/stopwatch
  void start();

  /// \brief Stop/pause the timer/stopwatch
  void stop();

  /// \brief Resets the timer/stopwatch.
  ///
  /// The next call to start will indicate the "zero" reference time.
  /// The reset method does not need to be called immediately after construction;
  /// it is only required if one wishes to re-use the timer.
  void reset();

  // Accounts for pauses using start/stop calls
  double elapsed_seconds() const;

  // total seconds since the last start call
  double elapsed_seconds_since_start() const;

private:
  using clock_type = std::chrono::high_resolution_clock;

  bool need_to_start_tmr_ = true;

  double elapsed_secs_ = 0;

  clock_type::time_point start_time_;
};

}  // xreg

#endif
