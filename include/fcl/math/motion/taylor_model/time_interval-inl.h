/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
// This code is based on code developed by Stephane Redon at UNC and Inria for the CATCH library: http://graphics.ewha.ac.kr/CATCH/
/** @author Jia Pan */

#ifndef FCL_CCD_TIMEINTERVAL_INL_H
#define FCL_CCD_TIMEINTERVAL_INL_H

#include "fcl/math/motion/taylor_model/time_interval.h"

namespace fcl
{

//==============================================================================
extern template
struct TimeInterval<double>;

//==============================================================================
template <typename S>
TimeInterval<S>::TimeInterval()
{
  // Do nothing
}

//==============================================================================
template <typename S>
TimeInterval<S>::TimeInterval(S l, S r)
{
  setValue(l, r);
}

//==============================================================================
template <typename S>
void TimeInterval<S>::setValue(S l, S r)
{
  t_.setValue(l, r);
  t2_.setValue(l * t_[0], r * t_[1]);
  t3_.setValue(l * t2_[0], r * t2_[1]);
  t4_.setValue(l * t3_[0], r * t3_[1]);
  t5_.setValue(l * t4_[0], r * t4_[1]);
  t6_.setValue(l * t5_[0], r * t5_[1]);
}

} // namespace fcl

#endif
