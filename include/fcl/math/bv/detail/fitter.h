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

/** @author Jia Pan */

#ifndef FCL_BV_DETAIL_FITTER_H
#define FCL_BV_DETAIL_FITTER_H

#include "fcl/math/bv/detail/fitter_OBB.h"
#include "fcl/math/bv/detail/fitter_RSS.h"
#include "fcl/math/bv/detail/fitter_OBBRSS.h"
#include "fcl/math/bv/detail/fitter_kIOS.h"

namespace fcl {
namespace detail {

template <typename S, typename BV>
struct Fitter
{
  static void fit(Vector3<S>* ps, int n, BV& bv);
};

template <typename S>
struct Fitter<S, OBB<S>>
{
  static void fit(Vector3<S>* ps, int n, OBB<S>& bv);
};

template <typename S>
struct Fitter<S, RSS<S>>
{
  static void fit(Vector3<S>* ps, int n, RSS<S>& bv);
};

template <typename S>
struct Fitter<S, kIOS<S>>
{
  static void fit(Vector3<S>* ps, int n, kIOS<S>& bv);
};

template <typename S>
struct Fitter<S, OBBRSS<S>>
{
  static void fit(Vector3<S>* ps, int n, OBBRSS<S>& bv);
};

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename S, typename BV>
void Fitter<S, BV>::fit(Vector3<S>* ps, int n, BV& bv)
{
  for(int i = 0; i < n; ++i)
    bv += ps[i];
}

//==============================================================================
template <typename S>
void Fitter<S, OBB<S>>::fit(Vector3<S>* ps, int n, OBB<S>& bv)
{
  switch(n)
  {
  case 1:
    OBB_fit_functions::fit1(ps, bv);
    break;
  case 2:
    OBB_fit_functions::fit2(ps, bv);
    break;
  case 3:
    OBB_fit_functions::fit3(ps, bv);
    break;
  case 6:
    OBB_fit_functions::fit6(ps, bv);
    break;
  default:
    OBB_fit_functions::fitn(ps, n, bv);
  }
}

//==============================================================================
template <typename S>
void Fitter<S, RSS<S>>::fit(Vector3<S>* ps, int n, RSS<S>& bv)
{
  switch(n)
  {
  case 1:
    RSS_fit_functions::fit1(ps, bv);
    break;
  case 2:
    RSS_fit_functions::fit2(ps, bv);
    break;
  case 3:
    RSS_fit_functions::fit3(ps, bv);
    break;
  default:
    RSS_fit_functions::fitn(ps, n, bv);
  }
}

//==============================================================================
template <typename S>
void Fitter<S, kIOS<S>>::fit(Vector3<S>* ps, int n, kIOS<S>& bv)
{
  switch(n)
  {
  case 1:
    kIOS_fit_functions::fit1(ps, bv);
    break;
  case 2:
    kIOS_fit_functions::fit2(ps, bv);
    break;
  case 3:
    kIOS_fit_functions::fit3(ps, bv);
    break;
  default:
    kIOS_fit_functions::fitn(ps, n, bv);
  }
}

//==============================================================================
template <typename S>
void Fitter<S, OBBRSS<S>>::fit(Vector3<S>* ps, int n, OBBRSS<S>& bv)
{
  switch(n)
  {
  case 1:
    OBBRSS_fit_functions::fit1(ps, bv);
    break;
  case 2:
    OBBRSS_fit_functions::fit2(ps, bv);
    break;
  case 3:
    OBBRSS_fit_functions::fit3(ps, bv);
    break;
  default:
    OBBRSS_fit_functions::fitn(ps, n, bv);
  }
}

} // namespace detail
} // namespace fcl

#endif
