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

#ifndef FCL_CCD_TBVMOTIONBOUNDVISITOR_H
#define FCL_CCD_TBVMOTIONBOUNDVISITOR_H

#include "fcl/math/motion/taylor_model/taylor_matrix.h"
#include "fcl/math/motion/taylor_model/taylor_vector.h"
#include "fcl/math/bv/RSS.h"
#include "fcl/math/motion/bv_motion_bound_visitor.h"

namespace fcl
{

template <typename S>
class MotionBase;

template <typename S>
class SplineMotion;

template <typename S>
class ScrewMotion;

template <typename S>
class InterpMotion;

template <typename S>
class TranslationMotion;

template<typename BV>
class FCL_EXPORT TBVMotionBoundVisitor : public BVMotionBoundVisitor<typename BV::S>
{
public:
  using S = typename BV::S;

  TBVMotionBoundVisitor(const BV& bv_, const Vector3<S>& n_);

  virtual S visit(const MotionBase<S>& motion) const;
  virtual S visit(const SplineMotion<S>& motion) const;
  virtual S visit(const ScrewMotion<S>& motion) const;
  virtual S visit(const InterpMotion<S>& motion) const;
  virtual S visit(const TranslationMotion<S>& motion) const;

protected:
  template <typename, typename, typename>
  friend struct TBVMotionBoundVisitorVisitImpl;

  BV bv;
  Vector3<S> n;
};

} // namespace fcl

#include "fcl/math/motion/tbv_motion_bound_visitor-inl.h"

#endif
