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

#ifndef FCL_CCD_MOTION_BASE_H
#define FCL_CCD_MOTION_BASE_H

#include "fcl/math/motion/taylor_model/taylor_matrix.h"
#include "fcl/math/motion/taylor_model/taylor_vector.h"
#include "fcl/math/bv/RSS.h"

namespace fcl
{

template <typename S>
class BVMotionBoundVisitor;

template <typename S>
class TriangleMotionBoundVisitor;

template <typename S>
class MotionBase
{
public:
  MotionBase();
  
  virtual ~MotionBase();

  /** @brief Integrate the motion from 0 to dt */
  virtual bool integrate(S dt) const = 0;

  /** @brief Compute the motion bound for a bounding volume, given the closest direction n between two query objects */
  virtual S computeMotionBound(const BVMotionBoundVisitor<S>& mb_visitor) const = 0;

  /** @brief Compute the motion bound for a triangle, given the closest direction n between two query objects */
  virtual S computeMotionBound(const TriangleMotionBoundVisitor<S>& mb_visitor) const = 0;

  /** @brief Get the rotation and translation in current step */
  void getCurrentTransform(Matrix3<S>& R, Vector3<S>& T) const;

  void getCurrentTransform(Quaternion<S>& Q, Vector3<S>& T) const;

  void getCurrentRotation(Matrix3<S>& R) const;

  void getCurrentRotation(Quaternion<S>& Q) const;

  void getCurrentTranslation(Vector3<S>& T) const;

  virtual void getCurrentTransform(Transform3<S>& tf) const = 0;

  virtual void getTaylorModel(TMatrix3<S>& tm, TVector3<S>& tv) const = 0;

  const std::shared_ptr<TimeInterval<S>>& getTimeInterval() const;
protected:

  std::shared_ptr<TimeInterval<S>> time_interval_;
  
};

using MotionBasef = MotionBase<float>;
using MotionBased = MotionBase<double>;

template <typename S>
using MotionBasePtr = std::shared_ptr<MotionBase<S>>;

} // namespace fcl

#include "fcl/math/motion/motion_base-inl.h"

#endif
