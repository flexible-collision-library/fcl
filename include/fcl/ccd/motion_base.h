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

/** \author Jia Pan */


#ifndef FCL_CCD_MOTION_BASE_H
#define FCL_CCD_MOTION_BASE_H


#include "fcl/math/transform.h"
#include "fcl/ccd/taylor_matrix.h"
#include "fcl/ccd/taylor_vector.h"
#include "fcl/BV/RSS.h"

namespace fcl
{

class MotionBase;

class SplineMotion;
class ScrewMotion;
class InterpMotion;
class TranslationMotion;

/// @brief Compute the motion bound for a bounding volume, given the closest direction n between two query objects
class BVMotionBoundVisitor
{
public:
  virtual FCL_REAL visit(const MotionBase& motion) const = 0;
  virtual FCL_REAL visit(const SplineMotion& motion) const = 0;
  virtual FCL_REAL visit(const ScrewMotion& motion) const = 0;
  virtual FCL_REAL visit(const InterpMotion& motion) const = 0;
  virtual FCL_REAL visit(const TranslationMotion& motion) const = 0;
};

template<typename BV>
class TBVMotionBoundVisitor : public BVMotionBoundVisitor
{
public:
  TBVMotionBoundVisitor(const BV& bv_, const Vec3f& n_) : bv(bv_), n(n_) {}

  virtual FCL_REAL visit(const MotionBase& motion) const { return 0; }
  virtual FCL_REAL visit(const SplineMotion& motion) const { return 0; }
  virtual FCL_REAL visit(const ScrewMotion& motion) const { return 0; }
  virtual FCL_REAL visit(const InterpMotion& motion) const { return 0; }
  virtual FCL_REAL visit(const TranslationMotion& motion) const { return 0; }

protected:
  BV bv;
  Vec3f n;
};

template<>
FCL_REAL TBVMotionBoundVisitor<RSS>::visit(const SplineMotion& motion) const;

template<>
FCL_REAL TBVMotionBoundVisitor<RSS>::visit(const ScrewMotion& motion) const;

template<>
FCL_REAL TBVMotionBoundVisitor<RSS>::visit(const InterpMotion& motion) const;

template<>
FCL_REAL TBVMotionBoundVisitor<RSS>::visit(const TranslationMotion& motion) const;


class TriangleMotionBoundVisitor
{
public:
  TriangleMotionBoundVisitor(const Vec3f& a_, const Vec3f& b_, const Vec3f& c_, const Vec3f& n_) :
    a(a_), b(b_), c(c_), n(n_) {}

  virtual FCL_REAL visit(const MotionBase& motion) const { return 0; }
  virtual FCL_REAL visit(const SplineMotion& motion) const;
  virtual FCL_REAL visit(const ScrewMotion& motion) const;
  virtual FCL_REAL visit(const InterpMotion& motion) const;
  virtual FCL_REAL visit(const TranslationMotion& motion) const;

protected:
  Vec3f a, b, c, n;
};



class MotionBase
{
public:
  MotionBase() : time_interval_(std::shared_ptr<TimeInterval>(new TimeInterval(0, 1)))
  {
  }
  
  virtual ~MotionBase() {}

  /** \brief Integrate the motion from 0 to dt */
  virtual bool integrate(double dt) const = 0;

  /** \brief Compute the motion bound for a bounding volume, given the closest direction n between two query objects */
  virtual FCL_REAL computeMotionBound(const BVMotionBoundVisitor& mb_visitor) const = 0;

  /** \brief Compute the motion bound for a triangle, given the closest direction n between two query objects */
  virtual FCL_REAL computeMotionBound(const TriangleMotionBoundVisitor& mb_visitor) const = 0;

  /** \brief Get the rotation and translation in current step */
  void getCurrentTransform(Matrix3f& R, Vec3f& T) const
  {
    Transform3f tf;
    getCurrentTransform(tf);
    R = tf.getRotation();
    T = tf.getTranslation();
  }

  void getCurrentTransform(Quaternion3f& Q, Vec3f& T) const
  {
    Transform3f tf;
    getCurrentTransform(tf);
    Q = tf.getQuatRotation();
    T = tf.getTranslation();
  }

  void getCurrentRotation(Matrix3f& R) const
  {
    Transform3f tf;
    getCurrentTransform(tf);
    R = tf.getRotation();
  }

  void getCurrentRotation(Quaternion3f& Q) const
  {
    Transform3f tf;
    getCurrentTransform(tf);
    Q = tf.getQuatRotation();
  }

  void getCurrentTranslation(Vec3f& T) const
  {
    Transform3f tf;
    getCurrentTransform(tf);
    T = tf.getTranslation();
  }

  virtual void getCurrentTransform(Transform3f& tf) const = 0;

  virtual void getTaylorModel(TMatrix3& tm, TVector3& tv) const = 0;

  const std::shared_ptr<TimeInterval>& getTimeInterval() const
  {
    return time_interval_;
  }
protected:

  std::shared_ptr<TimeInterval> time_interval_;
  
};

typedef std::shared_ptr<MotionBase> MotionBasePtr;


}

#endif
