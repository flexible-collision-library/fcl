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

#include "fcl/ccd/taylor_matrix.h"
#include "fcl/ccd/taylor_vector.h"
#include "fcl/BV/RSS.h"

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

/// @brief Compute the motion bound for a bounding volume, given the closest direction n between two query objects
template <typename S>
class BVMotionBoundVisitor
{
public:
  virtual S visit(const MotionBase<S>& motion) const = 0;
  virtual S visit(const SplineMotion<S>& motion) const = 0;
  virtual S visit(const ScrewMotion<S>& motion) const = 0;
  virtual S visit(const InterpMotion<S>& motion) const = 0;
  virtual S visit(const TranslationMotion<S>& motion) const = 0;
};

template<typename BV>
class TBVMotionBoundVisitor : public BVMotionBoundVisitor<typename BV::S>
{
public:
  using S = typename BV::S;

  TBVMotionBoundVisitor(const BV& bv_, const Vector3<S>& n_) : bv(bv_), n(n_) {}

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

template <typename S>
class TriangleMotionBoundVisitor
{
public:
  TriangleMotionBoundVisitor(const Vector3<S>& a_, const Vector3<S>& b_, const Vector3<S>& c_, const Vector3<S>& n_) :
    a(a_), b(b_), c(c_), n(n_) {}

  virtual S visit(const MotionBase<S>& motion) const { return 0; }
  virtual S visit(const SplineMotion<S>& motion) const;
  virtual S visit(const ScrewMotion<S>& motion) const;
  virtual S visit(const InterpMotion<S>& motion) const;
  virtual S visit(const TranslationMotion<S>& motion) const;

protected:
  template <typename, typename>
  friend struct TriangleMotionBoundVisitorVisitImpl;

  Vector3<S> a, b, c, n;
};

template <typename S>
class MotionBase
{
public:
  MotionBase()
    : time_interval_(std::shared_ptr<TimeInterval<S>>(new TimeInterval<S>(0, 1)))
  {
  }
  
  virtual ~MotionBase() {}

  /** \brief Integrate the motion from 0 to dt */
  virtual bool integrate(S dt) const = 0;

  /** \brief Compute the motion bound for a bounding volume, given the closest direction n between two query objects */
  virtual S computeMotionBound(const BVMotionBoundVisitor<S>& mb_visitor) const = 0;

  /** \brief Compute the motion bound for a triangle, given the closest direction n between two query objects */
  virtual S computeMotionBound(const TriangleMotionBoundVisitor<S>& mb_visitor) const = 0;

  /** \brief Get the rotation and translation in current step */
  void getCurrentTransform(Matrix3<S>& R, Vector3<S>& T) const
  {
    Transform3<S> tf;
    getCurrentTransform(tf);
    R = tf.linear();
    T = tf.translation();
  }

  void getCurrentTransform(Quaternion<S>& Q, Vector3<S>& T) const
  {
    Transform3<S> tf;
    getCurrentTransform(tf);
    Q = tf.linear();
    T = tf.translation();
  }

  void getCurrentRotation(Matrix3<S>& R) const
  {
    Transform3<S> tf;
    getCurrentTransform(tf);
    R = tf.linear();
  }

  void getCurrentRotation(Quaternion<S>& Q) const
  {
    Transform3<S> tf;
    getCurrentTransform(tf);
    Q = tf.linear();
  }

  void getCurrentTranslation(Vector3<S>& T) const
  {
    Transform3<S> tf;
    getCurrentTransform(tf);
    T = tf.translation();
  }

  virtual void getCurrentTransform(Transform3<S>& tf) const = 0;

  virtual void getTaylorModel(TMatrix3<S>& tm, TVector3<S>& tv) const = 0;

  const std::shared_ptr<TimeInterval<S>>& getTimeInterval() const
  {
    return time_interval_;
  }
protected:

  std::shared_ptr<TimeInterval<S>> time_interval_;
  
};

using MotionBasef = MotionBase<float>;
using MotionBased = MotionBase<double>;

template <typename S>
using MotionBasePtr = std::shared_ptr<MotionBase<S>>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename S, typename BV, typename MotionT>
struct TBVMotionBoundVisitorVisitImpl
{
  static S run(
      const TBVMotionBoundVisitor<BV>& /*visitor*/,
      const MotionT& /*motion*/)
  {
    return 0;
  }
};

//==============================================================================
template<typename BV>
typename BV::S TBVMotionBoundVisitor<BV>::visit(
    const MotionBase<S>& motion) const
{
  return 0;
}
//==============================================================================
template<typename BV>
typename BV::S TBVMotionBoundVisitor<BV>::visit(
    const SplineMotion<S>& motion) const
{
  using S = typename BV::S;

  return TBVMotionBoundVisitorVisitImpl<
      S, BV, SplineMotion<S>>::run(*this, motion);
}

//==============================================================================
template<typename BV>
typename BV::S TBVMotionBoundVisitor<BV>::visit(
    const ScrewMotion<S>& motion) const
{
  using S = typename BV::S;

  return TBVMotionBoundVisitorVisitImpl<
      S, BV, ScrewMotion<S>>::run(*this, motion);
}

//==============================================================================
template<typename BV>
typename BV::S TBVMotionBoundVisitor<BV>::visit(
    const InterpMotion<S>& motion) const
{
  using S = typename BV::S;

  return TBVMotionBoundVisitorVisitImpl<
      S, BV, InterpMotion<S>>::run(*this, motion);
}

//==============================================================================
template<typename BV>
typename BV::S TBVMotionBoundVisitor<BV>::visit(
    const TranslationMotion<S>& motion) const
{
  using S = typename BV::S;

  return TBVMotionBoundVisitorVisitImpl<
      S, BV, TranslationMotion<S>>::run(*this, motion);
}

//==============================================================================
template <typename S>
struct TBVMotionBoundVisitorVisitImpl<S, RSS<S>, SplineMotion<S>>
{
  static S run(
      const TBVMotionBoundVisitor<RSS<S>>& visitor,
      const SplineMotion<S>& motion)
  {
    S T_bound = motion.computeTBound(visitor.n);
    S tf_t = motion.getCurrentTime();

    Vector3<S> c1 = visitor.bv.To;
    Vector3<S> c2 = visitor.bv.To + visitor.bv.axis.col(0) * visitor.bv.l[0];
    Vector3<S> c3 = visitor.bv.To + visitor.bv.axis.col(1) * visitor.bv.l[1];
    Vector3<S> c4 = visitor.bv.To + visitor.bv.axis.col(0) * visitor.bv.l[0] + visitor.bv.axis.col(1) * visitor.bv.l[1];

    S tmp;
    // max_i |c_i * n|
    S cn_max = std::abs(c1.dot(visitor.n));
    tmp = std::abs(c2.dot(visitor.n));
    if(tmp > cn_max) cn_max = tmp;
    tmp = std::abs(c3.dot(visitor.n));
    if(tmp > cn_max) cn_max = tmp;
    tmp = std::abs(c4.dot(visitor.n));
    if(tmp > cn_max) cn_max = tmp;

    // max_i ||c_i||
    S cmax = c1.squaredNorm();
    tmp = c2.squaredNorm();
    if(tmp > cmax) cmax = tmp;
    tmp = c3.squaredNorm();
    if(tmp > cmax) cmax = tmp;
    tmp = c4.squaredNorm();
    if(tmp > cmax) cmax = tmp;
    cmax = sqrt(cmax);

    // max_i ||c_i x n||
    S cxn_max = (c1.cross(visitor.n)).squaredNorm();
    tmp = (c2.cross(visitor.n)).squaredNorm();
    if(tmp > cxn_max) cxn_max = tmp;
    tmp = (c3.cross(visitor.n)).squaredNorm();
    if(tmp > cxn_max) cxn_max = tmp;
    tmp = (c4.cross(visitor.n)).squaredNorm();
    if(tmp > cxn_max) cxn_max = tmp;
    cxn_max = sqrt(cxn_max);

    S dWdW_max = motion.computeDWMax();
    S ratio = std::min(1 - tf_t, dWdW_max);

    S R_bound = 2 * (cn_max + cmax + cxn_max + 3 * visitor.bv.r) * ratio;


    // std::cout << R_bound << " " << T_bound << std::endl;

    return R_bound + T_bound;
  }
};

//==============================================================================
/// @brief Compute the motion bound for a bounding volume along a given direction n
/// according to mu < |v * n| + ||w x n||(r + max(||ci*||)) where ||ci*|| = ||R0(ci) x w||. w is the angular axis (normalized)
/// and ci are the endpoints of the generator primitives of RSSd.
/// Notice that all bv parameters are in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
template <typename S>
struct TBVMotionBoundVisitorVisitImpl<S, RSS<S>, ScrewMotion<S>>
{
  static S run(
      const TBVMotionBoundVisitor<RSS<S>>& visitor,
      const ScrewMotion<S>& motion)
  {
    Transform3<S> tf;
    motion.getCurrentTransform(tf);

    const Vector3<S>& axis = motion.getAxis();
    S linear_vel = motion.getLinearVelocity();
    S angular_vel = motion.getAngularVelocity();
    const Vector3<S>& p = motion.getAxisOrigin();

    S c_proj_max = ((tf.linear() * visitor.bv.To).cross(axis)).squaredNorm();
    S tmp;
    tmp = ((tf.linear() * (visitor.bv.To + visitor.bv.axis.col(0) * visitor.bv.l[0])).cross(axis)).squaredNorm();
    if(tmp > c_proj_max) c_proj_max = tmp;
    tmp = ((tf.linear() * (visitor.bv.To + visitor.bv.axis.col(1) * visitor.bv.l[1])).cross(axis)).squaredNorm();
    if(tmp > c_proj_max) c_proj_max = tmp;
    tmp = ((tf.linear() * (visitor.bv.To + visitor.bv.axis.col(0) * visitor.bv.l[0] + visitor.bv.axis.col(1) * visitor.bv.l[1])).cross(axis)).squaredNorm();
    if(tmp > c_proj_max) c_proj_max = tmp;

    c_proj_max = sqrt(c_proj_max);

    S v_dot_n = axis.dot(visitor.n) * linear_vel;
    S w_cross_n = (axis.cross(visitor.n)).norm() * angular_vel;
    S origin_proj = ((tf.translation() - p).cross(axis)).norm();

    S mu = v_dot_n + w_cross_n * (c_proj_max + visitor.bv.r + origin_proj);

    return mu;
  }
};

//==============================================================================
/// @brief Compute the motion bound for a bounding volume along a given direction n
/// according to mu < |v * n| + ||w x n||(r + max(||ci*||)) where ||ci*|| = ||R0(ci) x w||. w is the angular axis (normalized)
/// and ci are the endpoints of the generator primitives of RSSd.
/// Notice that all bv parameters are in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
template <typename S>
struct TBVMotionBoundVisitorVisitImpl<S, RSS<S>, InterpMotion<S>>
{
  static S run(
      const TBVMotionBoundVisitor<RSS<S>>& visitor,
      const InterpMotion<S>& motion)
  {
    Transform3<S> tf;
    motion.getCurrentTransform(tf);

    const Vector3<S>& reference_p = motion.getReferencePoint();
    const Vector3<S>& angular_axis = motion.getAngularAxis();
    S angular_vel = motion.getAngularVelocity();
    const Vector3<S>& linear_vel = motion.getLinearVelocity();

    S c_proj_max = ((tf.linear() * (visitor.bv.To - reference_p)).cross(angular_axis)).squaredNorm();
    S tmp;
    tmp = ((tf.linear() * (visitor.bv.To + visitor.bv.axis.col(0) * visitor.bv.l[0] - reference_p)).cross(angular_axis)).squaredNorm();
    if(tmp > c_proj_max) c_proj_max = tmp;
    tmp = ((tf.linear() * (visitor.bv.To + visitor.bv.axis.col(1) * visitor.bv.l[1] - reference_p)).cross(angular_axis)).squaredNorm();
    if(tmp > c_proj_max) c_proj_max = tmp;
    tmp = ((tf.linear() * (visitor.bv.To + visitor.bv.axis.col(0) * visitor.bv.l[0] + visitor.bv.axis.col(1) * visitor.bv.l[1] - reference_p)).cross(angular_axis)).squaredNorm();
    if(tmp > c_proj_max) c_proj_max = tmp;

    c_proj_max = std::sqrt(c_proj_max);

    S v_dot_n = linear_vel.dot(visitor.n);
    S w_cross_n = (angular_axis.cross(visitor.n)).norm() * angular_vel;
    S mu = v_dot_n + w_cross_n * (visitor.bv.r + c_proj_max);

    return mu;
  }
};

//==============================================================================
/// @brief Compute the motion bound for a bounding volume along a given direction n
template <typename S>
struct TBVMotionBoundVisitorVisitImpl<S, RSS<S>, TranslationMotion<S>>
{
  static S run(
      const TBVMotionBoundVisitor<RSS<S>>& visitor,
      const TranslationMotion<S>& motion)
  {
    return motion.getVelocity().dot(visitor.n);
  }
};

//==============================================================================
template <typename S, typename MotionT>
struct TriangleMotionBoundVisitorVisitImpl
{
  static S run(
      const TriangleMotionBoundVisitor<S>& /*visitor*/,
      const MotionT& /*motion*/)
  {
    return 0;
  }
};

//==============================================================================
template<typename S>
S TriangleMotionBoundVisitor<S>::visit(
    const SplineMotion<S>& motion) const
{
  return TriangleMotionBoundVisitorVisitImpl<
      S, SplineMotion<S>>::run(*this, motion);
}

//==============================================================================
template<typename S>
S TriangleMotionBoundVisitor<S>::visit(
    const ScrewMotion<S>& motion) const
{
  return TriangleMotionBoundVisitorVisitImpl<
      S, ScrewMotion<S>>::run(*this, motion);
}

//==============================================================================
template<typename S>
S TriangleMotionBoundVisitor<S>::visit(
    const InterpMotion<S>& motion) const
{
  return TriangleMotionBoundVisitorVisitImpl<
      S, InterpMotion<S>>::run(*this, motion);
}

//==============================================================================
template<typename S>
S TriangleMotionBoundVisitor<S>::visit(
    const TranslationMotion<S>& motion) const
{
  return TriangleMotionBoundVisitorVisitImpl<
      S, TranslationMotion<S>>::run(*this, motion);
}

//==============================================================================
/// @brief Compute the motion bound for a triangle along a given direction n
/// according to mu < |v * n| + ||w x n||(max||ci*||) where ||ci*|| = ||R0(ci) x w|| / \|w\|. w is the angular velocity
/// and ci are the triangle vertex coordinates.
/// Notice that the triangle is in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
template <typename S>
struct TriangleMotionBoundVisitorVisitImpl<S, ScrewMotion<S>>
{
  static S run(
      const TriangleMotionBoundVisitor<S>& visitor,
      const ScrewMotion<S>& motion)
  {
    Transform3<S> tf;
    motion.getCurrentTransform(tf);

    const Vector3<S>& axis = motion.getAxis();
    S linear_vel = motion.getLinearVelocity();
    S angular_vel = motion.getAngularVelocity();
    const Vector3<S>& p = motion.getAxisOrigin();

    S proj_max = ((tf.linear() * visitor.a + tf.translation() - p).cross(axis)).squaredNorm();
    S tmp;
    tmp = ((tf.linear() * visitor.b + tf.translation() - p).cross(axis)).squaredNorm();
    if(tmp > proj_max) proj_max = tmp;
    tmp = ((tf.linear() * visitor.c + tf.translation() - p).cross(axis)).squaredNorm();
    if(tmp > proj_max) proj_max = tmp;

    proj_max = std::sqrt(proj_max);

    S v_dot_n = axis.dot(visitor.n) * linear_vel;
    S w_cross_n = (axis.cross(visitor.n)).norm() * angular_vel;
    S mu = v_dot_n + w_cross_n * proj_max;

    return mu;
  }
};

//==============================================================================
/// @brief Compute the motion bound for a triangle along a given direction n
/// according to mu < |v * n| + ||w x n||(max||ci*||) where ||ci*|| = ||R0(ci) x w|| / \|w\|. w is the angular velocity
/// and ci are the triangle vertex coordinates.
/// Notice that the triangle is in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
template <typename S>
struct TriangleMotionBoundVisitorVisitImpl<S, InterpMotion<S>>
{
  static S run(
      const TriangleMotionBoundVisitor<S>& visitor,
      const InterpMotion<S>& motion)
  {
    Transform3<S> tf;
    motion.getCurrentTransform(tf);

    const Vector3<S>& reference_p = motion.getReferencePoint();
    const Vector3<S>& angular_axis = motion.getAngularAxis();
    S angular_vel = motion.getAngularVelocity();
    const Vector3<S>& linear_vel = motion.getLinearVelocity();

    S proj_max = ((tf.linear() * (visitor.a - reference_p)).cross(angular_axis)).squaredNorm();
    S tmp;
    tmp = ((tf.linear() * (visitor.b - reference_p)).cross(angular_axis)).squaredNorm();
    if(tmp > proj_max) proj_max = tmp;
    tmp = ((tf.linear() * (visitor.c - reference_p)).cross(angular_axis)).squaredNorm();
    if(tmp > proj_max) proj_max = tmp;

    proj_max = std::sqrt(proj_max);

    S v_dot_n = linear_vel.dot(visitor.n);
    S w_cross_n = (angular_axis.cross(visitor.n)).norm() * angular_vel;
    S mu = v_dot_n + w_cross_n * proj_max;

    return mu;
  }
};

//==============================================================================
template <typename S>
struct TriangleMotionBoundVisitorVisitImpl<S, SplineMotion<S>>
{
  static S run(
      const TriangleMotionBoundVisitor<S>& visitor,
      const SplineMotion<S>& motion)
  {
    S T_bound = motion.computeTBound(visitor.n);
    S tf_t = motion.getCurrentTime();

    S R_bound = std::abs(visitor.a.dot(visitor.n)) + visitor.a.norm() + (visitor.a.cross(visitor.n)).norm();
    S R_bound_tmp = std::abs(visitor.b.dot(visitor.n)) + visitor.b.norm() + (visitor.b.cross(visitor.n)).norm();
    if(R_bound_tmp > R_bound) R_bound = R_bound_tmp;
    R_bound_tmp = std::abs(visitor.c.dot(visitor.n)) + visitor.c.norm() + (visitor.c.cross(visitor.n)).norm();
    if(R_bound_tmp > R_bound) R_bound = R_bound_tmp;

    S dWdW_max = motion.computeDWMax();
    S ratio = std::min(1 - tf_t, dWdW_max);

    R_bound *= 2 * ratio;

    // std::cout << R_bound << " " << T_bound << std::endl;

    return R_bound + T_bound;
  }
};

//==============================================================================
/// @brief Compute the motion bound for a triangle along a given direction n
template <typename S>
struct TriangleMotionBoundVisitorVisitImpl<S, TranslationMotion<S>>
{
  static S run(
      const TriangleMotionBoundVisitor<S>& visitor,
      const TranslationMotion<S>& motion)
  {
    return motion.getVelocity().dot(visitor.n);
  }
};

} // namespace fcl

#endif
