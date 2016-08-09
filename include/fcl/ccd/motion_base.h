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

template <typename Scalar>
class MotionBase;

template <typename Scalar>
class SplineMotion;

template <typename Scalar>
class ScrewMotion;

template <typename Scalar>
class InterpMotion;

template <typename Scalar>
class TranslationMotion;

/// @brief Compute the motion bound for a bounding volume, given the closest direction n between two query objects
template <typename Scalar>
class BVMotionBoundVisitor
{
public:
  virtual Scalar visit(const MotionBase<Scalar>& motion) const = 0;
  virtual Scalar visit(const SplineMotion<Scalar>& motion) const = 0;
  virtual Scalar visit(const ScrewMotion<Scalar>& motion) const = 0;
  virtual Scalar visit(const InterpMotion<Scalar>& motion) const = 0;
  virtual Scalar visit(const TranslationMotion<Scalar>& motion) const = 0;
};

template<typename BV>
class TBVMotionBoundVisitor : public BVMotionBoundVisitor<typename BV::Scalar>
{
public:
  using Scalar = typename BV::Scalar;

  TBVMotionBoundVisitor(const BV& bv_, const Vector3<Scalar>& n_) : bv(bv_), n(n_) {}

  virtual Scalar visit(const MotionBase<Scalar>& motion) const;
  virtual Scalar visit(const SplineMotion<Scalar>& motion) const;
  virtual Scalar visit(const ScrewMotion<Scalar>& motion) const;
  virtual Scalar visit(const InterpMotion<Scalar>& motion) const;
  virtual Scalar visit(const TranslationMotion<Scalar>& motion) const;

protected:
  template <typename, typename, typename>
  friend struct TBVMotionBoundVisitorVisitImpl;

  BV bv;
  Vector3<Scalar> n;
};

template <typename Scalar>
class TriangleMotionBoundVisitor
{
public:
  TriangleMotionBoundVisitor(const Vector3<Scalar>& a_, const Vector3<Scalar>& b_, const Vector3<Scalar>& c_, const Vector3<Scalar>& n_) :
    a(a_), b(b_), c(c_), n(n_) {}

  virtual Scalar visit(const MotionBase<Scalar>& motion) const { return 0; }
  virtual Scalar visit(const SplineMotion<Scalar>& motion) const;
  virtual Scalar visit(const ScrewMotion<Scalar>& motion) const;
  virtual Scalar visit(const InterpMotion<Scalar>& motion) const;
  virtual Scalar visit(const TranslationMotion<Scalar>& motion) const;

protected:
  template <typename, typename>
  friend struct TriangleMotionBoundVisitorVisitImpl;

  Vector3<Scalar> a, b, c, n;
};

template <typename Scalar>
class MotionBase
{
public:
  MotionBase()
    : time_interval_(std::shared_ptr<TimeInterval<Scalar>>(new TimeInterval<Scalar>(0, 1)))
  {
  }
  
  virtual ~MotionBase() {}

  /** \brief Integrate the motion from 0 to dt */
  virtual bool integrate(Scalar dt) const = 0;

  /** \brief Compute the motion bound for a bounding volume, given the closest direction n between two query objects */
  virtual Scalar computeMotionBound(const BVMotionBoundVisitor<Scalar>& mb_visitor) const = 0;

  /** \brief Compute the motion bound for a triangle, given the closest direction n between two query objects */
  virtual Scalar computeMotionBound(const TriangleMotionBoundVisitor<Scalar>& mb_visitor) const = 0;

  /** \brief Get the rotation and translation in current step */
  void getCurrentTransform(Matrix3<Scalar>& R, Vector3<Scalar>& T) const
  {
    Transform3<Scalar> tf;
    getCurrentTransform(tf);
    R = tf.linear();
    T = tf.translation();
  }

  void getCurrentTransform(Quaternion<Scalar>& Q, Vector3<Scalar>& T) const
  {
    Transform3<Scalar> tf;
    getCurrentTransform(tf);
    Q = tf.linear();
    T = tf.translation();
  }

  void getCurrentRotation(Matrix3<Scalar>& R) const
  {
    Transform3<Scalar> tf;
    getCurrentTransform(tf);
    R = tf.linear();
  }

  void getCurrentRotation(Quaternion<Scalar>& Q) const
  {
    Transform3<Scalar> tf;
    getCurrentTransform(tf);
    Q = tf.linear();
  }

  void getCurrentTranslation(Vector3<Scalar>& T) const
  {
    Transform3<Scalar> tf;
    getCurrentTransform(tf);
    T = tf.translation();
  }

  virtual void getCurrentTransform(Transform3<Scalar>& tf) const = 0;

  virtual void getTaylorModel(TMatrix3<Scalar>& tm, TVector3<Scalar>& tv) const = 0;

  const std::shared_ptr<TimeInterval<Scalar>>& getTimeInterval() const
  {
    return time_interval_;
  }
protected:

  std::shared_ptr<TimeInterval<Scalar>> time_interval_;
  
};

using MotionBasef = MotionBase<float>;
using MotionBased = MotionBase<double>;

template <typename Scalar>
using MotionBasePtr = std::shared_ptr<MotionBase<Scalar>>;

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar, typename BV, typename MotionT>
struct TBVMotionBoundVisitorVisitImpl
{
  static Scalar run(
      const TBVMotionBoundVisitor<BV>& /*visitor*/,
      const MotionT& /*motion*/)
  {
    return 0;
  }
};

//==============================================================================
template<typename BV>
typename BV::Scalar TBVMotionBoundVisitor<BV>::visit(
    const MotionBase<Scalar>& motion) const
{
  return 0;
}
//==============================================================================
template<typename BV>
typename BV::Scalar TBVMotionBoundVisitor<BV>::visit(
    const SplineMotion<Scalar>& motion) const
{
  using Scalar = typename BV::Scalar;

  return TBVMotionBoundVisitorVisitImpl<
      Scalar, BV, SplineMotion<Scalar>>::run(*this, motion);
}

//==============================================================================
template<typename BV>
typename BV::Scalar TBVMotionBoundVisitor<BV>::visit(
    const ScrewMotion<Scalar>& motion) const
{
  using Scalar = typename BV::Scalar;

  return TBVMotionBoundVisitorVisitImpl<
      Scalar, BV, ScrewMotion<Scalar>>::run(*this, motion);
}

//==============================================================================
template<typename BV>
typename BV::Scalar TBVMotionBoundVisitor<BV>::visit(
    const InterpMotion<Scalar>& motion) const
{
  using Scalar = typename BV::Scalar;

  return TBVMotionBoundVisitorVisitImpl<
      Scalar, BV, InterpMotion<Scalar>>::run(*this, motion);
}

//==============================================================================
template<typename BV>
typename BV::Scalar TBVMotionBoundVisitor<BV>::visit(
    const TranslationMotion<Scalar>& motion) const
{
  using Scalar = typename BV::Scalar;

  return TBVMotionBoundVisitorVisitImpl<
      Scalar, BV, TranslationMotion<Scalar>>::run(*this, motion);
}

//==============================================================================
template <typename Scalar>
struct TBVMotionBoundVisitorVisitImpl<Scalar, RSS<Scalar>, SplineMotion<Scalar>>
{
  static Scalar run(
      const TBVMotionBoundVisitor<RSS<Scalar>>& visitor,
      const SplineMotion<Scalar>& motion)
  {
    Scalar T_bound = motion.computeTBound(visitor.n);
    Scalar tf_t = motion.getCurrentTime();

    Vector3<Scalar> c1 = visitor.bv.frame.translation();
    Vector3<Scalar> c2 = visitor.bv.frame.translation() + visitor.bv.frame.linear().col(0) * visitor.bv.l[0];
    Vector3<Scalar> c3 = visitor.bv.frame.translation() + visitor.bv.frame.linear().col(1) * visitor.bv.l[1];
    Vector3<Scalar> c4 = visitor.bv.frame.translation() + visitor.bv.frame.linear().col(0) * visitor.bv.l[0] + visitor.bv.frame.linear().col(1) * visitor.bv.l[1];

    Scalar tmp;
    // max_i |c_i * n|
    Scalar cn_max = std::abs(c1.dot(visitor.n));
    tmp = std::abs(c2.dot(visitor.n));
    if(tmp > cn_max) cn_max = tmp;
    tmp = std::abs(c3.dot(visitor.n));
    if(tmp > cn_max) cn_max = tmp;
    tmp = std::abs(c4.dot(visitor.n));
    if(tmp > cn_max) cn_max = tmp;

    // max_i ||c_i||
    Scalar cmax = c1.squaredNorm();
    tmp = c2.squaredNorm();
    if(tmp > cmax) cmax = tmp;
    tmp = c3.squaredNorm();
    if(tmp > cmax) cmax = tmp;
    tmp = c4.squaredNorm();
    if(tmp > cmax) cmax = tmp;
    cmax = sqrt(cmax);

    // max_i ||c_i x n||
    Scalar cxn_max = (c1.cross(visitor.n)).squaredNorm();
    tmp = (c2.cross(visitor.n)).squaredNorm();
    if(tmp > cxn_max) cxn_max = tmp;
    tmp = (c3.cross(visitor.n)).squaredNorm();
    if(tmp > cxn_max) cxn_max = tmp;
    tmp = (c4.cross(visitor.n)).squaredNorm();
    if(tmp > cxn_max) cxn_max = tmp;
    cxn_max = sqrt(cxn_max);

    Scalar dWdW_max = motion.computeDWMax();
    Scalar ratio = std::min(1 - tf_t, dWdW_max);

    Scalar R_bound = 2 * (cn_max + cmax + cxn_max + 3 * visitor.bv.r) * ratio;


    // std::cout << R_bound << " " << T_bound << std::endl;

    return R_bound + T_bound;
  }
};

//==============================================================================
/// @brief Compute the motion bound for a bounding volume along a given direction n
/// according to mu < |v * n| + ||w x n||(r + max(||ci*||)) where ||ci*|| = ||R0(ci) x w||. w is the angular axis (normalized)
/// and ci are the endpoints of the generator primitives of RSSd.
/// Notice that all bv parameters are in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
template <typename Scalar>
struct TBVMotionBoundVisitorVisitImpl<Scalar, RSS<Scalar>, ScrewMotion<Scalar>>
{
  static Scalar run(
      const TBVMotionBoundVisitor<RSS<Scalar>>& visitor,
      const ScrewMotion<Scalar>& motion)
  {
    Transform3<Scalar> tf;
    motion.getCurrentTransform(tf);

    const Vector3<Scalar>& axis = motion.getAxis();
    Scalar linear_vel = motion.getLinearVelocity();
    Scalar angular_vel = motion.getAngularVelocity();
    const Vector3<Scalar>& p = motion.getAxisOrigin();

    Scalar c_proj_max = ((tf.linear() * visitor.bv.frame.translation()).cross(axis)).squaredNorm();
    Scalar tmp;
    tmp = ((tf.linear() * (visitor.bv.frame.translation() + visitor.bv.frame.linear().col(0) * visitor.bv.l[0])).cross(axis)).squaredNorm();
    if(tmp > c_proj_max) c_proj_max = tmp;
    tmp = ((tf.linear() * (visitor.bv.frame.translation() + visitor.bv.frame.linear().col(1) * visitor.bv.l[1])).cross(axis)).squaredNorm();
    if(tmp > c_proj_max) c_proj_max = tmp;
    tmp = ((tf.linear() * (visitor.bv.frame.translation() + visitor.bv.frame.linear().col(0) * visitor.bv.l[0] + visitor.bv.frame.linear().col(1) * visitor.bv.l[1])).cross(axis)).squaredNorm();
    if(tmp > c_proj_max) c_proj_max = tmp;

    c_proj_max = sqrt(c_proj_max);

    Scalar v_dot_n = axis.dot(visitor.n) * linear_vel;
    Scalar w_cross_n = (axis.cross(visitor.n)).norm() * angular_vel;
    Scalar origin_proj = ((tf.translation() - p).cross(axis)).norm();

    Scalar mu = v_dot_n + w_cross_n * (c_proj_max + visitor.bv.r + origin_proj);

    return mu;
  }
};

//==============================================================================
/// @brief Compute the motion bound for a bounding volume along a given direction n
/// according to mu < |v * n| + ||w x n||(r + max(||ci*||)) where ||ci*|| = ||R0(ci) x w||. w is the angular axis (normalized)
/// and ci are the endpoints of the generator primitives of RSSd.
/// Notice that all bv parameters are in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
template <typename Scalar>
struct TBVMotionBoundVisitorVisitImpl<Scalar, RSS<Scalar>, InterpMotion<Scalar>>
{
  static Scalar run(
      const TBVMotionBoundVisitor<RSS<Scalar>>& visitor,
      const InterpMotion<Scalar>& motion)
  {
    Transform3<Scalar> tf;
    motion.getCurrentTransform(tf);

    const Vector3<Scalar>& reference_p = motion.getReferencePoint();
    const Vector3<Scalar>& angular_axis = motion.getAngularAxis();
    Scalar angular_vel = motion.getAngularVelocity();
    const Vector3<Scalar>& linear_vel = motion.getLinearVelocity();

    Scalar c_proj_max = ((tf.linear() * (visitor.bv.frame.translation() - reference_p)).cross(angular_axis)).squaredNorm();
    Scalar tmp;
    tmp = ((tf.linear() * (visitor.bv.frame.translation() + visitor.bv.frame.linear().col(0) * visitor.bv.l[0] - reference_p)).cross(angular_axis)).squaredNorm();
    if(tmp > c_proj_max) c_proj_max = tmp;
    tmp = ((tf.linear() * (visitor.bv.frame.translation() + visitor.bv.frame.linear().col(1) * visitor.bv.l[1] - reference_p)).cross(angular_axis)).squaredNorm();
    if(tmp > c_proj_max) c_proj_max = tmp;
    tmp = ((tf.linear() * (visitor.bv.frame.translation() + visitor.bv.frame.linear().col(0) * visitor.bv.l[0] + visitor.bv.frame.linear().col(1) * visitor.bv.l[1] - reference_p)).cross(angular_axis)).squaredNorm();
    if(tmp > c_proj_max) c_proj_max = tmp;

    c_proj_max = std::sqrt(c_proj_max);

    Scalar v_dot_n = linear_vel.dot(visitor.n);
    Scalar w_cross_n = (angular_axis.cross(visitor.n)).norm() * angular_vel;
    Scalar mu = v_dot_n + w_cross_n * (visitor.bv.r + c_proj_max);

    return mu;
  }
};

//==============================================================================
/// @brief Compute the motion bound for a bounding volume along a given direction n
template <typename Scalar>
struct TBVMotionBoundVisitorVisitImpl<Scalar, RSS<Scalar>, TranslationMotion<Scalar>>
{
  static Scalar run(
      const TBVMotionBoundVisitor<RSS<Scalar>>& visitor,
      const TranslationMotion<Scalar>& motion)
  {
    return motion.getVelocity().dot(visitor.n);
  }
};

//==============================================================================
template <typename Scalar, typename MotionT>
struct TriangleMotionBoundVisitorVisitImpl
{
  static Scalar run(
      const TriangleMotionBoundVisitor<Scalar>& /*visitor*/,
      const MotionT& /*motion*/)
  {
    return 0;
  }
};

//==============================================================================
template<typename Scalar>
Scalar TriangleMotionBoundVisitor<Scalar>::visit(
    const SplineMotion<Scalar>& motion) const
{
  return TriangleMotionBoundVisitorVisitImpl<
      Scalar, SplineMotion<Scalar>>::run(*this, motion);
}

//==============================================================================
template<typename Scalar>
Scalar TriangleMotionBoundVisitor<Scalar>::visit(
    const ScrewMotion<Scalar>& motion) const
{
  return TriangleMotionBoundVisitorVisitImpl<
      Scalar, ScrewMotion<Scalar>>::run(*this, motion);
}

//==============================================================================
template<typename Scalar>
Scalar TriangleMotionBoundVisitor<Scalar>::visit(
    const InterpMotion<Scalar>& motion) const
{
  return TriangleMotionBoundVisitorVisitImpl<
      Scalar, InterpMotion<Scalar>>::run(*this, motion);
}

//==============================================================================
template<typename Scalar>
Scalar TriangleMotionBoundVisitor<Scalar>::visit(
    const TranslationMotion<Scalar>& motion) const
{
  return TriangleMotionBoundVisitorVisitImpl<
      Scalar, TranslationMotion<Scalar>>::run(*this, motion);
}

//==============================================================================
/// @brief Compute the motion bound for a triangle along a given direction n
/// according to mu < |v * n| + ||w x n||(max||ci*||) where ||ci*|| = ||R0(ci) x w|| / \|w\|. w is the angular velocity
/// and ci are the triangle vertex coordinates.
/// Notice that the triangle is in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
template <typename Scalar>
struct TriangleMotionBoundVisitorVisitImpl<Scalar, ScrewMotion<Scalar>>
{
  static Scalar run(
      const TriangleMotionBoundVisitor<Scalar>& visitor,
      const ScrewMotion<Scalar>& motion)
  {
    Transform3<Scalar> tf;
    motion.getCurrentTransform(tf);

    const Vector3<Scalar>& axis = motion.getAxis();
    Scalar linear_vel = motion.getLinearVelocity();
    Scalar angular_vel = motion.getAngularVelocity();
    const Vector3<Scalar>& p = motion.getAxisOrigin();

    Scalar proj_max = ((tf.linear() * visitor.a + tf.translation() - p).cross(axis)).squaredNorm();
    Scalar tmp;
    tmp = ((tf.linear() * visitor.b + tf.translation() - p).cross(axis)).squaredNorm();
    if(tmp > proj_max) proj_max = tmp;
    tmp = ((tf.linear() * visitor.c + tf.translation() - p).cross(axis)).squaredNorm();
    if(tmp > proj_max) proj_max = tmp;

    proj_max = std::sqrt(proj_max);

    Scalar v_dot_n = axis.dot(visitor.n) * linear_vel;
    Scalar w_cross_n = (axis.cross(visitor.n)).norm() * angular_vel;
    Scalar mu = v_dot_n + w_cross_n * proj_max;

    return mu;
  }
};

//==============================================================================
/// @brief Compute the motion bound for a triangle along a given direction n
/// according to mu < |v * n| + ||w x n||(max||ci*||) where ||ci*|| = ||R0(ci) x w|| / \|w\|. w is the angular velocity
/// and ci are the triangle vertex coordinates.
/// Notice that the triangle is in the local frame of the object, but n should be in the global frame (the reason is that the motion (t1, t2 and t) is in global frame)
template <typename Scalar>
struct TriangleMotionBoundVisitorVisitImpl<Scalar, InterpMotion<Scalar>>
{
  static Scalar run(
      const TriangleMotionBoundVisitor<Scalar>& visitor,
      const InterpMotion<Scalar>& motion)
  {
    Transform3<Scalar> tf;
    motion.getCurrentTransform(tf);

    const Vector3<Scalar>& reference_p = motion.getReferencePoint();
    const Vector3<Scalar>& angular_axis = motion.getAngularAxis();
    Scalar angular_vel = motion.getAngularVelocity();
    const Vector3<Scalar>& linear_vel = motion.getLinearVelocity();

    Scalar proj_max = ((tf.linear() * (visitor.a - reference_p)).cross(angular_axis)).squaredNorm();
    Scalar tmp;
    tmp = ((tf.linear() * (visitor.b - reference_p)).cross(angular_axis)).squaredNorm();
    if(tmp > proj_max) proj_max = tmp;
    tmp = ((tf.linear() * (visitor.c - reference_p)).cross(angular_axis)).squaredNorm();
    if(tmp > proj_max) proj_max = tmp;

    proj_max = std::sqrt(proj_max);

    Scalar v_dot_n = linear_vel.dot(visitor.n);
    Scalar w_cross_n = (angular_axis.cross(visitor.n)).norm() * angular_vel;
    Scalar mu = v_dot_n + w_cross_n * proj_max;

    return mu;
  }
};

//==============================================================================
template <typename Scalar>
struct TriangleMotionBoundVisitorVisitImpl<Scalar, SplineMotion<Scalar>>
{
  static Scalar run(
      const TriangleMotionBoundVisitor<Scalar>& visitor,
      const SplineMotion<Scalar>& motion)
  {
    Scalar T_bound = motion.computeTBound(visitor.n);
    Scalar tf_t = motion.getCurrentTime();

    Scalar R_bound = std::abs(visitor.a.dot(visitor.n)) + visitor.a.norm() + (visitor.a.cross(visitor.n)).norm();
    Scalar R_bound_tmp = std::abs(visitor.b.dot(visitor.n)) + visitor.b.norm() + (visitor.b.cross(visitor.n)).norm();
    if(R_bound_tmp > R_bound) R_bound = R_bound_tmp;
    R_bound_tmp = std::abs(visitor.c.dot(visitor.n)) + visitor.c.norm() + (visitor.c.cross(visitor.n)).norm();
    if(R_bound_tmp > R_bound) R_bound = R_bound_tmp;

    Scalar dWdW_max = motion.computeDWMax();
    Scalar ratio = std::min(1 - tf_t, dWdW_max);

    R_bound *= 2 * ratio;

    // std::cout << R_bound << " " << T_bound << std::endl;

    return R_bound + T_bound;
  }
};

//==============================================================================
/// @brief Compute the motion bound for a triangle along a given direction n
template <typename Scalar>
struct TriangleMotionBoundVisitorVisitImpl<Scalar, TranslationMotion<Scalar>>
{
  static Scalar run(
      const TriangleMotionBoundVisitor<Scalar>& visitor,
      const TranslationMotion<Scalar>& motion)
  {
    return motion.getVelocity().dot(visitor.n);
  }
};

} // namespace fcl

#endif
