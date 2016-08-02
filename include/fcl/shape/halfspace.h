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

#ifndef FCL_SHAPE_HALFSPACE_H
#define FCL_SHAPE_HALFSPACE_H

#include "fcl/shape/shape_base.h"
#include "fcl/shape/compute_bv.h"
#include "fcl/BV/OBB.h"
#include "fcl/BV/RSS.h"
#include "fcl/BV/OBBRSS.h"
#include "fcl/BV/kDOP.h"
#include "fcl/BV/kIOS.h"

namespace fcl
{

/// @brief Half Space: this is equivalent to the Planed in ODE. The separation plane is defined as n * x = d;
/// Points in the negative side of the separation plane (i.e. {x | n * x < d}) are inside the half space and points
/// in the positive side of the separation plane (i.e. {x | n * x > d}) are outside the half space
template <typename Scalar>
class Halfspace : public ShapeBase<Scalar>
{
public:
  /// @brief Construct a half space with normal direction and offset
  Halfspace(const Vector3<Scalar>& n, Scalar d);

  /// @brief Construct a plane with normal direction and offset
  Halfspace(Scalar a, Scalar b, Scalar c, Scalar d_);

  Halfspace();

  Scalar signedDistance(const Vector3<Scalar>& p) const;

  Scalar distance(const Vector3<Scalar>& p) const;

  /// @brief Compute AABBd
  void computeLocalAABB() override;

  /// @brief Get node type: a half space
  NODE_TYPE getNodeType() const override;
  
  /// @brief Planed normal
  Vector3<Scalar> n;
  
  /// @brief Planed offset
  Scalar d;

protected:

  /// @brief Turn non-unit normal into unit
  void unitNormalTest();
};

using Halfspacef = Halfspace<float>;
using Halfspaced = Halfspace<double>;

template <typename Scalar>
Halfspace<Scalar> transform(
    const Halfspace<Scalar>& a, const Transform3<Scalar>& tf)
{
  /// suppose the initial halfspace is n * x <= d
  /// after transform (R, T), x --> x' = R x + T
  /// and the new half space becomes n' * x' <= d'
  /// where n' = R * n
  ///   and d' = d + n' * T

  Vector3<Scalar> n = tf.linear() * a.n;
  Scalar d = a.d + n.dot(tf.translation());

  return Halfspace<Scalar>(n, d);
}

template <typename Scalar>
struct ComputeBVImpl<Scalar, AABBd, Halfspace<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, OBB<Scalar>, Halfspace<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, RSSd, Halfspace<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, OBBRSSd, Halfspace<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, kIOSd, Halfspace<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, KDOPd<16>, Halfspace<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, KDOPd<18>, Halfspace<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, KDOPd<24>, Halfspace<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, AABBd, Halfspace<Scalar>>
{
  void operator()(const Halfspace<Scalar>& s, const Transform3<Scalar>& tf, AABBd& bv)
  {
    Halfspace<Scalar> new_s = transform(s, tf);
    const Vector3d& n = new_s.n;
    const FCL_REAL& d = new_s.d;

    AABBd bv_;
    bv_.min_ = Vector3d::Constant(-std::numeric_limits<FCL_REAL>::max());
    bv_.max_ = Vector3d::Constant(std::numeric_limits<FCL_REAL>::max());
    if(n[1] == (FCL_REAL)0.0 && n[2] == (FCL_REAL)0.0)
    {
      // normal aligned with x axis
      if(n[0] < 0) bv_.min_[0] = -d;
      else if(n[0] > 0) bv_.max_[0] = d;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[2] == (FCL_REAL)0.0)
    {
      // normal aligned with y axis
      if(n[1] < 0) bv_.min_[1] = -d;
      else if(n[1] > 0) bv_.max_[1] = d;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[1] == (FCL_REAL)0.0)
    {
      // normal aligned with z axis
      if(n[2] < 0) bv_.min_[2] = -d;
      else if(n[2] > 0) bv_.max_[2] = d;
    }

    bv = bv_;
  }
};

template <typename Scalar>
struct ComputeBVImpl<Scalar, OBB<Scalar>, Halfspace<Scalar>>
{
  void operator()(const Halfspace<Scalar>& s, const Transform3<Scalar>& tf, OBB<Scalar>& bv)
  {
    /// Half space can only have very rough OBB<Scalar>
    bv.axis.setIdentity();
    bv.To.setZero();
    bv.extent.setConstant(std::numeric_limits<FCL_REAL>::max());
  }
};

template <typename Scalar>
struct ComputeBVImpl<Scalar, RSSd, Halfspace<Scalar>>
{
  void operator()(const Halfspace<Scalar>& s, const Transform3<Scalar>& tf, RSSd& bv)
  {
    /// Half space can only have very rough RSSd
    bv.axis.setIdentity();
    bv.Tr.setZero();
    bv.l[0] = bv.l[1] = bv.r = std::numeric_limits<FCL_REAL>::max();
  }
};

template <typename Scalar>
struct ComputeBVImpl<Scalar, OBBRSSd, Halfspace<Scalar>>
{
  void operator()(const Halfspace<Scalar>& s, const Transform3<Scalar>& tf, OBBRSSd& bv)
  {
    computeBV<Scalar, OBB<Scalar>, Halfspace<Scalar>>(s, tf, bv.obb);
    computeBV<Scalar, RSSd, Halfspace<Scalar>>(s, tf, bv.rss);
  }
};

template <typename Scalar>
struct ComputeBVImpl<Scalar, kIOSd, Halfspace<Scalar>>
{
  void operator()(const Halfspace<Scalar>& s, const Transform3<Scalar>& tf, kIOSd& bv)
  {
    bv.num_spheres = 1;
    computeBV<Scalar, OBB<Scalar>, Halfspace<Scalar>>(s, tf, bv.obb);
    bv.spheres[0].o.setZero();
    bv.spheres[0].r = std::numeric_limits<FCL_REAL>::max();
  }
};

template <typename Scalar>
struct ComputeBVImpl<Scalar, KDOPd<16>, Halfspace<Scalar>>
{
  void operator()(const Halfspace<Scalar>& s, const Transform3<Scalar>& tf, KDOPd<16>& bv)
  {
    Halfspace<Scalar> new_s = transform(s, tf);
    const Vector3d& n = new_s.n;
    const FCL_REAL& d = new_s.d;

    const std::size_t D = 8;
    for(std::size_t i = 0; i < D; ++i)
      bv.dist(i) = -std::numeric_limits<FCL_REAL>::max();
    for(std::size_t i = D; i < 2 * D; ++i)
      bv.dist(i) = std::numeric_limits<FCL_REAL>::max();

    if(n[1] == (FCL_REAL)0.0 && n[2] == (FCL_REAL)0.0)
    {
      if(n[0] > 0) bv.dist(D) = d;
      else bv.dist(0) = -d;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[2] == (FCL_REAL)0.0)
    {
      if(n[1] > 0) bv.dist(D + 1) = d;
      else bv.dist(1) = -d;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[1] == (FCL_REAL)0.0)
    {
      if(n[2] > 0) bv.dist(D + 2) = d;
      else bv.dist(2) = -d;
    }
    else if(n[2] == (FCL_REAL)0.0 && n[0] == n[1])
    {
      if(n[0] > 0) bv.dist(D + 3) = n[0] * d * 2;
      else bv.dist(3) = n[0] * d * 2;
    }
    else if(n[1] == (FCL_REAL)0.0 && n[0] == n[2])
    {
      if(n[1] > 0) bv.dist(D + 4) = n[0] * d * 2;
      else bv.dist(4) = n[0] * d * 2;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[1] == n[2])
    {
      if(n[1] > 0) bv.dist(D + 5) = n[1] * d * 2;
      else bv.dist(5) = n[1] * d * 2;
    }
    else if(n[2] == (FCL_REAL)0.0 && n[0] + n[1] == (FCL_REAL)0.0)
    {
      if(n[0] > 0) bv.dist(D + 6) = n[0] * d * 2;
      else bv.dist(6) = n[0] * d * 2;
    }
    else if(n[1] == (FCL_REAL)0.0 && n[0] + n[2] == (FCL_REAL)0.0)
    {
      if(n[0] > 0) bv.dist(D + 7) = n[0] * d * 2;
      else bv.dist(7) = n[0] * d * 2;
    }
  }
};

template <typename Scalar>
struct ComputeBVImpl<Scalar, KDOPd<18>, Halfspace<Scalar>>
{
  void operator()(const Halfspace<Scalar>& s, const Transform3<Scalar>& tf, KDOPd<18>& bv)
  {
    Halfspace<Scalar> new_s = transform(s, tf);
    const Vector3d& n = new_s.n;
    const FCL_REAL& d = new_s.d;

    const std::size_t D = 9;

    for(std::size_t i = 0; i < D; ++i)
      bv.dist(i) = -std::numeric_limits<FCL_REAL>::max();
    for(std::size_t i = D; i < 2 * D; ++i)
      bv.dist(i) = std::numeric_limits<FCL_REAL>::max();

    if(n[1] == (FCL_REAL)0.0 && n[2] == (FCL_REAL)0.0)
    {
      if(n[0] > 0) bv.dist(D) = d;
      else bv.dist(0) = -d;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[2] == (FCL_REAL)0.0)
    {
      if(n[1] > 0) bv.dist(D + 1) = d;
      else bv.dist(1) = -d;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[1] == (FCL_REAL)0.0)
    {
      if(n[2] > 0) bv.dist(D + 2) = d;
      else bv.dist(2) = -d;
    }
    else if(n[2] == (FCL_REAL)0.0 && n[0] == n[1])
    {
      if(n[0] > 0) bv.dist(D + 3) = n[0] * d * 2;
      else bv.dist(3) = n[0] * d * 2;
    }
    else if(n[1] == (FCL_REAL)0.0 && n[0] == n[2])
    {
      if(n[1] > 0) bv.dist(D + 4) = n[0] * d * 2;
      else bv.dist(4) = n[0] * d * 2;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[1] == n[2])
    {
      if(n[1] > 0) bv.dist(D + 5) = n[1] * d * 2;
      else bv.dist(5) = n[1] * d * 2;
    }
    else if(n[2] == (FCL_REAL)0.0 && n[0] + n[1] == (FCL_REAL)0.0)
    {
      if(n[0] > 0) bv.dist(D + 6) = n[0] * d * 2;
      else bv.dist(6) = n[0] * d * 2;
    }
    else if(n[1] == (FCL_REAL)0.0 && n[0] + n[2] == (FCL_REAL)0.0)
    {
      if(n[0] > 0) bv.dist(D + 7) = n[0] * d * 2;
      else bv.dist(7) = n[0] * d * 2;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[1] + n[2] == (FCL_REAL)0.0)
    {
      if(n[1] > 0) bv.dist(D + 8) = n[1] * d * 2;
      else bv.dist(8) = n[1] * d * 2;
    }
  }
};

template <typename Scalar>
struct ComputeBVImpl<Scalar, KDOPd<24>, Halfspace<Scalar>>
{
  void operator()(const Halfspace<Scalar>& s, const Transform3<Scalar>& tf, KDOPd<24>& bv)
  {
    Halfspace<Scalar> new_s = transform(s, tf);
    const Vector3d& n = new_s.n;
    const FCL_REAL& d = new_s.d;

    const std::size_t D = 12;

    for(std::size_t i = 0; i < D; ++i)
      bv.dist(i) = -std::numeric_limits<FCL_REAL>::max();
    for(std::size_t i = D; i < 2 * D; ++i)
      bv.dist(i) = std::numeric_limits<FCL_REAL>::max();

    if(n[1] == (FCL_REAL)0.0 && n[2] == (FCL_REAL)0.0)
    {
      if(n[0] > 0) bv.dist(D) = d;
      else bv.dist(0) = -d;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[2] == (FCL_REAL)0.0)
    {
      if(n[1] > 0) bv.dist(D + 1) = d;
      else bv.dist(1) = -d;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[1] == (FCL_REAL)0.0)
    {
      if(n[2] > 0) bv.dist(D + 2) = d;
      else bv.dist(2) = -d;
    }
    else if(n[2] == (FCL_REAL)0.0 && n[0] == n[1])
    {
      if(n[0] > 0) bv.dist(D + 3) = n[0] * d * 2;
      else bv.dist(3) = n[0] * d * 2;
    }
    else if(n[1] == (FCL_REAL)0.0 && n[0] == n[2])
    {
      if(n[1] > 0) bv.dist(D + 4) = n[0] * d * 2;
      else bv.dist(4) = n[0] * d * 2;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[1] == n[2])
    {
      if(n[1] > 0) bv.dist(D + 5) = n[1] * d * 2;
      else bv.dist(5) = n[1] * d * 2;
    }
    else if(n[2] == (FCL_REAL)0.0 && n[0] + n[1] == (FCL_REAL)0.0)
    {
      if(n[0] > 0) bv.dist(D + 6) = n[0] * d * 2;
      else bv.dist(6) = n[0] * d * 2;
    }
    else if(n[1] == (FCL_REAL)0.0 && n[0] + n[2] == (FCL_REAL)0.0)
    {
      if(n[0] > 0) bv.dist(D + 7) = n[0] * d * 2;
      else bv.dist(7) = n[0] * d * 2;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[1] + n[2] == (FCL_REAL)0.0)
    {
      if(n[1] > 0) bv.dist(D + 8) = n[1] * d * 2;
      else bv.dist(8) = n[1] * d * 2;
    }
    else if(n[0] + n[2] == (FCL_REAL)0.0 && n[0] + n[1] == (FCL_REAL)0.0)
    {
      if(n[0] > 0) bv.dist(D + 9) = n[0] * d * 3;
      else bv.dist(9) = n[0] * d * 3;
    }
    else if(n[0] + n[1] == (FCL_REAL)0.0 && n[1] + n[2] == (FCL_REAL)0.0)
    {
      if(n[0] > 0) bv.dist(D + 10) = n[0] * d * 3;
      else bv.dist(10) = n[0] * d * 3;
    }
    else if(n[0] + n[1] == (FCL_REAL)0.0 && n[0] + n[2] == (FCL_REAL)0.0)
    {
      if(n[1] > 0) bv.dist(D + 11) = n[1] * d * 3;
      else bv.dist(11) = n[1] * d * 3;
    }
  }
};

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
Halfspace<Scalar>::Halfspace(const Vector3<Scalar>& n, Scalar d)
  : ShapeBase<Scalar>(), n(n), d(d)
{
  unitNormalTest();
}

//==============================================================================
template <typename Scalar>
Halfspace<Scalar>::Halfspace(Scalar a, Scalar b, Scalar c, Scalar d)
  : ShapeBase<Scalar>(), n(a, b, c), d(d)
{
  unitNormalTest();
}

//==============================================================================
template <typename Scalar>
Halfspace<Scalar>::Halfspace() : ShapeBase<Scalar>(), n(1, 0, 0), d(0)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
Scalar Halfspace<Scalar>::signedDistance(const Vector3<Scalar>& p) const
{
  return n.dot(p) - d;
}

//==============================================================================
template <typename Scalar>
Scalar Halfspace<Scalar>::distance(const Vector3<Scalar>& p) const
{
  return std::abs(n.dot(p) - d);
}

//==============================================================================
template <typename Scalar>
void Halfspace<Scalar>::computeLocalAABB()
{
  computeBV<Scalar, AABBd>(*this, Transform3d::Identity(), this->aabb_local);
  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename Scalar>
NODE_TYPE Halfspace<Scalar>::getNodeType() const
{
  return GEOM_HALFSPACE;
}

//==============================================================================
template <typename Scalar>
void Halfspace<Scalar>::unitNormalTest()
{
  Scalar l = n.norm();
  if(l > 0)
  {
    Scalar inv_l = 1.0 / l;
    n *= inv_l;
    d *= inv_l;
  }
  else
  {
    n << 1, 0, 0;
    d = 0;
  }
}

}

#endif
