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

#ifndef FCL_SHAPE_PLANE_H
#define FCL_SHAPE_PLANE_H

#include "fcl/shape/shape_base.h"
#include "fcl/shape/compute_bv.h"
#include "fcl/shape/geometric_shapes_utility.h"
#include "fcl/BV/OBB.h"
#include "fcl/BV/RSS.h"
#include "fcl/BV/OBBRSS.h"
#include "fcl/BV/kDOP.h"
#include "fcl/BV/kIOS.h"

namespace fcl
{

/// @brief Infinite plane 
template <typename Scalar>
class Plane : public ShapeBase<Scalar>
{
public:
  /// @brief Construct a plane with normal direction and offset 
  Plane(const Vector3<Scalar>& n, Scalar d);
  
  /// @brief Construct a plane with normal direction and offset 
  Plane(Scalar a, Scalar b, Scalar c, Scalar d);

  Plane();

  Scalar signedDistance(const Vector3<Scalar>& p) const;

  Scalar distance(const Vector3<Scalar>& p) const;

  /// @brief Compute AABBd 
  void computeLocalAABB() override;

  /// @brief Get node type: a plane 
  NODE_TYPE getNodeType() const override;

  /// @brief Plane normal 
  Vector3<Scalar> n;

  /// @brief Plane offset 
  Scalar d;

protected:
  
  /// @brief Turn non-unit normal into unit 
  void unitNormalTest();
};

using Planef = Plane<float>;
using Planed = Plane<double>;

template <typename Scalar>
Plane<Scalar> transform(const Plane<Scalar>& a, const Transform3<Scalar>& tf)
{
  /// suppose the initial halfspace is n * x <= d
  /// after transform (R, T), x --> x' = R x + T
  /// and the new half space becomes n' * x' <= d'
  /// where n' = R * n
  ///   and d' = d + n' * T

  Vector3<Scalar> n = tf.linear() * a.n;
  Scalar d = a.d + n.dot(tf.translation());

  return Plane<Scalar>(n, d);
}

template <typename Scalar>
struct ComputeBVImpl<Scalar, AABBd, Plane<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, OBB<Scalar>, Plane<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, RSSd, Plane<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, OBBRSSd, Plane<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, kIOSd, Plane<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, KDOPd<16>, Plane<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, KDOPd<18>, Plane<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, KDOPd<24>, Plane<Scalar>>;

template <typename Scalar>
struct ComputeBVImpl<Scalar, AABBd, Plane<Scalar>>
{
  void operator()(const Plane<Scalar>& s, const Transform3<Scalar>& tf, AABBd& bv)
  {
    Plane<Scalar> new_s = transform(s, tf);
    const Vector3d& n = new_s.n;
    const FCL_REAL& d = new_s.d;

    AABBd bv_;
    bv_.min_ = Vector3d::Constant(-std::numeric_limits<FCL_REAL>::max());
    bv_.max_ = Vector3d::Constant(std::numeric_limits<FCL_REAL>::max());
    if(n[1] == (FCL_REAL)0.0 && n[2] == (FCL_REAL)0.0)
    {
      // normal aligned with x axis
      if(n[0] < 0) { bv_.min_[0] = bv_.max_[0] = -d; }
      else if(n[0] > 0) { bv_.min_[0] = bv_.max_[0] = d; }
    }
    else if(n[0] == (FCL_REAL)0.0 && n[2] == (FCL_REAL)0.0)
    {
      // normal aligned with y axis
      if(n[1] < 0) { bv_.min_[1] = bv_.max_[1] = -d; }
      else if(n[1] > 0) { bv_.min_[1] = bv_.max_[1] = d; }
    }
    else if(n[0] == (FCL_REAL)0.0 && n[1] == (FCL_REAL)0.0)
    {
      // normal aligned with z axis
      if(n[2] < 0) { bv_.min_[2] = bv_.max_[2] = -d; }
      else if(n[2] > 0) { bv_.min_[2] = bv_.max_[2] = d; }
    }

    bv = bv_;
  }
};

template <typename Scalar>
struct ComputeBVImpl<Scalar, OBB<Scalar>, Plane<Scalar>>
{
  void operator()(const Plane<Scalar>& s, const Transform3<Scalar>& tf, OBB<Scalar>& bv)
  {
    Vector3d n = tf.linear() * s.n;
    bv.axis.col(0) = n;
    generateCoordinateSystem(bv.axis);

    bv.extent << 0, std::numeric_limits<FCL_REAL>::max(), std::numeric_limits<FCL_REAL>::max();

    Vector3d p = s.n * s.d;
    bv.To = tf * p; /// n'd' = R * n * (d + (R * n) * T) = R * (n * d) + T
  }
};

template <typename Scalar>
struct ComputeBVImpl<Scalar, RSSd, Plane<Scalar>>
{
  void operator()(const Plane<Scalar>& s, const Transform3<Scalar>& tf, RSSd& bv)
  {
    Vector3d n = tf.linear() * s.n;

    bv.axis.col(0) = n;
    generateCoordinateSystem(bv.axis);

    bv.l[0] = std::numeric_limits<FCL_REAL>::max();
    bv.l[1] = std::numeric_limits<FCL_REAL>::max();

    bv.r = 0;

    Vector3d p = s.n * s.d;
    bv.Tr = tf * p;
  }
};

template <typename Scalar>
struct ComputeBVImpl<Scalar, OBBRSSd, Plane<Scalar>>
{
  void operator()(const Plane<Scalar>& s, const Transform3<Scalar>& tf, OBBRSSd& bv)
  {
    computeBV<Scalar, OBB<Scalar>, Plane<Scalar>>(s, tf, bv.obb);
    computeBV<Scalar, RSSd, Plane<Scalar>>(s, tf, bv.rss);
  }
};

template <typename Scalar>
struct ComputeBVImpl<Scalar, kIOSd, Plane<Scalar>>
{
  void operator()(const Plane<Scalar>& s, const Transform3<Scalar>& tf, kIOSd& bv)
  {
    bv.num_spheres = 1;
    computeBV<Scalar, OBB<Scalar>, Plane<Scalar>>(s, tf, bv.obb);
    bv.spheres[0].o.setZero();
    bv.spheres[0].r = std::numeric_limits<FCL_REAL>::max();
  }
};

template <typename Scalar>
struct ComputeBVImpl<Scalar, KDOPd<16>, Plane<Scalar>>
{
  void operator()(const Plane<Scalar>& s, const Transform3<Scalar>& tf, KDOPd<16>& bv)
  {
    Plane<Scalar> new_s = transform(s, tf);
    const Vector3d& n = new_s.n;
    const FCL_REAL& d = new_s.d;

    const std::size_t D = 8;

    for(std::size_t i = 0; i < D; ++i)
      bv.dist(i) = -std::numeric_limits<FCL_REAL>::max();
    for(std::size_t i = D; i < 2 * D; ++i)
      bv.dist(i) = std::numeric_limits<FCL_REAL>::max();

    if(n[1] == (FCL_REAL)0.0 && n[2] == (FCL_REAL)0.0)
    {
      if(n[0] > 0) bv.dist(0) = bv.dist(D) = d;
      else bv.dist(0) = bv.dist(D) = -d;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[2] == (FCL_REAL)0.0)
    {
      if(n[1] > 0) bv.dist(1) = bv.dist(D + 1) = d;
      else bv.dist(1) = bv.dist(D + 1) = -d;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[1] == (FCL_REAL)0.0)
    {
      if(n[2] > 0) bv.dist(2) = bv.dist(D + 2) = d;
      else bv.dist(2) = bv.dist(D + 2) = -d;
    }
    else if(n[2] == (FCL_REAL)0.0 && n[0] == n[1])
    {
      bv.dist(3) = bv.dist(D + 3) = n[0] * d * 2;
    }
    else if(n[1] == (FCL_REAL)0.0 && n[0] == n[2])
    {
      bv.dist(4) = bv.dist(D + 4) = n[0] * d * 2;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[1] == n[2])
    {
      bv.dist(6) = bv.dist(D + 5) = n[1] * d * 2;
    }
    else if(n[2] == (FCL_REAL)0.0 && n[0] + n[1] == (FCL_REAL)0.0)
    {
      bv.dist(6) = bv.dist(D + 6) = n[0] * d * 2;
    }
    else if(n[1] == (FCL_REAL)0.0 && n[0] + n[2] == (FCL_REAL)0.0)
    {
      bv.dist(7) = bv.dist(D + 7) = n[0] * d * 2;
    }
  }
};

template <typename Scalar>
struct ComputeBVImpl<Scalar, KDOPd<18>, Plane<Scalar>>
{
  void operator()(const Plane<Scalar>& s, const Transform3<Scalar>& tf, KDOPd<18>& bv)
  {
    Plane<Scalar> new_s = transform(s, tf);
    const Vector3d& n = new_s.n;
    const FCL_REAL& d = new_s.d;

    const std::size_t D = 9;

    for(std::size_t i = 0; i < D; ++i)
      bv.dist(i) = -std::numeric_limits<FCL_REAL>::max();
    for(std::size_t i = D; i < 2 * D; ++i)
      bv.dist(i) = std::numeric_limits<FCL_REAL>::max();

    if(n[1] == (FCL_REAL)0.0 && n[2] == (FCL_REAL)0.0)
    {
      if(n[0] > 0) bv.dist(0) = bv.dist(D) = d;
      else bv.dist(0) = bv.dist(D) = -d;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[2] == (FCL_REAL)0.0)
    {
      if(n[1] > 0) bv.dist(1) = bv.dist(D + 1) = d;
      else bv.dist(1) = bv.dist(D + 1) = -d;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[1] == (FCL_REAL)0.0)
    {
      if(n[2] > 0) bv.dist(2) = bv.dist(D + 2) = d;
      else bv.dist(2) = bv.dist(D + 2) = -d;
    }
    else if(n[2] == (FCL_REAL)0.0 && n[0] == n[1])
    {
      bv.dist(3) = bv.dist(D + 3) = n[0] * d * 2;
    }
    else if(n[1] == (FCL_REAL)0.0 && n[0] == n[2])
    {
      bv.dist(4) = bv.dist(D + 4) = n[0] * d * 2;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[1] == n[2])
    {
      bv.dist(5) = bv.dist(D + 5) = n[1] * d * 2;
    }
    else if(n[2] == (FCL_REAL)0.0 && n[0] + n[1] == (FCL_REAL)0.0)
    {
      bv.dist(6) = bv.dist(D + 6) = n[0] * d * 2;
    }
    else if(n[1] == (FCL_REAL)0.0 && n[0] + n[2] == (FCL_REAL)0.0)
    {
      bv.dist(7) = bv.dist(D + 7) = n[0] * d * 2;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[1] + n[2] == (FCL_REAL)0.0)
    {
      bv.dist(8) = bv.dist(D + 8) = n[1] * d * 2;
    }
  }
};

template <typename Scalar>
struct ComputeBVImpl<Scalar, KDOPd<24>, Plane<Scalar>>
{
  void operator()(const Plane<Scalar>& s, const Transform3<Scalar>& tf, KDOPd<24>& bv)
  {
    Plane<Scalar> new_s = transform(s, tf);
    const Vector3d& n = new_s.n;
    const FCL_REAL& d = new_s.d;

    const std::size_t D = 12;

    for(std::size_t i = 0; i < D; ++i)
      bv.dist(i) = -std::numeric_limits<FCL_REAL>::max();
    for(std::size_t i = D; i < 2 * D; ++i)
      bv.dist(i) = std::numeric_limits<FCL_REAL>::max();

    if(n[1] == (FCL_REAL)0.0 && n[2] == (FCL_REAL)0.0)
    {
      if(n[0] > 0) bv.dist(0) = bv.dist(D) = d;
      else bv.dist(0) = bv.dist(D) = -d;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[2] == (FCL_REAL)0.0)
    {
      if(n[1] > 0) bv.dist(1) = bv.dist(D + 1) = d;
      else bv.dist(1) = bv.dist(D + 1) = -d;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[1] == (FCL_REAL)0.0)
    {
      if(n[2] > 0) bv.dist(2) = bv.dist(D + 2) = d;
      else bv.dist(2) = bv.dist(D + 2) = -d;
    }
    else if(n[2] == (FCL_REAL)0.0 && n[0] == n[1])
    {
      bv.dist(3) = bv.dist(D + 3) = n[0] * d * 2;
    }
    else if(n[1] == (FCL_REAL)0.0 && n[0] == n[2])
    {
      bv.dist(4) = bv.dist(D + 4) = n[0] * d * 2;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[1] == n[2])
    {
      bv.dist(5) = bv.dist(D + 5) = n[1] * d * 2;
    }
    else if(n[2] == (FCL_REAL)0.0 && n[0] + n[1] == (FCL_REAL)0.0)
    {
      bv.dist(6) = bv.dist(D + 6) = n[0] * d * 2;
    }
    else if(n[1] == (FCL_REAL)0.0 && n[0] + n[2] == (FCL_REAL)0.0)
    {
      bv.dist(7) = bv.dist(D + 7) = n[0] * d * 2;
    }
    else if(n[0] == (FCL_REAL)0.0 && n[1] + n[2] == (FCL_REAL)0.0)
    {
      bv.dist(8) = bv.dist(D + 8) = n[1] * d * 2;
    }
    else if(n[0] + n[2] == (FCL_REAL)0.0 && n[0] + n[1] == (FCL_REAL)0.0)
    {
      bv.dist(9) = bv.dist(D + 9) = n[0] * d * 3;
    }
    else if(n[0] + n[1] == (FCL_REAL)0.0 && n[1] + n[2] == (FCL_REAL)0.0)
    {
      bv.dist(10) = bv.dist(D + 10) = n[0] * d * 3;
    }
    else if(n[0] + n[1] == (FCL_REAL)0.0 && n[0] + n[2] == (FCL_REAL)0.0)
    {
      bv.dist(11) = bv.dist(D + 11) = n[1] * d * 3;
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
Plane<Scalar>::Plane(const Vector3<Scalar>& n, Scalar d)
  : ShapeBase<Scalar>(), n(n), d(d)
{
  unitNormalTest();
}

//==============================================================================
template <typename Scalar>
Plane<Scalar>::Plane(Scalar a, Scalar b, Scalar c, Scalar d)
  : ShapeBase<Scalar>(), n(a, b, c), d(d)
{
  unitNormalTest();
}

//==============================================================================
template <typename Scalar>
Plane<Scalar>::Plane() : ShapeBased(), n(1, 0, 0), d(0)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
Scalar Plane<Scalar>::signedDistance(const Vector3<Scalar>& p) const
{
  return n.dot(p) - d;
}

//==============================================================================
template <typename Scalar>
Scalar Plane<Scalar>::distance(const Vector3<Scalar>& p) const
{
  return std::abs(n.dot(p) - d);
}

//==============================================================================
template <typename Scalar>
void Plane<Scalar>::computeLocalAABB()
{
  computeBV<Scalar, AABBd>(*this, Transform3<Scalar>::Identity(), this->aabb_local);
  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename Scalar>
NODE_TYPE Plane<Scalar>::getNodeType() const
{
  return GEOM_PLANE;
}

//==============================================================================
template <typename Scalar>
void Plane<Scalar>::unitNormalTest()
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
