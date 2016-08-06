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
template <typename ScalarT>
class Plane : public ShapeBase<ScalarT>
{
public:

  using Scalar = ScalarT;

  /// @brief Construct a plane with normal direction and offset 
  Plane(const Vector3<ScalarT>& n, ScalarT d);
  
  /// @brief Construct a plane with normal direction and offset 
  Plane(ScalarT a, ScalarT b, ScalarT c, ScalarT d);

  Plane();

  ScalarT signedDistance(const Vector3<ScalarT>& p) const;

  ScalarT distance(const Vector3<ScalarT>& p) const;

  /// @brief Compute AABB
  void computeLocalAABB() override;

  /// @brief Get node type: a plane 
  NODE_TYPE getNodeType() const override;

  /// @brief Plane normal 
  Vector3<ScalarT> n;

  /// @brief Plane offset 
  ScalarT d;

protected:
  
  /// @brief Turn non-unit normal into unit 
  void unitNormalTest();
};

using Planef = Plane<float>;
using Planed = Plane<double>;

template <typename ScalarT>
Plane<ScalarT> transform(const Plane<ScalarT>& a, const Transform3<ScalarT>& tf)
{
  /// suppose the initial halfspace is n * x <= d
  /// after transform (R, T), x --> x' = R x + T
  /// and the new half space becomes n' * x' <= d'
  /// where n' = R * n
  ///   and d' = d + n' * T

  Vector3<ScalarT> n = tf.linear() * a.n;
  ScalarT d = a.d + n.dot(tf.translation());

  return Plane<ScalarT>(n, d);
}

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, AABB<ScalarT>, Plane<ScalarT>>;

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, OBB<ScalarT>, Plane<ScalarT>>;

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, RSS<ScalarT>, Plane<ScalarT>>;

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, OBBRSS<ScalarT>, Plane<ScalarT>>;

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, kIOS<ScalarT>, Plane<ScalarT>>;

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, KDOP<ScalarT, 16>, Plane<ScalarT>>;

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, KDOP<ScalarT, 18>, Plane<ScalarT>>;

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, KDOP<ScalarT, 24>, Plane<ScalarT>>;

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, AABB<ScalarT>, Plane<ScalarT>>
{
  void operator()(const Plane<ScalarT>& s, const Transform3<ScalarT>& tf, AABB<ScalarT>& bv)
  {
    Plane<ScalarT> new_s = transform(s, tf);
    const Vector3<ScalarT>& n = new_s.n;
    const ScalarT& d = new_s.d;

    AABB<ScalarT> bv_;
    bv_.min_ = Vector3<ScalarT>::Constant(-std::numeric_limits<ScalarT>::max());
    bv_.max_ = Vector3<ScalarT>::Constant(std::numeric_limits<ScalarT>::max());
    if(n[1] == (ScalarT)0.0 && n[2] == (ScalarT)0.0)
    {
      // normal aligned with x axis
      if(n[0] < 0) { bv_.min_[0] = bv_.max_[0] = -d; }
      else if(n[0] > 0) { bv_.min_[0] = bv_.max_[0] = d; }
    }
    else if(n[0] == (ScalarT)0.0 && n[2] == (ScalarT)0.0)
    {
      // normal aligned with y axis
      if(n[1] < 0) { bv_.min_[1] = bv_.max_[1] = -d; }
      else if(n[1] > 0) { bv_.min_[1] = bv_.max_[1] = d; }
    }
    else if(n[0] == (ScalarT)0.0 && n[1] == (ScalarT)0.0)
    {
      // normal aligned with z axis
      if(n[2] < 0) { bv_.min_[2] = bv_.max_[2] = -d; }
      else if(n[2] > 0) { bv_.min_[2] = bv_.max_[2] = d; }
    }

    bv = bv_;
  }
};

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, OBB<ScalarT>, Plane<ScalarT>>
{
  void operator()(const Plane<ScalarT>& s, const Transform3<ScalarT>& tf, OBB<ScalarT>& bv)
  {
    Vector3<ScalarT> n = tf.linear() * s.n;
    bv.axis.col(0) = n;
    generateCoordinateSystem(bv.axis);

    bv.extent << 0, std::numeric_limits<ScalarT>::max(), std::numeric_limits<ScalarT>::max();

    Vector3<ScalarT> p = s.n * s.d;
    bv.To = tf * p; /// n'd' = R * n * (d + (R * n) * T) = R * (n * d) + T
  }
};

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, RSS<ScalarT>, Plane<ScalarT>>
{
  void operator()(const Plane<ScalarT>& s, const Transform3<ScalarT>& tf, RSS<ScalarT>& bv)
  {
    Vector3<ScalarT> n = tf.linear() * s.n;

    bv.axis.col(0) = n;
    generateCoordinateSystem(bv.axis);

    bv.l[0] = std::numeric_limits<ScalarT>::max();
    bv.l[1] = std::numeric_limits<ScalarT>::max();

    bv.r = 0;

    Vector3<ScalarT> p = s.n * s.d;
    bv.Tr = tf * p;
  }
};

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, OBBRSS<ScalarT>, Plane<ScalarT>>
{
  void operator()(const Plane<ScalarT>& s, const Transform3<ScalarT>& tf, OBBRSS<ScalarT>& bv)
  {
    computeBV<ScalarT, OBB<ScalarT>, Plane<ScalarT>>(s, tf, bv.obb);
    computeBV<ScalarT, RSS<ScalarT>, Plane<ScalarT>>(s, tf, bv.rss);
  }
};

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, kIOS<ScalarT>, Plane<ScalarT>>
{
  void operator()(const Plane<ScalarT>& s, const Transform3<ScalarT>& tf, kIOS<ScalarT>& bv)
  {
    bv.num_spheres = 1;
    computeBV<ScalarT, OBB<ScalarT>, Plane<ScalarT>>(s, tf, bv.obb);
    bv.spheres[0].o.setZero();
    bv.spheres[0].r = std::numeric_limits<ScalarT>::max();
  }
};

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, KDOP<ScalarT, 16>, Plane<ScalarT>>
{
  void operator()(const Plane<ScalarT>& s, const Transform3<ScalarT>& tf, KDOP<ScalarT, 16>& bv)
  {
    Plane<ScalarT> new_s = transform(s, tf);
    const Vector3<ScalarT>& n = new_s.n;
    const ScalarT& d = new_s.d;

    const std::size_t D = 8;

    for(std::size_t i = 0; i < D; ++i)
      bv.dist(i) = -std::numeric_limits<ScalarT>::max();
    for(std::size_t i = D; i < 2 * D; ++i)
      bv.dist(i) = std::numeric_limits<ScalarT>::max();

    if(n[1] == (ScalarT)0.0 && n[2] == (ScalarT)0.0)
    {
      if(n[0] > 0) bv.dist(0) = bv.dist(D) = d;
      else bv.dist(0) = bv.dist(D) = -d;
    }
    else if(n[0] == (ScalarT)0.0 && n[2] == (ScalarT)0.0)
    {
      if(n[1] > 0) bv.dist(1) = bv.dist(D + 1) = d;
      else bv.dist(1) = bv.dist(D + 1) = -d;
    }
    else if(n[0] == (ScalarT)0.0 && n[1] == (ScalarT)0.0)
    {
      if(n[2] > 0) bv.dist(2) = bv.dist(D + 2) = d;
      else bv.dist(2) = bv.dist(D + 2) = -d;
    }
    else if(n[2] == (ScalarT)0.0 && n[0] == n[1])
    {
      bv.dist(3) = bv.dist(D + 3) = n[0] * d * 2;
    }
    else if(n[1] == (ScalarT)0.0 && n[0] == n[2])
    {
      bv.dist(4) = bv.dist(D + 4) = n[0] * d * 2;
    }
    else if(n[0] == (ScalarT)0.0 && n[1] == n[2])
    {
      bv.dist(6) = bv.dist(D + 5) = n[1] * d * 2;
    }
    else if(n[2] == (ScalarT)0.0 && n[0] + n[1] == (ScalarT)0.0)
    {
      bv.dist(6) = bv.dist(D + 6) = n[0] * d * 2;
    }
    else if(n[1] == (ScalarT)0.0 && n[0] + n[2] == (ScalarT)0.0)
    {
      bv.dist(7) = bv.dist(D + 7) = n[0] * d * 2;
    }
  }
};

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, KDOP<ScalarT, 18>, Plane<ScalarT>>
{
  void operator()(const Plane<ScalarT>& s, const Transform3<ScalarT>& tf, KDOP<ScalarT, 18>& bv)
  {
    Plane<ScalarT> new_s = transform(s, tf);
    const Vector3<ScalarT>& n = new_s.n;
    const ScalarT& d = new_s.d;

    const std::size_t D = 9;

    for(std::size_t i = 0; i < D; ++i)
      bv.dist(i) = -std::numeric_limits<ScalarT>::max();
    for(std::size_t i = D; i < 2 * D; ++i)
      bv.dist(i) = std::numeric_limits<ScalarT>::max();

    if(n[1] == (ScalarT)0.0 && n[2] == (ScalarT)0.0)
    {
      if(n[0] > 0) bv.dist(0) = bv.dist(D) = d;
      else bv.dist(0) = bv.dist(D) = -d;
    }
    else if(n[0] == (ScalarT)0.0 && n[2] == (ScalarT)0.0)
    {
      if(n[1] > 0) bv.dist(1) = bv.dist(D + 1) = d;
      else bv.dist(1) = bv.dist(D + 1) = -d;
    }
    else if(n[0] == (ScalarT)0.0 && n[1] == (ScalarT)0.0)
    {
      if(n[2] > 0) bv.dist(2) = bv.dist(D + 2) = d;
      else bv.dist(2) = bv.dist(D + 2) = -d;
    }
    else if(n[2] == (ScalarT)0.0 && n[0] == n[1])
    {
      bv.dist(3) = bv.dist(D + 3) = n[0] * d * 2;
    }
    else if(n[1] == (ScalarT)0.0 && n[0] == n[2])
    {
      bv.dist(4) = bv.dist(D + 4) = n[0] * d * 2;
    }
    else if(n[0] == (ScalarT)0.0 && n[1] == n[2])
    {
      bv.dist(5) = bv.dist(D + 5) = n[1] * d * 2;
    }
    else if(n[2] == (ScalarT)0.0 && n[0] + n[1] == (ScalarT)0.0)
    {
      bv.dist(6) = bv.dist(D + 6) = n[0] * d * 2;
    }
    else if(n[1] == (ScalarT)0.0 && n[0] + n[2] == (ScalarT)0.0)
    {
      bv.dist(7) = bv.dist(D + 7) = n[0] * d * 2;
    }
    else if(n[0] == (ScalarT)0.0 && n[1] + n[2] == (ScalarT)0.0)
    {
      bv.dist(8) = bv.dist(D + 8) = n[1] * d * 2;
    }
  }
};

template <typename ScalarT>
struct ComputeBVImpl<ScalarT, KDOP<ScalarT, 24>, Plane<ScalarT>>
{
  void operator()(const Plane<ScalarT>& s, const Transform3<ScalarT>& tf, KDOP<ScalarT, 24>& bv)
  {
    Plane<ScalarT> new_s = transform(s, tf);
    const Vector3<ScalarT>& n = new_s.n;
    const ScalarT& d = new_s.d;

    const std::size_t D = 12;

    for(std::size_t i = 0; i < D; ++i)
      bv.dist(i) = -std::numeric_limits<ScalarT>::max();
    for(std::size_t i = D; i < 2 * D; ++i)
      bv.dist(i) = std::numeric_limits<ScalarT>::max();

    if(n[1] == (ScalarT)0.0 && n[2] == (ScalarT)0.0)
    {
      if(n[0] > 0) bv.dist(0) = bv.dist(D) = d;
      else bv.dist(0) = bv.dist(D) = -d;
    }
    else if(n[0] == (ScalarT)0.0 && n[2] == (ScalarT)0.0)
    {
      if(n[1] > 0) bv.dist(1) = bv.dist(D + 1) = d;
      else bv.dist(1) = bv.dist(D + 1) = -d;
    }
    else if(n[0] == (ScalarT)0.0 && n[1] == (ScalarT)0.0)
    {
      if(n[2] > 0) bv.dist(2) = bv.dist(D + 2) = d;
      else bv.dist(2) = bv.dist(D + 2) = -d;
    }
    else if(n[2] == (ScalarT)0.0 && n[0] == n[1])
    {
      bv.dist(3) = bv.dist(D + 3) = n[0] * d * 2;
    }
    else if(n[1] == (ScalarT)0.0 && n[0] == n[2])
    {
      bv.dist(4) = bv.dist(D + 4) = n[0] * d * 2;
    }
    else if(n[0] == (ScalarT)0.0 && n[1] == n[2])
    {
      bv.dist(5) = bv.dist(D + 5) = n[1] * d * 2;
    }
    else if(n[2] == (ScalarT)0.0 && n[0] + n[1] == (ScalarT)0.0)
    {
      bv.dist(6) = bv.dist(D + 6) = n[0] * d * 2;
    }
    else if(n[1] == (ScalarT)0.0 && n[0] + n[2] == (ScalarT)0.0)
    {
      bv.dist(7) = bv.dist(D + 7) = n[0] * d * 2;
    }
    else if(n[0] == (ScalarT)0.0 && n[1] + n[2] == (ScalarT)0.0)
    {
      bv.dist(8) = bv.dist(D + 8) = n[1] * d * 2;
    }
    else if(n[0] + n[2] == (ScalarT)0.0 && n[0] + n[1] == (ScalarT)0.0)
    {
      bv.dist(9) = bv.dist(D + 9) = n[0] * d * 3;
    }
    else if(n[0] + n[1] == (ScalarT)0.0 && n[1] + n[2] == (ScalarT)0.0)
    {
      bv.dist(10) = bv.dist(D + 10) = n[0] * d * 3;
    }
    else if(n[0] + n[1] == (ScalarT)0.0 && n[0] + n[2] == (ScalarT)0.0)
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
template <typename ScalarT>
Plane<ScalarT>::Plane(const Vector3<ScalarT>& n, ScalarT d)
  : ShapeBase<ScalarT>(), n(n), d(d)
{
  unitNormalTest();
}

//==============================================================================
template <typename ScalarT>
Plane<ScalarT>::Plane(ScalarT a, ScalarT b, ScalarT c, ScalarT d)
  : ShapeBase<ScalarT>(), n(a, b, c), d(d)
{
  unitNormalTest();
}

//==============================================================================
template <typename ScalarT>
Plane<ScalarT>::Plane() : ShapeBased(), n(1, 0, 0), d(0)
{
  // Do nothing
}

//==============================================================================
template <typename ScalarT>
ScalarT Plane<ScalarT>::signedDistance(const Vector3<ScalarT>& p) const
{
  return n.dot(p) - d;
}

//==============================================================================
template <typename ScalarT>
ScalarT Plane<ScalarT>::distance(const Vector3<ScalarT>& p) const
{
  return std::abs(n.dot(p) - d);
}

//==============================================================================
template <typename ScalarT>
void Plane<ScalarT>::computeLocalAABB()
{
  computeBV<ScalarT, AABB<ScalarT>>(*this, Transform3<ScalarT>::Identity(), this->aabb_local);
  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

//==============================================================================
template <typename ScalarT>
NODE_TYPE Plane<ScalarT>::getNodeType() const
{
  return GEOM_PLANE;
}

//==============================================================================
template <typename ScalarT>
void Plane<ScalarT>::unitNormalTest()
{
  ScalarT l = n.norm();
  if(l > 0)
  {
    ScalarT inv_l = 1.0 / l;
    n *= inv_l;
    d *= inv_l;
  }
  else
  {
    n << 1, 0, 0;
    d = 0;
  }
}

} // namespace fcl

#endif
