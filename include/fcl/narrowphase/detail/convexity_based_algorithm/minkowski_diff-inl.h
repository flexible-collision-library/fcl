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

#ifndef FCL_NARROWPHASE_DETAIL_MINKOWSKIDIFF_INL_H
#define FCL_NARROWPHASE_DETAIL_MINKOWSKIDIFF_INL_H

#include "fcl/narrowphase/detail/convexity_based_algorithm/minkowski_diff.h"

#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/shape/capsule.h"
#include "fcl/geometry/shape/cone.h"
#include "fcl/geometry/shape/convex.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/shape/ellipsoid.h"
#include "fcl/geometry/shape/halfspace.h"
#include "fcl/geometry/shape/plane.h"
#include "fcl/geometry/shape/sphere.h"
#include "fcl/geometry/shape/triangle_p.h"

namespace fcl
{

namespace detail
{

//==============================================================================
extern template
struct MinkowskiDiff<double>;

//==============================================================================
template <typename S, typename Derived>
FCL_EXPORT
Vector3<S> getSupport(
    const ShapeBase<S>* shape,
    const Eigen::MatrixBase<Derived>& dir)
{
  // Check the number of rows is 6 at compile time
  EIGEN_STATIC_ASSERT(
        Derived::RowsAtCompileTime == 3
        && Derived::ColsAtCompileTime == 1,
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  switch(shape->getNodeType())
  {
  case GEOM_TRIANGLE:
    {
      const auto* triangle = static_cast<const TriangleP<S>*>(shape);
      S dota = dir.dot(triangle->a);
      S dotb = dir.dot(triangle->b);
      S dotc = dir.dot(triangle->c);
      if(dota > dotb)
      {
        if(dotc > dota)
          return triangle->c;
        else
          return triangle->a;
      }
      else
      {
        if(dotc > dotb)
          return triangle->c;
        else
          return triangle->b;
      }
    }
    break;
  case GEOM_BOX:
    {
      const Box<S>* box = static_cast<const Box<S>*>(shape);
      return Vector3<S>((dir[0]>0)?(box->side[0]/2):(-box->side[0]/2),
                   (dir[1]>0)?(box->side[1]/2):(-box->side[1]/2),
                   (dir[2]>0)?(box->side[2]/2):(-box->side[2]/2));
    }
    break;
  case GEOM_SPHERE:
    {
      const Sphere<S>* sphere = static_cast<const Sphere<S>*>(shape);
      return dir * sphere->radius;
    }
    break;
  case GEOM_ELLIPSOID:
    {
      const Ellipsoid<S>* ellipsoid = static_cast<const Ellipsoid<S>*>(shape);

      const S a2 = ellipsoid->radii[0] * ellipsoid->radii[0];
      const S b2 = ellipsoid->radii[1] * ellipsoid->radii[1];
      const S c2 = ellipsoid->radii[2] * ellipsoid->radii[2];

      const Vector3<S> v(a2 * dir[0], b2 * dir[1], c2 * dir[2]);
      const S d = std::sqrt(v.dot(dir));

      return v / d;
    }
    break;
  case GEOM_CAPSULE:
    {
      const Capsule<S>* capsule = static_cast<const Capsule<S>*>(shape);
      S half_h = capsule->lz * 0.5;
      Vector3<S> pos1(0, 0, half_h);
      Vector3<S> pos2(0, 0, -half_h);
      Vector3<S> v = dir * capsule->radius;
      pos1 += v;
      pos2 += v;
      if(dir.dot(pos1) > dir.dot(pos2))
        return pos1;
      else return pos2;
    }
    break;
  case GEOM_CONE:
    {
      const Cone<S>* cone = static_cast<const Cone<S>*>(shape);
      S zdist = dir[0] * dir[0] + dir[1] * dir[1];
      S len = zdist + dir[2] * dir[2];
      zdist = std::sqrt(zdist);
      len = std::sqrt(len);
      S half_h = cone->lz * 0.5;
      S radius = cone->radius;

      S sin_a = radius / std::sqrt(radius * radius + 4 * half_h * half_h);

      if(dir[2] > len * sin_a)
        return Vector3<S>(0, 0, half_h);
      else if(zdist > 0)
      {
        S rad = radius / zdist;
        return Vector3<S>(rad * dir[0], rad * dir[1], -half_h);
      }
      else
        return Vector3<S>(0, 0, -half_h);
    }
    break;
  case GEOM_CYLINDER:
    {
      const Cylinder<S>* cylinder = static_cast<const Cylinder<S>*>(shape);
      S zdist = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1]);
      S half_h = cylinder->lz * 0.5;
      if(zdist == 0.0)
      {
        return Vector3<S>(0, 0, (dir[2]>0)? half_h:-half_h);
      }
      else
      {
        S d = cylinder->radius / zdist;
        return Vector3<S>(d * dir[0], d * dir[1], (dir[2]>0)?half_h:-half_h);
      }
    }
    break;
  case GEOM_CONVEX:
    {
      const Convex<S>* convex = static_cast<const Convex<S>*>(shape);
      S maxdot = - std::numeric_limits<S>::max();
      Vector3<S> bestv = Vector3<S>::Zero();
      for(const auto& vertex : convex->getVertices())
      {
        S dot = dir.dot(vertex);
        if(dot > maxdot)
        {
          bestv = vertex;
          maxdot = dot;
        }
      }
      return bestv;
    }
    break;
  case GEOM_PLANE:
  break;
  default:
    ; // nothing
  }

  return Vector3<S>::Zero();
}

//==============================================================================
template <typename S>
MinkowskiDiff<S>::MinkowskiDiff()
{
  // Do nothing
}

//==============================================================================
template <typename S>
Vector3<S> MinkowskiDiff<S>::support0(const Vector3<S>& d) const
{
  return getSupport(shapes[0], d);
}

//==============================================================================
template <typename S>
Vector3<S> MinkowskiDiff<S>::support1(const Vector3<S>& d) const
{
  return toshape0 * getSupport(shapes[1], toshape1 * d);
}

//==============================================================================
template <typename S>
Vector3<S> MinkowskiDiff<S>::support(const Vector3<S>& d) const
{
  return support0(d) - support1(-d);
}

//==============================================================================
template <typename S>
Vector3<S> MinkowskiDiff<S>::support(const Vector3<S>& d, size_t index) const
{
  if(index)
    return support1(d);
  else
    return support0(d);
}

//==============================================================================
template <typename S>
Vector3<S> MinkowskiDiff<S>::support0(const Vector3<S>& d, const Vector3<S>& v) const
{
  if(d.dot(v) <= 0)
    return getSupport(shapes[0], d);
  else
    return getSupport(shapes[0], d) + v;
}

//==============================================================================
template <typename S>
Vector3<S> MinkowskiDiff<S>::support(const Vector3<S>& d, const Vector3<S>& v) const
{
  return support0(d, v) - support1(-d);
}

//==============================================================================
template <typename S>
Vector3<S> MinkowskiDiff<S>::support(const Vector3<S>& d, const Vector3<S>& v, size_t index) const
{
  if(index)
    return support1(d);
  else
    return support0(d, v);
}

} // namespace detail
} // namespace fcl

#endif
