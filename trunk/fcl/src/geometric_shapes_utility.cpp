/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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


#include "fcl/geometric_shapes_utility.h"
#include "fcl/BV_fitter.h"

namespace fcl
{

template<>
void computeBV<AABB>(const Box& s, const SimpleTransform& tf, AABB& bv)
{
  Matrix3f R = tf.getRotation() * s.getLocalRotation();
  Vec3f T = tf.getRotation() * s.getLocalTranslation() + tf.getTranslation();

  BVH_REAL x_range = 0.5 * (fabs(R[0][0] * s.side[0]) + fabs(R[0][1] * s.side[1]) + fabs(R[0][2] * s.side[2]));
  BVH_REAL y_range = 0.5 * (fabs(R[1][0] * s.side[0]) + fabs(R[1][1] * s.side[1]) + fabs(R[1][2] * s.side[2]));
  BVH_REAL z_range = 0.5 * (fabs(R[2][0] * s.side[0]) + fabs(R[2][1] * s.side[1]) + fabs(R[2][2] * s.side[2]));

  bv.max_ = T + Vec3f(x_range, y_range, z_range);
  bv.min_ = T + Vec3f(-x_range, -y_range, -z_range);
}

template<>
void computeBV<AABB>(const Sphere& s, const SimpleTransform& tf, AABB& bv)
{
  Vec3f T = tf.getRotation() * s.getLocalTranslation() + tf.getTranslation();

  bv.max_ = T + Vec3f(s.radius, s.radius, s.radius);
  bv.min_ = T + Vec3f(-s.radius, -s.radius, -s.radius);
}

template<>
void computeBV<AABB>(const Capsule& s, const SimpleTransform& tf, AABB& bv)
{
  Matrix3f R = tf.getRotation() * s.getLocalRotation();
  Vec3f T = tf.getRotation() * s.getLocalTranslation() + tf.getTranslation();

  BVH_REAL x_range = 0.5 * fabs(R[0][2] * s.lz) + s.radius;
  BVH_REAL y_range = 0.5 * fabs(R[1][2] * s.lz) + s.radius;
  BVH_REAL z_range = 0.5 * fabs(R[2][2] * s.lz) + s.radius;

  bv.max_ = T + Vec3f(x_range, y_range, z_range);
  bv.min_ = T + Vec3f(-x_range, -y_range, -z_range);
}

template<>
void computeBV<AABB>(const Cone& s, const SimpleTransform& tf, AABB& bv)
{
  Matrix3f R = tf.getRotation() * s.getLocalRotation();
  Vec3f T = tf.getRotation() * s.getLocalTranslation() + tf.getTranslation();

  BVH_REAL x_range = fabs(R[0][0] * s.radius) + fabs(R[0][1] * s.radius) + 0.5 * fabs(R[0][2] * s.lz);
  BVH_REAL y_range = fabs(R[1][0] * s.radius) + fabs(R[1][1] * s.radius) + 0.5 * fabs(R[1][2] * s.lz);
  BVH_REAL z_range = fabs(R[2][0] * s.radius) + fabs(R[2][1] * s.radius) + 0.5 * fabs(R[2][2] * s.lz);

  bv.max_ = T + Vec3f(x_range, y_range, z_range);
  bv.min_ = T + Vec3f(-x_range, -y_range, -z_range);
}

template<>
void computeBV<AABB>(const Cylinder& s, const SimpleTransform& tf, AABB& bv)
{
  Matrix3f R = tf.getRotation() * s.getLocalRotation();
  Vec3f T = tf.getRotation() * s.getLocalTranslation() + tf.getTranslation();

  BVH_REAL x_range = fabs(R[0][0] * s.radius) + fabs(R[0][1] * s.radius) + 0.5 * fabs(R[0][2] * s.lz);
  BVH_REAL y_range = fabs(R[1][0] * s.radius) + fabs(R[1][1] * s.radius) + 0.5 * fabs(R[1][2] * s.lz);
  BVH_REAL z_range = fabs(R[2][0] * s.radius) + fabs(R[2][1] * s.radius) + 0.5 * fabs(R[2][2] * s.lz);

  bv.max_ = T + Vec3f(x_range, y_range, z_range);
  bv.min_ = T + Vec3f(-x_range, -y_range, -z_range);
}

template<>
void computeBV<AABB>(const Convex& s, const SimpleTransform& tf, AABB& bv)
{
  Matrix3f R = tf.getRotation() * s.getLocalRotation();
  Vec3f T = tf.getRotation() * s.getLocalTranslation() + tf.getTranslation();

  AABB bv_;
  for(int i = 0; i < s.num_points; ++i)
  {
    Vec3f new_p = R * s.points[i] + T;
    bv_ += new_p;
  }

  bv = bv_;
}

template<>
void computeBV<AABB>(const Plane& s, const SimpleTransform& tf, AABB& bv)
{
  Matrix3f R = tf.getRotation() * s.getLocalRotation();

  Vec3f n = R * n;

  AABB bv_;
  if(n[1] == (BVH_REAL)0.0 && n[2] == (BVH_REAL)0.0)
  {
    // normal aligned with x axis
    if(n[0] < 0) bv_.min_[0] = -s.d;
    else if(n[0] > 0) bv_.max_[0] = s.d;
  }
  else if(n[0] == (BVH_REAL)0.0 && n[2] == (BVH_REAL)0.0)
  {
    // normal aligned with y axis
    if(n[1] < 0) bv_.min_[1] = -s.d;
    else if(n[1] > 0) bv_.max_[1] = s.d;
  }
  else if(n[0] == (BVH_REAL)0.0 && n[1] == (BVH_REAL)0.0)
  {
    // normal aligned with z axis
    if(n[2] < 0) bv_.min_[2] = -s.d;
    else if(n[2] > 0) bv_.max_[2] = s.d;
  }

  bv = bv_;
}


template<>
void computeBV<OBB>(const Box& s, const SimpleTransform& tf, OBB& bv)
{
  Matrix3f R = tf.getRotation() * s.getLocalRotation();
  Vec3f T = tf.getRotation() * s.getLocalTranslation() + tf.getTranslation();

  bv.To = T;
  bv.axis[0] = R.getColumn(0);
  bv.axis[1] = R.getColumn(1);
  bv.axis[2] = R.getColumn(2);
  bv.extent = s.side * (BVH_REAL)0.5;
}

template<>
void computeBV<OBB>(const Sphere& s, const SimpleTransform& tf, OBB& bv)
{
  Vec3f T = tf.getRotation() * s.getLocalTranslation() + tf.getTranslation();

  bv.To = T;
  bv.axis[0].setValue(1, 0, 0);
  bv.axis[1].setValue(0, 1, 0);
  bv.axis[2].setValue(0, 0, 1);
  bv.extent.setValue(s.radius);
}

template<>
void computeBV<OBB>(const Capsule& s, const SimpleTransform& tf, OBB& bv)
{
  Matrix3f R = tf.getRotation() * s.getLocalRotation();
  Vec3f T = tf.getRotation() * s.getLocalTranslation() + tf.getTranslation();

  bv.To = T;
  bv.axis[0] = R.getColumn(0);
  bv.axis[1] = R.getColumn(1);
  bv.axis[2] = R.getColumn(2);
  bv.extent.setValue(s.radius, s.radius, s.lz / 2 + s.radius);
}

template<>
void computeBV<OBB>(const Cone& s, const SimpleTransform& tf, OBB& bv)
{
  Matrix3f R = tf.getRotation() * s.getLocalRotation();
  Vec3f T = tf.getRotation() * s.getLocalTranslation() + tf.getTranslation();

  bv.To = T;
  bv.axis[0] = R.getColumn(0);
  bv.axis[1] = R.getColumn(1);
  bv.axis[2] = R.getColumn(2);
  bv.extent.setValue(s.radius, s.radius, s.lz / 2);
}

template<>
void computeBV<OBB>(const Cylinder& s, const SimpleTransform& tf, OBB& bv)
{
  Matrix3f R = tf.getRotation() * s.getLocalRotation();
  Vec3f T = tf.getRotation() * s.getLocalTranslation() + tf.getTranslation();

  bv.To = T;
  bv.axis[0] = R.getColumn(0);
  bv.axis[1] = R.getColumn(1);
  bv.axis[2] = R.getColumn(2);
  bv.extent.setValue(s.radius, s.radius, s.lz / 2);
}

template<>
void computeBV<OBB>(const Convex& s, const SimpleTransform& tf, OBB& bv)
{
  Matrix3f R = tf.getRotation() * s.getLocalRotation();
  Vec3f T = tf.getRotation() * s.getLocalTranslation() + tf.getTranslation();

  fit(s.points, s.num_points, bv);

  bv.axis[0] = R * bv.axis[0];
  bv.axis[1] = R * bv.axis[1];
  bv.axis[2] = R * bv.axis[2];

  bv.To = R * bv.To + T;

}

template<>
void computeBV<OBB>(const Plane& s, const SimpleTransform& tf, OBB& bv)
{
  Matrix3f R = tf.getRotation() * s.getLocalRotation();
  Vec3f T = tf.getRotation() * s.getLocalTranslation() + tf.getTranslation();

  // generate other two axes orthonormal to plane normal
  generateCoordinateSystem(s.n, bv.axis[1], bv.axis[2]);
  bv.axis[0] = s.n;

  bv.extent.setValue(0, std::numeric_limits<BVH_REAL>::max(), std::numeric_limits<BVH_REAL>::max());

  Vec3f p = s.n * s.d;
  bv.To = R * p + T;
}

void Box::computeLocalAABB()
{
  AABB aabb;
  computeBV<AABB>(*this, SimpleTransform(), aabb);
  aabb_center = aabb.center();
  aabb_radius = (aabb.min_ - aabb_center).length();
}

void Sphere::computeLocalAABB()
{
  AABB aabb;
  computeBV<AABB>(*this, SimpleTransform(), aabb);
  aabb_center = aabb.center();
  aabb_radius = radius;
}

void Capsule::computeLocalAABB()
{
  AABB aabb;
  computeBV<AABB>(*this, SimpleTransform(), aabb);
  aabb_center = aabb.center();
  aabb_radius = (aabb.min_ - aabb_center).length();
}

void Cone::computeLocalAABB()
{
  AABB aabb;
  computeBV<AABB>(*this, SimpleTransform(), aabb);
  aabb_center = aabb.center();
  aabb_radius = (aabb.min_ - aabb_center).length();
}

void Cylinder::computeLocalAABB()
{
  AABB aabb;
  computeBV<AABB>(*this, SimpleTransform(), aabb);
  aabb_center = aabb.center();
  aabb_radius = (aabb.min_ - aabb_center).length();
}

void Convex::computeLocalAABB()
{
  AABB aabb;
  computeBV<AABB>(*this, SimpleTransform(), aabb);
  aabb_center = aabb.center();
  aabb_radius = (aabb.min_ - aabb_center).length();
}

void Plane::computeLocalAABB()
{
  AABB aabb;
  computeBV<AABB>(*this, SimpleTransform(), aabb);
  aabb_center = aabb.center();
  aabb_radius = (aabb.min_ - aabb_center).length();
}




}
