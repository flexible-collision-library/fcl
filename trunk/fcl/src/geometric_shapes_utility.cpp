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
void computeBV<AABB>(const Box& s, AABB& bv)
{
  Vec3f R[3];
  matMulMat(s.getRotation(), s.getLocalRotation(), R);
  Vec3f T = matMulVec(s.getRotation(), s.getLocalTranslation()) + s.getTranslation();

  BVH_REAL x_range = 0.5 * (fabs(R[0][0] * s.side[0]) + fabs(R[0][1] * s.side[1]) + fabs(R[0][2] * s.side[2]));
  BVH_REAL y_range = 0.5 * (fabs(R[1][0] * s.side[0]) + fabs(R[1][1] * s.side[1]) + fabs(R[1][2] * s.side[2]));
  BVH_REAL z_range = 0.5 * (fabs(R[2][0] * s.side[0]) + fabs(R[2][1] * s.side[1]) + fabs(R[2][2] * s.side[2]));

  bv.max_ = T + Vec3f(x_range, y_range, z_range);
  bv.min_ = T + Vec3f(-x_range, -y_range, -z_range);
}

template<>
void computeBV<AABB>(const Sphere& s, AABB& bv)
{
  Vec3f T = matMulVec(s.getRotation(), s.getLocalTranslation()) + s.getTranslation();

  bv.max_ = T + Vec3f(s.radius, s.radius, s.radius);
  bv.min_ = T + Vec3f(-s.radius, -s.radius, -s.radius);
}

template<>
void computeBV<AABB>(const Capsule& s, AABB& bv)
{
  Vec3f R[3];
  matMulMat(s.getRotation(), s.getLocalRotation(), R);
  Vec3f T = matMulVec(s.getRotation(), s.getLocalTranslation()) + s.getTranslation();

  BVH_REAL x_range = 0.5 * fabs(R[0][2] * s.lz) + s.radius;
  BVH_REAL y_range = 0.5 * fabs(R[1][2] * s.lz) + s.radius;
  BVH_REAL z_range = 0.5 * fabs(R[2][2] * s.lz) + s.radius;

  bv.max_ = T + Vec3f(x_range, y_range, z_range);
  bv.min_ = T + Vec3f(-x_range, -y_range, -z_range);
}

template<>
void computeBV<AABB>(const Cone& s, AABB& bv)
{
  Vec3f R[3];
  matMulMat(s.getRotation(), s.getLocalRotation(), R);
  Vec3f T = matMulVec(s.getRotation(), s.getLocalTranslation()) + s.getTranslation();

  BVH_REAL x_range = fabs(R[0][0] * s.radius) + fabs(R[0][1] * s.radius) + 0.5 * fabs(R[0][2] * s.lz);
  BVH_REAL y_range = fabs(R[1][0] * s.radius) + fabs(R[1][1] * s.radius) + 0.5 * fabs(R[1][2] * s.lz);
  BVH_REAL z_range = fabs(R[2][0] * s.radius) + fabs(R[2][1] * s.radius) + 0.5 * fabs(R[2][2] * s.lz);

  bv.max_ = T + Vec3f(x_range, y_range, z_range);
  bv.min_ = T + Vec3f(-x_range, -y_range, -z_range);
}

template<>
void computeBV<AABB>(const Cylinder& s, AABB& bv)
{
  Vec3f R[3];
  matMulMat(s.getRotation(), s.getLocalRotation(), R);
  Vec3f T = matMulVec(s.getRotation(), s.getLocalTranslation()) + s.getTranslation();

  BVH_REAL x_range = fabs(R[0][0] * s.radius) + fabs(R[0][1] * s.radius) + 0.5 * fabs(R[0][2] * s.lz);
  BVH_REAL y_range = fabs(R[1][0] * s.radius) + fabs(R[1][1] * s.radius) + 0.5 * fabs(R[1][2] * s.lz);
  BVH_REAL z_range = fabs(R[2][0] * s.radius) + fabs(R[2][1] * s.radius) + 0.5 * fabs(R[2][2] * s.lz);

  bv.max_ = T + Vec3f(x_range, y_range, z_range);
  bv.min_ = T + Vec3f(-x_range, -y_range, -z_range);
}

template<>
void computeBV<AABB>(const Convex& s, AABB& bv)
{
  Vec3f R[3];
  matMulMat(s.getRotation(), s.getLocalRotation(), R);
  Vec3f T = matMulVec(s.getRotation(), s.getLocalTranslation()) + s.getTranslation();

  AABB bv_;
  for(int i = 0; i < s.num_points; ++i)
  {
    Vec3f new_p = matMulVec(R, s.points[i]) + T;
    bv_ += new_p;
  }

  bv = bv_;
}

template<>
void computeBV<AABB>(const Plane& s, AABB& bv)
{
  Vec3f R[3];
  matMulMat(s.getRotation(), s.getLocalRotation(), R);

  Vec3f n = matMulVec(R, n);

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
void computeBV<OBB>(const Box& s, OBB& bv)
{
  Vec3f R[3];
  matMulMat(s.getRotation(), s.getLocalRotation(), R);
  Vec3f T = matMulVec(s.getRotation(), s.getLocalTranslation()) + s.getTranslation();

  bv.To = T;
  bv.axis[0] = Vec3f(R[0][0], R[1][0], R[2][0]);
  bv.axis[1] = Vec3f(R[0][1], R[1][1], R[2][1]);
  bv.axis[2] = Vec3f(R[0][2], R[1][2], R[2][2]);
  bv.extent = s.side * (BVH_REAL)0.5;
}

template<>
void computeBV<OBB>(const Sphere& s, OBB& bv)
{
  Vec3f T = matMulVec(s.getRotation(), s.getLocalTranslation()) + s.getTranslation();

  bv.To = T;
  bv.axis[0] = Vec3f(1, 0, 0);
  bv.axis[1] = Vec3f(0, 1, 0);
  bv.axis[2] = Vec3f(0, 0, 1);
  bv.extent = Vec3f(s.radius, s.radius, s.radius);
}

template<>
void computeBV<OBB>(const Capsule& s, OBB& bv)
{
  Vec3f R[3];
  matMulMat(s.getRotation(), s.getLocalRotation(), R);
  Vec3f T = matMulVec(s.getRotation(), s.getLocalTranslation()) + s.getTranslation();

  bv.To = T;
  bv.axis[0] = Vec3f(R[0][0], R[1][0], R[2][0]);
  bv.axis[1] = Vec3f(R[0][1], R[1][1], R[2][1]);
  bv.axis[2] = Vec3f(R[0][2], R[1][2], R[2][2]);
  bv.extent = Vec3f(s.radius, s.radius, s.lz / 2 + s.radius);
}

template<>
void computeBV<OBB>(const Cone& s, OBB& bv)
{
  Vec3f R[3];
  matMulMat(s.getRotation(), s.getLocalRotation(), R);
  Vec3f T = matMulVec(s.getRotation(), s.getLocalTranslation()) + s.getTranslation();

  bv.To = T;
  bv.axis[0] = Vec3f(R[0][0], R[1][0], R[2][0]);
  bv.axis[1] = Vec3f(R[0][1], R[1][1], R[2][1]);
  bv.axis[2] = Vec3f(R[0][2], R[1][2], R[2][2]);
  bv.extent = Vec3f(s.radius, s.radius, s.lz / 2);
}

template<>
void computeBV<OBB>(const Cylinder& s, OBB& bv)
{
  Vec3f R[3];
  matMulMat(s.getRotation(), s.getLocalRotation(), R);
  Vec3f T = matMulVec(s.getRotation(), s.getLocalTranslation()) + s.getTranslation();

  bv.To = T;
  bv.axis[0] = Vec3f(R[0][0], R[1][0], R[2][0]);
  bv.axis[1] = Vec3f(R[0][1], R[1][1], R[2][1]);
  bv.axis[2] = Vec3f(R[0][2], R[1][2], R[2][2]);
  bv.extent = Vec3f(s.radius, s.radius, s.lz / 2);
}

template<>
void computeBV<OBB>(const Convex& s, OBB& bv)
{
  Vec3f R[3];
  matMulMat(s.getRotation(), s.getLocalRotation(), R);
  Vec3f T = matMulVec(s.getRotation(), s.getLocalTranslation()) + s.getTranslation();

  fit(s.points, s.num_points, bv);

  Vec3f axis[3];
  axis[0] = matMulVec(R, bv.axis[0]);
  axis[1] = matMulVec(R, bv.axis[1]);
  axis[2] = matMulVec(R, bv.axis[2]);

  bv.axis[0] = axis[0];
  bv.axis[1] = axis[1];
  bv.axis[2] = axis[2];

  bv.To = matMulVec(R, bv.To) + T;

}

template<>
void computeBV<OBB>(const Plane& s, OBB& bv)
{
  Vec3f R[3];
  matMulMat(s.getRotation(), s.getLocalRotation(), R);
  Vec3f T = matMulVec(s.getRotation(), s.getLocalTranslation()) + s.getTranslation();

  // generate other two axes orthonormal to plane normal
  const Vec3f& w = s.n;
  Vec3f u, v;
  float inv_length;
  if(fabs(w[0]) >= fabs(w[1]))
  {
    inv_length = 1.0 / sqrt(w[0] * w[0] + w[2] * w[2]);
    u[0] = -w[2] * inv_length;
    u[1] = 0;
    u[2] = w[0] * inv_length;
    v[0] = w[1] * u[2];
    v[1] = w[2] * u[0] - w[0] * u[2];
    v[2] = -w[1] * u[0];
  }
  else
  {
    inv_length = 1.0 / sqrt(w[1] * w[1] + w[2] * w[2]);
    u[0] = 0;
    u[1] = w[2] * inv_length;
    u[2] = -w[1] * inv_length;
    v[0] = w[1] * u[2] - w[2] * u[1];
    v[1] = -w[0] * u[2];
    v[2] = w[0] * u[1];
  }

  bv.axis[0] = w;
  bv.axis[1] = u;
  bv.axis[2] = v;

  bv.extent = Vec3f(0, std::numeric_limits<BVH_REAL>::max(), std::numeric_limits<BVH_REAL>::max());

  Vec3f p = s.n * s.d;
  bv.To = matMulVec(R, p) + T;
}

void Box::computeLocalAABB()
{
  computeBV<AABB>(*this, aabb_local);
  aabb = aabb_local;
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).length();
}

void Sphere::computeLocalAABB()
{
  computeBV<AABB>(*this, aabb_local);
  aabb = aabb_local;
  aabb_center = aabb_local.center();
  aabb_radius = radius;
}

void Capsule::computeLocalAABB()
{
  computeBV<AABB>(*this, aabb_local);
  aabb = aabb_local;
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).length();
}

void Cone::computeLocalAABB()
{
  computeBV<AABB>(*this, aabb_local);
  aabb = aabb_local;
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).length();
}

void Cylinder::computeLocalAABB()
{
  computeBV<AABB>(*this, aabb_local);
  aabb = aabb_local;
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).length();
}

void Convex::computeLocalAABB()
{
  computeBV<AABB>(*this, aabb_local);
  aabb = aabb_local;
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).length();
}

void Plane::computeLocalAABB()
{
  computeBV<AABB>(*this, aabb_local);
  aabb = aabb_local;
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).length();
}




}
