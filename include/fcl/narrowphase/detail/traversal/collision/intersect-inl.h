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

#ifndef FCL_NARROWPHASE_DETAIL_INTERSECT_INL_H
#define FCL_NARROWPHASE_DETAIL_INTERSECT_INL_H

#include "fcl/narrowphase/detail/traversal/collision/intersect.h"

namespace fcl
{

namespace detail
{

//==============================================================================
extern template
class FCL_EXPORT Intersect<double>;

//==============================================================================
template <typename S>
bool Intersect<S>::isZero(S v)
{
  return (v < getNearZeroThreshold()) && (v > -getNearZeroThreshold());
}

//==============================================================================
/// @brief data: only used for EE, return the intersect point
template <typename S>
bool Intersect<S>::solveCubicWithIntervalNewton(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& d0,
                                             const Vector3<S>& va, const Vector3<S>& vb, const Vector3<S>& vc, const Vector3<S>& vd,
                                             S& l, S& r, bool bVF, S coeffs[], Vector3<S>* data)
{
  S v2[2]= {l*l,r*r};
  S v[2]= {l,r};
  S r_backup;

  unsigned char min3, min2, min1, max3, max2, max1;

  min3= *((unsigned char*)&coeffs[3]+7)>>7; max3=min3^1;
  min2= *((unsigned char*)&coeffs[2]+7)>>7; max2=min2^1;
  min1= *((unsigned char*)&coeffs[1]+7)>>7; max1=min1^1;

  // bound the cubic

  S minor = coeffs[3]*v2[min3]*v[min3]+coeffs[2]*v2[min2]+coeffs[1]*v[min1]+coeffs[0];
  S major = coeffs[3]*v2[max3]*v[max3]+coeffs[2]*v2[max2]+coeffs[1]*v[max1]+coeffs[0];

  if(major<0) return false;
  if(minor>0) return false;

  // starting here, the bounds have opposite values
  S m = 0.5 * (r + l);

  // bound the derivative
  S dminor = 3.0*coeffs[3]*v2[min3]+2.0*coeffs[2]*v[min2]+coeffs[1];
  S dmajor = 3.0*coeffs[3]*v2[max3]+2.0*coeffs[2]*v[max2]+coeffs[1];

  if((dminor > 0)||(dmajor < 0)) // we can use Newton
  {
    S m2 = m*m;
    S fm = coeffs[3]*m2*m+coeffs[2]*m2+coeffs[1]*m+coeffs[0];
    S nl = m;
    S nu = m;
    if(fm>0)
    {
      nl-=(fm/dminor);
      nu-=(fm/dmajor);
    }
    else
    {
      nu-=(fm/dminor);
      nl-=(fm/dmajor);
    }

    //intersect with [l,r]

    if(nl>r) return false;
    if(nu<l) return false;
    if(nl>l)
    {
      if(nu<r) { l=nl; r=nu; m=0.5*(l+r); }
      else { l=nl; m=0.5*(l+r); }
    }
    else
    {
      if(nu<r) { r=nu; m=0.5*(l+r); }
    }
  }

  // sufficient temporal resolution, check root validity
  if((r-l)< getCcdResolution())
  {
    if(bVF)
      return checkRootValidity_VF(a0, b0, c0, d0, va, vb, vc, vd, r);
    else
      return checkRootValidity_EE(a0, b0, c0, d0, va, vb, vc, vd, r, data);
  }

  r_backup = r, r = m;
  if(solveCubicWithIntervalNewton(a0, b0, c0, d0, va, vb, vc, vd, l, r, bVF, coeffs, data))
    return true;

  l = m, r = r_backup;
  return solveCubicWithIntervalNewton(a0, b0, c0, d0, va, vb, vc, vd, l, r, bVF, coeffs, data);
}

//==============================================================================
template <typename S>
bool Intersect<S>::insideTriangle(const Vector3<S>& a, const Vector3<S>& b, const Vector3<S>& c, const Vector3<S>&p)
{
  Vector3<S> ab = b - a;
  Vector3<S> ac = c - a;
  Vector3<S> n = ab.cross(ac);

  Vector3<S> pa = a - p;
  Vector3<S> pb = b - p;
  Vector3<S> pc = c - p;

  if((pb.cross(pc)).dot(n) < -getEpsilon()) return false;
  if((pc.cross(pa)).dot(n) < -getEpsilon()) return false;
  if((pa.cross(pb)).dot(n) < -getEpsilon()) return false;

  return true;
}

//==============================================================================
template <typename S>
bool Intersect<S>::insideLineSegment(const Vector3<S>& a, const Vector3<S>& b, const Vector3<S>& p)
{
  return (p - a).dot(p - b) <= 0;
}

//==============================================================================
/// @brief Calculate the line segment papb that is the shortest route between
/// two lines p1p2 and p3p4. Calculate also the values of mua and mub where
///    pa = p1 + mua (p2 - p1)
///    pb = p3 + mub (p4 - p3)
/// Return FALSE if no solution exists.
template <typename S>
bool Intersect<S>::linelineIntersect(const Vector3<S>& p1, const Vector3<S>& p2, const Vector3<S>& p3, const Vector3<S>& p4,
                                  Vector3<S>* pa, Vector3<S>* pb, S* mua, S* mub)
{
  Vector3<S> p31 = p1 - p3;
  Vector3<S> p34 = p4 - p3;
  if(fabs(p34[0]) < getEpsilon() && fabs(p34[1]) < getEpsilon() && fabs(p34[2]) < getEpsilon())
    return false;

  Vector3<S> p12 = p2 - p1;
  if(fabs(p12[0]) < getEpsilon() && fabs(p12[1]) < getEpsilon() && fabs(p12[2]) < getEpsilon())
    return false;

  S d3134 = p31.dot(p34);
  S d3412 = p34.dot(p12);
  S d3112 = p31.dot(p12);
  S d3434 = p34.dot(p34);
  S d1212 = p12.dot(p12);

  S denom = d1212 * d3434 - d3412 * d3412;
  if(fabs(denom) < getEpsilon())
    return false;
  S numer = d3134 * d3412 - d3112 * d3434;

  *mua = numer / denom;
  if(*mua < 0 || *mua > 1)
    return false;

  *mub = (d3134 + d3412 * (*mua)) / d3434;
  if(*mub < 0 || *mub > 1)
    return false;

  *pa = p1 + p12 * (*mua);
  *pb = p3 + p34 * (*mub);
  return true;
}

//==============================================================================
template <typename S>
bool Intersect<S>::checkRootValidity_VF(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& p0,
                                     const Vector3<S>& va, const Vector3<S>& vb, const Vector3<S>& vc, const Vector3<S>& vp,
                                     S t)
{
  return insideTriangle(a0 + va * t, b0 + vb * t, c0 + vc * t, p0 + vp * t);
}

//==============================================================================
template <typename S>
bool Intersect<S>::checkRootValidity_EE(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& d0,
                                     const Vector3<S>& va, const Vector3<S>& vb, const Vector3<S>& vc, const Vector3<S>& vd,
                                     S t, Vector3<S>* q_i)
{
  Vector3<S> a = a0 + va * t;
  Vector3<S> b = b0 + vb * t;
  Vector3<S> c = c0 + vc * t;
  Vector3<S> d = d0 + vd * t;
  Vector3<S> p1, p2;
  S t_ab, t_cd;
  if(linelineIntersect(a, b, c, d, &p1, &p2, &t_ab, &t_cd))
  {
    if(q_i) *q_i = p1;
    return true;
  }

  return false;
}

//==============================================================================
template <typename S>
bool Intersect<S>::checkRootValidity_VE(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& p0,
                                     const Vector3<S>& va, const Vector3<S>& vb, const Vector3<S>& vp,
                                     S t)
{
  return insideLineSegment(a0 + va * t, b0 + vb * t, p0 + vp * t);
}

//==============================================================================
template <typename S>
bool Intersect<S>::solveSquare(S a, S b, S c,
                            const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& d0,
                            const Vector3<S>& va, const Vector3<S>& vb, const Vector3<S>& vc, const Vector3<S>& vd,
                            bool bVF,
                            S* ret)
{
  S discriminant = b * b - 4 * a * c;
  if(discriminant < 0)
    return false;

  S sqrt_dis = sqrt(discriminant);
  S r1 = (-b + sqrt_dis) / (2 * a);
  bool v1 = (r1 >= 0.0 && r1 <= 1.0) ? ((bVF) ? checkRootValidity_VF(a0, b0, c0, d0, va, vb, vc, vd, r1) : checkRootValidity_EE(a0, b0, c0, d0, va, vb, vc, vd, r1)) : false;

  S r2 = (-b - sqrt_dis) / (2 * a);
  bool v2 = (r2 >= 0.0 && r2 <= 1.0) ? ((bVF) ? checkRootValidity_VF(a0, b0, c0, d0, va, vb, vc, vd, r2) : checkRootValidity_EE(a0, b0, c0, d0, va, vb, vc, vd, r2)) : false;

  if(v1 && v2)
  {
    *ret = (r1 > r2) ? r2 : r1;
    return true;
  }
  if(v1)
  {
    *ret = r1;
    return true;
  }
  if(v2)
  {
    *ret = r2;
    return true;
  }

  return false;
}

//==============================================================================
template <typename S>
bool Intersect<S>::solveSquare(S a, S b, S c,
                            const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& p0,
                            const Vector3<S>& va, const Vector3<S>& vb, const Vector3<S>& vp)
{
  if(isZero(a))
  {
    S t = -c/b;
    return (t >= 0 && t <= 1) ? checkRootValidity_VE(a0, b0, p0, va, vb, vp, t) : false;
  }

  S discriminant = b*b-4*a*c;
  if(discriminant < 0)
    return false;

  S sqrt_dis = sqrt(discriminant);

  S r1 = (-b+sqrt_dis) / (2 * a);
  bool v1 = (r1 >= 0.0 && r1 <= 1.0) ? checkRootValidity_VE(a0, b0, p0, va, vb, vp, r1) : false;
  if(v1) return true;

  S r2 = (-b-sqrt_dis) / (2 * a);
  bool v2 = (r2 >= 0.0 && r2 <= 1.0) ? checkRootValidity_VE(a0, b0, p0, va, vb, vp, r2) : false;
  return v2;
}

//==============================================================================
/// @brief Compute the cubic coefficients for VF case
/// See Paper "Interactive Continuous Collision Detection between Deformable Models using Connectivity-Based Culling", Equation 1.
template <typename S>
void Intersect<S>::computeCubicCoeff_VF(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& p0,
                                     const Vector3<S>& va, const Vector3<S>& vb, const Vector3<S>& vc, const Vector3<S>& vp,
                                     S* a, S* b, S* c, S* d)
{
  Vector3<S> vavb = vb - va;
  Vector3<S> vavc = vc - va;
  Vector3<S> vavp = vp - va;
  Vector3<S> a0b0 = b0 - a0;
  Vector3<S> a0c0 = c0 - a0;
  Vector3<S> a0p0 = p0 - a0;

  Vector3<S> vavb_cross_vavc = vavb.cross(vavc);
  Vector3<S> vavb_cross_a0c0 = vavb.cross(a0c0);
  Vector3<S> a0b0_cross_vavc = a0b0.cross(vavc);
  Vector3<S> a0b0_cross_a0c0 = a0b0.cross(a0c0);

  *a = vavp.dot(vavb_cross_vavc);
  *b = a0p0.dot(vavb_cross_vavc) + vavp.dot(vavb_cross_a0c0 + a0b0_cross_vavc);
  *c = vavp.dot(a0b0_cross_a0c0) + a0p0.dot(vavb_cross_a0c0 + a0b0_cross_vavc);
  *d = a0p0.dot(a0b0_cross_a0c0);
}

//==============================================================================
template <typename S>
void Intersect<S>::computeCubicCoeff_EE(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& d0,
                                     const Vector3<S>& va, const Vector3<S>& vb, const Vector3<S>& vc, const Vector3<S>& vd,
                                     S* a, S* b, S* c, S* d)
{
  Vector3<S> vavb = vb - va;
  Vector3<S> vcvd = vd - vc;
  Vector3<S> vavc = vc - va;
  Vector3<S> c0d0 = d0 - c0;
  Vector3<S> a0b0 = b0 - a0;
  Vector3<S> a0c0 = c0 - a0;
  Vector3<S> vavb_cross_vcvd = vavb.cross(vcvd);
  Vector3<S> vavb_cross_c0d0 = vavb.cross(c0d0);
  Vector3<S> a0b0_cross_vcvd = a0b0.cross(vcvd);
  Vector3<S> a0b0_cross_c0d0 = a0b0.cross(c0d0);

  *a = vavc.dot(vavb_cross_vcvd);
  *b = a0c0.dot(vavb_cross_vcvd) + vavc.dot(vavb_cross_c0d0 + a0b0_cross_vcvd);
  *c = vavc.dot(a0b0_cross_c0d0) + a0c0.dot(vavb_cross_c0d0 + a0b0_cross_vcvd);
  *d = a0c0.dot(a0b0_cross_c0d0);
}

//==============================================================================
template <typename S>
void Intersect<S>::computeCubicCoeff_VE(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& p0,
                                     const Vector3<S>& va, const Vector3<S>& vb, const Vector3<S>& vp,
                                     const Vector3<S>& L,
                                     S* a, S* b, S* c)
{
  Vector3<S> vbva = va - vb;
  Vector3<S> vbvp = vp - vb;
  Vector3<S> b0a0 = a0 - b0;
  Vector3<S> b0p0 = p0 - b0;

  Vector3<S> L_cross_vbvp = L.cross(vbvp);
  Vector3<S> L_cross_b0p0 = L.cross(b0p0);

  *a = L_cross_vbvp.dot(vbva);
  *b = L_cross_vbvp.dot(b0a0) + L_cross_b0p0.dot(vbva);
  *c = L_cross_b0p0.dot(b0a0);
}

//==============================================================================
template <typename S>
bool Intersect<S>::intersect_VF(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& p0,
                             const Vector3<S>& a1, const Vector3<S>& b1, const Vector3<S>& c1, const Vector3<S>& p1,
                             S* collision_time, Vector3<S>* p_i, bool useNewton)
{
  *collision_time = 2.0;

  Vector3<S> vp, va, vb, vc;
  vp = p1 - p0;
  va = a1 - a0;
  vb = b1 - b0;
  vc = c1 - c0;

  S a, b, c, d;
  computeCubicCoeff_VF(a0, b0, c0, p0, va, vb, vc, vp, &a, &b, &c, &d);

  if(isZero(a) && isZero(b) && isZero(c) && isZero(d))
  {
    return false;
  }


  /// if(isZero(a))
  /// {
  ///   return solveSquare(b, c, d, a0, b0, c0, p0, va, vb, vc, vp, true, collision_time);
  /// }

  S coeffs[4];
  coeffs[3] = a, coeffs[2] = b, coeffs[1] = c, coeffs[0] = d;

  if(useNewton)
  {
    S l = 0;
    S r = 1;

    if(solveCubicWithIntervalNewton(a0, b0, c0, p0, va, vb, vc, vp, l, r, true, coeffs))
    {
      *collision_time = 0.5 * (l + r);
    }
  }
  else
  {
    S roots[3];
    int num = PolySolver<S>::solveCubic(coeffs, roots);
    for(int i = 0; i < num; ++i)
    {
      S r = roots[i];
      if(r < 0 || r > 1) continue;
      if(checkRootValidity_VF(a0, b0, c0, p0, va, vb, vc, vp, r))
      {
        *collision_time = r;
        break;
      }
    }
  }

  if(*collision_time > 1)
  {
    return false;
  }

  *p_i = vp * (*collision_time) + p0;
  return true;
}

//==============================================================================
template <typename S>
bool Intersect<S>::intersect_EE(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& d0,
                             const Vector3<S>& a1, const Vector3<S>& b1, const Vector3<S>& c1, const Vector3<S>& d1,
                             S* collision_time, Vector3<S>* p_i, bool useNewton)
{
  *collision_time = 2.0;

  Vector3<S> va, vb, vc, vd;
  va = a1 - a0;
  vb = b1 - b0;
  vc = c1 - c0;
  vd = d1 - d0;

  S a, b, c, d;
  computeCubicCoeff_EE(a0, b0, c0, d0, va, vb, vc, vd, &a, &b, &c, &d);

  if(isZero(a) && isZero(b) && isZero(c) && isZero(d))
  {
    return false;
  }

  /// if(isZero(a))
  /// {
  ///   return solveSquare(b, c, d, a0, b0, c0, d0, va, vb, vc, vd, collision_time, false);
  /// }


  S coeffs[4];
  coeffs[3] = a, coeffs[2] = b, coeffs[1] = c, coeffs[0] = d;

  if(useNewton)
  {
    S l = 0;
    S r = 1;

    if(solveCubicWithIntervalNewton(a0, b0, c0, d0, va, vb, vc, vd, l, r, false, coeffs, p_i))
    {
      *collision_time  = (l + r) * 0.5;
    }
  }
  else
  {
    S roots[3];
    int num = PolySolver<S>::solveCubic(coeffs, roots);
    for(int i = 0; i < num; ++i)
    {
      S r = roots[i];
      if(r < 0 || r > 1) continue;

      if(checkRootValidity_EE(a0, b0, c0, d0, va, vb, vc, vd, r, p_i))
      {
        *collision_time = r;
        break;
      }
    }
  }

  if(*collision_time > 1)
  {
    return false;
  }

  return true;
}

//==============================================================================
template <typename S>
bool Intersect<S>::intersect_VE(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& p0,
                             const Vector3<S>& a1, const Vector3<S>& b1, const Vector3<S>& p1,
                             const Vector3<S>& L)
{
  Vector3<S> va, vb, vp;
  va = a1 - a0;
  vb = b1 - b0;
  vp = p1 - p0;

  S a, b, c;
  computeCubicCoeff_VE(a0, b0, p0, va, vb, vp, L, &a, &b, &c);

  if(isZero(a) && isZero(b) && isZero(c))
    return true;

  return solveSquare(a, b, c, a0, b0, p0, va, vb, vp);

}

//==============================================================================
/// @brief Prefilter for intersection, works for both VF and EE
template <typename S>
bool Intersect<S>::intersectPreFiltering(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& d0,
                                      const Vector3<S>& a1, const Vector3<S>& b1, const Vector3<S>& c1, const Vector3<S>& d1)
{
  Vector3<S> n0 = (b0 - a0).cross(c0 - a0);
  Vector3<S> n1 = (b1 - a1).cross(c1 - a1);
  Vector3<S> a0a1 = a1 - a0;
  Vector3<S> b0b1 = b1 - b0;
  Vector3<S> c0c1 = c1 - c0;
  Vector3<S> delta = (b0b1 - a0a1).cross(c0c1 - a0a1);
  Vector3<S> nx = (n0 + n1 - delta) * 0.5;

  Vector3<S> a0d0 = d0 - a0;
  Vector3<S> a1d1 = d1 - a1;

  S A = n0.dot(a0d0);
  S B = n1.dot(a1d1);
  S C = nx.dot(a0d0);
  S D = nx.dot(a1d1);
  S E = n1.dot(a0d0);
  S F = n0.dot(a1d1);

  if(A > 0 && B > 0 && (2*C +F) > 0 && (2*D+E) > 0)
    return false;
  if(A < 0 && B < 0 && (2*C +F) < 0 && (2*D+E) < 0)
    return false;

  return true;
}

//==============================================================================
template <typename S>
bool Intersect<S>::intersect_VF_filtered(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& p0,
                                      const Vector3<S>& a1, const Vector3<S>& b1, const Vector3<S>& c1, const Vector3<S>& p1,
                                      S* collision_time, Vector3<S>* p_i, bool useNewton)
{
  if(intersectPreFiltering(a0, b0, c0, p0, a1, b1, c1, p1))
  {
    return intersect_VF(a0, b0, c0, p0, a1, b1, c1, p1, collision_time, p_i, useNewton);
  }
  else
    return false;
}

//==============================================================================
template <typename S>
bool Intersect<S>::intersect_EE_filtered(const Vector3<S>& a0, const Vector3<S>& b0, const Vector3<S>& c0, const Vector3<S>& d0,
                                      const Vector3<S>& a1, const Vector3<S>& b1, const Vector3<S>& c1, const Vector3<S>& d1,
                                      S* collision_time, Vector3<S>* p_i, bool useNewton)
{
  if(intersectPreFiltering(a0, b0, c0, d0, a1, b1, c1, d1))
  {
    return intersect_EE(a0, b0, c0, d0, a1, b1, c1, d1, collision_time, p_i, useNewton);
  }
  else
    return false;
}

//==============================================================================
template <typename S>
bool Intersect<S>::intersect_Triangle(
    const Vector3<S>& P1,
    const Vector3<S>& P2,
    const Vector3<S>& P3,
    const Vector3<S>& Q1,
    const Vector3<S>& Q2,
    const Vector3<S>& Q3,
    const Matrix3<S>& R,
    const Vector3<S>& T,
    Vector3<S>* contact_points,
    unsigned int* num_contact_points,
    S* penetration_depth,
    Vector3<S>* normal)
{
  Vector3<S> Q1_ = R * Q1 + T;
  Vector3<S> Q2_ = R * Q2 + T;
  Vector3<S> Q3_ = R * Q3 + T;

  return intersect_Triangle(P1, P2, P3, Q1_, Q2_, Q3_, contact_points, num_contact_points, penetration_depth, normal);
}

//==============================================================================
template <typename S>
bool Intersect<S>::intersect_Triangle(
    const Vector3<S>& P1,
    const Vector3<S>& P2,
    const Vector3<S>& P3,
    const Vector3<S>& Q1,
    const Vector3<S>& Q2,
    const Vector3<S>& Q3,
    const Transform3<S>& tf,
    Vector3<S>* contact_points,
    unsigned int* num_contact_points,
    S* penetration_depth,
    Vector3<S>* normal)
{
  Vector3<S> Q1_ = tf * Q1;
  Vector3<S> Q2_ = tf * Q2;
  Vector3<S> Q3_ = tf * Q3;

  return intersect_Triangle(P1, P2, P3, Q1_, Q2_, Q3_, contact_points, num_contact_points, penetration_depth, normal);
}

//==============================================================================
template <typename S>
bool Intersect<S>::intersect_Triangle_ODE_style(
    const Vector3<S>& P1, const Vector3<S>& P2, const Vector3<S>& P3,
    const Vector3<S>& Q1, const Vector3<S>& Q2, const Vector3<S>& Q3,
    Vector3<S>* contact_points,
    unsigned int* num_contact_points,
    S* penetration_depth,
    Vector3<S>* normal)
{
  Vector3<S> n1;
  S t1;
  bool b1 = buildTrianglePlane(P1, P2, P3, &n1, &t1);
  if(!b1) return false;

  Vector3<S> n2;
  S t2;
  bool b2 = buildTrianglePlane(Q1, Q2, Q3, &n2, &t2);
  if(!b2) return false;

  if(sameSideOfPlane(P1, P2, P3, n2, t2))
    return false;

  if(sameSideOfPlane(Q1, Q2, Q3, n1, t1))
    return false;

  Vector3<S> clipped_points1[getMaxTriangleClips()];
  unsigned int num_clipped_points1 = 0;
  Vector3<S> clipped_points2[getMaxTriangleClips()];
  unsigned int num_clipped_points2 = 0;

  Vector3<S> deepest_points1[getMaxTriangleClips()];
  unsigned int num_deepest_points1 = 0;
  Vector3<S> deepest_points2[getMaxTriangleClips()];
  unsigned int num_deepest_points2 = 0;
  S penetration_depth1 = -1, penetration_depth2 = -1;

  clipTriangleByTriangleAndEdgePlanes(Q1, Q2, Q3, P1, P2, P3, n1, t1, clipped_points2, &num_clipped_points2);

  if(num_clipped_points2 == 0)
    return false;

  computeDeepestPoints(clipped_points2, num_clipped_points2, n1, t1, &penetration_depth2, deepest_points2, &num_deepest_points2);
  if(num_deepest_points2 == 0)
    return false;

  clipTriangleByTriangleAndEdgePlanes(P1, P2, P3, Q1, Q2, Q3, n2, t2, clipped_points1, &num_clipped_points1);
  if(num_clipped_points1 == 0)
    return false;

  computeDeepestPoints(clipped_points1, num_clipped_points1, n2, t2, &penetration_depth1, deepest_points1, &num_deepest_points1);
  if(num_deepest_points1 == 0)
    return false;


  /// Return contact information
  if(contact_points && num_contact_points && penetration_depth && normal)
  {
    if(penetration_depth1 > penetration_depth2)
    {
      *num_contact_points = num_deepest_points2;
      for(unsigned int i = 0; i < num_deepest_points2; ++i)
      {
        contact_points[i] = deepest_points2[i];
      }

      *normal = n1;
      *penetration_depth = penetration_depth2;
    }
    else
    {
      *num_contact_points = num_deepest_points1;
      for(unsigned int i = 0; i < num_deepest_points1; ++i)
      {
        contact_points[i] = deepest_points1[i];
      }

      *normal = -n2;
      *penetration_depth = penetration_depth1;
    }
  }

  return true;
}

//==============================================================================
template <typename S>
bool Intersect<S>::intersect_Triangle(
    const Vector3<S>& P1, const Vector3<S>& P2, const Vector3<S>& P3,
    const Vector3<S>& Q1, const Vector3<S>& Q2, const Vector3<S>& Q3,
    Vector3<S>* contact_points,
    unsigned int* num_contact_points,
    S* penetration_depth,
    Vector3<S>* normal)
{
  Vector3<S> p1 = P1 - P1;
  Vector3<S> p2 = P2 - P1;
  Vector3<S> p3 = P3 - P1;
  Vector3<S> q1 = Q1 - P1;
  Vector3<S> q2 = Q2 - P1;
  Vector3<S> q3 = Q3 - P1;

  Vector3<S> e1 = p2 - p1;
  Vector3<S> e2 = p3 - p2;
  Vector3<S> n1 = e1.cross(e2);
  if (!project6(n1, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<S> f1 = q2 - q1;
  Vector3<S> f2 = q3 - q2;
  Vector3<S> m1 = f1.cross(f2);
  if (!project6(m1, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<S> ef11 = e1.cross(f1);
  if (!project6(ef11, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<S> ef12 = e1.cross(f2);
  if (!project6(ef12, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<S> f3 = q1 - q3;
  Vector3<S> ef13 = e1.cross(f3);
  if (!project6(ef13, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<S> ef21 = e2.cross(f1);
  if (!project6(ef21, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<S> ef22 = e2.cross(f2);
  if (!project6(ef22, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<S> ef23 = e2.cross(f3);
  if (!project6(ef23, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<S> e3 = p1 - p3;
  Vector3<S> ef31 = e3.cross(f1);
  if (!project6(ef31, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<S> ef32 = e3.cross(f2);
  if (!project6(ef32, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<S> ef33 = e3.cross(f3);
  if (!project6(ef33, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<S> g1 = e1.cross(n1);
  if (!project6(g1, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<S> g2 = e2.cross(n1);
  if (!project6(g2, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<S> g3 = e3.cross(n1);
  if (!project6(g3, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<S> h1 = f1.cross(m1);
  if (!project6(h1, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<S> h2 = f2.cross(m1);
  if (!project6(h2, p1, p2, p3, q1, q2, q3)) return false;

  Vector3<S> h3 = f3.cross(m1);
  if (!project6(h3, p1, p2, p3, q1, q2, q3)) return false;

  if(contact_points && num_contact_points && penetration_depth && normal)
  {
    Vector3<S> n1, n2;
    S t1, t2;
    buildTrianglePlane(P1, P2, P3, &n1, &t1);
    buildTrianglePlane(Q1, Q2, Q3, &n2, &t2);

    Vector3<S> deepest_points1[3];
    unsigned int num_deepest_points1 = 0;
    Vector3<S> deepest_points2[3];
    unsigned int num_deepest_points2 = 0;
    S penetration_depth1, penetration_depth2;

    Vector3<S> P[3] = {P1, P2, P3};
    Vector3<S> Q[3] = {Q1, Q2, Q3};

    computeDeepestPoints(Q, 3, n1, t1, &penetration_depth2, deepest_points2, &num_deepest_points2);
    computeDeepestPoints(P, 3, n2, t2, &penetration_depth1, deepest_points1, &num_deepest_points1);


    if(penetration_depth1 > penetration_depth2)
    {
      *num_contact_points = std::min(num_deepest_points2, (unsigned int)2);
      for(unsigned int i = 0; i < *num_contact_points; ++i)
      {
        contact_points[i] = deepest_points2[i];
      }

      *normal = n1;
      *penetration_depth = penetration_depth2;
    }
    else
    {
      *num_contact_points = std::min(num_deepest_points1, (unsigned int)2);
      for(unsigned int i = 0; i < *num_contact_points; ++i)
      {
        contact_points[i] = deepest_points1[i];
      }

      *normal = -n2;
      *penetration_depth = penetration_depth1;
    }
  }

  return true;
}

//==============================================================================
template <typename S>
void Intersect<S>::computeDeepestPoints(Vector3<S>* clipped_points, unsigned int num_clipped_points, const Vector3<S>& n, S t, S* penetration_depth, Vector3<S>* deepest_points, unsigned int* num_deepest_points)
{
  *num_deepest_points = 0;
  S max_depth = -std::numeric_limits<S>::max();
  unsigned int num_deepest_points_ = 0;
  unsigned int num_neg = 0;
  unsigned int num_pos = 0;
  unsigned int num_zero = 0;

  for(unsigned int i = 0; i < num_clipped_points; ++i)
  {
    S dist = -distanceToPlane(n, t, clipped_points[i]);
    if(dist > getEpsilon()) num_pos++;
    else if(dist < -getEpsilon()) num_neg++;
    else num_zero++;
    if(dist > max_depth)
    {
      max_depth = dist;
      num_deepest_points_ = 1;
      deepest_points[num_deepest_points_ - 1] = clipped_points[i];
    }
    else if(dist + 1e-6 >= max_depth)
    {
      num_deepest_points_++;
      deepest_points[num_deepest_points_ - 1] = clipped_points[i];
    }
  }

  if(max_depth < -getEpsilon())
    num_deepest_points_ = 0;

  if(num_zero == 0 && ((num_neg == 0) || (num_pos == 0)))
    num_deepest_points_ = 0;

  *penetration_depth = max_depth;
  *num_deepest_points = num_deepest_points_;
}

//==============================================================================
template <typename S>
void Intersect<S>::clipTriangleByTriangleAndEdgePlanes(const Vector3<S>& v1, const Vector3<S>& v2, const Vector3<S>& v3,
                                                    const Vector3<S>& t1, const Vector3<S>& t2, const Vector3<S>& t3,
                                                    const Vector3<S>& tn, S to,
                                                    Vector3<S> clipped_points[], unsigned int* num_clipped_points,
                                                    bool clip_triangle)
{
  *num_clipped_points = 0;
  Vector3<S> temp_clip[getMaxTriangleClips()];
  Vector3<S> temp_clip2[getMaxTriangleClips()];
  unsigned int num_temp_clip = 0;
  unsigned int num_temp_clip2 = 0;
  Vector3<S> v[3] = {v1, v2, v3};

  Vector3<S> plane_n;
  S plane_dist;

  if(buildEdgePlane(t1, t2, tn, &plane_n, &plane_dist))
  {
    clipPolygonByPlane(v, 3, plane_n, plane_dist, temp_clip, &num_temp_clip);
    if(num_temp_clip > 0)
    {
      if(buildEdgePlane(t2, t3, tn, &plane_n, &plane_dist))
      {
        clipPolygonByPlane(temp_clip, num_temp_clip, plane_n, plane_dist, temp_clip2, &num_temp_clip2);
        if(num_temp_clip2 > 0)
        {
          if(buildEdgePlane(t3, t1, tn, &plane_n, &plane_dist))
          {
            if(clip_triangle)
            {
              num_temp_clip = 0;
              clipPolygonByPlane(temp_clip2, num_temp_clip2, plane_n, plane_dist, temp_clip, &num_temp_clip);
              if(num_temp_clip > 0)
              {
                clipPolygonByPlane(temp_clip, num_temp_clip, tn, to, clipped_points, num_clipped_points);
              }
            }
            else
            {
              clipPolygonByPlane(temp_clip2, num_temp_clip2, plane_n, plane_dist, clipped_points, num_clipped_points);
            }
          }
        }
      }
    }
  }
}

//==============================================================================
template <typename S>
void Intersect<S>::clipPolygonByPlane(Vector3<S>* polygon_points, unsigned int num_polygon_points, const Vector3<S>& n, S t, Vector3<S> clipped_points[], unsigned int* num_clipped_points)
{
  *num_clipped_points = 0;

  unsigned int num_clipped_points_ = 0;
  unsigned int vi;
  unsigned int prev_classify = 2;
  unsigned int classify;
  for(unsigned int i = 0; i <= num_polygon_points; ++i)
  {
    vi = (i % num_polygon_points);
    S d = distanceToPlane(n, t, polygon_points[i]);
    classify = ((d > getEpsilon()) ? 1 : 0);
    if(classify == 0)
    {
      if(prev_classify == 1)
      {
        if(num_clipped_points_ < getMaxTriangleClips())
        {
          Vector3<S> tmp;
          clipSegmentByPlane(polygon_points[i - 1], polygon_points[vi], n, t, &tmp);
          if(num_clipped_points_ > 0)
          {
            if((tmp - clipped_points[num_clipped_points_ - 1]).squaredNorm() > getEpsilon())
            {
              clipped_points[num_clipped_points_] = tmp;
              num_clipped_points_++;
            }
          }
          else
          {
            clipped_points[num_clipped_points_] = tmp;
            num_clipped_points_++;
          }
        }
      }

      if(num_clipped_points_ < getMaxTriangleClips() && i < num_polygon_points)
      {
        clipped_points[num_clipped_points_] = polygon_points[vi];
        num_clipped_points_++;
      }
    }
    else
    {
      if(prev_classify == 0)
      {
        if(num_clipped_points_ < getMaxTriangleClips())
        {
          Vector3<S> tmp;
          clipSegmentByPlane(polygon_points[i - 1], polygon_points[vi], n, t, &tmp);
          if(num_clipped_points_ > 0)
          {
            if((tmp - clipped_points[num_clipped_points_ - 1]).squaredNorm() > getEpsilon())
            {
              clipped_points[num_clipped_points_] = tmp;
              num_clipped_points_++;
            }
          }
          else
          {
            clipped_points[num_clipped_points_] = tmp;
            num_clipped_points_++;
          }
        }
      }
    }

    prev_classify = classify;
  }

  if(num_clipped_points_ > 2)
  {
    if((clipped_points[0] - clipped_points[num_clipped_points_ - 1]).squaredNorm() < getEpsilon())
    {
      num_clipped_points_--;
    }
  }

  *num_clipped_points = num_clipped_points_;
}

//==============================================================================
template <typename S>
void Intersect<S>::clipSegmentByPlane(const Vector3<S>& v1, const Vector3<S>& v2, const Vector3<S>& n, S t, Vector3<S>* clipped_point)
{
  S dist1 = distanceToPlane(n, t, v1);
  Vector3<S> tmp = v2 - v1;
  S dist2 = tmp.dot(n);
  *clipped_point = tmp * (-dist1 / dist2) + v1;
}

//==============================================================================
template <typename S>
S Intersect<S>::distanceToPlane(const Vector3<S>& n, S t, const Vector3<S>& v)
{
  return n.dot(v) - t;
}

//==============================================================================
template <typename S>
bool Intersect<S>::buildTrianglePlane(const Vector3<S>& v1, const Vector3<S>& v2, const Vector3<S>& v3, Vector3<S>* n, S* t)
{
  Vector3<S> n_ = (v2 - v1).cross(v3 - v1);
  bool can_normalize = false;
  normalize(n_, &can_normalize);
  if(can_normalize)
  {
    *n = n_;
    *t = n_.dot(v1);
    return true;
  }

  return false;
}

//==============================================================================
template <typename S>
bool Intersect<S>::buildEdgePlane(const Vector3<S>& v1, const Vector3<S>& v2, const Vector3<S>& tn, Vector3<S>* n, S* t)
{
  Vector3<S> n_ = (v2 - v1).cross(tn);
  bool can_normalize = false;
  normalize(n_, &can_normalize);
  if(can_normalize)
  {
    *n = n_;
    *t = n_.dot(v1);
    return true;
  }

  return false;
}

//==============================================================================
template <typename S>
bool Intersect<S>::sameSideOfPlane(const Vector3<S>& v1, const Vector3<S>& v2, const Vector3<S>& v3, const Vector3<S>& n, S t)
{
  S dist1 = distanceToPlane(n, t, v1);
  S dist2 = dist1 * distanceToPlane(n, t, v2);
  S dist3 = dist1 * distanceToPlane(n, t, v3);
  if((dist2 > 0) && (dist3 > 0))
    return true;
  return false;
}

//==============================================================================
template <typename S>
int Intersect<S>::project6(const Vector3<S>& ax,
                        const Vector3<S>& p1, const Vector3<S>& p2, const Vector3<S>& p3,
                        const Vector3<S>& q1, const Vector3<S>& q2, const Vector3<S>& q3)
{
  S P1 = ax.dot(p1);
  S P2 = ax.dot(p2);
  S P3 = ax.dot(p3);
  S Q1 = ax.dot(q1);
  S Q2 = ax.dot(q2);
  S Q3 = ax.dot(q3);

  S mn1 = std::min(P1, std::min(P2, P3));
  S mx2 = std::max(Q1, std::max(Q2, Q3));
  if(mn1 > mx2) return 0;

  S mx1 = std::max(P1, std::max(P2, P3));
  S mn2 = std::min(Q1, std::min(Q2, Q3));

  if(mn2 > mx1) return 0;
  return 1;
}

//==============================================================================
template <typename S>
S Intersect<S>::gaussianCDF(S x)
{
  return 0.5 * std::erfc(-x / sqrt(2.0));
}

//==============================================================================
template <typename S>
constexpr S Intersect<S>::getEpsilon()
{
  return 1e-5;
}

//==============================================================================
template <typename S>
constexpr S Intersect<S>::getNearZeroThreshold()
{
  return 1e-7;
}

//==============================================================================
template <typename S>
constexpr S Intersect<S>::getCcdResolution()
{
  return 1e-7;
}

//==============================================================================
template <typename S>
constexpr unsigned int Intersect<S>::getMaxTriangleClips()
{
  return 8;
}

} // namespace detail
} // namespace fcl

#endif
