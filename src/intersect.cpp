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

#include "fcl/intersect.h"
#include <iostream>
#include <limits>
#include <vector>

namespace fcl
{
const FCL_REAL PolySolver::NEAR_ZERO_THRESHOLD = 1e-9;


bool PolySolver::isZero(FCL_REAL v)
{
  return (v < NEAR_ZERO_THRESHOLD) && (v > -NEAR_ZERO_THRESHOLD);
}

bool PolySolver::cbrt(FCL_REAL v)
{
  return powf(v, 1.0 / 3.0);
}

int PolySolver::solveLinear(FCL_REAL c[2], FCL_REAL s[1])
{
  if(isZero(c[1]))
    return 0;
  s[0] = - c[0] / c[1];
  return 1;
}

int PolySolver::solveQuadric(FCL_REAL c[3], FCL_REAL s[2])
{
  FCL_REAL p, q, D;

  // make sure we have a d2 equation

  if(isZero(c[2]))
    return solveLinear(c, s);

  // normal for: x^2 + px + q
  p = c[1] / (2.0 * c[2]);
  q = c[0] / c[2];
  D = p * p - q;

  if(isZero(D))
  {
    // one FCL_REAL root
    s[0] = s[1] = -p;
    return 1;
  }

  if(D < 0.0)
    // no real root
    return 0;
  else
  {
    // two real roots
    FCL_REAL sqrt_D = sqrt(D);
    s[0] = sqrt_D - p;
    s[1] = -sqrt_D - p;
    return 2;
  }
}

int PolySolver::solveCubic(FCL_REAL c[4], FCL_REAL s[3])
{
  int i, num;
  FCL_REAL sub, A, B, C, sq_A, p, q, cb_p, D;
  const FCL_REAL ONE_OVER_THREE = 1 / 3.0;
  const FCL_REAL PI = 3.14159265358979323846;

  // make sure we have a d2 equation
  if(isZero(c[3]))
    return solveQuadric(c, s);

  // normalize the equation:x ^ 3 + Ax ^ 2 + Bx  + C = 0
  A = c[2] / c[3];
  B = c[1] / c[3];
  C = c[0] / c[3];

  // substitute x = y - A / 3 to eliminate the quadratic term: x^3 + px + q = 0
  sq_A = A * A;
  p = (-ONE_OVER_THREE * sq_A + B) * ONE_OVER_THREE;
  q = 0.5 * (2.0 / 27.0 * A * sq_A - ONE_OVER_THREE * A * B + C);

  // use Cardano's formula
  cb_p = p * p * p;
  D = q * q + cb_p;

  if(isZero(D))
  {
    if(isZero(q))
    {
      // one triple solution
      s[0] = 0.0;
      num = 1;
    }
    else
    {
      // one single and one FCL_REAL solution
      FCL_REAL u = cbrt(-q);
      s[0] = 2.0 * u;
      s[1] = -u;
      num = 2;
    }
  }
  else
  {
    if(D < 0.0)
    {
      // three real solutions
      FCL_REAL phi = ONE_OVER_THREE * acos(-q / sqrt(-cb_p));
      FCL_REAL t = 2.0 * sqrt(-p);
      s[0] = t * cos(phi);
      s[1] = -t * cos(phi + PI / 3.0);
      s[2] = -t * cos(phi - PI / 3.0);
      num = 3;
    }
    else
    {
      // one real solution
      FCL_REAL sqrt_D = sqrt(D);
      FCL_REAL u = cbrt(sqrt_D + fabs(q));
      if(q > 0.0)
        s[0] = - u + p / u ;
      else
        s[0] = u - p / u;
      num = 1;
    }
  }

  // re-substitute
  sub = ONE_OVER_THREE * A;
  for(i = 0; i < num; i++)
    s[i] -= sub;
  return num;
}



const FCL_REAL Intersect::EPSILON = 1e-5;
const FCL_REAL Intersect::NEAR_ZERO_THRESHOLD = 1e-7;
const FCL_REAL Intersect::CCD_RESOLUTION = 1e-7;


bool Intersect::isZero(FCL_REAL v)
{
  return (v < NEAR_ZERO_THRESHOLD) && (v > -NEAR_ZERO_THRESHOLD);
}

/// @brief data: only used for EE, return the intersect point
bool Intersect::solveCubicWithIntervalNewton(const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& d0,
                                             const Vec3f& va, const Vec3f& vb, const Vec3f& vc, const Vec3f& vd,
                                             FCL_REAL& l, FCL_REAL& r, bool bVF, FCL_REAL coeffs[], Vec3f* data)
{
  FCL_REAL v2[2]= {l*l,r*r};
  FCL_REAL v[2]= {l,r};
  FCL_REAL r_backup;

  unsigned char min3, min2, min1, max3, max2, max1;

  min3= *((unsigned char*)&coeffs[3]+7)>>7; max3=min3^1;
  min2= *((unsigned char*)&coeffs[2]+7)>>7; max2=min2^1;
  min1= *((unsigned char*)&coeffs[1]+7)>>7; max1=min1^1;

  // bound the cubic

  FCL_REAL minor = coeffs[3]*v2[min3]*v[min3]+coeffs[2]*v2[min2]+coeffs[1]*v[min1]+coeffs[0];
  FCL_REAL major = coeffs[3]*v2[max3]*v[max3]+coeffs[2]*v2[max2]+coeffs[1]*v[max1]+coeffs[0];

  if(major<0) return false;
  if(minor>0) return false;

  // starting here, the bounds have opposite values
  FCL_REAL m = 0.5 * (r + l);

  // bound the derivative
  FCL_REAL dminor = 3.0*coeffs[3]*v2[min3]+2.0*coeffs[2]*v[min2]+coeffs[1];
  FCL_REAL dmajor = 3.0*coeffs[3]*v2[max3]+2.0*coeffs[2]*v[max2]+coeffs[1];

  if((dminor > 0)||(dmajor < 0)) // we can use Newton
  {
    FCL_REAL m2 = m*m;
    FCL_REAL fm = coeffs[3]*m2*m+coeffs[2]*m2+coeffs[1]*m+coeffs[0];
    FCL_REAL nl = m;
    FCL_REAL nu = m;
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
  if((r-l)< CCD_RESOLUTION)
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



bool Intersect::insideTriangle(const Vec3f& a, const Vec3f& b, const Vec3f& c, const Vec3f&p)
{
  Vec3f ab = b - a;
  Vec3f ac = c - a;
  Vec3f n = ab.cross(ac);

  Vec3f pa = a - p;
  Vec3f pb = b - p;
  Vec3f pc = c - p;

  if((pb.cross(pc)).dot(n) < -EPSILON) return false;
  if((pc.cross(pa)).dot(n) < -EPSILON) return false;
  if((pa.cross(pb)).dot(n) < -EPSILON) return false;

  return true;
}

bool Intersect::insideLineSegment(const Vec3f& a, const Vec3f& b, const Vec3f& p)
{
  return (p - a).dot(p - b) <= 0;
}

/// @brief Calculate the line segment papb that is the shortest route between
/// two lines p1p2 and p3p4. Calculate also the values of mua and mub where
///    pa = p1 + mua (p2 - p1)
///    pb = p3 + mub (p4 - p3)
/// Return FALSE if no solution exists.
bool Intersect::linelineIntersect(const Vec3f& p1, const Vec3f& p2, const Vec3f& p3, const Vec3f& p4,
                                  Vec3f* pa, Vec3f* pb, FCL_REAL* mua, FCL_REAL* mub)
{
  Vec3f p31 = p1 - p3;
  Vec3f p34 = p4 - p3;
  if(fabs(p34[0]) < EPSILON && fabs(p34[1]) < EPSILON && fabs(p34[2]) < EPSILON)
    return false;

  Vec3f p12 = p2 - p1;
  if(fabs(p12[0]) < EPSILON && fabs(p12[1]) < EPSILON && fabs(p12[2]) < EPSILON)
    return false;

  FCL_REAL d3134 = p31.dot(p34);
  FCL_REAL d3412 = p34.dot(p12);
  FCL_REAL d3112 = p31.dot(p12);
  FCL_REAL d3434 = p34.dot(p34);
  FCL_REAL d1212 = p12.dot(p12);

  FCL_REAL denom = d1212 * d3434 - d3412 * d3412;
  if(fabs(denom) < EPSILON)
    return false;
  FCL_REAL numer = d3134 * d3412 - d3112 * d3434;

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

bool Intersect::checkRootValidity_VF(const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& p0,
                                     const Vec3f& va, const Vec3f& vb, const Vec3f& vc, const Vec3f& vp,
                                     FCL_REAL t)
{
  return insideTriangle(a0 + va * t, b0 + vb * t, c0 + vc * t, p0 + vp * t);
}

bool Intersect::checkRootValidity_EE(const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& d0,
                                     const Vec3f& va, const Vec3f& vb, const Vec3f& vc, const Vec3f& vd,
                                     FCL_REAL t, Vec3f* q_i)
{
  Vec3f a = a0 + va * t;
  Vec3f b = b0 + vb * t;
  Vec3f c = c0 + vc * t;
  Vec3f d = d0 + vd * t;
  Vec3f p1, p2;
  FCL_REAL t_ab, t_cd;
  if(linelineIntersect(a, b, c, d, &p1, &p2, &t_ab, &t_cd))
  {
    if(q_i) *q_i = p1;
    return true;
  }

  return false;
}

bool Intersect::checkRootValidity_VE(const Vec3f& a0, const Vec3f& b0, const Vec3f& p0,
                                     const Vec3f& va, const Vec3f& vb, const Vec3f& vp,
                                     FCL_REAL t)
{
  return insideLineSegment(a0 + va * t, b0 + vb * t, p0 + vp * t);
}

bool Intersect::solveSquare(FCL_REAL a, FCL_REAL b, FCL_REAL c,
                            const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& d0,
                            const Vec3f& va, const Vec3f& vb, const Vec3f& vc, const Vec3f& vd,
                            bool bVF,
                            FCL_REAL* ret)
{
  FCL_REAL discriminant = b * b - 4 * a * c;
  if(discriminant < 0)
    return false;

  FCL_REAL sqrt_dis = sqrt(discriminant);
  FCL_REAL r1 = (-b + sqrt_dis) / (2 * a);
  bool v1 = (r1 >= 0.0 && r1 <= 1.0) ? ((bVF) ? checkRootValidity_VF(a0, b0, c0, d0, va, vb, vc, vd, r1) : checkRootValidity_EE(a0, b0, c0, d0, va, vb, vc, vd, r1)) : false;

  FCL_REAL r2 = (-b - sqrt_dis) / (2 * a);
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

bool Intersect::solveSquare(FCL_REAL a, FCL_REAL b, FCL_REAL c,
                            const Vec3f& a0, const Vec3f& b0, const Vec3f& p0,
                            const Vec3f& va, const Vec3f& vb, const Vec3f& vp)
{
  if(isZero(a))
  {
    FCL_REAL t = -c/b;
    return (t >= 0 && t <= 1) ? checkRootValidity_VE(a0, b0, p0, va, vb, vp, t) : false;
  }

  FCL_REAL discriminant = b*b-4*a*c;
  if(discriminant < 0)
    return false;

  FCL_REAL sqrt_dis = sqrt(discriminant);

  FCL_REAL r1 = (-b+sqrt_dis) / (2 * a);
  bool v1 = (r1 >= 0.0 && r1 <= 1.0) ? checkRootValidity_VE(a0, b0, p0, va, vb, vp, r1) : false;
  if(v1) return true;

  FCL_REAL r2 = (-b-sqrt_dis) / (2 * a);
  bool v2 = (r2 >= 0.0 && r2 <= 1.0) ? checkRootValidity_VE(a0, b0, p0, va, vb, vp, r2) : false;
  return v2;
}



/// @brief Compute the cubic coefficients for VF case
/// See Paper "Interactive Continuous Collision Detection between Deformable Models using Connectivity-Based Culling", Equation 1.
void Intersect::computeCubicCoeff_VF(const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& p0,
                                     const Vec3f& va, const Vec3f& vb, const Vec3f& vc, const Vec3f& vp,
                                     FCL_REAL* a, FCL_REAL* b, FCL_REAL* c, FCL_REAL* d)
{
  Vec3f vavb = vb - va;
  Vec3f vavc = vc - va;
  Vec3f vavp = vp - va;
  Vec3f a0b0 = b0 - a0;
  Vec3f a0c0 = c0 - a0;
  Vec3f a0p0 = p0 - a0;

  Vec3f vavb_cross_vavc = vavb.cross(vavc);
  Vec3f vavb_cross_a0c0 = vavb.cross(a0c0);
  Vec3f a0b0_cross_vavc = a0b0.cross(vavc);
  Vec3f a0b0_cross_a0c0 = a0b0.cross(a0c0);

  *a = vavp.dot(vavb_cross_vavc);
  *b = a0p0.dot(vavb_cross_vavc) + vavp.dot(vavb_cross_a0c0 + a0b0_cross_vavc);
  *c = vavp.dot(a0b0_cross_a0c0) + a0p0.dot(vavb_cross_a0c0 + a0b0_cross_vavc);
  *d = a0p0.dot(a0b0_cross_a0c0);
}

void Intersect::computeCubicCoeff_EE(const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& d0,
                                     const Vec3f& va, const Vec3f& vb, const Vec3f& vc, const Vec3f& vd,
                                     FCL_REAL* a, FCL_REAL* b, FCL_REAL* c, FCL_REAL* d)
{
  Vec3f vavb = vb - va;
  Vec3f vcvd = vd - vc;
  Vec3f vavc = vc - va;
  Vec3f c0d0 = d0 - c0;
  Vec3f a0b0 = b0 - a0;
  Vec3f a0c0 = c0 - a0;
  Vec3f vavb_cross_vcvd = vavb.cross(vcvd);
  Vec3f vavb_cross_c0d0 = vavb.cross(c0d0);
  Vec3f a0b0_cross_vcvd = a0b0.cross(vcvd);
  Vec3f a0b0_cross_c0d0 = a0b0.cross(c0d0);

  *a = vavc.dot(vavb_cross_vcvd);
  *b = a0c0.dot(vavb_cross_vcvd) + vavc.dot(vavb_cross_c0d0 + a0b0_cross_vcvd);
  *c = vavc.dot(a0b0_cross_c0d0) + a0c0.dot(vavb_cross_c0d0 + a0b0_cross_vcvd);
  *d = a0c0.dot(a0b0_cross_c0d0);
}

void Intersect::computeCubicCoeff_VE(const Vec3f& a0, const Vec3f& b0, const Vec3f& p0,
                                     const Vec3f& va, const Vec3f& vb, const Vec3f& vp,
                                     const Vec3f& L,
                                     FCL_REAL* a, FCL_REAL* b, FCL_REAL* c)
{
  Vec3f vbva = va - vb;
  Vec3f vbvp = vp - vb;
  Vec3f b0a0 = a0 - b0;
  Vec3f b0p0 = p0 - b0;

  Vec3f L_cross_vbvp = L.cross(vbvp);
  Vec3f L_cross_b0p0 = L.cross(b0p0);

  *a = L_cross_vbvp.dot(vbva);
  *b = L_cross_vbvp.dot(b0a0) + L_cross_b0p0.dot(vbva);
  *c = L_cross_b0p0.dot(b0a0);
}


bool Intersect::intersect_VF(const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& p0,
                             const Vec3f& a1, const Vec3f& b1, const Vec3f& c1, const Vec3f& p1,
                             FCL_REAL* collision_time, Vec3f* p_i, bool useNewton)
{
  *collision_time = 2.0;

  Vec3f vp, va, vb, vc;
  vp = p1 - p0;
  va = a1 - a0;
  vb = b1 - b0;
  vc = c1 - c0;

  FCL_REAL a, b, c, d;
  computeCubicCoeff_VF(a0, b0, c0, p0, va, vb, vc, vp, &a, &b, &c, &d);

  if(isZero(a) && isZero(b) && isZero(c) && isZero(d))
  {
    return false;
  }


  /// if(isZero(a))
  /// {
  ///   return solveSquare(b, c, d, a0, b0, c0, p0, va, vb, vc, vp, true, collision_time);
  /// }

  FCL_REAL coeffs[4];
  coeffs[3] = a, coeffs[2] = b, coeffs[1] = c, coeffs[0] = d;

  if(useNewton)
  {
    FCL_REAL l = 0;
    FCL_REAL r = 1;

    if(solveCubicWithIntervalNewton(a0, b0, c0, p0, va, vb, vc, vp, l, r, true, coeffs))
    {
      *collision_time = 0.5 * (l + r);
    }
  }
  else
  {
    FCL_REAL roots[3];
    int num = PolySolver::solveCubic(coeffs, roots);
    for(int i = 0; i < num; ++i)
    {
      FCL_REAL r = roots[i];
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

bool Intersect::intersect_EE(const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& d0,
                             const Vec3f& a1, const Vec3f& b1, const Vec3f& c1, const Vec3f& d1,
                             FCL_REAL* collision_time, Vec3f* p_i, bool useNewton)
{
  *collision_time = 2.0;

  Vec3f va, vb, vc, vd;
  va = a1 - a0;
  vb = b1 - b0;
  vc = c1 - c0;
  vd = d1 - d0;

  FCL_REAL a, b, c, d;
  computeCubicCoeff_EE(a0, b0, c0, d0, va, vb, vc, vd, &a, &b, &c, &d);

  if(isZero(a) && isZero(b) && isZero(c) && isZero(d))
  {
    return false;
  }
  
  /// if(isZero(a))
  /// {
  ///   return solveSquare(b, c, d, a0, b0, c0, d0, va, vb, vc, vd, collision_time, false);
  /// }
 

  FCL_REAL coeffs[4];
  coeffs[3] = a, coeffs[2] = b, coeffs[1] = c, coeffs[0] = d;

  if(useNewton)
  {
    FCL_REAL l = 0;
    FCL_REAL r = 1;

    if(solveCubicWithIntervalNewton(a0, b0, c0, d0, va, vb, vc, vd, l, r, false, coeffs, p_i))
    {
      *collision_time  = (l + r) * 0.5;
    }
  }
  else
  {
    FCL_REAL roots[3];
    int num = PolySolver::solveCubic(coeffs, roots);
    for(int i = 0; i < num; ++i)
    {
      FCL_REAL r = roots[i];
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


bool Intersect::intersect_VE(const Vec3f& a0, const Vec3f& b0, const Vec3f& p0,
                             const Vec3f& a1, const Vec3f& b1, const Vec3f& p1,
                             const Vec3f& L)
{
  Vec3f va, vb, vp;
  va = a1 - a0;
  vb = b1 - b0;
  vp = p1 - p0;

  FCL_REAL a, b, c;
  computeCubicCoeff_VE(a0, b0, p0, va, vb, vp, L, &a, &b, &c);

  if(isZero(a) && isZero(b) && isZero(c))
    return true;

  return solveSquare(a, b, c, a0, b0, p0, va, vb, vp);

}


/// @brief Prefilter for intersection, works for both VF and EE
bool Intersect::intersectPreFiltering(const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& d0,
                                      const Vec3f& a1, const Vec3f& b1, const Vec3f& c1, const Vec3f& d1)
{
  Vec3f n0 = (b0 - a0).cross(c0 - a0);
  Vec3f n1 = (b1 - a1).cross(c1 - a1);
  Vec3f a0a1 = a1 - a0;
  Vec3f b0b1 = b1 - b0;
  Vec3f c0c1 = c1 - c0;
  Vec3f delta = (b0b1 - a0a1).cross(c0c1 - a0a1);
  Vec3f nx = (n0 + n1 - delta) * 0.5;

  Vec3f a0d0 = d0 - a0;
  Vec3f a1d1 = d1 - a1;

  FCL_REAL A = n0.dot(a0d0);
  FCL_REAL B = n1.dot(a1d1);
  FCL_REAL C = nx.dot(a0d0);
  FCL_REAL D = nx.dot(a1d1);
  FCL_REAL E = n1.dot(a0d0);
  FCL_REAL F = n0.dot(a1d1);

  if(A > 0 && B > 0 && (2*C +F) > 0 && (2*D+E) > 0)
    return false;
  if(A < 0 && B < 0 && (2*C +F) < 0 && (2*D+E) < 0)
    return false;

  return true;
}

bool Intersect::intersect_VF_filtered(const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& p0,
                                      const Vec3f& a1, const Vec3f& b1, const Vec3f& c1, const Vec3f& p1,
                                      FCL_REAL* collision_time, Vec3f* p_i, bool useNewton)
{
  if(intersectPreFiltering(a0, b0, c0, p0, a1, b1, c1, p1))
  {
    return intersect_VF(a0, b0, c0, p0, a1, b1, c1, p1, collision_time, p_i, useNewton);
  }
  else
    return false;
}

bool Intersect::intersect_EE_filtered(const Vec3f& a0, const Vec3f& b0, const Vec3f& c0, const Vec3f& d0,
                                      const Vec3f& a1, const Vec3f& b1, const Vec3f& c1, const Vec3f& d1,
                                      FCL_REAL* collision_time, Vec3f* p_i, bool useNewton)
{
  if(intersectPreFiltering(a0, b0, c0, d0, a1, b1, c1, d1))
  {
    return intersect_EE(a0, b0, c0, d0, a1, b1, c1, d1, collision_time, p_i, useNewton);
  }
  else
    return false;
}

bool Intersect::intersect_Triangle(const Vec3f& P1, const Vec3f& P2, const Vec3f& P3,
                                   const Vec3f& Q1, const Vec3f& Q2, const Vec3f& Q3,
                                   const Matrix3f& R, const Vec3f& T,
                                   Vec3f* contact_points,
                                   unsigned int* num_contact_points,
                                   FCL_REAL* penetration_depth,
                                   Vec3f* normal)
{
  Vec3f Q1_ = R * Q1 + T;
  Vec3f Q2_ = R * Q2 + T;
  Vec3f Q3_ = R * Q3 + T;

  return intersect_Triangle(P1, P2, P3, Q1_, Q2_, Q3_, contact_points, num_contact_points, penetration_depth, normal);
}

bool Intersect::intersect_Triangle(const Vec3f& P1, const Vec3f& P2, const Vec3f& P3,
                                   const Vec3f& Q1, const Vec3f& Q2, const Vec3f& Q3,
                                   const Transform3f& tf,
                                   Vec3f* contact_points,
                                   unsigned int* num_contact_points,
                                   FCL_REAL* penetration_depth,
                                   Vec3f* normal)
{
  Vec3f Q1_ = tf.transform(Q1);
  Vec3f Q2_ = tf.transform(Q2);
  Vec3f Q3_ = tf.transform(Q3);

  return intersect_Triangle(P1, P2, P3, Q1_, Q2_, Q3_, contact_points, num_contact_points, penetration_depth, normal);
}


#if ODE_STYLE
bool Intersect::intersect_Triangle(const Vec3f& P1, const Vec3f& P2, const Vec3f& P3,
                                   const Vec3f& Q1, const Vec3f& Q2, const Vec3f& Q3,
                                   Vec3f* contact_points,
                                   unsigned int* num_contact_points,
                                   FCL_REAL* penetration_depth,
                                   Vec3f* normal)
{


  Vec3f n1;
  FCL_REAL t1;
  bool b1 = buildTrianglePlane(P1, P2, P3, &n1, &t1);
  if(!b1) return false;

  Vec3f n2;
  FCL_REAL t2;
  bool b2 = buildTrianglePlane(Q1, Q2, Q3, &n2, &t2);
  if(!b2) return false;

  if(sameSideOfPlane(P1, P2, P3, n2, t2))
    return false;

  if(sameSideOfPlane(Q1, Q2, Q3, n1, t1))
    return false;

  Vec3f clipped_points1[MAX_TRIANGLE_CLIPS];
  unsigned int num_clipped_points1 = 0;
  Vec3f clipped_points2[MAX_TRIANGLE_CLIPS];
  unsigned int num_clipped_points2 = 0;

  Vec3f deepest_points1[MAX_TRIANGLE_CLIPS];
  unsigned int num_deepest_points1 = 0;
  Vec3f deepest_points2[MAX_TRIANGLE_CLIPS];
  unsigned int num_deepest_points2 = 0;
  FCL_REAL penetration_depth1 = -1, penetration_depth2 = -1;

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
#else
bool Intersect::intersect_Triangle(const Vec3f& P1, const Vec3f& P2, const Vec3f& P3,
                                   const Vec3f& Q1, const Vec3f& Q2, const Vec3f& Q3,
                                   Vec3f* contact_points,
                                   unsigned int* num_contact_points,
                                   FCL_REAL* penetration_depth,
                                   Vec3f* normal)
{
  Vec3f p1 = P1 - P1;
  Vec3f p2 = P2 - P1;
  Vec3f p3 = P3 - P1;
  Vec3f q1 = Q1 - P1;
  Vec3f q2 = Q2 - P1;
  Vec3f q3 = Q3 - P1;

  Vec3f e1 = p2 - p1;
  Vec3f e2 = p3 - p2;
  Vec3f n1 = e1.cross(e2);
  if (!project6(n1, p1, p2, p3, q1, q2, q3)) return false;

  Vec3f f1 = q2 - q1;
  Vec3f f2 = q3 - q2;
  Vec3f m1 = f1.cross(f2);
  if (!project6(m1, p1, p2, p3, q1, q2, q3)) return false;

  Vec3f ef11 = e1.cross(f1);
  if (!project6(ef11, p1, p2, p3, q1, q2, q3)) return false;

  Vec3f ef12 = e1.cross(f2);
  if (!project6(ef12, p1, p2, p3, q1, q2, q3)) return false;

  Vec3f f3 = q1 - q3;
  Vec3f ef13 = e1.cross(f3);
  if (!project6(ef13, p1, p2, p3, q1, q2, q3)) return false;

  Vec3f ef21 = e2.cross(f1);
  if (!project6(ef21, p1, p2, p3, q1, q2, q3)) return false;

  Vec3f ef22 = e2.cross(f2);
  if (!project6(ef22, p1, p2, p3, q1, q2, q3)) return false;

  Vec3f ef23 = e2.cross(f3);
  if (!project6(ef23, p1, p2, p3, q1, q2, q3)) return false;

  Vec3f e3 = p1 - p3;
  Vec3f ef31 = e3.cross(f1);
  if (!project6(ef31, p1, p2, p3, q1, q2, q3)) return false;

  Vec3f ef32 = e3.cross(f2);
  if (!project6(ef32, p1, p2, p3, q1, q2, q3)) return false;

  Vec3f ef33 = e3.cross(f3);
  if (!project6(ef33, p1, p2, p3, q1, q2, q3)) return false;

  Vec3f g1 = e1.cross(n1);
  if (!project6(g1, p1, p2, p3, q1, q2, q3)) return false;

  Vec3f g2 = e2.cross(n1);
  if (!project6(g2, p1, p2, p3, q1, q2, q3)) return false;

  Vec3f g3 = e3.cross(n1);
  if (!project6(g3, p1, p2, p3, q1, q2, q3)) return false;

  Vec3f h1 = f1.cross(m1);
  if (!project6(h1, p1, p2, p3, q1, q2, q3)) return false;

  Vec3f h2 = f2.cross(m1);
  if (!project6(h2, p1, p2, p3, q1, q2, q3)) return false;

  Vec3f h3 = f3.cross(m1);
  if (!project6(h3, p1, p2, p3, q1, q2, q3)) return false;

  if(contact_points && num_contact_points && penetration_depth && normal)
  {
    Vec3f n1, n2;
    FCL_REAL t1, t2;
    buildTrianglePlane(P1, P2, P3, &n1, &t1);
    buildTrianglePlane(Q1, Q2, Q3, &n2, &t2);

    Vec3f deepest_points1[3];
    unsigned int num_deepest_points1 = 0;
    Vec3f deepest_points2[3];
    unsigned int num_deepest_points2 = 0;
    FCL_REAL penetration_depth1, penetration_depth2;

    Vec3f P[3] = {P1, P2, P3};
    Vec3f Q[3] = {Q1, Q2, Q3};

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
#endif


void Intersect::computeDeepestPoints(Vec3f* clipped_points, unsigned int num_clipped_points, const Vec3f& n, FCL_REAL t, FCL_REAL* penetration_depth, Vec3f* deepest_points, unsigned int* num_deepest_points)
{
  *num_deepest_points = 0;
  FCL_REAL max_depth = -std::numeric_limits<FCL_REAL>::max();
  unsigned int num_deepest_points_ = 0;
  unsigned int num_neg = 0;
  unsigned int num_pos = 0;
  unsigned int num_zero = 0;

  for(unsigned int i = 0; i < num_clipped_points; ++i)
  {
    FCL_REAL dist = -distanceToPlane(n, t, clipped_points[i]);
    if(dist > EPSILON) num_pos++;
    else if(dist < -EPSILON) num_neg++;
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

  if(max_depth < -EPSILON)
    num_deepest_points_ = 0;

  if(num_zero == 0 && ((num_neg == 0) || (num_pos == 0)))
    num_deepest_points_ = 0;

  *penetration_depth = max_depth;
  *num_deepest_points = num_deepest_points_;
}

void Intersect::clipTriangleByTriangleAndEdgePlanes(const Vec3f& v1, const Vec3f& v2, const Vec3f& v3,
                                                    const Vec3f& t1, const Vec3f& t2, const Vec3f& t3,
                                                    const Vec3f& tn, FCL_REAL to,
                                                    Vec3f clipped_points[], unsigned int* num_clipped_points,
                                                    bool clip_triangle)
{
  *num_clipped_points = 0;
  Vec3f temp_clip[MAX_TRIANGLE_CLIPS];
  Vec3f temp_clip2[MAX_TRIANGLE_CLIPS];
  unsigned int num_temp_clip = 0;
  unsigned int num_temp_clip2 = 0;
  Vec3f v[3] = {v1, v2, v3};

  Vec3f plane_n;
  FCL_REAL plane_dist;

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

void Intersect::clipPolygonByPlane(Vec3f* polygon_points, unsigned int num_polygon_points, const Vec3f& n, FCL_REAL t, Vec3f clipped_points[], unsigned int* num_clipped_points)
{
  *num_clipped_points = 0;

  unsigned int num_clipped_points_ = 0;
  unsigned int vi;
  unsigned int prev_classify = 2;
  unsigned int classify;
  for(unsigned int i = 0; i <= num_polygon_points; ++i)
  {
    vi = (i % num_polygon_points);
    FCL_REAL d = distanceToPlane(n, t, polygon_points[i]);
    classify = ((d > EPSILON) ? 1 : 0);
    if(classify == 0)
    {
      if(prev_classify == 1)
      {
        if(num_clipped_points_ < MAX_TRIANGLE_CLIPS)
        {
          Vec3f tmp;
          clipSegmentByPlane(polygon_points[i - 1], polygon_points[vi], n, t, &tmp);
          if(num_clipped_points_ > 0)
          {
            if((tmp - clipped_points[num_clipped_points_ - 1]).sqrLength() > EPSILON)
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

      if(num_clipped_points_ < MAX_TRIANGLE_CLIPS && i < num_polygon_points)
      {
        clipped_points[num_clipped_points_] = polygon_points[vi];
        num_clipped_points_++;
      }
    }
    else
    {
      if(prev_classify == 0)
      {
        if(num_clipped_points_ < MAX_TRIANGLE_CLIPS)
        {
          Vec3f tmp;
          clipSegmentByPlane(polygon_points[i - 1], polygon_points[vi], n, t, &tmp);
          if(num_clipped_points_ > 0)
          {
            if((tmp - clipped_points[num_clipped_points_ - 1]).sqrLength() > EPSILON)
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
    if((clipped_points[0] - clipped_points[num_clipped_points_ - 1]).sqrLength() < EPSILON)
    {
      num_clipped_points_--;
    }
  }

  *num_clipped_points = num_clipped_points_;
}

void Intersect::clipSegmentByPlane(const Vec3f& v1, const Vec3f& v2, const Vec3f& n, FCL_REAL t, Vec3f* clipped_point)
{
  FCL_REAL dist1 = distanceToPlane(n, t, v1);
  Vec3f tmp = v2 - v1;
  FCL_REAL dist2 = tmp.dot(n);
  *clipped_point = tmp * (-dist1 / dist2) + v1;
}

FCL_REAL Intersect::distanceToPlane(const Vec3f& n, FCL_REAL t, const Vec3f& v)
{
  return n.dot(v) - t;
}

bool Intersect::buildTrianglePlane(const Vec3f& v1, const Vec3f& v2, const Vec3f& v3, Vec3f* n, FCL_REAL* t)
{
  Vec3f n_ = (v2 - v1).cross(v3 - v1);
  bool can_normalize = false;
  n_.normalize(&can_normalize);
  if(can_normalize)
  {
    *n = n_;
    *t = n_.dot(v1);
    return true;
  }

  return false;
}

bool Intersect::buildEdgePlane(const Vec3f& v1, const Vec3f& v2, const Vec3f& tn, Vec3f* n, FCL_REAL* t)
{
  Vec3f n_ = (v2 - v1).cross(tn);
  bool can_normalize = false;
  n_.normalize(&can_normalize);
  if(can_normalize)
  {
    *n = n_;
    *t = n_.dot(v1);
    return true;
  }

  return false;
}

bool Intersect::sameSideOfPlane(const Vec3f& v1, const Vec3f& v2, const Vec3f& v3, const Vec3f& n, FCL_REAL t)
{
  FCL_REAL dist1 = distanceToPlane(n, t, v1);
  FCL_REAL dist2 = dist1 * distanceToPlane(n, t, v2);
  FCL_REAL dist3 = dist1 * distanceToPlane(n, t, v3);
  if((dist2 > 0) && (dist3 > 0))
    return true;
  return false;
}

int Intersect::project6(const Vec3f& ax,
                        const Vec3f& p1, const Vec3f& p2, const Vec3f& p3,
                        const Vec3f& q1, const Vec3f& q2, const Vec3f& q3)
{
  FCL_REAL P1 = ax.dot(p1);
  FCL_REAL P2 = ax.dot(p2);
  FCL_REAL P3 = ax.dot(p3);
  FCL_REAL Q1 = ax.dot(q1);
  FCL_REAL Q2 = ax.dot(q2);
  FCL_REAL Q3 = ax.dot(q3);

  FCL_REAL mn1 = std::min(P1, std::min(P2, P3));
  FCL_REAL mx2 = std::max(Q1, std::max(Q2, Q3));
  if(mn1 > mx2) return 0;

  FCL_REAL mx1 = std::max(P1, std::max(P2, P3));
  FCL_REAL mn2 = std::min(Q1, std::min(Q2, Q3));

  if(mn2 > mx1) return 0;
  return 1;
}



void TriangleDistance::segPoints(const Vec3f& P, const Vec3f& A, const Vec3f& Q, const Vec3f& B,
                                 Vec3f& VEC, Vec3f& X, Vec3f& Y)
{
  Vec3f T;
  FCL_REAL A_dot_A, B_dot_B, A_dot_B, A_dot_T, B_dot_T;
  Vec3f TMP;

  T = Q - P;
  A_dot_A = A.dot(A);
  B_dot_B = B.dot(B);
  A_dot_B = A.dot(B);
  A_dot_T = A.dot(T);
  B_dot_T = B.dot(T);

  // t parameterizes ray P,A
  // u parameterizes ray Q,B

  FCL_REAL t, u;

  // compute t for the closest point on ray P,A to
  // ray Q,B

  FCL_REAL denom = A_dot_A*B_dot_B - A_dot_B*A_dot_B;

  t = (A_dot_T*B_dot_B - B_dot_T*A_dot_B) / denom;

  // clamp result so t is on the segment P,A

  if((t < 0) || std::isnan(t)) t = 0; else if(t > 1) t = 1;

  // find u for point on ray Q,B closest to point at t

  u = (t*A_dot_B - B_dot_T) / B_dot_B;

  // if u is on segment Q,B, t and u correspond to
  // closest points, otherwise, clamp u, recompute and
  // clamp t

  if((u <= 0) || std::isnan(u))
  {
    Y = Q;

    t = A_dot_T / A_dot_A;

    if((t <= 0) || std::isnan(t))
    {
      X = P;
      VEC = Q - P;
    }
    else if(t >= 1)
    {
      X = P + A;
      VEC = Q - X;
    }
    else
    {
      X = P + A * t;
      TMP = T.cross(A);
      VEC = A.cross(TMP);
    }
  }
  else if (u >= 1)
  {
    Y = Q + B;

    t = (A_dot_B + A_dot_T) / A_dot_A;

    if((t <= 0) || std::isnan(t))
    {
      X = P;
      VEC = Y - P;
    }
    else if(t >= 1)
    {
      X = P + A;
      VEC = Y - X;
    }
    else
    {
      X = P + A * t;
      T = Y - P;
      TMP = T.cross(A);
      VEC= A.cross(TMP);
    }
  }
  else
  {
    Y = Q + B * u;

    if((t <= 0) || std::isnan(t))
    {
      X = P;
      TMP = T.cross(B);
      VEC = B.cross(TMP);
    }
    else if(t >= 1)
    {
      X = P + A;
      T = Q - X;
      TMP = T.cross(B);
      VEC = B.cross(TMP);
    }
    else
    {
      X = P + A * t;
      VEC = A.cross(B);
      if(VEC.dot(T) < 0)
      {
        VEC = VEC * (-1);
      }
    }
  }
}


FCL_REAL TriangleDistance::triDistance(const Vec3f S[3], const Vec3f T[3], Vec3f& P, Vec3f& Q)
{
  // Compute vectors along the 6 sides

  Vec3f Sv[3];
  Vec3f Tv[3];
  Vec3f VEC;

  Sv[0] = S[1] - S[0];
  Sv[1] = S[2] - S[1];
  Sv[2] = S[0] - S[2];

  Tv[0] = T[1] - T[0];
  Tv[1] = T[2] - T[1];
  Tv[2] = T[0] - T[2];

  // For each edge pair, the vector connecting the closest points
  // of the edges defines a slab (parallel planes at head and tail
  // enclose the slab). If we can show that the off-edge vertex of
  // each triangle is outside of the slab, then the closest points
  // of the edges are the closest points for the triangles.
  // Even if these tests fail, it may be helpful to know the closest
  // points found, and whether the triangles were shown disjoint

  Vec3f V, Z, minP, minQ;
  FCL_REAL mindd;
  int shown_disjoint = 0;

  mindd = (S[0] - T[0]).sqrLength() + 1; // Set first minimum safely high

  for(int i = 0; i < 3; ++i)
  {
    for(int j = 0; j < 3; ++j)
    {
      // Find closest points on edges i & j, plus the
      // vector (and distance squared) between these points
      segPoints(S[i], Sv[i], T[j], Tv[j], VEC, P, Q);

      V = Q - P;
      FCL_REAL dd = V.dot(V);

      // Verify this closest point pair only if the distance
      // squared is less than the minimum found thus far.

      if(dd <= mindd)
      {
        minP = P;
        minQ = Q;
        mindd = dd;

        Z = S[(i+2)%3] - P;
        FCL_REAL a = Z.dot(VEC);
        Z = T[(j+2)%3] - Q;
        FCL_REAL b = Z.dot(VEC);

        if((a <= 0) && (b >= 0)) return sqrt(dd);

        FCL_REAL p = V.dot(VEC);

        if(a < 0) a = 0;
        if(b > 0) b = 0;
        if((p - a + b) > 0) shown_disjoint = 1;
      }
    }
  }

  // No edge pairs contained the closest points.
  // either:
  // 1. one of the closest points is a vertex, and the
  //    other point is interior to a face.
  // 2. the triangles are overlapping.
  // 3. an edge of one triangle is parallel to the other's face. If
  //    cases 1 and 2 are not true, then the closest points from the 9
  //    edge pairs checks above can be taken as closest points for the
  //    triangles.
  // 4. possibly, the triangles were degenerate.  When the
  //    triangle points are nearly colinear or coincident, one
  //    of above tests might fail even though the edges tested
  //    contain the closest points.

  // First check for case 1

  Vec3f Sn;
  FCL_REAL Snl;

  Sn = Sv[0].cross(Sv[1]); // Compute normal to S triangle
  Snl = Sn.dot(Sn);        // Compute square of length of normal

  // If cross product is long enough,

  if(Snl > 1e-15)
  {
    // Get projection lengths of T points

    Vec3f Tp;

    V = S[0] - T[0];
    Tp[0] = V.dot(Sn);

    V = S[0] - T[1];
    Tp[1] = V.dot(Sn);

    V = S[0] - T[2];
    Tp[2] = V.dot(Sn);

    // If Sn is a separating direction,
    // find point with smallest projection

    int point = -1;
    if((Tp[0] > 0) && (Tp[1] > 0) && (Tp[2] > 0))
    {
      if(Tp[0] < Tp[1]) point = 0; else point = 1;
      if(Tp[2] < Tp[point]) point = 2;
    }
    else if((Tp[0] < 0) && (Tp[1] < 0) && (Tp[2] < 0))
    {
      if(Tp[0] > Tp[1]) point = 0; else point = 1;
      if(Tp[2] > Tp[point]) point = 2;
    }

    // If Sn is a separating direction,

    if(point >= 0)
    {
      shown_disjoint = 1;

      // Test whether the point found, when projected onto the
      // other triangle, lies within the face.

      V = T[point] - S[0];
      Z = Sn.cross(Sv[0]);
      if(V.dot(Z) > 0)
      {
        V = T[point] - S[1];
        Z = Sn.cross(Sv[1]);
        if(V.dot(Z) > 0)
        {
          V = T[point] - S[2];
          Z = Sn.cross(Sv[2]);
          if(V.dot(Z) > 0)
          {
            // T[point] passed the test - it's a closest point for
            // the T triangle; the other point is on the face of S
            P = T[point] + Sn * (Tp[point] / Snl);
            Q = T[point];
            return (P - Q).length();
          }
        }
      }
    }
  }

  Vec3f Tn;
  FCL_REAL Tnl;

  Tn = Tv[0].cross(Tv[1]);
  Tnl = Tn.dot(Tn);

  if(Tnl > 1e-15)
  {
    Vec3f Sp;

    V = T[0] - S[0];
    Sp[0] = V.dot(Tn);

    V = T[0] - S[1];
    Sp[1] = V.dot(Tn);

    V = T[0] - S[2];
    Sp[2] = V.dot(Tn);

    int point = -1;
    if((Sp[0] > 0) && (Sp[1] > 0) && (Sp[2] > 0))
    {
      if(Sp[0] < Sp[1]) point = 0; else point = 1;
      if(Sp[2] < Sp[point]) point = 2;
    }
    else if((Sp[0] < 0) && (Sp[1] < 0) && (Sp[2] < 0))
    {
      if(Sp[0] > Sp[1]) point = 0; else point = 1;
      if(Sp[2] > Sp[point]) point = 2;
    }

    if(point >= 0)
    {
      shown_disjoint = 1;

      V = S[point] - T[0];
      Z = Tn.cross(Tv[0]);
      if(V.dot(Z) > 0)
      {
        V = S[point] - T[1];
        Z = Tn.cross(Tv[1]);
        if(V.dot(Z) > 0)
        {
          V = S[point] - T[2];
          Z = Tn.cross(Tv[2]);
          if(V.dot(Z) > 0)
          {
            P = S[point];
            Q = S[point] + Tn * (Sp[point] / Tnl);
            return (P - Q).length();
          }
        }
      }
    }
  }

  // Case 1 can't be shown.
  // If one of these tests showed the triangles disjoint,
  // we assume case 3 or 4, otherwise we conclude case 2,
  // that the triangles overlap.

  if(shown_disjoint)
  {
    P = minP;
    Q = minQ;
    return sqrt(mindd);
  }
  else return 0;
}


FCL_REAL TriangleDistance::triDistance(const Vec3f& S1, const Vec3f& S2, const Vec3f& S3,
                                       const Vec3f& T1, const Vec3f& T2, const Vec3f& T3,
                                       Vec3f& P, Vec3f& Q)
{
  Vec3f S[3];
  Vec3f T[3];
  S[0] = S1; S[1] = S2; S[2] = S3;
  T[0] = T1; T[1] = T2; T[2] = T3;

  return triDistance(S, T, P, Q);
}

FCL_REAL TriangleDistance::triDistance(const Vec3f S[3], const Vec3f T[3],
                                       const Matrix3f& R, const Vec3f& Tl,
                                       Vec3f& P, Vec3f& Q)
{
  Vec3f T_transformed[3];
  T_transformed[0] = R * T[0] + Tl;
  T_transformed[1] = R * T[1] + Tl;
  T_transformed[2] = R * T[2] + Tl;

  return triDistance(S, T_transformed, P, Q);
}


FCL_REAL TriangleDistance::triDistance(const Vec3f S[3], const Vec3f T[3],
                                       const Transform3f& tf,
                                       Vec3f& P, Vec3f& Q)
{
  Vec3f T_transformed[3];
  T_transformed[0] = tf.transform(T[0]);
  T_transformed[1] = tf.transform(T[1]);
  T_transformed[2] = tf.transform(T[2]);

  return triDistance(S, T_transformed, P, Q);
}


FCL_REAL TriangleDistance::triDistance(const Vec3f& S1, const Vec3f& S2, const Vec3f& S3,
                                       const Vec3f& T1, const Vec3f& T2, const Vec3f& T3,
                                       const Matrix3f& R, const Vec3f& Tl,
                                       Vec3f& P, Vec3f& Q)
{
  Vec3f T1_transformed = R * T1 + Tl;
  Vec3f T2_transformed = R * T2 + Tl;
  Vec3f T3_transformed = R * T3 + Tl;
  return triDistance(S1, S2, S3, T1_transformed, T2_transformed, T3_transformed, P, Q);
}

FCL_REAL TriangleDistance::triDistance(const Vec3f& S1, const Vec3f& S2, const Vec3f& S3,
                                       const Vec3f& T1, const Vec3f& T2, const Vec3f& T3,
                                       const Transform3f& tf,
                                       Vec3f& P, Vec3f& Q)
{
  Vec3f T1_transformed = tf.transform(T1);
  Vec3f T2_transformed = tf.transform(T2);
  Vec3f T3_transformed = tf.transform(T3);
  return triDistance(S1, S2, S3, T1_transformed, T2_transformed, T3_transformed, P, Q);
}





Project::ProjectResult Project::projectLine(const Vec3f& a, const Vec3f& b, const Vec3f& p)
{
  ProjectResult res;

  const Vec3f d = b - a;
  const FCL_REAL l = d.sqrLength();

  if(l > 0)
  {
    const FCL_REAL t = (p - a).dot(d);
    res.parameterization[1] = (t >= l) ? 1 : ((t <= 0) ? 0 : (t / l));
    res.parameterization[0] = 1 - res.parameterization[1];
    if(t >= l) { res.sqr_distance = (p - b).sqrLength(); res.encode = 2; /* 0x10 */ }
    else if(t <= 0) { res.sqr_distance = (p - a).sqrLength(); res.encode = 1; /* 0x01 */ }
    else { res.sqr_distance = (a + d * res.parameterization[1] - p).sqrLength(); res.encode = 3; /* 0x00 */ }
  }

  return res;
}

Project::ProjectResult Project::projectTriangle(const Vec3f& a, const Vec3f& b, const Vec3f& c, const Vec3f& p)
{
  ProjectResult res;
	
  static const size_t nexti[3] = {1, 2, 0};
  const Vec3f* vt[] = {&a, &b, &c};
  const Vec3f dl[] = {a - b, b - c, c - a};
  const Vec3f& n = dl[0].cross(dl[1]);
  const FCL_REAL l = n.sqrLength();
	
  if(l > 0)
  {
    FCL_REAL mindist = -1;
    for(size_t i = 0; i < 3; ++i)
    {
      if((*vt[i] - p).dot(dl[i].cross(n)) > 0) // origin is to the outside part of the triangle edge, then the optimal can only be on the edge
      {
        size_t j = nexti[i];
        ProjectResult res_line = projectLine(*vt[i], *vt[j], p);

        if(mindist < 0 || res_line.sqr_distance < mindist)
        {
          mindist = res_line.sqr_distance;
          res.encode = static_cast<size_t>(((res_line.encode&1)?1<<i:0) + ((res_line.encode&2)?1<<j:0));
          res.parameterization[i] = res_line.parameterization[0];
          res.parameterization[j] = res_line.parameterization[1];
          res.parameterization[nexti[j]] = 0;
        }
      }
    }
    
    if(mindist < 0) // the origin project is within the triangle
    {
      FCL_REAL d = (a - p).dot(n);
      FCL_REAL s = sqrt(l);
      Vec3f p_to_project = n * (d / l);
      mindist = p_to_project.sqrLength();
      res.encode = 7; // m = 0x111
      res.parameterization[0] = dl[1].cross(b - p -p_to_project).length() / s;
      res.parameterization[1] = dl[2].cross(c - p -p_to_project).length() / s;
      res.parameterization[2] = 1 - res.parameterization[0] - res.parameterization[1];
    }

    res.sqr_distance = mindist;
  }

  return  res;

}

Project::ProjectResult Project::projectTetrahedra(const Vec3f& a, const Vec3f& b, const Vec3f& c, const Vec3f& d, const Vec3f& p)
{
  ProjectResult res;

  static const size_t nexti[] = {1, 2, 0};
  const Vec3f* vt[] = {&a, &b, &c, &d};
  const Vec3f dl[3] = {a-d, b-d, c-d};
  FCL_REAL vl = triple(dl[0], dl[1], dl[2]); 
  bool ng = (vl * (a-p).dot((b-c).cross(a-b))) <= 0;
  if(ng && std::abs(vl) > 0) // abs(vl) == 0, the tetrahedron is degenerated; if ng is false, then the last vertex in the tetrahedron does not grow toward the origin (in fact origin is on the other side of the abc face)
  {
    FCL_REAL mindist = -1;

    for(size_t i = 0; i < 3; ++i)
    {
      size_t j = nexti[i];
      FCL_REAL s = vl * (d-p).dot(dl[i].cross(dl[j]));
      if(s > 0) // the origin is to the outside part of a triangle face, then the optimal can only be on the triangle face
      {
        ProjectResult res_triangle = projectTriangle(*vt[i], *vt[j], d, p);
        if(mindist < 0 || res_triangle.sqr_distance < mindist)
        {
          mindist = res_triangle.sqr_distance;
          res.encode = static_cast<size_t>( (res_triangle.encode&1?1<<i:0) + (res_triangle.encode&2?1<<j:0) + (res_triangle.encode&4?8:0) );
          res.parameterization[i] = res_triangle.parameterization[0];
          res.parameterization[j] = res_triangle.parameterization[1];
          res.parameterization[nexti[j]] = 0;
          res.parameterization[3] = res_triangle.parameterization[2];
        }
      }
    }

    if(mindist < 0)
    {
      mindist = 0;
      res.encode = 15;
      res.parameterization[0] = triple(c - p, b - p, d - p) / vl;
      res.parameterization[1] = triple(a - p, c - p, d - p) / vl;
      res.parameterization[2] = triple(b - p, a - p, d - p) / vl;
      res.parameterization[3] = 1 - (res.parameterization[0] + res.parameterization[1] + res.parameterization[2]);
    }

    res.sqr_distance = mindist;
  }
  else if(!ng)
  {
    res = projectTriangle(a, b, c, p);
    res.parameterization[3] = 0;
  }
  return res;
}

Project::ProjectResult Project::projectLineOrigin(const Vec3f& a, const Vec3f& b)
{
  ProjectResult res;

  const Vec3f d = b - a;
  const FCL_REAL l = d.sqrLength();

  if(l > 0)
  {
    const FCL_REAL t = - a.dot(d);
    res.parameterization[1] = (t >= l) ? 1 : ((t <= 0) ? 0 : (t / l));
    res.parameterization[0] = 1 - res.parameterization[1];
    if(t >= l) { res.sqr_distance = b.sqrLength(); res.encode = 2; /* 0x10 */ }
    else if(t <= 0) { res.sqr_distance = a.sqrLength(); res.encode = 1; /* 0x01 */ }
    else { res.sqr_distance = (a + d * res.parameterization[1]).sqrLength(); res.encode = 3; /* 0x00 */ }
  }

  return res;
}

Project::ProjectResult Project::projectTriangleOrigin(const Vec3f& a, const Vec3f& b, const Vec3f& c)
{
  ProjectResult res;
	
  static const size_t nexti[3] = {1, 2, 0};
  const Vec3f* vt[] = {&a, &b, &c};
  const Vec3f dl[] = {a - b, b - c, c - a};
  const Vec3f& n = dl[0].cross(dl[1]);
  const FCL_REAL l = n.sqrLength();
	
  if(l > 0)
  {
    FCL_REAL mindist = -1;
    for(size_t i = 0; i < 3; ++i)
    {
      if(vt[i]->dot(dl[i].cross(n)) > 0) // origin is to the outside part of the triangle edge, then the optimal can only be on the edge
      {
        size_t j = nexti[i];
        ProjectResult res_line = projectLineOrigin(*vt[i], *vt[j]);

        if(mindist < 0 || res_line.sqr_distance < mindist)
        {
          mindist = res_line.sqr_distance;
          res.encode = static_cast<size_t>(((res_line.encode&1)?1<<i:0) + ((res_line.encode&2)?1<<j:0));
          res.parameterization[i] = res_line.parameterization[0];
          res.parameterization[j] = res_line.parameterization[1];
          res.parameterization[nexti[j]] = 0;
        }
      }
    }
    
    if(mindist < 0) // the origin project is within the triangle
    {
      FCL_REAL d = a.dot(n);
      FCL_REAL s = sqrt(l);
      Vec3f o_to_project = n * (d / l);
      mindist = o_to_project.sqrLength();
      res.encode = 7; // m = 0x111
      res.parameterization[0] = dl[1].cross(b - o_to_project).length() / s;
      res.parameterization[1] = dl[2].cross(c - o_to_project).length() / s;
      res.parameterization[2] = 1 - res.parameterization[0] - res.parameterization[1];
    }

    res.sqr_distance = mindist;
  }

  return  res;

}

Project::ProjectResult Project::projectTetrahedraOrigin(const Vec3f& a, const Vec3f& b, const Vec3f& c, const Vec3f& d)
{
  ProjectResult res;

  static const size_t nexti[] = {1, 2, 0};
  const Vec3f* vt[] = {&a, &b, &c, &d};
  const Vec3f dl[3] = {a-d, b-d, c-d};
  FCL_REAL vl = triple(dl[0], dl[1], dl[2]); 
  bool ng = (vl * a.dot((b-c).cross(a-b))) <= 0;
  if(ng && std::abs(vl) > 0) // abs(vl) == 0, the tetrahedron is degenerated; if ng is false, then the last vertex in the tetrahedron does not grow toward the origin (in fact origin is on the other side of the abc face)
  {
    FCL_REAL mindist = -1;

    for(size_t i = 0; i < 3; ++i)
    {
      size_t j = nexti[i];
      FCL_REAL s = vl * d.dot(dl[i].cross(dl[j]));
      if(s > 0) // the origin is to the outside part of a triangle face, then the optimal can only be on the triangle face
      {
        ProjectResult res_triangle = projectTriangleOrigin(*vt[i], *vt[j], d);
        if(mindist < 0 || res_triangle.sqr_distance < mindist)
        {
          mindist = res_triangle.sqr_distance;
          res.encode = static_cast<size_t>( (res_triangle.encode&1?1<<i:0) + (res_triangle.encode&2?1<<j:0) + (res_triangle.encode&4?8:0) );
          res.parameterization[i] = res_triangle.parameterization[0];
          res.parameterization[j] = res_triangle.parameterization[1];
          res.parameterization[nexti[j]] = 0;
          res.parameterization[3] = res_triangle.parameterization[2];
        }
      }
    }

    if(mindist < 0)
    {
      mindist = 0;
      res.encode = 15;
      res.parameterization[0] = triple(c, b, d) / vl;
      res.parameterization[1] = triple(a, c, d) / vl;
      res.parameterization[2] = triple(b, a, d) / vl;
      res.parameterization[3] = 1 - (res.parameterization[0] + res.parameterization[1] + res.parameterization[2]);
    }

    res.sqr_distance = mindist;
  }
  else if(!ng)
  {
    res = projectTriangleOrigin(a, b, c);
    res.parameterization[3] = 0;
  }
  return res;
}


}
