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

#ifndef FCL_NARROWPHASE_DETAIL_BOXBOX_INL_H
#define FCL_NARROWPHASE_DETAIL_BOXBOX_INL_H

#include "fcl/narrowphase/detail/primitive_shape_algorithm/box_box.h"

#include <algorithm>
#include <unordered_map>

namespace fcl
{

namespace detail
{

//==============================================================================
extern template
void lineClosestApproach(const Vector3<double>& pa, const Vector3<double>& ua,
                         const Vector3<double>& pb, const Vector3<double>& ub,
                         double* alpha, double* beta);

//==============================================================================
extern template
int intersectRectQuad2(double h[2], double p[8], double ret[16]);

//==============================================================================
extern template
void cullPoints2(int n, double p[], int m, int i0, int iret[]);

//==============================================================================
extern template
int boxBox2(
    const Vector3<double>& side1,
    const Transform3<double>& tf1,
    const Vector3<double>& side2,
    const Transform3<double>& tf2,
    Vector3<double>& normal,
    double* depth,
    int* return_code,
    int maxc,
    std::vector<ContactPoint<double>>& contacts);

//==============================================================================
extern template
bool boxBoxIntersect(const Box<double>& s1, const Transform3<double>& tf1,
                     const Box<double>& s2, const Transform3<double>& tf2,
                     std::vector<ContactPoint<double>>* contacts_);

//==============================================================================
template <typename S>
void lineClosestApproach(const Vector3<S>& pa, const Vector3<S>& ua,
                         const Vector3<S>& pb, const Vector3<S>& ub,
                         S* alpha, S* beta)
{
  Vector3<S> p = pb - pa;
  S uaub = ua.dot(ub);
  S q1 = ua.dot(p);
  S q2 = -ub.dot(p);
  S d = 1 - uaub * uaub;
  if(d <= (S)(0.0001f))
  {
    *alpha = 0;
    *beta = 0;
  }
  else
  {
    d = 1 / d;
    *alpha = (q1 + uaub * q2) * d;
    *beta = (uaub * q1 + q2) * d;
  }
}

//==============================================================================
template <typename S>
int intersectRectQuad2(S h[2], S p[8], S ret[16])
{
  // q (and r) contain nq (and nr) coordinate points for the current (and
  // chopped) polygons
  int nq = 4, nr = 0;
  S buffer[16];
  S* q = p;
  S* r = ret;
  for(int dir = 0; dir <= 1; ++dir)
  {
    // direction notation: xy[0] = x axis, xy[1] = y axis
    for(int sign = -1; sign <= 1; sign += 2)
    {
      // chop q along the line xy[dir] = sign*h[dir]
      S* pq = q;
      S* pr = r;
      nr = 0;
      for(int i = nq; i > 0; --i)
      {
        // go through all points in q and all lines between adjacent points
        if(sign * pq[dir] < h[dir])
        {
          // this point is inside the chopping line
          pr[0] = pq[0];
          pr[1] = pq[1];
          pr += 2;
          nr++;
          if(nr & 8)
          {
            q = r;
            goto done;
          }
        }
        S* nextq = (i > 1) ? pq+2 : q;
        if((sign*pq[dir] < h[dir]) ^ (sign*nextq[dir] < h[dir]))
        {
          // this line crosses the chopping line
          pr[1-dir] = pq[1-dir] + (nextq[1-dir]-pq[1-dir]) /
            (nextq[dir]-pq[dir]) * (sign*h[dir]-pq[dir]);
          pr[dir] = sign*h[dir];
          pr += 2;
          nr++;
          if(nr & 8)
          {
            q = r;
            goto done;
          }
        }
        pq += 2;
      }
      q = r;
      r = (q == ret) ? buffer : ret;
      nq = nr;
    }
  }

 done:
  if(q != ret) memcpy(ret, q, nr*2*sizeof(S));
  return nr;
}

//==============================================================================
template <typename S>
void cullPoints2(int n, S p[], int m, int i0, int iret[])
{
  // compute the centroid of the polygon in cx,cy
  S a, cx, cy, q;
  switch(n)
  {
  case 1:
    cx = p[0];
    cy = p[1];
    break;
  case 2:
    cx = 0.5 * (p[0] + p[2]);
    cy = 0.5 * (p[1] + p[3]);
    break;
  default:
    a = 0;
    cx = 0;
    cy = 0;
    for(int i = 0; i < n-1; ++i)
    {
      q = p[i*2]*p[i*2+3] - p[i*2+2]*p[i*2+1];
      a += q;
      cx += q*(p[i*2]+p[i*2+2]);
      cy += q*(p[i*2+1]+p[i*2+3]);
    }
    q = p[n*2-2]*p[1] - p[0]*p[n*2-1];
    if(std::abs(a+q) > std::numeric_limits<S>::epsilon())
      a = 1/(3*(a+q));
    else
      a= 1e18f;

    cx = a*(cx + q*(p[n*2-2]+p[0]));
    cy = a*(cy + q*(p[n*2-1]+p[1]));
  }


  // compute the angle of each point w.r.t. the centroid
  S A[8];
  for(int i = 0; i < n; ++i)
    A[i] = atan2(p[i*2+1]-cy,p[i*2]-cx);

  // search for points that have angles closest to A[i0] + i*(2*pi/m).
  int avail[8];
  for(int i = 0; i < n; ++i) avail[i] = 1;
  avail[i0] = 0;
  iret[0] = i0;
  iret++;
  const S pi = constants<S>::pi();
  for(int j = 1; j < m; ++j)
  {
    a = j*(2*pi/m) + A[i0];
    if (a > pi) a -= 2*pi;
    S maxdiff= 1e9, diff;

    *iret = i0;	// iret is not allowed to keep this value, but it sometimes does, when diff=#QNAN0
    for(int i = 0; i < n; ++i)
    {
      if(avail[i])
      {
        diff = std::abs(A[i]-a);
        if(diff > pi) diff = 2*pi - diff;
        if(diff < maxdiff)
        {
          maxdiff = diff;
          *iret = i;
        }
      }
    }
    avail[*iret] = 0;
    iret++;
  }
}

class CiStream {
 public:
  CiStream(bool active = false) : active_(active), stream_(std::cout) {}
  template <typename U>
  CiStream& operator<<(const U& object) {
    if (active_) stream_ << object;
    return *this;
  }

 private:
  bool active_{false};
  std::ostream& stream_;
};

//==============================================================================
template <typename S, typename DerivedA, typename DerivedB>
int boxBox2(
    const Vector3<S>& side1,
    const Eigen::MatrixBase<DerivedA>& R1,
    const Eigen::MatrixBase<DerivedB>& T1,
    const Vector3<S>& side2,
    const Eigen::MatrixBase<DerivedA>& R2,
    const Eigen::MatrixBase<DerivedB>& T2,
    Vector3<S>& normal,
    S* depth,
    int* return_code,
    int maxc,
    std::vector<ContactPoint<S>>& contacts,
    bool verbose)
{
  CiStream cout(verbose);


//  cout << "boxBox2\n";
//  cout << " side1: " << side1.transpose() << "\n";
//  cout << " R1: " << R1 << "\n";
//  cout << " T1: " << T1.transpose() << "\n";
//  cout << " side2: " << side2.transpose() << "\n";
//  cout << " R2: " << R2 << "\n";
//  cout << " T2: " << T2.transpose() << "\n";
//  cout << "\n";

  Vector3<S> p = T2 - T1; // get vector from centers of box 1 to box 2, relative to box 1
  Vector3<S> pp = R1.transpose() * p; // get pp = p relative to body 1
//  cout << " r_12_W: " << p.transpose() << "\n";
//  cout << " r_12_1: " << pp.transpose() << "\n";
//  cout << "\n";
  // get side lengths / 2
  Vector3<S> A = side1 * 0.5;
  Vector3<S> B = side2 * 0.5;

  // Rij is R1'*R2, i.e. the relative rotation between R1 and R2

  // NOTE: R1 --> R_W1 and R2 --> R_W2, so, R_W1ᵀ·R_W2 = R_1W · R_W2 = R_12
  Matrix3<S> R = R1.transpose() * R2;
  Matrix3<S> Q = R.cwiseAbs();
//  cout << " R: " << R << "\n";
//  cout << " Q: " << Q << "\n";

  // for all 15 possible separating axes:
  //   * see if the axis separates the boxes. if so, return 0.
  //   * find the depth of the penetration along the separating axis (s2)
  //   * if this is the largest depth so far, record it.
  // the normal vector will be set to the separating axis with the smallest
  // depth. note: normalR is set to point to a column of R1 or R2 if that is
  // the smallest depth normal so far. otherwise normalR is 0 and normalC is
  // set to a vector relative to body 1. invert_normal is 1 if the sign of
  // the normal should be flipped.

  int best_col_id = -1;
  const Eigen::MatrixBase<DerivedA>* normalR = 0;
  S tmp = 0;

  S s = -std::numeric_limits<S>::max();
  int invert_normal = 0;
  int code = 0;

  auto assign_face_code = [&cout, &best_col_id, &normalR, &invert_normal](int c) {
    (void)c;
//  cout << " Assigning code " << c << ":\n";
//  cout << "   best_col_id: " << best_col_id << "\n";
//  cout << "   normalR:     " << normalR->transpose() << "\n";
//  cout << "   invert_norm: " << invert_normal << "\n";
  };

  // separating axis = u1, u2, u3
  tmp = pp[0];
  S s2 = std::abs(tmp) - (Q.row(0).dot(B) + A[0]);
//  cout << "Code 1 - s: " << s << ", s2: " << s2 << "\n";
  if(s2 > 0) { cout << " non-intersecting!\n"; *return_code = 0; return 0; }
  if(s2 > s)
  {
    code = 1;
    s = s2;
    best_col_id = 0;
    normalR = &R1;
    invert_normal = (tmp < 0);
    assign_face_code(code);
  }
//  cout << "\n";

  tmp = pp[1];
  s2 = std::abs(tmp) - (Q.row(1).dot(B) + A[1]);
//  cout << "Code 2 - s: " << s << ", s2: " << s2 << "\n";
  if(s2 > 0) { cout << " non-intersecting!\n"; *return_code = 0; return 0; }
  if(s2 > s)
  {
    code = 2;
    s = s2;
    best_col_id = 1;
    normalR = &R1;
    invert_normal = (tmp < 0);
    assign_face_code(code);
  }
//  cout << "\n";

  tmp = pp[2];
  s2 = std::abs(tmp) - (Q.row(2).dot(B) + A[2]);
//  cout << "Code 3 - s: " << s << ", s2: " << s2 << "\n";
  if(s2 > 0) { cout << " non-intersecting!\n"; *return_code = 0; return 0; }
  if(s2 > s)
  {
    code = 3;
    s = s2;
    best_col_id = 2;
    normalR = &R1;
    invert_normal = (tmp < 0);
    assign_face_code(code);
  }
//  cout << "\n";

  // separating axis = v1, v2, v3
  tmp = R2.col(0).dot(p);
  s2 = std::abs(tmp) - (Q.col(0).dot(A) + B[0]);
//  cout << "Code 4 - s: " << s << ", s2: " << s2 << "\n";
  if(s2 > 0) { cout << " non-intersecting!\n"; *return_code = 0; return 0; }
  if(s2 > s)
  {
    code = 4;
    s = s2;
    best_col_id = 0;
    normalR = &R2;
    invert_normal = (tmp < 0);
    assign_face_code(code);
  }
//  cout << "\n";

  tmp = R2.col(1).dot(p);
  s2 = std::abs(tmp) - (Q.col(1).dot(A) + B[1]);
  if(s2 > 0) { cout << " non-intersecting!\n"; *return_code = 0; return 0; }
//  cout << "Code 5 - s: " << s << ", s2: " << s2 << "\n";
  if(s2 > s)
  {
    code = 5;
    s = s2;
    best_col_id = 1;
    normalR = &R2;
    invert_normal = (tmp < 0);
    assign_face_code(code);
  }
//  cout << "\n";

  tmp = R2.col(2).dot(p);
  s2 =  std::abs(tmp) - (Q.col(2).dot(A) + B[2]);
//  cout << "Code 6 - s: " << s << ", s2: " << s2 << "\n";
  if(s2 > 0) { cout << " non-intersecting!\n"; *return_code = 0; return 0; }
  if(s2 > s)
  {
    code = 6;
    s = s2;
    best_col_id = 2;
    normalR = &R2;
    invert_normal = (tmp < 0);
    assign_face_code(code);
  }
//  cout << "\n";

  // This is used to detect zero-length axes which arise from taking the cross
  // product of parallel edges.
  typename constants<S>::Real eps = constants<S>::eps();

  // We are performing the mathematical test: 0 > 0 (which should always be
  // false). However, zero can sometimes be 1e-16 or 1.5e-16. Thus,
  // mathematically we would interpret a scenario as 0 > 0 but end up with
  // 1.5e-16 > 1e-16. The former evaluates to false, the latter evaluates to
  // true; a clear falsehood.
  // (See http://gamma.cs.unc.edu/users/gottschalk/main.pdf, page 83).
  //
  // Gottschalk simply offers the value 1e-6 without any justification.
  // For double-precision values, that is quite a large epsilon; we can do
  // better. The idea is that this fictional zero can be scaled by the boxes'
  // dimensions and summed up four times. To account for that zero, we need to
  // make sure our epsilon is large enough to account for the *maximum* scale
  // factor and the sum. We pad Q by that amount so that the error will *not*
  // be the largest term.
  {
    using std::max;
    // For small boxes (dimensions all less than 1), limit the scale factor to
    // be no smaller than 10 * eps. This assumes all dimensions are strictly
    // non-negative.
    S scale_factor = max(max(A.maxCoeff(), B.maxCoeff()), 1.0) * 10 * eps;
    Q.array() += scale_factor;
  }




  // NOTE: This fudge factor is used to prefer face-feature contact over
  // edge-edge. It is unclear why edge-edge contact receives this 5% penalty.
  // Someone should document this.
  const S fudge_factor = S(1.05);

  // separating axis = u1 x (v1,v2,v3)
  tmp = pp[2] * R(1, 0) - pp[1] * R(2, 0);
  s2 = std::abs(tmp) - (A[1] * Q(2, 0) + A[2] * Q(1, 0) + B[1] * Q(0, 2) + B[2] * Q(0, 1));
//  cout << "Code 7 - s: " << s << ", s2: " << s2 << ", s2 * fudge factor: ";
//  cout << (s2 * fudge_factor) << "\n";
  if(s2 > 0) { cout << " non-intersecting!\n"; *return_code = 0; return 0; }
  Vector3<S> n(0, -R(2, 0), R(1, 0));
  S l = n.norm();
  Vector3<S> normalC(n);

  auto assign_edge_code = [&cout, &best_col_id, &normalC, &invert_normal](int c) {
    (void)c;
//  cout << " Assigning code " << c << ":\n";
//  cout << "   best_col_id: " << best_col_id << "\n";
//  cout << "   normalC:     " << normalC.transpose() << "\n";
//  cout << "   invert_norm: " << invert_normal << "\n";
//  cout << "\n";
  };

  if(l > eps)
  {
    s2 /= l;
    if(s2 * fudge_factor > s)
    {
      code = 7;
      s = s2;
      best_col_id = -1;
      normalC = n / l;
      invert_normal = (tmp < 0);
      assign_edge_code(code);
    }
  } else {
//  cout << " l: " << l << "\n";
  }
//  cout << "\n";

  tmp = pp[2] * R(1, 1) - pp[1] * R(2, 1);
  s2 = std::abs(tmp) - (A[1] * Q(2, 1) + A[2] * Q(1, 1) + B[0] * Q(0, 2) + B[2] * Q(0, 0));
//  cout << "Code 8 - s: " << s << ", s2: " << s2 << ", s2 * fudge factor: ";
//  cout << (s2 * fudge_factor) << "\n";
  if(s2 > 0) { cout << " non-intersecting!\n"; *return_code = 0; return 0; }
  n << 0, -R(2, 1), R(1, 1);
  l = n.norm();
  if(l > eps)
  {
    s2 /= l;
    if(s2 * fudge_factor > s)
    {
      code = 8;
      s = s2;
      best_col_id = -1;
      normalC = n / l;
      invert_normal = (tmp < 0);
      assign_edge_code(code);
    }
  } else {
//  cout << " l: " << l << "\n";
  }
//  cout << "\n";

  tmp = pp[2] * R(1, 2) - pp[1] * R(2, 2);
  s2 = std::abs(tmp) - (A[1] * Q(2, 2) + A[2] * Q(1, 2) + B[0] * Q(0, 1) + B[1] * Q(0, 0));
//  cout << "Code 9 - s: " << s << ", s2: " << s2 << ", s2 * fudge factor: ";
//  cout << (s2 * fudge_factor) << "\n";
  if(s2 > 0) { cout << " non-intersecting!\n"; *return_code = 0; return 0; }
  n << 0, -R(2, 2), R(1, 2);
  l = n.norm();
  if(l > eps)
  {
    s2 /= l;
    if(s2 * fudge_factor > s)
    {
      code = 9;
      s = s2;
      best_col_id = -1;
      normalC = n / l;
      invert_normal = (tmp < 0);
      assign_edge_code(code);
    }
  } else {
//  cout << " l: " << l << "\n";
  }
//  cout << "\n";

  // separating axis = u2 x (v1,v2,v3)
  tmp = pp[0] * R(2, 0) - pp[2] * R(0, 0);
  s2 = std::abs(tmp) - (A[0] * Q(2, 0) + A[2] * Q(0, 0) + B[1] * Q(1, 2) + B[2] * Q(1, 1));
//  cout << "Code 10 - s: " << s << ", s2: " << s2 << ", s2 * fudge factor: ";
//  cout << (s2 * fudge_factor) << "\n";
  if(s2 > 0) { cout << " non-intersecting!\n"; *return_code = 0; return 0; }
  n << R(2, 0), 0, -R(0, 0);
  l = n.norm();
  if(l > eps)
  {
    s2 /= l;
    if(s2 * fudge_factor > s)
    {
      code = 10;
      s = s2;
      best_col_id = -1;
      normalC = n / l;
      invert_normal = (tmp < 0);
      assign_edge_code(code);
    }
  } else {
//  cout << " l: " << l << "\n";
  }
//  cout << "\n";

  tmp = pp[0] * R(2, 1) - pp[2] * R(0, 1);
  s2 = std::abs(tmp) - (A[0] * Q(2, 1) + A[2] * Q(0, 1) + B[0] * Q(1, 2) + B[2] * Q(1, 0));
//  cout << "Code 11 - s: " << s << ", s2: " << s2 << ", s2 * fudge factor: ";
//  cout << (s2 * fudge_factor) << "\n";
  if(s2 > 0) { cout << " non-intersecting!\n"; *return_code = 0; return 0; }
  n << R(2, 1), 0, -R(0, 1);
  l = n.norm();
  if(l > eps)
  {
    s2 /= l;
    if(s2 * fudge_factor > s)
    {
      code = 11;
      s = s2;
      best_col_id = -1;
      normalC = n / l;
      invert_normal = (tmp < 0);
      assign_edge_code(code);
    }
  } else {
//  cout << " l: " << l << "\n";
  }
//  cout << "\n";

  tmp = pp[0] * R(2, 2) - pp[2] * R(0, 2);
  s2 = std::abs(tmp) - (A[0] * Q(2, 2) + A[2] * Q(0, 2) + B[0] * Q(1, 1) + B[1] * Q(1, 0));
//  cout << "Code 12 - s: " << s << ", s2: " << s2 << ", s2 * fudge factor: ";
//  cout << (s2 * fudge_factor) << "\n";
  if(s2 > 0) { cout << " non-intersecting!\n"; *return_code = 0; return 0; }
  n << R(2, 2), 0, -R(0, 2);
  l = n.norm();
  if(l > eps)
  {
    s2 /= l;
    if(s2 * fudge_factor > s)
    {
      code = 12;
      s = s2;
      best_col_id = -1;
      normalC = n / l;
      invert_normal = (tmp < 0);
      assign_edge_code(code);
    }
  } else {
//  cout << " l: " << l << "\n";
  }
//  cout << "\n";

  // separating axis = u3 x (v1,v2,v3)
  tmp = pp[1] * R(0, 0) - pp[0] * R(1, 0);
  s2 = std::abs(tmp) - (A[0] * Q(1, 0) + A[1] * Q(0, 0) + B[1] * Q(2, 2) + B[2] * Q(2, 1));
//  cout << "Code 13 - s: " << s << ", s2: " << s2 << ", s2 * fudge factor: ";
//  cout << (s2 * fudge_factor) << "\n";
  if(s2 > 0) { cout << " non-intersecting!\n"; *return_code = 0; return 0; }
  n << -R(1, 0), R(0, 0), 0;
  l = n.norm();
  if(l > eps)
  {
    s2 /= l;
    if(s2 * fudge_factor > s)
    {
      code = 13;
      s = s2;
      best_col_id = -1;
      normalC = n / l;
      invert_normal = (tmp < 0);
      assign_edge_code(code);
    }
  } else {
//  cout << " l: " << l << "\n";
  }
//  cout << "\n";

  tmp = pp[1] * R(0, 1) - pp[0] * R(1, 1);
  s2 = std::abs(tmp) - (A[0] * Q(1, 1) + A[1] * Q(0, 1) + B[0] * Q(2, 2) + B[2] * Q(2, 0));
//  cout << "Code 14 - s: " << s << ", s2: " << s2 << ", s2 * fudge factor: ";
//  cout << (s2 * fudge_factor) << "\n";
  if(s2 > 0) { cout << " non-intersecting!\n"; *return_code = 0; return 0; }
  n << -R(1, 1), R(0, 1), 0;
  l = n.norm();
  if(l > eps)
  {
    s2 /= l;
    if(s2 * fudge_factor > s)
    {
      code = 14;
      s = s2;
      best_col_id = -1;
      normalC = n / l;
      invert_normal = (tmp < 0);
      assign_edge_code(code);
    }
  } else {
//  cout << " l: " << l << "\n";
  }
//  cout << "\n";

  tmp = pp[1] * R(0, 2) - pp[0] * R(1, 2);
  s2 = std::abs(tmp) - (A[0] * Q(1, 2) + A[1] * Q(0, 2) + B[0] * Q(2, 1) + B[1] * Q(2, 0));
//  cout << "Code 15 - s: " << s << ", s2: " << s2 << ", s2 * fudge factor: ";
//  cout << (s2 * fudge_factor) << "\n";
  if(s2 > 0) { cout << " non-intersecting!\n"; *return_code = 0; return 0; }
  n << -R(1, 2), R(0, 2), 0;
  l = n.norm();
  if(l > eps)
  {
    s2 /= l;
    if(s2 * fudge_factor > s)
    {
      code = 15;
      s = s2;
      best_col_id = -1;
      normalC = n / l;
      invert_normal = (tmp < 0);
      assign_edge_code(code);
    }
  } else {
//  cout << " l: " << l << "\n";
  }
//  cout << "\n";

  if (!code) { *return_code = code; return 0; }

  // if we get to this point, the boxes interpenetrate. compute the normal
  // in global coordinates.
  if(best_col_id != -1)
    normal = normalR->col(best_col_id);
  else
    normal = R1 * normalC;

  if(invert_normal)
    normal = -normal;

  *depth = -s; // s is negative when the boxes are in collision

//  cout << "normal: " << normal.transpose() << "\n";
//  cout << "depth:  " << (*depth) << "\n";
//  cout << "\n";

  // compute contact point(s)

  if(code > 6)
  {
//  cout << "--------------------------------------------------------\n";
//  cout << "Compute contact point from edge-edge: " << code << "\n";
    // an edge from box 1 touches an edge from box 2.
    // find a point pa on the intersecting edge of box 1
    Vector3<S> pa(T1);
    S sign;

    for(int j = 0; j < 3; ++j)
    {
      sign = (R1.col(j).dot(normal) > 0) ? 1 : -1;
      pa += R1.col(j) * (A[j] * sign);
    }

    // find a point pb on the intersecting edge of box 2
    Vector3<S> pb(T2);

    for(int j = 0; j < 3; ++j)
    {
      sign = (R2.col(j).dot(normal) > 0) ? -1 : 1;
      pb += R2.col(j) * (B[j] * sign);
    }

    S alpha, beta;
    Vector3<S> ua(R1.col((code-7)/3));
    Vector3<S> ub(R2.col((code-7)%3));

//  cout << " pa: " << pa.transpose() << "\n";
//  cout << " pb: " << pb.transpose() << "\n";
//  cout << " ua: " << ua.transpose() << "\n";
//  cout << " ub: " << ub.transpose() << "\n";

    lineClosestApproach(pa, ua, pb, ub, &alpha, &beta);
    pa += ua * alpha;
    pb += ub * beta;

    Vector3<S> pointInWorld((pa + pb) * 0.5);
//  cout << " pa: " << pa.transpose() << "\n";
//  cout << " pb: " << pb.transpose() << "\n";
//  cout << " point in world: " << pointInWorld.transpose() << "\n";
    contacts.emplace_back(normal, pointInWorld, *depth);
    *return_code = code;

    return 1;
  }

//  cout << "--------------------------------------------------------\n";
//  cout << "Compute contact point from face-something: " << code << "\n";

  // okay, we have a face-something intersection (because the separating
  // axis is perpendicular to a face). define face 'a' to be the reference
  // face (i.e. the normal vector is perpendicular to this) and face 'b' to be
  // the incident face (the closest face of the other box).

  const Eigen::MatrixBase<DerivedA> *Ra, *Rb;
  const Eigen::MatrixBase<DerivedB> *pa, *pb;
  const Vector3<S> *Sa, *Sb;

  if(code <= 3)
  {
    Ra = &R1;
    Rb = &R2;
    pa = &T1;
    pb = &T2;
    Sa = &A;
    Sb = &B;
  }
  else
  {
    Ra = &R2;
    Rb = &R1;
    pa = &T2;
    pb = &T1;
    Sa = &B;
    Sb = &A;
  }

//  cout << " Ra: " << Ra->transpose() << "\n";
//  cout << " Rb: " << Rb->transpose() << "\n";
//  cout << " pa: " << pa->transpose() << "\n";
//  cout << " pb: " << pb->transpose() << "\n";
//  cout << " Sa: " << Sa->transpose() << "\n";
//  cout << " Sb: " << Sb->transpose() << "\n";
//  cout << "\n";

  // nr = normal vector of reference face dotted with axes of incident box.
  // anr = absolute values of nr.
  Vector3<S> normal2(normal);
  if(code <= 3)
    normal2 = normal;
  else
    normal2 = -normal;

  Vector3<S> nr = Rb->transpose() * normal2;
  Vector3<S> anr = nr.cwiseAbs();

//  cout << " normal2: " << normal2.transpose() << "\n";
//  cout << " nr:      " << nr.transpose() << "\n";
//  cout << " anr:     " << anr.transpose() << "\n";
//  cout << "\n";

  // find the largest component of anr: this corresponds to the normal
  // for the incident face. the other axis numbers of the incident face
  // are stored in a1,a2.
  int lanr{-1}, a1{-1}, a2{-1};
  if(anr[1] > anr[0])
  {
    if(anr[1] > anr[2])
    {
      a1 = 0;
      lanr = 1;
      a2 = 2;
    }
    else
    {
      a1 = 0;
      a2 = 1;
      lanr = 2;
    }
  }
  else
  {
    if(anr[0] > anr[2])
    {
      lanr = 0;
      a1 = 1;
      a2 = 2;
    }
    else
    {
      a1 = 0;
      a2 = 1;
      lanr = 2;
    }
  }

//  cout << " lanr: " << lanr << "\n";
//  cout << " a1:   " << a1 << "\n";
//  cout << " a2:   " << a2 << "\n";
//  cout << "\n";

  // compute center point of incident face, in reference-face coordinates
  Vector3<S> center;
  if(nr[lanr] < 0)
    center = (*pb) - (*pa) + Rb->col(lanr) * ((*Sb)[lanr]);
  else
    center = (*pb) - (*pa) - Rb->col(lanr) * ((*Sb)[lanr]);

//  cout << " center: " << center.transpose() << "\n";
//  cout << "\n";

  // find the normal and non-normal axis numbers of the reference box
  int codeN{-1}, code1{-1}, code2{-1};
  if(code <= 3)
    codeN = code-1;
  else codeN = code-4;

  if(codeN == 0)
  {
    code1 = 1;
    code2 = 2;
  }
  else if(codeN == 1)
  {
    code1 = 0;
    code2 = 2;
  }
  else
  {
    code1 = 0;
    code2 = 1;
  }

//  cout << " codeN: " << codeN << "\n";
//  cout << " code1: " << code1 << "\n";
//  cout << " code2: " << code2 << "\n";
//  cout << "\n";

  // TODO(SeanCurtis-TRI): Remove this test once the mac woes are debugged.
  if (code1 == -1 || code2 == -1 || codeN == -1) {
    throw std::logic_error("Failed to initialize codes");
  }

  // find the four corners of the incident face, in reference-face coordinates
  S quad[8]; // 2D coordinate of incident face (x,y pairs)
  S c1{std::numeric_limits<S>::quiet_NaN()};
  S c2{std::numeric_limits<S>::quiet_NaN()};
  S m11{std::numeric_limits<S>::quiet_NaN()};
  S m12{std::numeric_limits<S>::quiet_NaN()};
  S m21{std::numeric_limits<S>::quiet_NaN()};
  S m22{std::numeric_limits<S>::quiet_NaN()};

  c1 = Ra->col(code1).dot(center);
  c2 = Ra->col(code2).dot(center);
  // optimize this? - we have already computed this data above, but it is not
  // stored in an easy-to-index format. for now it's quicker just to recompute
  // the four dot products.
  Vector3<S> tempRac = Ra->col(code1);
  m11 = Rb->col(a1).dot(tempRac);
  m12 = Rb->col(a2).dot(tempRac);
  tempRac = Ra->col(code2);
  m21 = Rb->col(a1).dot(tempRac);
  m22 = Rb->col(a2).dot(tempRac);

  S k1 = m11 * (*Sb)[a1];
  S k2 = m21 * (*Sb)[a1];
  S k3 = m12 * (*Sb)[a2];
  S k4 = m22 * (*Sb)[a2];
  quad[0] = c1 - k1 - k3;
  quad[1] = c2 - k2 - k4;
  quad[2] = c1 - k1 + k3;
  quad[3] = c2 - k2 + k4;
  quad[4] = c1 + k1 + k3;
  quad[5] = c2 + k2 + k4;
  quad[6] = c1 + k1 - k3;
  quad[7] = c2 + k2 - k4;

  // TODO(SeanCurtis-TRI): Remove this test once the mac woes are debugged.
  using std::isnan;
  for (int i = 0; i < 8; ++i) {
    if (isnan(quad[i])) {
      throw std::logic_error("Quad contains NaN");
    }
  }

  // find the size of the reference face
  S rect[2] = {(*Sa)[code1], (*Sa)[code2]};

//  cout << " rect: " << rect[0] << ", " << rect[1] << "\n";
//  cout << " quad:\n";
  for (int i = 0; i < 8; i += 2) {
//  cout << "   " << quad[i] << ", " << quad[i + 1] << "\n";
  }

  // intersect the incident and reference faces
  S ret[16];
  int n_intersect = intersectRectQuad2(rect, quad, ret);
//  cout << " n_intersect: " << n_intersect << "\n";
//  for (int i = 0; i < n_intersect; ++i) {
//    int idx = i * 2;
//  cout << "  " << ret[idx] << ", " << ret[idx + 1] << "\n";
//  }
//  cout << "\n";

  if(n_intersect < 1) {
    // Historical documentation says that this "should never happen" but it
    // still returned values that allowed computation to proceed. This arrests
    // that evaluation in place and precludes the possibility of silently
    // reaching this ostensibly unreachable code.
    throw std::logic_error("Reached 'unreachable' code -- n_intersect < 1");
  }

  // convert the intersection points into reference-face coordinates,
  // and compute the contact position and depth for each point. only keep
  // those points that have a positive (penetrating) depth. delete points in
  // the 'ret' array as necessary so that 'point' and 'ret' correspond.
  Vector3<S> points[8]; // penetrating contact points
  S dep[8]; // depths for those points
  S det1 = 1.f/(m11*m22 - m12*m21);
  m11 *= det1;
  m12 *= det1;
  m21 *= det1;
  m22 *= det1;
  int cnum = 0;	// number of penetrating contact points found
  for(int j = 0; j < n_intersect; ++j)
  {
    S k1 =  m22*(ret[j*2]-c1) - m12*(ret[j*2+1]-c2);
    S k2 = -m21*(ret[j*2]-c1) + m11*(ret[j*2+1]-c2);
    points[cnum] = center + Rb->col(a1) * k1 + Rb->col(a2) * k2;
    dep[cnum] = (*Sa)[codeN] - normal2.dot(points[cnum]);
    if(dep[cnum] >= 0)
    {
      ret[cnum*2] = ret[j*2];
      ret[cnum*2+1] = ret[j*2+1];
      cnum++;
    }
  }
  if(cnum < 1) { *return_code = code; return 0; } // this should never happen

  // we can't generate more contacts than we actually have
  if(maxc > cnum) maxc = cnum;
  if(maxc < 1) maxc = 1;

  // The determination of these contact computations are tested in:
  // test_fcl_box_box.cpp.
  //  The case where cnum <= maxc is tested by the test:
  //    test_collision_box_box_all_contacts
  //  The case where cnum > maxc is tested by the test:
  //    test_collision_box_box_cull_contacts
  //
  // Each of those tests is exercised twice: once when code < 4 and once when
  // 4 <= code < 7.
  int iret[] = {0, 1, 2, 3, 4, 5, 6, 7};
  if (cnum > maxc) {
    int i1 = 0;
    S maxdepth = dep[0];
    for(int i = 1; i < cnum; ++i)
    {
      if(dep[i] > maxdepth)
      {
        maxdepth = dep[i];
        i1 = i;
      }
    }

    cullPoints2(cnum, ret, maxc, i1, iret);
    cnum = maxc;
  }

  if (code < 4) {
    for(int j = 0; j < cnum; ++j)
    {
      int i = iret[j];
      Vector3<S> pointInWorld = points[i] + (*pa) + normal * (dep[i] / 2);
      contacts.emplace_back(normal, pointInWorld, dep[i]);
    }
  } else {
    for(int j = 0; j < cnum; ++j)
    {
      int i = iret[j];
      Vector3<S> pointInWorld = points[i] + (*pa) - normal * (dep[i] / 2);
      contacts.emplace_back(normal, pointInWorld, dep[i]);
    }
  }

  *return_code = code;
  return cnum;
}

//==============================================================================
template <typename S>
int boxBox2(
    const Vector3<S>& side1,
    const Transform3<S>& tf1,
    const Vector3<S>& side2,
    const Transform3<S>& tf2,
    Vector3<S>& normal,
    S* depth,
    int* return_code,
    int maxc,
    std::vector<ContactPoint<S>>& contacts)
{
  return boxBox2(side1, tf1.linear(), tf1.translation(), side2, tf2.linear(),
                 tf2.translation(), normal, depth, return_code, maxc, contacts);
}

//==============================================================================
template <typename S>
bool boxBoxIntersect(const Box<S>& s1, const Transform3<S>& tf1,
                     const Box<S>& s2, const Transform3<S>& tf2,
                     std::vector<ContactPoint<S>>* contacts_)
{
  std::vector<ContactPoint<S>> contacts;
  int return_code;
  Vector3<S> normal;
  S depth;
  /* int cnum = */ boxBox2(s1.side, tf1,
                           s2.side, tf2,
                           normal, &depth, &return_code,
                           4, contacts);

  if(contacts_)
    *contacts_ = contacts;

  return return_code != 0;
}

} // namespace detail
} // namespace fcl

#endif
