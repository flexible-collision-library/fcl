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


#include "fcl/BVH/BVH_utility.h"

namespace fcl
{

void BVHExpand(BVHModel<OBB>& model, const Variance3f* ucs, FCL_REAL r = 1.0)
{
  for(int i = 0; i < model.getNumBVs(); ++i)
  {
    BVNode<OBB>& bvnode = model.getBV(i);

    Vec3f* vs = new Vec3f[bvnode.num_primitives * 6];

    for(int j = 0; j < bvnode.num_primitives; ++j)
    {
      int v_id = bvnode.first_primitive + j;
      const Variance3f& uc = ucs[v_id];

      Vec3f&v = model.vertices[bvnode.first_primitive + j];

      for(int k = 0; k < 3; ++k)
      {
        vs[6 * j + 2 * k] = v + uc.axis[k] * (r * uc.sigma[k]);
        vs[6 * j + 2 * k + 1] = v - uc.axis[k] * (r * uc.sigma[k]);
      }
    }

    OBB bv;
    fit(vs, bvnode.num_primitives * 6, bv);

    delete [] vs;

    bvnode.bv = bv;
  }
}

void BVHExpand(BVHModel<RSS>& model, const Variance3f* ucs, FCL_REAL r = 1.0)
{
  for(int i = 0; i < model.getNumBVs(); ++i)
  {
    BVNode<RSS>& bvnode = model.getBV(i);

    Vec3f* vs = new Vec3f[bvnode.num_primitives * 6];

    for(int j = 0; j < bvnode.num_primitives; ++j)
    {
      int v_id = bvnode.first_primitive + j;
      const Variance3f& uc = ucs[v_id];

      Vec3f&v = model.vertices[bvnode.first_primitive + j];

      for(int k = 0; k < 3; ++k)
      {
        vs[6 * j + 2 * k] = v + uc.axis[k] * (r * uc.sigma[k]);
        vs[6 * j + 2 * k + 1] = v - uc.axis[k] * (r * uc.sigma[k]);
      }
    }

    RSS bv;
    fit(vs, bvnode.num_primitives * 6, bv);

    delete [] vs;

    bvnode.bv = bv;
  }
}


void getCovariance(Vec3f* ps, Vec3f* ps2, Triangle* ts, unsigned int* indices, int n, Matrix3f& M)
{
  Vec3f S1;
  Vec3f S2[3];

  if(ts)
  {
    for(int i = 0; i < n; ++i)
    {
      const Triangle& t = (indices) ? ts[indices[i]] : ts[i];

      const Vec3f& p1 = ps[t[0]];
      const Vec3f& p2 = ps[t[1]];
      const Vec3f& p3 = ps[t[2]];

      S1[0] += (p1[0] + p2[0] + p3[0]);
      S1[1] += (p1[1] + p2[1] + p3[1]);
      S1[2] += (p1[2] + p2[2] + p3[2]);
      S2[0][0] += (p1[0] * p1[0] + p2[0] * p2[0] + p3[0] * p3[0]);
      S2[1][1] += (p1[1] * p1[1] + p2[1] * p2[1] + p3[1] * p3[1]);
      S2[2][2] += (p1[2] * p1[2] + p2[2] * p2[2] + p3[2] * p3[2]);
      S2[0][1] += (p1[0] * p1[1] + p2[0] * p2[1] + p3[0] * p3[1]);
      S2[0][2] += (p1[0] * p1[2] + p2[0] * p2[2] + p3[0] * p3[2]);
      S2[1][2] += (p1[1] * p1[2] + p2[1] * p2[2] + p3[1] * p3[2]);

      if(ps2)
      {
        const Vec3f& p1 = ps2[t[0]];
        const Vec3f& p2 = ps2[t[1]];
        const Vec3f& p3 = ps2[t[2]];

        S1[0] += (p1[0] + p2[0] + p3[0]);
        S1[1] += (p1[1] + p2[1] + p3[1]);
        S1[2] += (p1[2] + p2[2] + p3[2]);

        S2[0][0] += (p1[0] * p1[0] + p2[0] * p2[0] + p3[0] * p3[0]);
        S2[1][1] += (p1[1] * p1[1] + p2[1] * p2[1] + p3[1] * p3[1]);
        S2[2][2] += (p1[2] * p1[2] + p2[2] * p2[2] + p3[2] * p3[2]);
        S2[0][1] += (p1[0] * p1[1] + p2[0] * p2[1] + p3[0] * p3[1]);
        S2[0][2] += (p1[0] * p1[2] + p2[0] * p2[2] + p3[0] * p3[2]);
        S2[1][2] += (p1[1] * p1[2] + p2[1] * p2[2] + p3[1] * p3[2]);
      }
    }
  }
  else
  {
    for(int i = 0; i < n; ++i)
    {
      const Vec3f& p = (indices) ? ps[indices[i]] : ps[i];
      S1 += p;
      S2[0][0] += (p[0] * p[0]);
      S2[1][1] += (p[1] * p[1]);
      S2[2][2] += (p[2] * p[2]);
      S2[0][1] += (p[0] * p[1]);
      S2[0][2] += (p[0] * p[2]);
      S2[1][2] += (p[1] * p[2]);

      if(ps2) // another frame
      {
        const Vec3f& p = (indices) ? ps2[indices[i]] : ps2[i];
        S1 += p;
        S2[0][0] += (p[0] * p[0]);
        S2[1][1] += (p[1] * p[1]);
        S2[2][2] += (p[2] * p[2]);
        S2[0][1] += (p[0] * p[1]);
        S2[0][2] += (p[0] * p[2]);
        S2[1][2] += (p[1] * p[2]);
      }
    }
  }

  int n_points = ((ps2) ? 2 : 1) * ((ts) ? 3 : 1) * n;

  M(0, 0) = S2[0][0] - S1[0]*S1[0] / n_points;
  M(1, 1) = S2[1][1] - S1[1]*S1[1] / n_points;
  M(2, 2) = S2[2][2] - S1[2]*S1[2] / n_points;
  M(0, 1) = S2[0][1] - S1[0]*S1[1] / n_points;
  M(1, 2) = S2[1][2] - S1[1]*S1[2] / n_points;
  M(0, 2) = S2[0][2] - S1[0]*S1[2] / n_points;
  M(1, 0) = M(0, 1);
  M(2, 0) = M(0, 2);
  M(2, 1) = M(1, 2);
}


/** \brief Compute the RSS bounding volume parameters: radius, rectangle size and the origin.
 * The bounding volume axes are known.
 */
void getRadiusAndOriginAndRectangleSize(Vec3f* ps, Vec3f* ps2, Triangle* ts, unsigned int* indices, int n, Vec3f axis[3], Vec3f& origin, FCL_REAL l[2], FCL_REAL& r)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  int size_P = ((ps2) ? 2 : 1) * ((ts) ? 3 : 1) * n;

  FCL_REAL (*P)[3] = new FCL_REAL[size_P][3];

  int P_id = 0;
  
  if(ts)
  {
    for(int i = 0; i < n; ++i)
    {
      int index = indirect_index ? indices[i] : i;
      const Triangle& t = ts[index];

      for(int j = 0; j < 3; ++j)
      {
        int point_id = t[j];
        const Vec3f& p = ps[point_id];
        Vec3f v(p[0], p[1], p[2]);
        P[P_id][0] = axis[0].dot(v);
        P[P_id][1] = axis[1].dot(v);
        P[P_id][2] = axis[2].dot(v);
        P_id++;
      }

      if(ps2)
      {
        for(int j = 0; j < 3; ++j)
        {
          int point_id = t[j];
          const Vec3f& p = ps2[point_id];
          Vec3f v(p[0], p[1], p[2]);
          P[P_id][0] = axis[0].dot(v);
          P[P_id][1] = axis[0].dot(v);
          P[P_id][2] = axis[1].dot(v);
          P_id++;
        }
      }
    }
  }
  else
  {
    for(int i = 0; i < n; ++i)
    {
      int index = indirect_index ? indices[i] : i;

      const Vec3f& p = ps[index];
      Vec3f v(p[0], p[1], p[2]);
      P[P_id][0] = axis[0].dot(v);
      P[P_id][1] = axis[1].dot(v);
      P[P_id][2] = axis[2].dot(v);
      P_id++;

      if(ps2)
      {
        const Vec3f& v = ps2[index];
        P[P_id][0] = axis[0].dot(v);
        P[P_id][1] = axis[1].dot(v);
        P[P_id][2] = axis[2].dot(v);
        P_id++;
      }
    }
  }
    
  FCL_REAL minx, maxx, miny, maxy, minz, maxz;

  FCL_REAL cz, radsqr;

  minz = maxz = P[0][2];

  for(int i = 1; i < size_P; ++i)
  {
    FCL_REAL z_value = P[i][2];
    if(z_value < minz) minz = z_value;
    else if(z_value > maxz) maxz = z_value;
  }

  r = (FCL_REAL)0.5 * (maxz - minz);
  radsqr = r * r;
  cz = (FCL_REAL)0.5 * (maxz + minz);

  // compute an initial length of rectangle along x direction

  // find minx and maxx as starting points

  int minindex, maxindex;
  minindex = maxindex = 0;
  FCL_REAL mintmp, maxtmp;
  mintmp = maxtmp = P[0][0];

  for(int i = 1; i < size_P; ++i)
  {
    FCL_REAL x_value = P[i][0];
    if(x_value < mintmp)
    {
      minindex = i;
      mintmp = x_value;
    }
    else if(x_value > maxtmp)
    {
      maxindex = i;
      maxtmp = x_value;
    }
  }

  FCL_REAL x, dz;
  dz = P[minindex][2] - cz;
  minx = P[minindex][0] + std::sqrt(std::max<FCL_REAL>(radsqr - dz * dz, 0));
  dz = P[maxindex][2] - cz;
  maxx = P[maxindex][0] - std::sqrt(std::max<FCL_REAL>(radsqr - dz * dz, 0));


  // grow minx

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] < minx)
    {
      dz = P[i][2] - cz;
      x = P[i][0] + std::sqrt(std::max<FCL_REAL>(radsqr - dz * dz, 0));
      if(x < minx) minx = x;
    }
  }

  // grow maxx

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] > maxx)
    {
      dz = P[i][2] - cz;
      x = P[i][0] - std::sqrt(std::max<FCL_REAL>(radsqr - dz * dz, 0));
      if(x > maxx) maxx = x;
    }
  }

  // compute an initial length of rectangle along y direction

  // find miny and maxy as starting points

  minindex = maxindex = 0;
  mintmp = maxtmp = P[0][1];
  for(int i = 1; i < size_P; ++i)
  {
    FCL_REAL y_value = P[i][1];
    if(y_value < mintmp)
    {
      minindex = i;
      mintmp = y_value;
    }
    else if(y_value > maxtmp)
    {
      maxindex = i;
      maxtmp = y_value;
    }
  }

  FCL_REAL y;
  dz = P[minindex][2] - cz;
  miny = P[minindex][1] + std::sqrt(std::max<FCL_REAL>(radsqr - dz * dz, 0));
  dz = P[maxindex][2] - cz;
  maxy = P[maxindex][1] - std::sqrt(std::max<FCL_REAL>(radsqr - dz * dz, 0));

  // grow miny

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][1] < miny)
    {
      dz = P[i][2] - cz;
      y = P[i][1] + std::sqrt(std::max<FCL_REAL>(radsqr - dz * dz, 0));
      if(y < miny) miny = y;
    }
  }

  // grow maxy

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][1] > maxy)
    {
      dz = P[i][2] - cz;
      y = P[i][1] - std::sqrt(std::max<FCL_REAL>(radsqr - dz * dz, 0));
      if(y > maxy) maxy = y;
    }
  }

  // corners may have some points which are not covered - grow lengths if necessary
  // quite conservative (can be improved)
  FCL_REAL dx, dy, u, t;
  FCL_REAL a = std::sqrt((FCL_REAL)0.5);
  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] > maxx)
    {
      if(P[i][1] > maxy)
      {
        dx = P[i][0] - maxx;
        dy = P[i][1] - maxy;
        u = dx * a + dy * a;
        t = (a*u - dx)*(a*u - dx) +
            (a*u - dy)*(a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - std::sqrt(std::max<FCL_REAL>(radsqr - t, 0));
        if(u > 0)
        {
          maxx += u*a;
          maxy += u*a;
        }
      }
      else if(P[i][1] < miny)
      {
        dx = P[i][0] - maxx;
        dy = P[i][1] - miny;
        u = dx * a - dy * a;
        t = (a*u - dx)*(a*u - dx) +
            (-a*u - dy)*(-a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - std::sqrt(std::max<FCL_REAL>(radsqr - t, 0));
        if(u > 0)
        {
          maxx += u*a;
          miny -= u*a;
        }
      }
    }
    else if(P[i][0] < minx)
    {
      if(P[i][1] > maxy)
      {
        dx = P[i][0] - minx;
        dy = P[i][1] - maxy;
        u = dy * a - dx * a;
        t = (-a*u - dx)*(-a*u - dx) +
            (a*u - dy)*(a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - std::sqrt(std::max<FCL_REAL>(radsqr - t, 0));
        if(u > 0)
        {
          minx -= u*a;
          maxy += u*a;
        }
      }
      else if(P[i][1] < miny)
      {
        dx = P[i][0] - minx;
        dy = P[i][1] - miny;
        u = -dx * a - dy * a;
        t = (-a*u - dx)*(-a*u - dx) +
            (-a*u - dy)*(-a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - std::sqrt(std::max<FCL_REAL>(radsqr - t, 0));
        if (u > 0)
        {
          minx -= u*a;
          miny -= u*a;
        }
      }
    }
  }

  origin = axis[0] * minx + axis[1] * miny + axis[2] * cz;

  l[0] = maxx - minx;
  if(l[0] < 0) l[0] = 0;
  l[1] = maxy - miny;
  if(l[1] < 0) l[1] = 0;

  delete [] P;

}


/** \brief Compute the bounding volume extent and center for a set or subset of points.
 * The bounding volume axes are known.
 */
static inline void getExtentAndCenter_pointcloud(Vec3f* ps, Vec3f* ps2, unsigned int* indices, int n, Vec3f axis[3], Vec3f& center, Vec3f& extent)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  FCL_REAL real_max = std::numeric_limits<FCL_REAL>::max();

  FCL_REAL min_coord[3] = {real_max, real_max, real_max};
  FCL_REAL max_coord[3] = {-real_max, -real_max, -real_max};

  for(int i = 0; i < n; ++i)
  {
    int index = indirect_index ? indices[i] : i;

    const Vec3f& p = ps[index];
    Vec3f v(p[0], p[1], p[2]);
    FCL_REAL proj[3];
    proj[0] = axis[0].dot(v);
    proj[1] = axis[1].dot(v);
    proj[2] = axis[2].dot(v);

    for(int j = 0; j < 3; ++j)
    {
      if(proj[j] > max_coord[j]) max_coord[j] = proj[j];
      if(proj[j] < min_coord[j]) min_coord[j] = proj[j];
    }

    if(ps2)
    {
      const Vec3f& v = ps2[index];
      proj[0] = axis[0].dot(v);
      proj[1] = axis[1].dot(v);
      proj[2] = axis[2].dot(v);

      for(int j = 0; j < 3; ++j)
      {
        if(proj[j] > max_coord[j]) max_coord[j] = proj[j];
        if(proj[j] < min_coord[j]) min_coord[j] = proj[j];
      }
    }
  }

  Vec3f o((max_coord[0] + min_coord[0]) / 2,
          (max_coord[1] + min_coord[1]) / 2,
          (max_coord[2] + min_coord[2]) / 2);

  center = axis[0] * o[0] + axis[1] * o[1] + axis[2] * o[2];

  extent.setValue((max_coord[0] - min_coord[0]) / 2,
                  (max_coord[1] - min_coord[1]) / 2,
                  (max_coord[2] - min_coord[2]) / 2);

}


/** \brief Compute the bounding volume extent and center for a set or subset of points.
 * The bounding volume axes are known.
 */
static inline void getExtentAndCenter_mesh(Vec3f* ps, Vec3f* ps2, Triangle* ts, unsigned int* indices, int n, Vec3f axis[3], Vec3f& center, Vec3f& extent)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  FCL_REAL real_max = std::numeric_limits<FCL_REAL>::max();

  FCL_REAL min_coord[3] = {real_max, real_max, real_max};
  FCL_REAL max_coord[3] = {-real_max, -real_max, -real_max};

  for(int i = 0; i < n; ++i)
  {
    unsigned int index = indirect_index? indices[i] : i;
    const Triangle& t = ts[index];

    for(int j = 0; j < 3; ++j)
    {
      int point_id = t[j];
      const Vec3f& p = ps[point_id];
      Vec3f v(p[0], p[1], p[2]);
      FCL_REAL proj[3];
      proj[0] = axis[0].dot(v);
      proj[1] = axis[1].dot(v);
      proj[2] = axis[2].dot(v);

      for(int k = 0; k < 3; ++k)
      {
        if(proj[k] > max_coord[k]) max_coord[k] = proj[k];
        if(proj[k] < min_coord[k]) min_coord[k] = proj[k];
      }
    }

    if(ps2)
    {
      for(int j = 0; j < 3; ++j)
      {
        int point_id = t[j];
        const Vec3f& p = ps2[point_id];
        Vec3f v(p[0], p[1], p[2]);
        FCL_REAL proj[3];
        proj[0] = axis[0].dot(v);
        proj[1] = axis[1].dot(v);
        proj[2] = axis[2].dot(v);

        for(int k = 0; k < 3; ++k)
        {
          if(proj[k] > max_coord[k]) max_coord[k] = proj[k];
          if(proj[k] < min_coord[k]) min_coord[k] = proj[k];
        }
      }
    }
  }

  Vec3f o((max_coord[0] + min_coord[0]) / 2,
          (max_coord[1] + min_coord[1]) / 2,
          (max_coord[2] + min_coord[2]) / 2);

  center = axis[0] * o[0] + axis[1] * o[1] + axis[2] * o[2];

  extent.setValue((max_coord[0] - min_coord[0]) / 2,
                  (max_coord[1] - min_coord[1]) / 2,
                  (max_coord[2] - min_coord[2]) / 2);

}

void getExtentAndCenter(Vec3f* ps, Vec3f* ps2, Triangle* ts, unsigned int* indices, int n, Vec3f axis[3], Vec3f& center, Vec3f& extent)
{
  if(ts)
    getExtentAndCenter_mesh(ps, ps2, ts, indices, n, axis, center, extent);
  else
    getExtentAndCenter_pointcloud(ps, ps2, indices, n, axis, center, extent);
}

void circumCircleComputation(const Vec3f& a, const Vec3f& b, const Vec3f& c, Vec3f& center, FCL_REAL& radius)
{
  Vec3f e1 = a - c;
  Vec3f e2 = b - c;
  FCL_REAL e1_len2 = e1.sqrLength();
  FCL_REAL e2_len2 = e2.sqrLength();
  Vec3f e3 = e1.cross(e2);
  FCL_REAL e3_len2 = e3.sqrLength();
  radius = e1_len2 * e2_len2 * (e1 - e2).sqrLength() / e3_len2;
  radius = std::sqrt(radius) * 0.5;

  center = (e2 * e1_len2 - e1 * e2_len2).cross(e3) * (0.5 * 1 / e3_len2) + c;
}


static inline FCL_REAL maximumDistance_mesh(Vec3f* ps, Vec3f* ps2, Triangle* ts, unsigned int* indices, int n, const Vec3f& query)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;
  
  FCL_REAL maxD = 0;
  for(int i = 0; i < n; ++i)
  {
    unsigned int index = indirect_index ? indices[i] : i;
    const Triangle& t = ts[index];

    for(int j = 0; j < 3; ++j)
    {
      int point_id = t[j];
      const Vec3f& p = ps[point_id];
      
      FCL_REAL d = (p - query).sqrLength();
      if(d > maxD) maxD = d;
    }

    if(ps2)
    {
      for(int j = 0; j < 3; ++j)
      {
        int point_id = t[j];
        const Vec3f& p = ps2[point_id];
        
        FCL_REAL d = (p - query).sqrLength();
        if(d > maxD) maxD = d;
      }
    }
  }

  return std::sqrt(maxD);
}

static inline FCL_REAL maximumDistance_pointcloud(Vec3f* ps, Vec3f* ps2, unsigned int* indices, int n, const Vec3f& query)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  FCL_REAL maxD = 0;
  for(int i = 0; i < n; ++i)
  {
    int index = indirect_index ? indices[i] : i;

    const Vec3f& p = ps[index];
    FCL_REAL d = (p - query).sqrLength();
    if(d > maxD) maxD = d;

    if(ps2)
    {
      const Vec3f& v = ps2[index];
      FCL_REAL d = (v - query).sqrLength();
      if(d > maxD) maxD = d;
    }
  }

  return std::sqrt(maxD);
}

FCL_REAL maximumDistance(Vec3f* ps, Vec3f* ps2, Triangle* ts, unsigned int* indices, int n, const Vec3f& query)
{
  if(ts)
    return maximumDistance_mesh(ps, ps2, ts, indices, n, query);
  else
    return maximumDistance_pointcloud(ps, ps2, indices, n, query);
}


}
