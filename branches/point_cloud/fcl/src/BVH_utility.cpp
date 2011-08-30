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


#include "fcl/BVH_utility.h"
#include <ann/ANN.h>

namespace fcl
{

void BVHExpand(BVHModel<OBB>& model, const Uncertainty* ucs, BVH_REAL r = 1.0)
{
  for(int i = 0; i < model.getNumBVs(); ++i)
  {
    BVNode<OBB>& bvnode = model.getBV(i);

    Vec3f* vs = new Vec3f[bvnode.num_primitives * 6];

    for(int j = 0; j < bvnode.num_primitives; ++j)
    {
      int v_id = bvnode.first_primitive + j;
      const Uncertainty& uc = ucs[v_id];

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

void BVHExpand(BVHModel<RSS>& model, const Uncertainty* ucs, BVH_REAL r = 1.0)
{
  for(int i = 0; i < model.getNumBVs(); ++i)
  {
    BVNode<RSS>& bvnode = model.getBV(i);

    Vec3f* vs = new Vec3f[bvnode.num_primitives * 6];

    for(int j = 0; j < bvnode.num_primitives; ++j)
    {
      int v_id = bvnode.first_primitive + j;
      const Uncertainty& uc = ucs[v_id];

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


void estimateSamplingUncertainty(Vec3f* vertices, int num_vertices, Uncertainty* ucs)
{
  int nPts = num_vertices;
  ANNpointArray dataPts;
  ANNpoint queryPt;

  ANNidxArray nnIdx;
  ANNdistArray dists;
  ANNkd_tree* kdTree;

  int knn_k = 10;
  if(knn_k > nPts) knn_k = nPts;
  int dim = 3;

  queryPt = annAllocPt(dim);
  dataPts = annAllocPts(nPts, dim);
  nnIdx = new ANNidx[knn_k];
  dists = new ANNdist[knn_k];

  for(int i = 0; i < nPts; ++i)
  {
    for(int j = 0; j < 3; ++j)
      dataPts[i][j] = vertices[i][j];
  }

  kdTree = new ANNkd_tree(dataPts, nPts, dim);

  double eps = 0;
  double scale = 2;

  for(int i = 0; i < nPts; ++i)
  {
    float C[3][3];
    for(int j = 0; j < 3; ++j)
    {
      for(int k = 0; k < 3; ++k)
        C[j][k] = 0;
    }

    for(int j = 0; j < 3; ++j)
      queryPt[j] = vertices[i][j];

    kdTree->annkSearch(queryPt, knn_k, nnIdx, dists, eps);

    for(int j = 0; j < knn_k; ++j)
      dists[j] = sqrt(dists[j]);

    double r = dists[knn_k - 1];
    double sigma = scale * r;

    double weight_sum = 0;
    for(int j = 1; j < knn_k; ++j)
    {
      int id = nnIdx[j];
      Vec3f p = vertices[i] - vertices[id];
      double norm2p = p.sqrLength();
      double weight = exp(-norm2p / (sigma * sigma));

      weight_sum += weight;

      for(int k = 0; k < 3; ++k)
      {
        for(int l = 0; l < 3; ++l)
          C[k][l] += p[k] * p[l] * weight;
      }
    }

    for(int j = 0; j < 3; ++j)
    {
      for(int k = 0; k < 3; ++k)
      {
        C[j][k] /= weight_sum;
        ucs[i].Sigma[j][k] = C[j][k];
      }
    }

    ucs[i].preprocess();
    ucs[i].sqrt();
  }

  delete [] nnIdx;
  delete [] dists;
  delete kdTree;
  annClose();
}

/** \brief Compute the covariance matrix for a set or subset of points. */
void getCovariance(Vec3f* ps, Vec3f* ps2, unsigned int* indices, int n, Vec3f M[3])
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  Vec3f S1;
  Vec3f S2[3];

  for(int i = 0; i < n; ++i)
  {
    const Vec3f& p = indirect_index ? ps[indices[i]] : ps[i];
    S1 += p;
    S2[0][0] += (p[0] * p[0]);
    S2[1][1] += (p[1] * p[1]);
    S2[2][2] += (p[2] * p[2]);
    S2[0][1] += (p[0] * p[1]);
    S2[0][2] += (p[0] * p[2]);
    S2[1][2] += (p[1] * p[2]);

    if(ps2) // another frame
    {
      const Vec3f& p = indirect_index ? ps2[indices[i]] : ps2[i];
      S1 += p;
      S2[0][0] += (p[0] * p[0]);
      S2[1][1] += (p[1] * p[1]);
      S2[2][2] += (p[2] * p[2]);
      S2[0][1] += (p[0] * p[1]);
      S2[0][2] += (p[0] * p[2]);
      S2[1][2] += (p[1] * p[2]);
    }
  }

  int n_points = ((ps2 == NULL) ? n : 2*n);

  M[0][0] = S2[0][0] - S1[0]*S1[0] / n_points;
  M[1][1] = S2[1][1] - S1[1]*S1[1] / n_points;
  M[2][2] = S2[2][2] - S1[2]*S1[2] / n_points;
  M[0][1] = S2[0][1] - S1[0]*S1[1] / n_points;
  M[1][2] = S2[1][2] - S1[1]*S1[2] / n_points;
  M[0][2] = S2[0][2] - S1[0]*S1[2] / n_points;
  M[1][0] = M[0][1];
  M[2][0] = M[0][2];
  M[2][1] = M[1][2];
}

/** \brief Compute the covariance matrix for a set or subset of triangles. */
void getCovariance(Vec3f* ps, Vec3f* ps2, Triangle* ts, unsigned int* indices, int n, Vec3f M[3])
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  Vec3f S1;
  Vec3f S2[3];

  for(int i = 0; i < n; ++i)
  {
    const Triangle& t = indirect_index ? ts[indices[i]] : ts[i];

    const Vec3f& p1 = ps[t[0]];
    const Vec3f& p2 = ps[t[1]];
    const Vec3f& p3 = ps[t[2]];

    S1[0] += (p1[0] + p2[0] + p3[0]);
    S1[1] += (p1[1] + p2[1] + p3[1]);
    S1[2] += (p1[2] + p2[2] + p3[2]);

    S2[0][0] += (p1[0] * p1[0] +
                 p2[0] * p2[0] +
                 p3[0] * p3[0]);
    S2[1][1] += (p1[1] * p1[1] +
                 p2[1] * p2[1] +
                 p3[1] * p3[1]);
    S2[2][2] += (p1[2] * p1[2] +
                 p2[2] * p2[2] +
                 p3[2] * p3[2]);
    S2[0][1] += (p1[0] * p1[1] +
                 p2[0] * p2[1] +
                 p3[0] * p3[1]);
    S2[0][2] += (p1[0] * p1[2] +
                 p2[0] * p2[2] +
                 p3[0] * p3[2]);
    S2[1][2] += (p1[1] * p1[2] +
                 p2[1] * p2[2] +
                 p3[1] * p3[2]);

    if(ps2)
    {
      const Vec3f& p1 = ps2[t[0]];
      const Vec3f& p2 = ps2[t[1]];
      const Vec3f& p3 = ps2[t[2]];

      S1[0] += (p1[0] + p2[0] + p3[0]);
      S1[1] += (p1[1] + p2[1] + p3[1]);
      S1[2] += (p1[2] + p2[2] + p3[2]);

      S2[0][0] += (p1[0] * p1[0] +
                   p2[0] * p2[0] +
                   p3[0] * p3[0]);
      S2[1][1] += (p1[1] * p1[1] +
                   p2[1] * p2[1] +
                   p3[1] * p3[1]);
      S2[2][2] += (p1[2] * p1[2] +
                   p2[2] * p2[2] +
                   p3[2] * p3[2]);
      S2[0][1] += (p1[0] * p1[1] +
                   p2[0] * p2[1] +
                   p3[0] * p3[1]);
      S2[0][2] += (p1[0] * p1[2] +
                   p2[0] * p2[2] +
                   p3[0] * p3[2]);
      S2[1][2] += (p1[1] * p1[2] +
                   p2[1] * p2[2] +
                   p3[1] * p3[2]);
    }
  }

  int n_points = ((ps2 == NULL) ? 3*n : 6*n);

  M[0][0] = S2[0][0] - S1[0]*S1[0] / n_points;
  M[1][1] = S2[1][1] - S1[1]*S1[1] / n_points;
  M[2][2] = S2[2][2] - S1[2]*S1[2] / n_points;
  M[0][1] = S2[0][1] - S1[0]*S1[1] / n_points;
  M[1][2] = S2[1][2] - S1[1]*S1[2] / n_points;
  M[0][2] = S2[0][2] - S1[0]*S1[2] / n_points;
  M[1][0] = M[0][1];
  M[2][0] = M[0][2];
  M[2][1] = M[1][2];
}


/** \brief Compute the RSS bounding volume parameters: radius, rectangle size and the origin.
 * The bounding volume axes are known.
 */
void getRadiusAndOriginAndRectangleSize(Vec3f* ps, Vec3f* ps2, unsigned int* indices, int n, Vec3f axis[3], Vec3f& origin, BVH_REAL l[2], BVH_REAL& r)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  int size_P = (ps2 == NULL) ? n : (2 * n);

  BVH_REAL (*P)[3] = new BVH_REAL[size_P][3];


  int P_id = 0;
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

  BVH_REAL minx, maxx, miny, maxy, minz, maxz;

  BVH_REAL cz, radsqr;

  minz = maxz = P[0][2];

  for(int i = 1; i < size_P; ++i)
  {
    BVH_REAL z_value = P[i][2];
    if(z_value < minz) minz = z_value;
    else if(z_value > maxz) maxz = z_value;
  }

  r = (BVH_REAL)0.5 * (maxz - minz);
  radsqr = r * r;
  cz = (BVH_REAL)0.5 * (maxz + minz);

  // compute an initial length of rectangle along x direction

  // find minx and maxx as starting points

  int minindex, maxindex;
  minindex = maxindex = 0;
  BVH_REAL mintmp, maxtmp;
  mintmp = maxtmp = P[0][0];

  for(int i = 1; i < size_P; ++i)
  {
    BVH_REAL x_value = P[i][0];
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

  BVH_REAL x, dz;
  dz = P[minindex][2] - cz;
  minx = P[minindex][0] + sqrt(std::max(radsqr - dz * dz, 0.0));
  dz = P[maxindex][2] - cz;
  maxx = P[maxindex][0] - sqrt(std::max(radsqr - dz * dz, 0.0));


  // grow minx

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] < minx)
    {
      dz = P[i][2] - cz;
      x = P[i][0] + sqrt(std::max(radsqr - dz * dz, 0.0));
      if(x < minx) minx = x;
    }
  }

  // grow maxx

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] > maxx)
    {
      dz = P[i][2] - cz;
      x = P[i][0] - sqrt(std::max(radsqr - dz * dz, 0.0));
      if(x > maxx) maxx = x;
    }
  }

  // compute an initial length of rectangle along y direction

  // find miny and maxy as starting points

  minindex = maxindex = 0;
  mintmp = maxtmp = P[0][1];
  for(int i = 1; i < size_P; ++i)
  {
    BVH_REAL y_value = P[i][1];
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

  BVH_REAL y;
  dz = P[minindex][2] - cz;
  miny = P[minindex][1] + sqrt(std::max(radsqr - dz * dz, 0.0));
  dz = P[maxindex][2] - cz;
  maxy = P[maxindex][1] - sqrt(std::max(radsqr - dz * dz, 0.0));

  // grow miny

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][1] < miny)
    {
      dz = P[i][2] - cz;
      y = P[i][1] + sqrt(std::max(radsqr - dz * dz, 0.0));
      if(y < miny) miny = y;
    }
  }

  // grow maxy

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][1] > maxy)
    {
      dz = P[i][2] - cz;
      y = P[i][1] - sqrt(std::max(radsqr - dz * dz, 0.0));
      if(y > maxy) maxy = y;
    }
  }

  // corners may have some points which are not covered - grow lengths if necessary
  // quite conservative (can be improved)

  BVH_REAL dx, dy, u, t;
  BVH_REAL a = sqrt((BVH_REAL)0.5);
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
        u = u - sqrt(std::max(radsqr - t, 0.0));
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
        u = u - sqrt(std::max(radsqr - t, 0.0));
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
        u = u - sqrt(std::max(radsqr - t, 0.0));
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
        u = u - sqrt(std::max(radsqr - t, 0.0));
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


/** \brief Compute the RSS bounding volume parameters: radius, rectangle size and the origin.
 * The bounding volume axes are known.
 */
void getRadiusAndOriginAndRectangleSize(Vec3f* ps, Vec3f* ps2, Triangle* ts, unsigned int* indices, int n, Vec3f axis[3], Vec3f& origin, BVH_REAL l[2], BVH_REAL& r)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  int size_P = (ps2 == NULL) ? n * 3 : (2 * n * 3);

  BVH_REAL (*P)[3] = new BVH_REAL[size_P][3];


  int P_id = 0;
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

  BVH_REAL minx, maxx, miny, maxy, minz, maxz;

  BVH_REAL cz, radsqr;

  minz = maxz = P[0][2];

  for(int i = 1; i < size_P; ++i)
  {
    BVH_REAL z_value = P[i][2];
    if(z_value < minz) minz = z_value;
    else if(z_value > maxz) maxz = z_value;
  }

  r = (BVH_REAL)0.5 * (maxz - minz);
  radsqr = r * r;
  cz = (BVH_REAL)0.5 * (maxz + minz);

  // compute an initial length of rectangle along x direction

  // find minx and maxx as starting points

  int minindex, maxindex;
  minindex = maxindex = 0;
  BVH_REAL mintmp, maxtmp;
  mintmp = maxtmp = P[0][0];

  for(int i = 1; i < size_P; ++i)
  {
    BVH_REAL x_value = P[i][0];
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

  BVH_REAL x, dz;
  dz = P[minindex][2] - cz;
  minx = P[minindex][0] + sqrt(std::max(radsqr - dz * dz, 0.0));
  dz = P[maxindex][2] - cz;
  maxx = P[maxindex][0] - sqrt(std::max(radsqr - dz * dz, 0.0));


  // grow minx

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] < minx)
    {
      dz = P[i][2] - cz;
      x = P[i][0] + sqrt(std::max(radsqr - dz * dz, 0.0));
      if(x < minx) minx = x;
    }
  }

  // grow maxx

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] > maxx)
    {
      dz = P[i][2] - cz;
      x = P[i][0] - sqrt(std::max(radsqr - dz * dz, 0.0));
      if(x > maxx) maxx = x;
    }
  }

  // compute an initial length of rectangle along y direction

  // find miny and maxy as starting points

  minindex = maxindex = 0;
  mintmp = maxtmp = P[0][1];
  for(int i = 1; i < size_P; ++i)
  {
    BVH_REAL y_value = P[i][1];
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

  BVH_REAL y;
  dz = P[minindex][2] - cz;
  miny = P[minindex][1] + sqrt(std::max(radsqr - dz * dz, 0.0));
  dz = P[maxindex][2] - cz;
  maxy = P[maxindex][1] - sqrt(std::max(radsqr - dz * dz, 0.0));

  // grow miny

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][1] < miny)
    {
      dz = P[i][2] - cz;
      y = P[i][1] + sqrt(std::max(radsqr - dz * dz, 0.0));
      if(y < miny) miny = y;
    }
  }

  // grow maxy

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][1] > maxy)
    {
      dz = P[i][2] - cz;
      y = P[i][1] - sqrt(std::max(radsqr - dz * dz, 0.0));
      if(y > maxy) maxy = y;
    }
  }

  // corners may have some points which are not covered - grow lengths if necessary
  // quite conservative (can be improved)
  BVH_REAL dx, dy, u, t;
  BVH_REAL a = sqrt((BVH_REAL)0.5);
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
        u = u - sqrt(std::max(radsqr - t, 0.0));
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
        u = u - sqrt(std::max(radsqr - t, 0.0));
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
        u = u - sqrt(std::max(radsqr - t, 0.0));
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
        u = u - sqrt(std::max(radsqr - t, 0.0));
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
void getExtentAndCenter(Vec3f* ps, Vec3f* ps2, unsigned int* indices, int n, Vec3f axis[3], Vec3f& center, Vec3f& extent)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  BVH_REAL real_max = std::numeric_limits<BVH_REAL>::max();

  BVH_REAL min_coord[3] = {real_max, real_max, real_max};
  BVH_REAL max_coord[3] = {-real_max, -real_max, -real_max};

  for(int i = 0; i < n; ++i)
  {
    int index = indirect_index ? indices[i] : i;

    const Vec3f& p = ps[index];
    Vec3f v(p[0], p[1], p[2]);
    BVH_REAL proj[3];
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

  Vec3f o = Vec3f((max_coord[0] + min_coord[0]) / 2,
                 (max_coord[1] + min_coord[1]) / 2,
                 (max_coord[2] + min_coord[2]) / 2);

  center = axis[0] * o[0] + axis[1] * o[1] + axis[2] * o[2];

  extent = Vec3f((max_coord[0] - min_coord[0]) / 2,
                 (max_coord[1] - min_coord[1]) / 2,
                 (max_coord[2] - min_coord[2]) / 2);

}


/** \brief Compute the bounding volume extent and center for a set or subset of points.
 * The bounding volume axes are known.
 */
void getExtentAndCenter(Vec3f* ps, Vec3f* ps2, Triangle* ts, unsigned int* indices, int n, Vec3f axis[3], Vec3f& center, Vec3f& extent)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  BVH_REAL real_max = std::numeric_limits<BVH_REAL>::max();

  BVH_REAL min_coord[3] = {real_max, real_max, real_max};
  BVH_REAL max_coord[3] = {-real_max, -real_max, -real_max};

  for(int i = 0; i < n; ++i)
  {
    unsigned int index = indirect_index? indices[i] : i;
    const Triangle& t = ts[index];

    for(int j = 0; j < 3; ++j)
    {
      int point_id = t[j];
      const Vec3f& p = ps[point_id];
      Vec3f v(p[0], p[1], p[2]);
      BVH_REAL proj[3];
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
        BVH_REAL proj[3];
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

  Vec3f o = Vec3f((max_coord[0] + min_coord[0]) / 2,
                 (max_coord[1] + min_coord[1]) / 2,
                 (max_coord[2] + min_coord[2]) / 2);

  center = axis[0] * o[0] + axis[1] * o[1] + axis[2] * o[2];

  extent = Vec3f((max_coord[0] - min_coord[0]) / 2,
                 (max_coord[1] - min_coord[1]) / 2,
                 (max_coord[2] - min_coord[2]) / 2);

}



}
