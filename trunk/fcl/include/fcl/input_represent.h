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

#ifndef FCL_INPUT_REPRESENT
#define FCL_INPUT_REPRESENT

#include "fcl/data_types.h"
#include "fcl/vec_3f.h"


namespace fcl
{


struct PointFunctor
{
  virtual void operator() (const Vec3f& v);
};

struct VarianceStatistics : public PointFunctor
{
  VarianceStatistics()
  {
    for(unsigned int i = 0; i < 3; ++i)
      S[i] = Vec3f();
    n = 0;
  }
  
  void operator() (const Vec3f& v)
  {
    m += v;
    S[0][0] += v[0] * v[0];
    S[0][1] += v[0] * v[1];
    S[0][2] += v[0] * v[2];
    S[1][1] += v[1] * v[1];
    S[1][2] += v[1] * v[2];
    S[2][2] += v[2] * v[2];
    n++;
  }

  void reduce(Vec3f& mean, Matrix3f& var)
  {
    mean = m;
    var[0][0] = S[0][0] - m[0] * m[0] / n;
    var[1][1] = S[1][1] - m[1] * m[1] / n;
    var[2][2] = S[2][2] - m[2] * m[2] / n;
    var[0][1] = S[0][1] - m[0] * m[1] / n;
    var[0][2] = S[0][2] - m[0] * m[2] / n;
    var[1][2] = S[1][2] - m[1] * m[2] / n;
  }

  Vec3f m;
  Vec3f S[3];
  unsigned int n;
};

class Mesh
{
public:
  typedef PrimitiveType Triangle;
  
  Vec3f* vertices;
  Vec3f* prev_vertices;
  Triangle* triangles;

  unsigned int n_vertices;
  unsigned int n_tris;

  Mesh(Vec3f* vertices_, Vec3f* prev_vertices_, Triangle* triangles_)
  {
    vertices = vertices_;
    prev_vertices = prev_vertices_;
    triangles = triangles_;
  }

  void operate(PointFunctor& func)
  {
    for(unsigned int i = 0; i < n_tris; ++i)
    {
      const Triangle& t = *(triangles + i);
      
      for(unsigned int j = 0; j < 3; ++j)
      {
        unsigned int point_id = t[j];
        func(vertices[point_id]);
        if(prev_vertices) func(prev_vertices[point_id]);
      }
    }
  }

  void operate(PointFunctor& func, unsigned int* indices, unsigned int n)
  {
    for(unsigned int i = 0; i < n; ++i)
    {
      const Triangle& t = *(triangles + indices[i]);
      for(unsigned int j = 0; j < 3; ++j)
      {
        unsigned int point_id = t[j];
        func(vertices[point_id]);
        if(prev_vertices) func(prev_vertices[point_id]);
      }
    }
  }
};

class PointCloud
{
public:
  typedef PrimitiveType Vec3f;

  Vec3f* vertices;
  Vec3f* prev_vertices;

  unsigned int n_vertices;
  
  PointCloud(Vec3f* vertices_, Vec3f* prev_vertices_)
  {
    vertices = vertices_;
    prev_vertices = prev_vertices_;
  }

  void operate(PointFunctor& func)
  {
    for(unsigned int i = 0; i < n_vertices; ++i)
    {
      func(vertices[i]);
      if(prev_vertices) func(prev_vertices[i]);
    }
  }

  void operate(PointFunctor& func, unsigned int* indices, unsigned int n)
  {
    for(unsigned int i = 0; i < n; ++i)
    {
      unsigned int point_id = indices[i];
      func(vertices[point_id]);
      if(prev_vertices) func(prev_vertices[point_id]);
    }
  }
};


}

#endif
