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


#ifndef FCL_TEST_CORE_UTILITY_H
#define FCL_TEST_CORE_UTILITY_H

#include "fcl/vec_3f.h"
#include "fcl/matrix_3f.h"
#include "fcl/primitive.h"
#include <cstdio>
#include <vector>
#include <iostream>
#include <string.h>
using namespace fcl;

#if USE_PQP
#include <PQP.h>
#endif

struct Transform
{
  Matrix3f R;
  Vec3f T;

  Transform()
  {
    R.setIdentity();
  }
};


#if USE_PQP

struct CollisionPairComp_PQP
{
  bool operator()(const CollisionPair& a, const CollisionPair& b)
  {
    if(a.id1 == b.id1)
      return a.id2 < b.id2;
    return a.id1 < b.id1;
  }
};

inline void sortCollisionPair_PQP(CollisionPair* pairs, int n)
{
  std::vector<CollisionPair> pair_array(n);
  for(int i = 0; i < n; ++i)
    pair_array[i] = pairs[i];

  std::sort(pair_array.begin(), pair_array.end(), CollisionPairComp_PQP());

  for(int i = 0; i < n; ++i)
    pairs[i] = pair_array[i];
}

inline bool collide_PQP(const Transform& tf,
                        const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                        const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                        std::vector<std::pair<int, int> >& results,
                        bool verbose = true)
{
  PQP_Model m1, m2;

  m1.BeginModel();
  for(unsigned int i = 0; i < triangles1.size(); ++i)
  {
    const Triangle& t = triangles1[i];
    const Vec3f& p1_ = vertices1[t[0]];
    const Vec3f& p2_ = vertices1[t[1]];
    const Vec3f& p3_ = vertices1[t[2]];

    Vec3f p1 = tf.R * p1_ + tf.T;
    Vec3f p2 = tf.R * p2_ + tf.T;
    Vec3f p3 = tf.R * p3_ + tf.T;

    PQP_REAL q1[3];
    PQP_REAL q2[3];
    PQP_REAL q3[3];

    q1[0] = p1[0]; q1[1] = p1[1]; q1[2] = p1[2];
    q2[0] = p2[0]; q2[1] = p2[1]; q2[2] = p2[2];
    q3[0] = p3[0]; q3[1] = p3[1]; q3[2] = p3[2];

    m1.AddTri(q1, q2, q3, i);
  }

  m1.EndModel();


  m2.BeginModel();
  for(unsigned int i = 0; i < triangles2.size(); ++i)
  {
    const Triangle& t = triangles2[i];
    const Vec3f& p1 = vertices2[t[0]];
    const Vec3f& p2 = vertices2[t[1]];
    const Vec3f& p3 = vertices2[t[2]];

    PQP_REAL q1[3];
    PQP_REAL q2[3];
    PQP_REAL q3[3];

    q1[0] = p1[0]; q1[1] = p1[1]; q1[2] = p1[2];
    q2[0] = p2[0]; q2[1] = p2[1]; q2[2] = p2[2];
    q3[0] = p3[0]; q3[1] = p3[1]; q3[2] = p3[2];

    m2.AddTri(q1, q2, q3, i);
  }

  m2.EndModel();


  PQP_CollideResult res;
  PQP_REAL R1[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  PQP_REAL R2[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  PQP_REAL T1[3] = {0, 0, 0};
  PQP_REAL T2[3] = {0, 0, 0};
  PQP_Collide(&res, R1, T1, &m1, R2, T2, &m2);

  if(res.NumPairs() > 0)
  {
    sortCollisionPair_PQP(res.pairs, res.NumPairs());
    results.resize(res.NumPairs());

    for(int i = 0; i < res.NumPairs(); ++i)
    {
      results[i] = std::make_pair(res.Id1(i), res.Id2(i));
    }

    if(verbose)
      std::cout << "in collision " << res.NumPairs() << ": " << std::endl;
    if(verbose) std::cout << res.num_bv_tests << " " << res.num_tri_tests << std::endl;
    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << res.num_bv_tests << " " << res.num_tri_tests << std::endl;
    return false;
  }
}


inline bool collide_PQP2(const Transform& tf,
                         const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                         const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                         std::vector<std::pair<int, int> >& results, bool verbose = true)
{
  PQP_Model m1, m2;

  m1.BeginModel();
  for(unsigned int i = 0; i < triangles1.size(); ++i)
  {
    const Triangle& t = triangles1[i];
    const Vec3f& p1 = vertices1[t[0]];
    const Vec3f& p2 = vertices1[t[1]];
    const Vec3f& p3 = vertices1[t[2]];

    PQP_REAL q1[3];
    PQP_REAL q2[3];
    PQP_REAL q3[3];

    q1[0] = p1[0]; q1[1] = p1[1]; q1[2] = p1[2];
    q2[0] = p2[0]; q2[1] = p2[1]; q2[2] = p2[2];
    q3[0] = p3[0]; q3[1] = p3[1]; q3[2] = p3[2];

    m1.AddTri(q1, q2, q3, i);
  }

  m1.EndModel();


  m2.BeginModel();
  for(unsigned int i = 0; i < triangles2.size(); ++i)
  {
    const Triangle& t = triangles2[i];
    const Vec3f& p1 = vertices2[t[0]];
    const Vec3f& p2 = vertices2[t[1]];
    const Vec3f& p3 = vertices2[t[2]];

    PQP_REAL q1[3];
    PQP_REAL q2[3];
    PQP_REAL q3[3];

    q1[0] = p1[0]; q1[1] = p1[1]; q1[2] = p1[2];
    q2[0] = p2[0]; q2[1] = p2[1]; q2[2] = p2[2];
    q3[0] = p3[0]; q3[1] = p3[1]; q3[2] = p3[2];

    m2.AddTri(q1, q2, q3, i);
  }

  m2.EndModel();


  PQP_CollideResult res;
  PQP_REAL R1[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  PQP_REAL R2[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  PQP_REAL T1[3] = {0, 0, 0};
  PQP_REAL T2[3] = {0, 0, 0};
  T1[0] = tf.T[0];
  T1[1] = tf.T[1];
  T1[2] = tf.T[2];
  for(int i = 0; i < 3; ++i)
  {
    R1[i][0] = tf.R[i][0];
    R1[i][1] = tf.R[i][1];
    R1[i][2] = tf.R[i][2];
  }

  PQP_Collide(&res, R1, T1, &m1, R2, T2, &m2);

  if(res.NumPairs() > 0)
  {
    sortCollisionPair_PQP(res.pairs, res.NumPairs());
    results.resize(res.NumPairs());
    for(int i = 0; i < res.NumPairs(); ++i)
    {
      results[i] = std::make_pair(res.Id1(i), res.Id2(i));
    }

    if(verbose)
      std::cout << "in collision " << res.NumPairs() << ": " << std::endl;
    if(verbose) std::cout << res.num_bv_tests << " " << res.num_tri_tests << std::endl;
    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << res.num_bv_tests << " " << res.num_tri_tests << std::endl;
    return false;
  }
}

#endif


struct DistanceRes
{
  double distance;
  Vec3f p1;
  Vec3f p2;
};

#if USE_PQP
inline bool distance_PQP(const Transform& tf,
                         const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                         const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                         DistanceRes& distance_result,
                         bool verbose = true)
{
  PQP_Model m1, m2;

  m1.BeginModel();
  for(unsigned int i = 0; i < triangles1.size(); ++i)
  {
    const Triangle& t = triangles1[i];
    const Vec3f& p1_ = vertices1[t[0]];
    const Vec3f& p2_ = vertices1[t[1]];
    const Vec3f& p3_ = vertices1[t[2]];

    Vec3f p1 = tf.R * p1_ + tf.T;
    Vec3f p2 = tf.R * p2_ + tf.T;
    Vec3f p3 = tf.R * p3_ + tf.T;

    PQP_REAL q1[3];
    PQP_REAL q2[3];
    PQP_REAL q3[3];

    q1[0] = p1[0]; q1[1] = p1[1]; q1[2] = p1[2];
    q2[0] = p2[0]; q2[1] = p2[1]; q2[2] = p2[2];
    q3[0] = p3[0]; q3[1] = p3[1]; q3[2] = p3[2];

    m1.AddTri(q1, q2, q3, i);
  }

  m1.EndModel();


  m2.BeginModel();
  for(unsigned int i = 0; i < triangles2.size(); ++i)
  {
    const Triangle& t = triangles2[i];
    const Vec3f& p1 = vertices2[t[0]];
    const Vec3f& p2 = vertices2[t[1]];
    const Vec3f& p3 = vertices2[t[2]];

    PQP_REAL q1[3];
    PQP_REAL q2[3];
    PQP_REAL q3[3];

    q1[0] = p1[0]; q1[1] = p1[1]; q1[2] = p1[2];
    q2[0] = p2[0]; q2[1] = p2[1]; q2[2] = p2[2];
    q3[0] = p3[0]; q3[1] = p3[1]; q3[2] = p3[2];

    m2.AddTri(q1, q2, q3, i);
  }

  m2.EndModel();


  PQP_DistanceResult res;
  PQP_REAL R1[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  PQP_REAL R2[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  PQP_REAL T1[3] = {0, 0, 0};
  PQP_REAL T2[3] = {0, 0, 0};

  PQP_Distance(&res, R1, T1, &m1, R2, T2, &m2, 0.01, 0.01);


  distance_result.distance = res.distance;
  distance_result.p1 = Vec3f(res.p1[0], res.p1[1], res.p1[2]);
  distance_result.p2 = Vec3f(res.p2[0], res.p2[1], res.p2[2]);


  if(verbose)
  {
    std::cout << "distance " << res.distance << std::endl;
    std::cout << res.p1[0] << " " << res.p1[1] << " " << res.p1[2] << std::endl;
    std::cout << res.p2[0] << " " << res.p2[1] << " " << res.p2[2] << std::endl;
    std::cout << res.num_bv_tests << " " << res.num_tri_tests << std::endl;
  }

  return true;
}


inline bool distance_PQP2(const Transform& tf,
                          const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                          const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2,
                          DistanceRes& distance_result, bool verbose = true)
{
  PQP_Model m1, m2;

  m1.BeginModel();
  for(unsigned int i = 0; i < triangles1.size(); ++i)
  {
    const Triangle& t = triangles1[i];
    const Vec3f& p1 = vertices1[t[0]];
    const Vec3f& p2 = vertices1[t[1]];
    const Vec3f& p3 = vertices1[t[2]];

    PQP_REAL q1[3];
    PQP_REAL q2[3];
    PQP_REAL q3[3];

    q1[0] = p1[0]; q1[1] = p1[1]; q1[2] = p1[2];
    q2[0] = p2[0]; q2[1] = p2[1]; q2[2] = p2[2];
    q3[0] = p3[0]; q3[1] = p3[1]; q3[2] = p3[2];

    m1.AddTri(q1, q2, q3, i);
  }

  m1.EndModel();


  m2.BeginModel();
  for(unsigned int i = 0; i < triangles2.size(); ++i)
  {
    const Triangle& t = triangles2[i];
    const Vec3f& p1 = vertices2[t[0]];
    const Vec3f& p2 = vertices2[t[1]];
    const Vec3f& p3 = vertices2[t[2]];

    PQP_REAL q1[3];
    PQP_REAL q2[3];
    PQP_REAL q3[3];

    q1[0] = p1[0]; q1[1] = p1[1]; q1[2] = p1[2];
    q2[0] = p2[0]; q2[1] = p2[1]; q2[2] = p2[2];
    q3[0] = p3[0]; q3[1] = p3[1]; q3[2] = p3[2];

    m2.AddTri(q1, q2, q3, i);
  }

  m2.EndModel();


  PQP_DistanceResult res;
  PQP_REAL R1[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  PQP_REAL R2[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  PQP_REAL T1[3] = {0, 0, 0};
  PQP_REAL T2[3] = {0, 0, 0};

  T1[0] = tf.T[0];
  T1[1] = tf.T[1];
  T1[2] = tf.T[2];
  for(int i = 0; i < 3; ++i)
  {
    R1[i][0] = tf.R[i][0];
    R1[i][1] = tf.R[i][1];
    R1[i][2] = tf.R[i][2];
  }

  PQP_Distance(&res, R1, T1, &m1, R2, T2, &m2, 0.01, 0.01);


  Vec3f p1_temp(res.p1[0], res.p1[1], res.p1[2]);
  Vec3f p2_temp(res.p2[0], res.p2[1], res.p2[2]);

  Vec3f p1 = tf.R * p1_temp + tf.T;
  Vec3f p2 = p2_temp;


  distance_result.distance = res.distance;
  distance_result.p1 = p1;
  distance_result.p2 = p2;


  if(verbose)
  {
    std::cout << "distance " << res.distance << std::endl;
    std::cout << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;
    std::cout << p2[0] << " " << p2[1] << " " << p2[2] << std::endl;
    std::cout << res.num_bv_tests << " " << res.num_tri_tests << std::endl;
  }

  return true;
}

#endif

inline BVH_REAL rand_interval(BVH_REAL rmin, BVH_REAL rmax)
{
  BVH_REAL t = rand() / ((BVH_REAL)RAND_MAX + 1);
  return (t * (rmax - rmin) + rmin);
}

inline void loadOBJFile(const char* filename, std::vector<Vec3f>& points, std::vector<Triangle>& triangles)
{

  FILE* file = fopen(filename, "rb");
  if(!file)
  {
    std::cerr << "file not exist" << std::endl;
    return;
  }

  bool has_normal = false;
  bool has_texture = false;
  char line_buffer[2000];
  while(fgets(line_buffer, 2000, file))
  {
    char* first_token = strtok(line_buffer, "\r\n\t ");
    if(!first_token || first_token[0] == '#' || first_token[0] == 0)
      continue;

    switch(first_token[0])
    {
      case 'v':
      {
        if(first_token[1] == 'n')
        {
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          has_normal = true;
        }
        else if(first_token[1] == 't')
        {
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          has_texture = true;
        }
        else
        {
          BVH_REAL x = (BVH_REAL)atof(strtok(NULL, "\t "));
          BVH_REAL y = (BVH_REAL)atof(strtok(NULL, "\t "));
          BVH_REAL z = (BVH_REAL)atof(strtok(NULL, "\t "));
          Vec3f p(x, y, z);
          points.push_back(p);
        }
      }
      break;
      case 'f':
      {
        Triangle tri;
        char* data[30];
        int n = 0;
        while((data[n] = strtok(NULL, "\t \r\n")) != NULL)
        {
          if(strlen(data[n]))
            n++;
        }

        for(int t = 0; t < (n - 2); ++t)
        {
          if((!has_texture) && (!has_normal))
          {
            tri[0] = atoi(data[0]) - 1;
            tri[1] = atoi(data[1]) - 1;
            tri[2] = atoi(data[2]) - 1;
          }
          else
          {
            const char *v1;
            for(int i = 0; i < 3; i++)
            {
               // vertex ID
               if(i == 0)
                 v1 = data[0];
               else
                 v1 = data[t + i];

               tri[i] = atoi(v1) - 1;

               // texture coordinate
               const char *v2 = strchr(v1 + 1, '/');

               if(v2)
               {
                 strchr(v2 + 1, '/');
               }
            }
          }
          triangles.push_back(tri);
        }
      }
    }
  }
}


inline void eulerToMatrix(BVH_REAL a, BVH_REAL b, BVH_REAL c, Matrix3f& R)
{
  BVH_REAL c1 = cos(a);
  BVH_REAL c2 = cos(b);
  BVH_REAL c3 = cos(c);
  BVH_REAL s1 = sin(a);
  BVH_REAL s2 = sin(b);
  BVH_REAL s3 = sin(c);

  R.setValue(c1 * c2, - c2 * s1, s2,
             c3 * s1 + c1 * s2 * s3, c1 * c3 - s1 * s2 * s3, - c2 * s3,
             s1 * s3 - c1 * c3 * s2, c3 * s1 * s2 + c1 * s3, c2 * c3);
}

inline void generateRandomTransform(BVH_REAL extents[6], Transform& transform)
{
  BVH_REAL x = rand_interval(extents[0], extents[3]);
  BVH_REAL y = rand_interval(extents[1], extents[4]);
  BVH_REAL z = rand_interval(extents[2], extents[5]);

  const BVH_REAL pi = 3.1415926;
  BVH_REAL a = rand_interval(0, 2 * pi);
  BVH_REAL b = rand_interval(0, 2 * pi);
  BVH_REAL c = rand_interval(0, 2 * pi);

  eulerToMatrix(a, b, c, transform.R);
  transform.T.setValue(x, y, z);
}

inline void generateRandomTransform(BVH_REAL extents[6], std::vector<Transform>& transforms, std::vector<Transform>& transforms2, BVH_REAL delta_trans[3], BVH_REAL delta_rot, int n)
{
  transforms.resize(n);
  transforms2.resize(n);
  for(int i = 0; i < n; ++i)
  {
    BVH_REAL x = rand_interval(extents[0], extents[3]);
    BVH_REAL y = rand_interval(extents[1], extents[4]);
    BVH_REAL z = rand_interval(extents[2], extents[5]);

    const BVH_REAL pi = 3.1415926;
    BVH_REAL a = rand_interval(0, 2 * pi);
    BVH_REAL b = rand_interval(0, 2 * pi);
    BVH_REAL c = rand_interval(0, 2 * pi);

    eulerToMatrix(a, b, c, transforms[i].R);
    transforms[i].T.setValue(x, y, z);

    BVH_REAL deltax = rand_interval(-delta_trans[0], delta_trans[0]);
    BVH_REAL deltay = rand_interval(-delta_trans[1], delta_trans[1]);
    BVH_REAL deltaz = rand_interval(-delta_trans[2], delta_trans[2]);

    BVH_REAL deltaa = rand_interval(-delta_rot, delta_rot);
    BVH_REAL deltab = rand_interval(-delta_rot, delta_rot);
    BVH_REAL deltac = rand_interval(-delta_rot, delta_rot);

    eulerToMatrix(a + deltaa, b + deltab, c + deltac, transforms2[i].R);
    transforms2[i].T.setValue(x + deltax, y + deltay, z + deltaz);
  }
}


inline void generateRandomTransform_ccd(BVH_REAL extents[6], std::vector<Transform>& transforms, std::vector<Transform>& transforms2, BVH_REAL delta_trans[3], BVH_REAL delta_rot, int n,
                                        const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                                        const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2)
{
  transforms.resize(n);
  transforms2.resize(n);

  for(int i = 0; i < n;)
  {
    BVH_REAL x = rand_interval(extents[0], extents[3]);
    BVH_REAL y = rand_interval(extents[1], extents[4]);
    BVH_REAL z = rand_interval(extents[2], extents[5]);

    const BVH_REAL pi = 3.1415926;
    BVH_REAL a = rand_interval(0, 2 * pi);
    BVH_REAL b = rand_interval(0, 2 * pi);
    BVH_REAL c = rand_interval(0, 2 * pi);

    Transform tf;

    eulerToMatrix(a, b, c, tf.R);
    tf.T.setValue(x, y, z);

    std::vector<std::pair<int, int> > results;
#if USE_PQP
    if(!collide_PQP(tf, vertices1, triangles1, vertices2, triangles2, results, false))
#endif
    {
      transforms[i] = tf;

      BVH_REAL deltax = rand_interval(-delta_trans[0], delta_trans[0]);
      BVH_REAL deltay = rand_interval(-delta_trans[1], delta_trans[1]);
      BVH_REAL deltaz = rand_interval(-delta_trans[2], delta_trans[2]);

      BVH_REAL deltaa = rand_interval(-delta_rot, delta_rot);
      BVH_REAL deltab = rand_interval(-delta_rot, delta_rot);
      BVH_REAL deltac = rand_interval(-delta_rot, delta_rot);

      eulerToMatrix(a + deltaa, b + deltab, c + deltac, transforms2[i].R);
      transforms2[i].T.setValue(x + deltax, y + deltay, z + deltaz);
      ++i;
    }
  }
}




#endif
