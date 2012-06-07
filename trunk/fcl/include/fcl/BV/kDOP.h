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

#ifndef FCL_KDOP_H
#define FCL_KDOP_H

#include "fcl/BVH_internal.h"
#include "fcl/vec_3f.h"

#include <cstdlib>
#include <limits>
#include <iostream>

/** \brief Main namespace */
namespace fcl
{

/** \brief Find the smaller and larger one of two values */
inline void minmax(BVH_REAL a, BVH_REAL b, BVH_REAL& minv, BVH_REAL& maxv)
{
  if(a > b)
  {
    minv = b;
    maxv = a;
  }
  else
  {
    minv = a;
    maxv = b;
  }
}
/** \brief Merge the interval [minv, maxv] and value p */
inline void minmax(BVH_REAL p, BVH_REAL& minv, BVH_REAL& maxv)
{
  if(p > maxv) maxv = p;
  if(p < minv) minv = p;
}


/** \brief Compute the distances to planes with normals from KDOP vectors except those of AABB face planes */
template<size_t N>
void getDistances(const Vec3f& p, BVH_REAL d[]) {}

/** \brief Specification of getDistances */
template<>
inline void getDistances<5>(const Vec3f& p, BVH_REAL d[])
{
  d[0] = p[0] + p[1];
  d[1] = p[0] + p[2];
  d[2] = p[1] + p[2];
  d[3] = p[0] - p[1];
  d[4] = p[0] - p[2];
}

template<>
inline void getDistances<6>(const Vec3f& p, BVH_REAL d[])
{
  d[0] = p[0] + p[1];
  d[1] = p[0] + p[2];
  d[2] = p[1] + p[2];
  d[3] = p[0] - p[1];
  d[4] = p[0] - p[2];
  d[5] = p[1] - p[2];
}

template<>
inline void getDistances<9>(const Vec3f& p, BVH_REAL d[])
{
  d[0] = p[0] + p[1];
  d[1] = p[0] + p[2];
  d[2] = p[1] + p[2];
  d[3] = p[0] - p[1];
  d[4] = p[0] - p[2];
  d[5] = p[1] - p[2];
  d[6] = p[0] + p[1] - p[2];
  d[7] = p[0] + p[2] - p[1];
  d[8] = p[1] + p[2] - p[0];
}


/** \brief KDOP class describes the KDOP collision structures. K is set as the template parameter, which should be 16, 18, or 24
    The KDOP structure is defined by some pairs of parallel planes defined by some axes. 
    For K = 18, the planes are 6 AABB planes and 12 diagonal planes that cut off some space of the edges:
    (-1,0,0) and (1,0,0)  -> indices 0 and 9
    (0,-1,0) and (0,1,0)  -> indices 1 and 10
    (0,0,-1) and (0,0,1)  -> indices 2 and 11
    (-1,-1,0) and (1,1,0) -> indices 3 and 12
    (-1,0,-1) and (1,0,1) -> indices 4 and 13
    (0,-1,-1) and (0,1,1) -> indices 5 and 14
    (-1,1,0) and (1,-1,0) -> indices 6 and 15
    (-1,0,1) and (1,0,-1) -> indices 7 and 16
    (0,-1,1) and (0,1,-1) -> indices 8 and 17
 */
template<size_t N>
class KDOP
{
public:
  KDOP()
  {
    BVH_REAL real_max = std::numeric_limits<BVH_REAL>::max();
    for(size_t i = 0; i < N / 2; ++i)
    {
      dist_[i] = real_max;
      dist_[i + N / 2] = -real_max;
    }
  }


  KDOP(const Vec3f& v)
  {
    for(size_t i = 0; i < 3; ++i)
    {
      dist_[i] = dist_[N / 2 + i] = v[i];
    }

    BVH_REAL d[(N - 6) / 2];
    getDistances<(N - 6) / 2>(v, d);
    for(size_t i = 0; i < (N - 6) / 2; ++i)
    {
      dist_[3 + i] = dist_[3 + i + N / 2] = d[i];
    }
  }

  KDOP(const Vec3f& a, const Vec3f& b)
  {
    for(size_t i = 0; i < 3; ++i)
    {
      minmax(a[i], b[i], dist_[i], dist_[i + N / 2]);
    }

    BVH_REAL ad[(N - 6) / 2], bd[(N - 6) / 2];
    getDistances<(N - 6) / 2>(a, ad);
    getDistances<(N - 6) / 2>(b, bd);
    for(size_t i = 0; i < (N - 6) / 2; ++i)
    {
      minmax(ad[i], bd[i], dist_[3 + i], dist_[3 + i + N / 2]);
    }
  }
  
  /** \brief Check whether two KDOPs are overlapped */
  inline bool overlap(const KDOP<N>& other) const
  {
    for(size_t i = 0; i < N / 2; ++i)
    {
      if(dist_[i] > other.dist_[i + N / 2]) return false;
      if(dist_[i + N / 2] < other.dist_[i]) return false;
    }

    return true;
  }

  /** \brief Check whether one point is inside the KDOP */
  inline bool inside(const Vec3f& p) const
  {
    for(size_t i = 0; i < 3; ++i)
    {
      if(p[i] < dist_[i] || p[i] > dist_[i + N / 2])
        return false;
    }

    BVH_REAL d[(N - 6) / 2];
    getDistances(p, d);
    for(size_t i = 0; i < (N - 6) / 2; ++i)
    {
      if(d[i] < dist_[3 + i] || d[i] > dist_[i + 3 + N / 2])
        return false;
    }

    return true;
  }

  /** \brief Merge the point and the KDOP */
  inline KDOP<N>& operator += (const Vec3f& p)
  {
    for(size_t i = 0; i < 3; ++i)
    {
      minmax(p[i], dist_[i], dist_[N / 2 + i]);
    }
    
    BVH_REAL pd[(N - 6) / 2];
    getDistances<(N - 6) / 2>(p, pd);
    for(size_t i = 0; i < (N - 6) / 2; ++i)
    {
      minmax(pd[i], dist_[3 + i], dist_[3 + N / 2 + i]);
    }

    return *this;
  }

  /** \brief Merge two KDOPs */
  inline  KDOP<N>& operator += (const KDOP<N>& other)
  {
    for(size_t i = 0; i < N / 2; ++i)
    {
      dist_[i] = std::min(other.dist_[i], dist_[i]);
      dist_[i + N / 2] = std::max(other.dist_[i + N / 2], dist_[i + N / 2]);
    }
    return *this;
  }

  /** \brief Create a KDOP by mergin two KDOPs */
  inline KDOP<N> operator + (const KDOP<N>& other) const
  {
    KDOP<N> res(*this);
    return res += other;
  }

  /** \brief The (AABB) width */
  inline BVH_REAL width() const
  {
    return dist_[N / 2] - dist_[0];
  }

  /** \brief The (AABB) height */
  inline BVH_REAL height() const
  {
    return dist_[N / 2 + 1] - dist_[1];
  }

  /** \brief The (AABB) depth */
  inline BVH_REAL depth() const
  {
    return dist_[N / 2 + 2] - dist_[2];
  }

  /** \brief The (AABB) volume */
  inline BVH_REAL volume() const
  {
    return width() * height() * depth();
  }

  inline BVH_REAL size() const
  {
    return width() * width() + height() * height() + depth() * depth();
  }

  /** \brief The (AABB) center */
  inline Vec3f center() const
  {
    return Vec3f(dist_[0] + dist_[N / 2], dist_[1] + dist_[N / 2 + 1], dist_[2] + dist_[N / 2 + 2]) * 0.5;
  }

  /** \brief The distance between two KDOP<N>
   * Not implemented.
   */
  BVH_REAL distance(const KDOP<N>& other, Vec3f* P = NULL, Vec3f* Q = NULL) const
  {
    std::cerr << "KDOP distance not implemented!" << std::endl;
    return 0.0;
  }

private:
  /** \brief distances to N KDOP planes */
  BVH_REAL dist_[N];


};

}

#endif
