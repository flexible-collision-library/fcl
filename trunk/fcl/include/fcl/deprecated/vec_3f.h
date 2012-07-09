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


#ifndef FCL_DEPRECATED_VEC_3F_H
#define FCL_DEPRECATED_VEC_3F_H

namespace fcl
{

namespace deprecated
{


/** \brief A class describing a three-dimensional vector */
class Vec3f
{
public:
  /** \brief vector data */
  FCL_REAL v_[3];

  Vec3f() { v_[0] = 0; v_[1] = 0; v_[2] = 0; }

  Vec3f(const Vec3f& other)
  {
    memcpy(v_, other.v_, sizeof(FCL_REAL) * 3);
  }

  Vec3f(const FCL_REAL* v)
  {
    memcpy(v_, v, sizeof(FCL_REAL) * 3);
  }

  Vec3f(FCL_REAL x, FCL_REAL y, FCL_REAL z)
  {
    v_[0] = x;
    v_[1] = y;
    v_[2] = z;
  }

  ~Vec3f() {}

  /** \brief Get the ith element */
  inline FCL_REAL operator [] (size_t i) const
  {
    return v_[i];
  }

  inline FCL_REAL& operator[] (size_t i)
  {
    return v_[i];
  }

  /** \brief Add the other vector */
  inline Vec3f& operator += (const Vec3f& other)
  {
    v_[0] += other.v_[0];
    v_[1] += other.v_[1];
    v_[2] += other.v_[2];
    return *this;
  }


  /** \brief Minus the other vector */
  inline Vec3f& operator -= (const Vec3f& other)
  {
    v_[0] -= other.v_[0];
    v_[1] -= other.v_[1];
    v_[2] -= other.v_[2];
    return *this;
  }

  inline Vec3f& operator *= (FCL_REAL t)
  {
    v_[0] *= t;
    v_[1] *= t;
    v_[2] *= t;
    return *this;
  }

  /** \brief Negate the vector */
  inline Vec3f& negate()
  {
    v_[0] = - v_[0];
    v_[1] = - v_[1];
    v_[2] = - v_[2];
    return *this;
  }

  /** \brief Return a negated vector */
  inline Vec3f operator - () const
  {
    return Vec3f(-v_[0], -v_[1], -v_[2]);
  }

  /** \brief Return a summation vector */
  inline Vec3f operator + (const Vec3f& other) const
  {
    return Vec3f(v_[0] + other.v_[0], v_[1] + other.v_[1], v_[2] + other.v_[2]);
  }

  /** \brief Return a substraction vector */
  inline Vec3f operator - (const Vec3f& other) const
  {
    return Vec3f(v_[0] - other.v_[0], v_[1] - other.v_[1], v_[2] - other.v_[2]);
  }

  /** \brief Scale the vector by t */
  inline Vec3f operator * (FCL_REAL t) const
  {
    return Vec3f(v_[0] * t, v_[1] * t, v_[2] * t);
  }

  /** \brief Return the cross product with another vector */
  inline Vec3f cross(const Vec3f& other) const
  {
    return Vec3f(v_[1] * other.v_[2] - v_[2] * other.v_[1],
                 v_[2] * other.v_[0] - v_[0] * other.v_[2],
                 v_[0] * other.v_[1] - v_[1] * other.v_[0]);
  }

  /** \brief Return the dot product with another vector */
  inline FCL_REAL dot(const Vec3f& other) const
  {
    return v_[0] * other.v_[0] + v_[1] * other.v_[1] + v_[2] * other.v_[2];
  }

  /** \brief Normalization */
  inline bool normalize()
  {
    const FCL_REAL EPSILON = 1e-11;
    FCL_REAL sqr_length = v_[0] * v_[0] + v_[1] * v_[1] + v_[2] * v_[2];
    if(sqr_length > EPSILON * EPSILON)
    {
      FCL_REAL inv_length = (FCL_REAL)1.0 / (FCL_REAL)sqrt(sqr_length);
      v_[0] *= inv_length;
      v_[1] *= inv_length;
      v_[2] *= inv_length;
      return true;
    }
    return false;
  }

  inline Vec3f normalized() const
  {
    const FCL_REAL EPSILON = 1e-11;
    FCL_REAL sqr_length = v_[0] * v_[0] + v_[1] * v_[1] + v_[2] * v_[2];
    if(sqr_length > EPSILON * EPSILON)
    {
      FCL_REAL inv_length = (FCL_REAL)1.0 / (FCL_REAL)sqrt(sqr_length);
      return *this * inv_length;
    }
    else
    {
      return *this;
    }
  }


  /** \brief Return vector length */
  inline FCL_REAL length() const
  {
    return sqrt(v_[0] * v_[0] + v_[1] * v_[1] + v_[2] * v_[2]);
  }

  /** \brief Return vector square length */
  inline FCL_REAL sqrLength() const
  {
    return v_[0] * v_[0] + v_[1] * v_[1] + v_[2] * v_[2];
  }

  /** \brief Set the vector using new values */
  inline void setValue(FCL_REAL x, FCL_REAL y, FCL_REAL z)
  {
    v_[0] = x; v_[1] = y; v_[2] = z;
  }

  /** \brief Set the vector using new values */
  inline void setValue(FCL_REAL x)
  {
    v_[0] = x; v_[1] = x; v_[2] = x;
  }

  /** \brief Check whether two vectors are the same in value */
  inline bool equal(const Vec3f& other) const
  {
    const FCL_REAL EPSILON = 1e-11;
    return ((v_[0] - other.v_[0] < EPSILON) &&
            (v_[0] - other.v_[0] > -EPSILON) &&
            (v_[1] - other.v_[1] < EPSILON) &&
            (v_[1] - other.v_[1] > -EPSILON) &&
            (v_[2] - other.v_[2] < EPSILON) &&
            (v_[2] - other.v_[2] > -EPSILON));
  }

  inline FCL_REAL triple(const Vec3f& v1, const Vec3f& v2) const
  {
    return v_[0] * (v1.v_[1] * v2.v_[2] - v1.v_[2] * v2.v_[1]) +
      v_[1] * (v1.v_[2] * v2.v_[0] - v1.v_[0] * v2.v_[2]) +
      v_[2] * (v1.v_[0] * v2.v_[1] - v1.v_[1] * v2.v_[0]);
  }
};




/** \brief The minimum of two vectors */
inline Vec3f min(const Vec3f& a, const Vec3f& b)
{
  Vec3f ret(std::min(a[0], b[0]), std::min(a[1], b[1]), std::min(a[2], b[2]));
  return ret;
}

/** \brief the maximum of two vectors */
inline Vec3f max(const Vec3f& a, const Vec3f& b)
{
  Vec3f ret(std::max(a[0], b[0]), std::max(a[1], b[1]), std::max(a[2], b[2]));
  return ret;
}

inline Vec3f abs(const Vec3f& v)
{
  FCL_REAL x = v[0] < 0 ? -v[0] : v[0];
  FCL_REAL y = v[1] < 0 ? -v[1] : v[1];
  FCL_REAL z = v[2] < 0 ? -v[2] : v[2];

  return Vec3f(x, y, z);
}


inline FCL_REAL triple(const Vec3f& a, const Vec3f& b, const Vec3f& c)
{
  return a.triple(b, c);
}

/** \brief generate a coordinate given a vector (i.e., generating three orthonormal vectors given a vector)
 * w should be normalized
 */
static void generateCoordinateSystem(const Vec3f& w, Vec3f& u, Vec3f& v)
{
  FCL_REAL inv_length;
  if(fabs(w[0]) >= fabs(w[1]))
  {
    inv_length = (FCL_REAL)1.0 / sqrt(w[0] * w[0] + w[2] * w[2]);
    u[0] = -w[2] * inv_length;
    u[1] = (FCL_REAL)0;
    u[2] = w[0] * inv_length;
    v[0] = w[1] * u[2];
    v[1] = w[2] * u[0] - w[0] * u[2];
    v[2] = -w[1] * u[0];
  }
  else
  {
    inv_length = (FCL_REAL)1.0 / sqrt(w[1] * w[1] + w[2] * w[2]);
    u[0] = (FCL_REAL)0;
    u[1] = w[2] * inv_length;
    u[2] = -w[1] * inv_length;
    v[0] = w[1] * u[2] - w[2] * u[1];
    v[1] = -w[0] * u[2];
    v[2] = w[0] * u[1];
  }
}

} // deprecated

} // fcl

#endif
