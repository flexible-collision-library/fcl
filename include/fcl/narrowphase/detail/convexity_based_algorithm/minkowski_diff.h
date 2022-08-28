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

/** @author Jia Pan, Shi Shenglei */

#ifndef FCL_NARROWPHASE_DETAIL_MINKOWSKIDIFF_H
#define FCL_NARROWPHASE_DETAIL_MINKOWSKIDIFF_H

#include "fcl/math/detail/project.h"
#include "fcl/geometry/shape/shape_base.h"

namespace fcl
{

namespace detail
{

template <typename S>
struct Support 
{
  Vector3<S> v, v1, v2;

  Support()
  {
    v.setZero(); v1.setZero(); v2.setZero(); 
  }

  Support(const Support& s)
  {
    v = s.v; v1 = s.v1; v2 = s.v2; 
  }

  inline Support& operator=(const Support& s)
  {
    v = s.v; v1 = s.v1; v2 = s.v2; 
    return *this;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename S>
struct Simplex 
{
  Support<S> ps[4];
  int last; //!< index of last added point
  Simplex() : last(-1){}

  inline int size() const
  {
      return last + 1;
  }
  inline const Support<S>& lastSupport() const
  {
      return ps[last];
  }
  inline const Support<S>& at(int idx) const
  {
      return ps[idx];
  }
  inline Support<S>& at(int idx)
  {
      return ps[idx];
  }
  inline void add(const Support<S> &v)
  {
      last++;
      ps[last] = v;
  }
  inline void set(int pos, const Support<S> &a)
  {
      ps[pos] = a;
  }
  inline void setSize(int size)
  {
      last = size - 1;
  }
  inline void swap(int pos1, int pos2)
  {
      Support<S> temp = ps[pos1];
      ps[pos1] = ps[pos2];
      ps[pos2] = temp;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @brief Minkowski difference class of two shapes
template <typename S>
struct FCL_EXPORT MinkowskiDiff
{
  /// @brief points to two shapes
  const ShapeBase<S>* shapes[2];

  /// @brief rotation from shape0 to shape1
  Matrix3<S> toshape1;

  /// @brief transform from shape1 to shape0 
  Transform3<S> toshape0;

  MinkowskiDiff();

  /// @brief support function for shape0
  Vector3<S> support0(const Vector3<S>& d) const;
  
  /// @brief support function for shape1
  Vector3<S> support1(const Vector3<S>& d) const;

  /// @brief support function for the pair of shapes
  Vector3<S> support(const Vector3<S>& d) const;

  /// @brief support function for the d-th shape (d = 0 or 1)
  Vector3<S> support(const Vector3<S>& d, size_t index) const;

  /// @brief support function for translating shape0, which is translating at velocity v
  Vector3<S> support0(const Vector3<S>& d, const Vector3<S>& v) const;

  /// @brief support function for the pair of shapes, where shape0 is translating at velocity v
  Vector3<S> support(const Vector3<S>& d, const Vector3<S>& v) const;

  /// @brief support function for the d-th shape (d = 0 or 1), where shape0 is translating at velocity v
  Vector3<S> support(const Vector3<S>& d, const Vector3<S>& v, size_t index) const;

  /// @brief support function for the pair of shapes
  Support<S> supportS(const Vector3<S>& d) const;

  /// @brief support function for the pair of shapes, where shape0 is translating at velocity v
  Support<S> supportS(const Vector3<S>& d, const Vector3<S>& v) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using MinkowskiDifff = MinkowskiDiff<float>;
using MinkowskiDiffd = MinkowskiDiff<double>;

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/convexity_based_algorithm/minkowski_diff-inl.h"

#endif
