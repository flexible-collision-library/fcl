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

#ifndef FCL_BV_SPLITTER_H
#define FCL_BV_SPLITTER_H

#include "fcl/math/triangle.h"
#include "fcl/BVH/BVH_internal.h"
#include "fcl/BV/kIOS.h"
#include "fcl/BV/OBBRSS.h"
#include <vector>
#include <iostream>

namespace fcl
{

/// @brief Base interface for BV splitting algorithm
template <typename BV>
class BVSplitterBase
{
public:

  using Scalar = typename BV::Scalar;

  /// @brief Set the geometry data needed by the split rule
  virtual void set(
      Vector3<Scalar>* vertices_,
      Triangle* tri_indices_,
      BVHModelType type_) = 0;

  /// @brief Compute the split rule according to a subset of geometry and the
  /// corresponding BV node
  virtual void computeRule(
      const BV& bv, unsigned int* primitive_indices, int num_primitives) = 0;

  /// @brief Apply the split rule on a given point
  virtual bool apply(const Vector3<Scalar>& q) const = 0;

  /// @brief Clear the geometry data set before
  virtual void clear() = 0;
};

/// @brief Three types of split algorithms are provided in FCL as default
enum SplitMethodType
{
  SPLIT_METHOD_MEAN,
  SPLIT_METHOD_MEDIAN,
  SPLIT_METHOD_BV_CENTER
};

/// @brief A class describing the split rule that splits each BV node
template <typename BV>
class BVSplitter : public BVSplitterBase<BV>
{
public:

  using Scalar = typename BV::Scalar;

  BVSplitter(SplitMethodType method);

  /// @brief Default deconstructor
  virtual ~BVSplitter();

  /// @brief Set the geometry data needed by the split rule
  void set(
      Vector3<Scalar>* vertices_, Triangle* tri_indices_, BVHModelType type_);

  /// @brief Compute the split rule according to a subset of geometry and the
  /// corresponding BV node
  void computeRule(
      const BV& bv, unsigned int* primitive_indices, int num_primitives);

  /// @brief Apply the split rule on a given point
  bool apply(const Vector3<Scalar>& q) const;

  /// @brief Clear the geometry data set before
  void clear();

private:

  /// @brief The axis based on which the split decision is made. For most BV,
  /// the axis is aligned with one of the world coordinate, so only split_axis
  /// is needed. For oriented node, we can use a vector to make a better split
  /// decision.
  int split_axis;
  Vector3<Scalar> split_vector;

  /// @brief The split threshold, different primitives are splitted according
  /// whether their projection on the split_axis is larger or smaller than the
  /// threshold
  Scalar split_value;

  /// @brief The mesh vertices or points handled by the splitter
  Vector3<Scalar>* vertices;

  /// @brief The triangles handled by the splitter
  Triangle* tri_indices;

  /// @brief Whether the geometry is mesh or point cloud
  BVHModelType type;

  /// @brief The split algorithm used
  SplitMethodType split_method;

  /// @brief Split algorithm 1: Split the node from center
  void computeRule_bvcenter(
      const BV& bv, unsigned int* primitive_indices, int num_primitives);

  /// @brief Split algorithm 2: Split the node according to the mean of the data
  ///  contained
  void computeRule_mean(
      const BV& bv, unsigned int* primitive_indices, int num_primitives);

  /// @brief Split algorithm 3: Split the node according to the median of the
  /// data contained
  void computeRule_median(
      const BV& bv, unsigned int* primitive_indices, int num_primitives);

  template <typename, typename>
  friend struct ApplyImpl;

  template <typename, typename>
  friend struct ComputeRuleCenterImpl;

  template <typename, typename>
  friend struct ComputeRuleMeanImpl;

  template <typename, typename>
  friend struct ComputeRuleMedianImpl;
};

template <typename Scalar, typename BV>
void computeSplitVector(const BV& bv, Vector3<Scalar>& split_vector);

template <typename Scalar, typename BV>
void computeSplitValue_bvcenter(const BV& bv, Scalar& split_value);

template <typename Scalar, typename BV>
void computeSplitValue_mean(
    const BV& bv,
    Vector3<Scalar>* vertices,
    Triangle* triangles,
    unsigned int* primitive_indices,
    int num_primitives,
    BVHModelType type,
    const Vector3<Scalar>& split_vector,
    Scalar& split_value);

template <typename Scalar, typename BV>
void computeSplitValue_median(
    const BV& bv,
    Vector3<Scalar>* vertices,
    Triangle* triangles,
    unsigned int* primitive_indices,
    int num_primitives,
    BVHModelType type,
    const Vector3<Scalar>& split_vector,
    Scalar& split_value);

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename BV>
BVSplitter<BV>::BVSplitter(SplitMethodType method)
  : split_method(method)
{
}

//==============================================================================
template <typename BV>
BVSplitter<BV>::~BVSplitter()
{
  // Do nothing
}

//==============================================================================
template <typename BV>
void BVSplitter<BV>::set(
    Vector3<Scalar>* vertices_, Triangle* tri_indices_, BVHModelType type_)
{
  vertices = vertices_;
  tri_indices = tri_indices_;
  type = type_;
}

//==============================================================================
template <typename BV>
void BVSplitter<BV>::computeRule(
    const BV& bv, unsigned int* primitive_indices, int num_primitives)
{
  switch(split_method)
  {
  case SPLIT_METHOD_MEAN:
    computeRule_mean(bv, primitive_indices, num_primitives);
    break;
  case SPLIT_METHOD_MEDIAN:
    computeRule_median(bv, primitive_indices, num_primitives);
    break;
  case SPLIT_METHOD_BV_CENTER:
    computeRule_bvcenter(bv, primitive_indices, num_primitives);
    break;
  default:
    std::cerr << "Split method not supported" << std::endl;
  }
}

//==============================================================================
template <typename Scalar, typename BV>
struct ApplyImpl
{
  static bool run(
      const BVSplitter<BV>& splitter, const Vector3<Scalar>& q)
  {
    return q[splitter.split_axis] > splitter.split_value;
  }
};

//==============================================================================
template <typename BV>
bool BVSplitter<BV>::apply(const Vector3<Scalar>& q) const
{
  return ApplyImpl<Scalar, BV>::run(*this, q);
}

//==============================================================================
template <typename Scalar, typename BV>
struct ComputeRuleCenterImpl
{
  static void run(
      BVSplitter<BV>& splitter,
      const BV& bv,
      unsigned int* /*primitive_indices*/,
      int /*num_primitives*/)
  {
    Vector3<Scalar> center = bv.center();
    int axis = 2;

    if(bv.width() >= bv.height() && bv.width() >= bv.depth())
      axis = 0;
    else if(bv.height() >= bv.width() && bv.height() >= bv.depth())
      axis = 1;

    splitter.split_axis = axis;
    splitter.split_value = center[axis];
  }
};

//==============================================================================
template <typename BV>
void BVSplitter<BV>::computeRule_bvcenter(
    const BV& bv, unsigned int* primitive_indices, int num_primitives)
{
  ComputeRuleCenterImpl<Scalar, BV>::run(
        *this, bv, primitive_indices, num_primitives);
}

//==============================================================================
template <typename Scalar, typename BV>
struct ComputeRuleMeanImpl
{
  static void run(
      BVSplitter<BV>& splitter,
      const BV& bv,
      unsigned int* primitive_indices,
      int num_primitives)
  {
    int axis = 2;

    if(bv.width() >= bv.height() && bv.width() >= bv.depth())
      axis = 0;
    else if(bv.height() >= bv.width() && bv.height() >= bv.depth())
      axis = 1;

    splitter.split_axis = axis;
    Scalar sum = 0;

    if(splitter.type == BVH_MODEL_TRIANGLES)
    {
      for(int i = 0; i < num_primitives; ++i)
      {
        const Triangle& t = splitter.tri_indices[primitive_indices[i]];
        sum += (splitter.vertices[t[0]][splitter.split_axis]
            + splitter.vertices[t[1]][splitter.split_axis]
            + splitter.vertices[t[2]][splitter.split_axis]);
      }

      sum /= 3;
    }
    else if(splitter.type == BVH_MODEL_POINTCLOUD)
    {
      for(int i = 0; i < num_primitives; ++i)
      {
        sum += splitter.vertices[primitive_indices[i]][splitter.split_axis];
      }
    }

    splitter.split_value = sum / num_primitives;
  }
};

//==============================================================================
template <typename BV>
void BVSplitter<BV>::computeRule_mean(
    const BV& bv, unsigned int* primitive_indices, int num_primitives)
{
  ComputeRuleMeanImpl<Scalar, BV>::run(
        *this, bv, primitive_indices, num_primitives);
}

//==============================================================================
template <typename Scalar, typename BV>
struct ComputeRuleMedianImpl
{
  static void run(
      BVSplitter<BV>& splitter,
      const BV& bv,
      unsigned int* primitive_indices,
      int num_primitives)
  {
    int axis = 2;

    if(bv.width() >= bv.height() && bv.width() >= bv.depth())
      axis = 0;
    else if(bv.height() >= bv.width() && bv.height() >= bv.depth())
      axis = 1;

    splitter.split_axis = axis;
    std::vector<Scalar> proj(num_primitives);

    if(splitter.type == BVH_MODEL_TRIANGLES)
    {
      for(int i = 0; i < num_primitives; ++i)
      {
        const Triangle& t = splitter.tri_indices[primitive_indices[i]];
        proj[i] = (splitter.vertices[t[0]][splitter.split_axis]
            + splitter.vertices[t[1]][splitter.split_axis]
            + splitter.vertices[t[2]][splitter.split_axis]) / 3;
      }
    }
    else if(splitter.type == BVH_MODEL_POINTCLOUD)
    {
      for(int i = 0; i < num_primitives; ++i)
        proj[i] = splitter.vertices[primitive_indices[i]][splitter.split_axis];
    }

    std::sort(proj.begin(), proj.end());

    if(num_primitives % 2 == 1)
    {
      splitter.split_value = proj[(num_primitives - 1) / 2];
    }
    else
    {
      splitter.split_value = (proj[num_primitives / 2] + proj[num_primitives / 2 - 1]) / 2;
    }
  }
};

//==============================================================================
template <typename BV>
void BVSplitter<BV>::computeRule_median(
    const BV& bv, unsigned int* primitive_indices, int num_primitives)
{
  ComputeRuleMedianImpl<Scalar, BV>::run(
        *this, bv, primitive_indices, num_primitives);
}

//==============================================================================
template <typename Scalar>
struct ComputeRuleCenterImpl<Scalar, OBB<Scalar>>
{
  static void run(
      BVSplitter<OBB<Scalar>>& splitter,
      const OBB<Scalar>& bv,
      unsigned int* /*primitive_indices*/,
      int /*num_primitives*/)
  {
    computeSplitVector<Scalar, OBB<Scalar>>(bv, splitter.split_vector);
    computeSplitValue_bvcenter<Scalar, OBB<Scalar>>(bv, splitter.split_value);
  }
};

//==============================================================================
template <typename Scalar>
struct ComputeRuleMeanImpl<Scalar, OBB<Scalar>>
{
  static void run(
      BVSplitter<OBB<Scalar>>& splitter,
      const OBB<Scalar>& bv,
      unsigned int* primitive_indices,
      int num_primitives)
  {
    computeSplitVector<Scalar, OBB<Scalar>>(bv, splitter.split_vector);
    computeSplitValue_mean<Scalar, OBB<Scalar>>(
          bv, splitter.vertices, splitter.tri_indices, primitive_indices,
          num_primitives, splitter.type, splitter.split_vector, splitter.split_value);
  }
};

//==============================================================================
template <typename Scalar>
struct ComputeRuleMedianImpl<Scalar, OBB<Scalar>>
{
  static void run(
      BVSplitter<OBB<Scalar>>& splitter,
      const OBB<Scalar>& bv,
      unsigned int* primitive_indices,
      int num_primitives)
  {
    computeSplitVector<Scalar, OBB<Scalar>>(bv, splitter.split_vector);
    computeSplitValue_median<Scalar, OBB<Scalar>>(
          bv, splitter.vertices, splitter.tri_indices, primitive_indices,
          num_primitives, splitter.type, splitter.split_vector, splitter.split_value);
  }
};

//==============================================================================
template <typename Scalar>
struct ComputeRuleCenterImpl<Scalar, RSS<Scalar>>
{
  static void run(
      BVSplitter<RSS<Scalar>>& splitter,
      const RSS<Scalar>& bv,
      unsigned int* /*primitive_indices*/,
      int /*num_primitives*/)
  {
    computeSplitVector<Scalar, RSS<Scalar>>(bv, splitter.split_vector);
    computeSplitValue_bvcenter<Scalar, RSS<Scalar>>(bv, splitter.split_value);
  }
};

//==============================================================================
template <typename Scalar>
struct ComputeRuleMeanImpl<Scalar, RSS<Scalar>>
{
  static void run(
      BVSplitter<RSS<Scalar>>& splitter,
      const RSS<Scalar>& bv,
      unsigned int* primitive_indices,
      int num_primitives)
  {
    computeSplitVector<Scalar, RSS<Scalar>>(bv, splitter.split_vector);
    computeSplitValue_mean<Scalar, RSS<Scalar>>(
          bv, splitter.vertices, splitter.tri_indices, primitive_indices,
          num_primitives, splitter.type, splitter.split_vector, splitter.split_value);
  }
};

//==============================================================================
template <typename Scalar>
struct ComputeRuleMedianImpl<Scalar, RSS<Scalar>>
{
  static void run(
      BVSplitter<RSS<Scalar>>& splitter,
      const RSS<Scalar>& bv,
      unsigned int* primitive_indices,
      int num_primitives)
  {
    computeSplitVector<Scalar, RSS<Scalar>>(bv, splitter.split_vector);
    computeSplitValue_median<Scalar, RSS<Scalar>>(
          bv, splitter.vertices, splitter.tri_indices, primitive_indices,
          num_primitives, splitter.type, splitter.split_vector, splitter.split_value);
  }
};

//==============================================================================
template <typename Scalar>
struct ComputeRuleCenterImpl<Scalar, kIOS<Scalar>>
{
  static void run(
      BVSplitter<kIOS<Scalar>>& splitter,
      const kIOS<Scalar>& bv,
      unsigned int* /*primitive_indices*/,
      int /*num_primitives*/)
  {
    computeSplitVector<Scalar, kIOS<Scalar>>(bv, splitter.split_vector);
    computeSplitValue_bvcenter<Scalar, kIOS<Scalar>>(bv, splitter.split_value);
  }
};

//==============================================================================
template <typename Scalar>
struct ComputeRuleMeanImpl<Scalar, kIOS<Scalar>>
{
  static void run(
      BVSplitter<kIOS<Scalar>>& splitter,
      const kIOS<Scalar>& bv,
      unsigned int* primitive_indices,
      int num_primitives)
  {
    computeSplitVector<Scalar, kIOS<Scalar>>(bv, splitter.split_vector);
    computeSplitValue_mean<Scalar, kIOS<Scalar>>(
          bv, splitter.vertices, splitter.tri_indices, primitive_indices,
          num_primitives, splitter.type, splitter.split_vector, splitter.split_value);
  }
};

//==============================================================================
template <typename Scalar>
struct ComputeRuleMedianImpl<Scalar, kIOS<Scalar>>
{
  static void run(
      BVSplitter<kIOS<Scalar>>& splitter,
      const kIOS<Scalar>& bv,
      unsigned int* primitive_indices,
      int num_primitives)
  {
    computeSplitVector<Scalar, kIOS<Scalar>>(bv, splitter.split_vector);
    computeSplitValue_median<Scalar, kIOS<Scalar>>(
          bv, splitter.vertices, splitter.tri_indices, primitive_indices,
          num_primitives, splitter.type, splitter.split_vector, splitter.split_value);
  }
};

//==============================================================================
template <typename Scalar>
struct ComputeRuleCenterImpl<Scalar, OBBRSS<Scalar>>
{
  static void run(
      BVSplitter<OBBRSS<Scalar>>& splitter,
      const OBBRSS<Scalar>& bv,
      unsigned int* /*primitive_indices*/,
      int /*num_primitives*/)
  {
    computeSplitVector<Scalar, OBBRSS<Scalar>>(bv, splitter.split_vector);
    computeSplitValue_bvcenter<Scalar, OBBRSS<Scalar>>(bv, splitter.split_value);
  }
};

//==============================================================================
template <typename Scalar>
struct ComputeRuleMeanImpl<Scalar, OBBRSS<Scalar>>
{
  static void run(
      BVSplitter<OBBRSS<Scalar>>& splitter,
      const OBBRSS<Scalar>& bv,
      unsigned int* primitive_indices,
      int num_primitives)
  {
    computeSplitVector<Scalar, OBBRSS<Scalar>>(bv, splitter.split_vector);
    computeSplitValue_mean<Scalar, OBBRSS<Scalar>>(
          bv, splitter.vertices, splitter.tri_indices, primitive_indices,
          num_primitives, splitter.type, splitter.split_vector, splitter.split_value);
  }
};

//==============================================================================
template <typename Scalar>
struct ComputeRuleMedianImpl<Scalar, OBBRSS<Scalar>>
{
  static void run(
      BVSplitter<OBBRSS<Scalar>>& splitter,
      const OBBRSS<Scalar>& bv,
      unsigned int* primitive_indices,
      int num_primitives)
  {
    computeSplitVector<Scalar, OBBRSS<Scalar>>(bv, splitter.split_vector);
    computeSplitValue_median<Scalar, OBBRSS<Scalar>>(
          bv, splitter.vertices, splitter.tri_indices, primitive_indices,
          num_primitives, splitter.type, splitter.split_vector, splitter.split_value);
  }
};

//==============================================================================
template <typename Scalar>
struct ApplyImpl<Scalar, OBB<Scalar>>
{
  static bool run(
      const BVSplitter<OBB<Scalar>>& splitter,
      const Vector3<Scalar>& q)
  {
    return splitter.split_vector.dot(q) > splitter.split_value;
  }
};

//==============================================================================
template <typename Scalar>
struct ApplyImpl<Scalar, RSS<Scalar>>
{
  static bool run(
      const BVSplitter<RSS<Scalar>>& splitter,
      const Vector3<Scalar>& q)
  {
    return splitter.split_vector.dot(q) > splitter.split_value;
  }
};

//==============================================================================
template <typename Scalar>
struct ApplyImpl<Scalar, kIOS<Scalar>>
{
  static bool run(
      const BVSplitter<kIOS<Scalar>>& splitter,
      const Vector3<Scalar>& q)
  {
    return splitter.split_vector.dot(q) > splitter.split_value;
  }
};

//==============================================================================
template <typename Scalar>
struct ApplyImpl<Scalar, OBBRSS<Scalar>>
{
  static bool run(
      const BVSplitter<OBBRSS<Scalar>>& splitter,
      const Vector3<Scalar>& q)
  {
    return splitter.split_vector.dot(q) > splitter.split_value;
  }
};

//==============================================================================
template <typename BV>
void BVSplitter<BV>::clear()
{
  vertices = NULL;
  tri_indices = NULL;
  type = BVH_MODEL_UNKNOWN;
}

//==============================================================================
template <typename Scalar, typename BV>
struct ComputeSplitVectorImpl
{
  static void run(const BV& bv, Vector3<Scalar>& split_vector)
  {
    split_vector = bv.frame.linear().col(0);
  }
};

//==============================================================================
template <typename Scalar, typename BV>
void computeSplitVector(const BV& bv, Vector3<Scalar>& split_vector)
{
  ComputeSplitVectorImpl<Scalar, BV>::run(bv, split_vector);
}

//==============================================================================
template <typename Scalar>
struct ComputeSplitVectorImpl<Scalar, kIOS<Scalar>>
{
  static void run(const kIOS<Scalar>& bv, Vector3<Scalar>& split_vector)
  {
    /*
      switch(bv.num_spheres)
      {
      case 1:
      split_vector = Vector3<Scalar>(1, 0, 0);
      break;
      case 3:
      {
      Vector3<Scalar> v[3];
      v[0] = bv.spheres[1].o - bv.spheres[0].o;
      v[0].normalize();
      generateCoordinateSystem(v[0], v[1], v[2]);
      split_vector = v[1];
      }
      break;
      case 5:
      {
      Vector3<Scalar> v[2];
      v[0] = bv.spheres[1].o - bv.spheres[0].o;
      v[1] = bv.spheres[3].o - bv.spheres[0].o;
      split_vector = v[0].cross(v[1]);
      split_vector.normalize();
      }
      break;
      default:
      ;
      }
    */
    split_vector = bv.obb.frame.linear().col(0);
  }
};

//==============================================================================
template <typename Scalar>
struct ComputeSplitVectorImpl<Scalar, OBBRSS<Scalar>>
{
  static void run(const OBBRSS<Scalar>& bv, Vector3<Scalar>& split_vector)
  {
    split_vector = bv.obb.frame.linear().col(0);
  }
};

//==============================================================================
template <typename Scalar, typename BV>
void computeSplitValue_bvcenter(const BV& bv, Scalar& split_value)
{
  Vector3<Scalar> center = bv.center();
  split_value = center[0];
}

//==============================================================================
template <typename Scalar, typename BV>
void computeSplitValue_mean(
    const BV& bv,
    Vector3<Scalar>* vertices,
    Triangle* triangles,
    unsigned int* primitive_indices,
    int num_primitives,
    BVHModelType type,
    const Vector3<Scalar>& split_vector,
    Scalar& split_value)
{
  Scalar sum = 0.0;
  if(type == BVH_MODEL_TRIANGLES)
  {
    Scalar c[3] = {0.0, 0.0, 0.0};

    for(int i = 0; i < num_primitives; ++i)
    {
      const Triangle& t = triangles[primitive_indices[i]];
      const Vector3<Scalar>& p1 = vertices[t[0]];
      const Vector3<Scalar>& p2 = vertices[t[1]];
      const Vector3<Scalar>& p3 = vertices[t[2]];

      c[0] += (p1[0] + p2[0] + p3[0]);
      c[1] += (p1[1] + p2[1] + p3[1]);
      c[2] += (p1[2] + p2[2] + p3[2]);
    }
    split_value = (c[0] * split_vector[0] + c[1] * split_vector[1] + c[2] * split_vector[2]) / (3 * num_primitives);
  }
  else if(type == BVH_MODEL_POINTCLOUD)
  {
    for(int i = 0; i < num_primitives; ++i)
    {
      const Vector3<Scalar>& p = vertices[primitive_indices[i]];
      Vector3<Scalar> v(p[0], p[1], p[2]);
      sum += v.dot(split_vector);
    }

    split_value = sum / num_primitives;
  }
}

//==============================================================================
template <typename Scalar, typename BV>
void computeSplitValue_median(
    const BV& bv,
    Vector3<Scalar>* vertices,
    Triangle* triangles,
    unsigned int* primitive_indices,
    int num_primitives,
    BVHModelType type,
    const Vector3<Scalar>& split_vector,
    Scalar& split_value)
{
  std::vector<Scalar> proj(num_primitives);

  if(type == BVH_MODEL_TRIANGLES)
  {
    for(int i = 0; i < num_primitives; ++i)
    {
      const Triangle& t = triangles[primitive_indices[i]];
      const Vector3<Scalar>& p1 = vertices[t[0]];
      const Vector3<Scalar>& p2 = vertices[t[1]];
      const Vector3<Scalar>& p3 = vertices[t[2]];
      Vector3<Scalar> centroid3(p1[0] + p2[0] + p3[0],
                      p1[1] + p2[1] + p3[1],
                      p1[2] + p2[2] + p3[2]);

      proj[i] = centroid3.dot(split_vector) / 3;
    }
  }
  else if(type == BVH_MODEL_POINTCLOUD)
  {
    for(int i = 0; i < num_primitives; ++i)
    {
      const Vector3<Scalar>& p = vertices[primitive_indices[i]];
      Vector3<Scalar> v(p[0], p[1], p[2]);
      proj[i] = v.dot(split_vector);
    }
  }

  std::sort(proj.begin(), proj.end());

  if(num_primitives % 2 == 1)
  {
    split_value = proj[(num_primitives - 1) / 2];
  }
  else
  {
    split_value = (proj[num_primitives / 2] + proj[num_primitives / 2 - 1]) / 2;
  }
}

} // namespace fcl

#endif
