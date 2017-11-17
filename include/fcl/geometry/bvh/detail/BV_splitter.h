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

#ifndef FCL_BV_SPLITTER_H
#define FCL_BV_SPLITTER_H

#include <vector>
#include <iostream>
#include "fcl/math/triangle.h"
#include "fcl/math/bv/kIOS.h"
#include "fcl/math/bv/OBBRSS.h"
#include "fcl/geometry/bvh/BVH_internal.h"
#include "fcl/geometry/bvh/detail/BV_splitter_base.h"

namespace fcl
{

namespace detail
{

/// @brief Three types of split algorithms are provided in FCL as default
enum SplitMethodType
{
  SPLIT_METHOD_MEAN,
  SPLIT_METHOD_MEDIAN,
  SPLIT_METHOD_BV_CENTER
};

/// @brief A class describing the split rule that splits each BV node
template <typename BV>
class FCL_EXPORT BVSplitter : public BVSplitterBase<BV>
{
public:

  using S = typename BV::S;

  BVSplitter(SplitMethodType method);

  /// @brief Default deconstructor
  virtual ~BVSplitter();

  /// @brief Set the geometry data needed by the split rule
  void set(
      Vector3<S>* vertices_, Triangle* tri_indices_, BVHModelType type_);

  /// @brief Compute the split rule according to a subset of geometry and the
  /// corresponding BV node
  void computeRule(
      const BV& bv, unsigned int* primitive_indices, int num_primitives);

  /// @brief Apply the split rule on a given point
  bool apply(const Vector3<S>& q) const;

  /// @brief Clear the geometry data set before
  void clear();

private:

  /// @brief The axis based on which the split decision is made. For most BV,
  /// the axis is aligned with one of the world coordinate, so only split_axis
  /// is needed. For oriented node, we can use a vector to make a better split
  /// decision.
  int split_axis;
  Vector3<S> split_vector;

  /// @brief The split threshold, different primitives are splitted according
  /// whether their projection on the split_axis is larger or smaller than the
  /// threshold
  S split_value;

  /// @brief The mesh vertices or points handled by the splitter
  Vector3<S>* vertices;

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

template <typename S, typename BV>
void computeSplitVector(const BV& bv, Vector3<S>& split_vector);

template <typename S, typename BV>
void computeSplitValue_bvcenter(const BV& bv, S& split_value);

template <typename S, typename BV>
void computeSplitValue_mean(
    const BV& bv,
    Vector3<S>* vertices,
    Triangle* triangles,
    unsigned int* primitive_indices,
    int num_primitives,
    BVHModelType type,
    const Vector3<S>& split_vector,
    S& split_value);

template <typename S, typename BV>
void computeSplitValue_median(
    const BV& bv,
    Vector3<S>* vertices,
    Triangle* triangles,
    unsigned int* primitive_indices,
    int num_primitives,
    BVHModelType type,
    const Vector3<S>& split_vector,
    S& split_value);

} // namespace detail
} // namespace fcl

#include "fcl/geometry/bvh/detail/BV_splitter-inl.h"

#endif
