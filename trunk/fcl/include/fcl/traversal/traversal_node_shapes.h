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


#ifndef FCL_TRAVERSAL_NODE_SHAPES_H
#define FCL_TRAVERSAL_NODE_SHAPES_H

#include "fcl/collision_data.h"
#include "fcl/traversal/traversal_node_base.h"
#include "fcl/narrowphase/narrowphase.h"
#include "fcl/shape/geometric_shapes_utility.h"

namespace fcl
{


/// @brief Traversal node for collision between two shapes
template<typename S1, typename S2, typename NarrowPhaseSolver>
class ShapeCollisionTraversalNode : public CollisionTraversalNodeBase
{
public:
  ShapeCollisionTraversalNode() : CollisionTraversalNodeBase()
  {
    model1 = NULL;
    model2 = NULL;

    nsolver = NULL;
  }

  /// @brief BV culling test in one BVTT node
  bool BVTesting(int, int) const
  {
    return false;
  }

  /// @brief Intersection testing between leaves (two shapes)
  void leafTesting(int, int) const
  {
    if(model1->isOccupied() && model2->isOccupied())
    {
      bool is_collision = false;
      if(request.enable_contact)
      {
        Vec3f contact_point, normal;
        FCL_REAL penetration_depth;
        if(nsolver->shapeIntersect(*model1, tf1, *model2, tf2, &contact_point, &penetration_depth, &normal))
        {
          is_collision = true;
          if(request.num_max_contacts > result->numContacts())
            result->addContact(Contact(model1, model2, Contact::NONE, Contact::NONE, contact_point, normal, penetration_depth));
        }
      }
      else
      {
        if(nsolver->shapeIntersect(*model1, tf1, *model2, tf2, NULL, NULL, NULL))
        {
          is_collision = true;
          if(request.num_max_contacts > result->numContacts())
            result->addContact(Contact(model1, model2, Contact::NONE, Contact::NONE));
        }
      }

      if(is_collision && request.enable_cost)
      {
        AABB aabb1, aabb2;
        computeBV<AABB, S1>(*model1, tf1, aabb1);
        computeBV<AABB, S2>(*model2, tf2, aabb2);
        AABB overlap_part;
        aabb1.overlap(aabb2, overlap_part);
        result->addCostSource(CostSource(overlap_part, cost_density), request.num_max_cost_sources);
      }
    }
    else if((!model1->isFree() && !model2->isFree()) && request.enable_cost)
    {
      if(nsolver->shapeIntersect(*model1, tf1, *model2, tf2, NULL, NULL, NULL))
      {
        AABB aabb1, aabb2;
        computeBV<AABB, S1>(*model1, tf1, aabb1);
        computeBV<AABB, S2>(*model2, tf2, aabb2);
        AABB overlap_part;
        aabb1.overlap(aabb2, overlap_part);
        result->addCostSource(CostSource(overlap_part, cost_density), request.num_max_cost_sources);        
      }      
    }
  }

  const S1* model1;
  const S2* model2;

  FCL_REAL cost_density;

  const NarrowPhaseSolver* nsolver;
};

/// @brief Traversal node for distance between two shapes
template<typename S1, typename S2, typename NarrowPhaseSolver>
class ShapeDistanceTraversalNode : public DistanceTraversalNodeBase
{
public:
  ShapeDistanceTraversalNode() : DistanceTraversalNodeBase()
  {
    model1 = NULL;
    model2 = NULL;

    nsolver = NULL;
  }

  /// @brief BV culling test in one BVTT node
  FCL_REAL BVTesting(int, int) const
  {
    return -1; // should not be used 
  }

  /// @brief Distance testing between leaves (two shapes)
  void leafTesting(int, int) const
  {
    FCL_REAL distance;
    nsolver->shapeDistance(*model1, tf1, *model2, tf2, &distance);
    result->update(distance, model1, model2, DistanceResult::NONE, DistanceResult::NONE);
  }

  const S1* model1;
  const S2* model2;

  const NarrowPhaseSolver* nsolver;
};


}

#endif
