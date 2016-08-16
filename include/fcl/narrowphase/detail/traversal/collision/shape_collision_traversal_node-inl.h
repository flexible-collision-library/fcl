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

#ifndef FCL_TRAVERSAL_SHAPECOLLISIONTRAVERSALNODE_INL_H
#define FCL_TRAVERSAL_SHAPECOLLISIONTRAVERSALNODE_INL_H

#include "fcl/narrowphase/detail/traversal/collision/shape_collision_traversal_node.h"

namespace fcl
{

namespace detail
{

//==============================================================================
template <typename Shape1, typename Shape2, typename NarrowPhaseSolver>
ShapeCollisionTraversalNode<Shape1, Shape2, NarrowPhaseSolver>::
ShapeCollisionTraversalNode()
  : CollisionTraversalNodeBase<typename Shape1::S>()
{
  model1 = nullptr;
  model2 = nullptr;

  nsolver = nullptr;
}

//==============================================================================
template <typename Shape1, typename Shape2, typename NarrowPhaseSolver>
bool ShapeCollisionTraversalNode<Shape1, Shape2, NarrowPhaseSolver>::
BVTesting(int, int) const
{
  return false;
}

//==============================================================================
template <typename Shape1, typename Shape2, typename NarrowPhaseSolver>
void ShapeCollisionTraversalNode<Shape1, Shape2, NarrowPhaseSolver>::
leafTesting(int, int) const
{
  if(model1->isOccupied() && model2->isOccupied())
  {
    bool is_collision = false;
    if(this->request.enable_contact)
    {
      std::vector<ContactPoint<S>> contacts;
      if(nsolver->shapeIntersect(*model1, this->tf1, *model2, this->tf2, &contacts))
      {
        is_collision = true;
        if(this->request.num_max_contacts > this->result->numContacts())
        {
          const size_t free_space = this->request.num_max_contacts - this->result->numContacts();
          size_t num_adding_contacts;

          // If the free space is not enough to add all the new contacts, we add contacts in descent order of penetration depth.
          if (free_space < contacts.size())
          {
            std::partial_sort(contacts.begin(), contacts.begin() + free_space, contacts.end(), std::bind(comparePenDepth<S>, std::placeholders::_2, std::placeholders::_1));
            num_adding_contacts = free_space;
          }
          else
          {
            num_adding_contacts = contacts.size();
          }

          for(size_t i = 0; i < num_adding_contacts; ++i)
            this->result->addContact(Contact<S>(model1, model2, Contact<S>::NONE, Contact<S>::NONE, contacts[i].pos, contacts[i].normal, contacts[i].penetration_depth));
        }
      }
    }
    else
    {
      if(nsolver->shapeIntersect(*model1, this->tf1, *model2, this->tf2, nullptr))
      {
        is_collision = true;
        if(this->request.num_max_contacts > this->result->numContacts())
          this->result->addContact(Contact<S>(model1, model2, Contact<S>::NONE, Contact<S>::NONE));
      }
    }

    if(is_collision && this->request.enable_cost)
    {
      AABB<S> aabb1, aabb2;
      computeBV(*model1, this->tf1, aabb1);
      computeBV(*model2, this->tf2, aabb2);
      AABB<S> overlap_part;
      aabb1.overlap(aabb2, overlap_part);
      this->result->addCostSource(CostSource<S>(overlap_part, cost_density), this->request.num_max_cost_sources);
    }
  }
  else if((!model1->isFree() && !model2->isFree()) && this->request.enable_cost)
  {
    if(nsolver->shapeIntersect(*model1, this->tf1, *model2, this->tf2, nullptr))
    {
      AABB<S> aabb1, aabb2;
      computeBV(*model1, this->tf1, aabb1);
      computeBV(*model2, this->tf2, aabb2);
      AABB<S> overlap_part;
      aabb1.overlap(aabb2, overlap_part);
      this->result->addCostSource(CostSource<S>(overlap_part, cost_density), this->request.num_max_cost_sources);
    }
  }
}

//==============================================================================
template <typename Shape1, typename Shape2, typename NarrowPhaseSolver>
bool initialize(
    ShapeCollisionTraversalNode<Shape1, Shape2, NarrowPhaseSolver>& node,
    const Shape1& shape1,
    const Transform3<typename Shape1::S>& tf1,
    const Shape2& shape2,
    const Transform3<typename Shape1::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const CollisionRequest<typename Shape1::S>& request,
    CollisionResult<typename Shape1::S>& result)
{
  node.model1 = &shape1;
  node.tf1 = tf1;
  node.model2 = &shape2;
  node.tf2 = tf2;
  node.nsolver = nsolver;

  node.request = request;
  node.result = &result;

  node.cost_density = shape1.cost_density * shape2.cost_density;

  return true;
}

} // namespace detail
} // namespace fcl

#endif
