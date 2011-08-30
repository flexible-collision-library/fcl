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

/*
#include "fcl/collision.h"
#include "fcl/collision_func_matrix.h"

#include <iostream>

namespace fcl
{

static CollisionFunctionMatrix CollisionFunctionLookTable;

int collide(const CollisionObject* o1, const CollisionObject* o2,
             int num_max_contacts, bool exhaustive, bool enable_contact,
             std::vector<Contact>& contacts)
{
  if(num_max_contacts <= 0 && !exhaustive)
  {
    std::cerr << "Warning: should stop early as num_max_contact is " << num_max_contacts << " !" << std::endl;
    return 0;
  }

  const OBJECT_TYPE object_type1 = o1->getObjectType();
  const NODE_TYPE node_type1 = o1->getNodeType();

  const OBJECT_TYPE object_type2 = o2->getObjectType();
  const NODE_TYPE node_type2 = o2->getNodeType();


  if(object_type1 == OT_GEOM && object_type2 == OT_GEOM)
  {
    return CollisionFunctionLookTable.collision_matrix[node_type1][node_type2](o1, o2, num_max_contacts, exhaustive, enable_contact, contacts);
  }
  else if(object_type1 == OT_GEOM && object_type2 == OT_BVH)
  {
    return CollisionFunctionLookTable.collision_matrix[node_type2][node_type1](o2, o1, num_max_contacts, exhaustive, enable_contact, contacts);
  }
  else if(object_type1 == OT_BVH && object_type2 == OT_GEOM)
  {
    return CollisionFunctionLookTable.collision_matrix[node_type1][node_type2](o1, o2, num_max_contacts, exhaustive, enable_contact, contacts);
  }
  else if(object_type1 == OT_BVH && object_type2 == OT_BVH)
  {
    if(node_type1 == node_type2)
    {
      return CollisionFunctionLookTable.collision_matrix[node_type1][node_type2](o1, o2, num_max_contacts, exhaustive, enable_contact, contacts);
    }
  }

  return 0;
}

}

*/

#include "fcl/collision.h"
#include "fcl/simple_setup.h"
#include "fcl/geometric_shapes.h"
#include "fcl/BVH_model.h"
#include "fcl/collision_node.h"
#include "fcl/geometric_shapes_intersect.h"

namespace fcl
{

int collide(const CollisionObject* o1, const CollisionObject* o2,
             int num_max_contacts, bool exhaustive, bool enable_contact,
             std::vector<Contact>& contacts)
{
  const BVHModel<OBB>* obj1 = (BVHModel<OBB>*)o1;
  const BVHModel<OBB>* obj2 = (BVHModel<OBB>*)o2;

  PointCloudCollisionTraversalNodeOBB node;

  node.model1 = obj1;
  node.model2 = obj2;
  node.vertices1 = obj1->vertices;
  node.vertices2 = obj2->vertices;



  Uncertainty* uc1 = new Uncertainty[obj1->num_vertices];
  Uncertainty* uc2 = new Uncertainty[obj2->num_vertices];
  memcpy(uc1, obj1->uc, sizeof(Uncertainty) * obj1->num_vertices);
  memcpy(uc2, obj2->uc, sizeof(Uncertainty) * obj2->num_vertices);

  node.uc1.reset(uc1);
  node.uc2.reset(uc2);

  node.num_max_contacts = num_max_contacts;
  node.exhaustive = exhaustive;
  node.enable_contact = enable_contact;
  node.collision_prob_threshold = 0.6;
  node.leaf_size_threshold = 20;

  relativeTransform(obj1->getRotation(), obj1->getTranslation(), obj2->getRotation(), obj2->getTranslation(), node.R, node.T);

  collide(&node);
  int num_contacts = node.pairs.size();
  if(num_contacts > 0)
  {
    if((!exhaustive) && (num_contacts > num_max_contacts)) num_contacts = num_max_contacts;
    contacts.resize(num_contacts);
    for(int i = 0; i < num_contacts; ++i)
    {
      contacts[i] = Contact(obj1, obj2, node.pairs[i].id1_start, node.pairs[i].id2_start);
    }
  }
  return num_contacts;
}

}
