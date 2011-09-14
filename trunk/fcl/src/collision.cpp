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
    if(!CollisionFunctionLookTable.collision_matrix[node_type1][node_type2])
    {
      std::cerr << "Warning: collision function between node type " << node_type1 << " and node type " << node_type2 << " is not supported"<< std::endl;
      return 0;
    }

    return CollisionFunctionLookTable.collision_matrix[node_type1][node_type2](o1, o2, num_max_contacts, exhaustive, enable_contact, contacts);
  }
  else if(object_type1 == OT_GEOM && object_type2 == OT_BVH)
  {
    //if(!CollisionFunctionLookTable.collision_matrix[node_type1][node_type2])
    if(!CollisionFunctionLookTable.collision_matrix[node_type2][node_type1])
    {
      std::cerr << "Warning: collision function between node type " << node_type1 << " and node type " << node_type2 << " is not supported"<< std::endl;
      return 0;
    }

    //return CollisionFunctionLookTable.collision_matrix[node_type1][node_type2](o1, o2, num_max_contacts, exhaustive, enable_contact, contacts);
    return CollisionFunctionLookTable.collision_matrix[node_type2][node_type1](o2, o1, num_max_contacts, exhaustive, enable_contact, contacts);
  }
  else if(object_type1 == OT_BVH && object_type2 == OT_GEOM)
  {
    if(!CollisionFunctionLookTable.collision_matrix[node_type1][node_type2])
    {
      std::cerr << "Warning: collision function between node type " << node_type1 << " and node type " << node_type2 << " is not supported"<< std::endl;
      return 0;
    }

    return CollisionFunctionLookTable.collision_matrix[node_type1][node_type2](o1, o2, num_max_contacts, exhaustive, enable_contact, contacts);
  }
  else if(object_type1 == OT_BVH && object_type2 == OT_BVH)
  {
    if(!CollisionFunctionLookTable.collision_matrix[node_type1][node_type2])
    {
      std::cerr << "Warning: collision function between node type " << node_type1 << " and node type " << node_type2 << " is not supported"<< std::endl;
      return 0;
    }

    if(node_type1 == node_type2)
    {
      return CollisionFunctionLookTable.collision_matrix[node_type1][node_type2](o1, o2, num_max_contacts, exhaustive, enable_contact, contacts);
    }
  }

  return 0;
}

}
