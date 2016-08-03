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

/** \author Jeongseok Lee */

#ifndef FCL_TRAVERSAL_TRAVERSALNODES_H
#define FCL_TRAVERSAL_TRAVERSALNODES_H

#include "fcl/config.h"

#include "fcl/traversal/collision/bvh_collision_traversal_node.h"
#include "fcl/traversal/collision/bvh_shape_collision_traversal_node.h"
#include "fcl/traversal/collision/collision_traversal_node_base.h"
#include "fcl/traversal/collision/mesh_collision_traversal_node.h"
#include "fcl/traversal/collision/mesh_continuous_collision_traversal_node.h"
#include "fcl/traversal/collision/mesh_shape_collision_traversal_node.h"
#include "fcl/traversal/collision/shape_bvh_collision_traversal_node.h"
#include "fcl/traversal/collision/shape_collision_traversal_node.h"
#include "fcl/traversal/collision/shape_mesh_collision_traversal_node.h"

#include "fcl/traversal/distance/bvh_distance_traversal_node.h"
#include "fcl/traversal/distance/bvh_shape_distance_traversal_node.h"
#include "fcl/traversal/distance/distance_traversal_node_base.h"
#include "fcl/traversal/distance/mesh_distance_traversal_node.h"
#include "fcl/traversal/distance/mesh_conservative_advancement_traversal_node.h"
#include "fcl/traversal/distance/mesh_shape_distance_traversal_node.h"
#include "fcl/traversal/distance/mesh_shape_conservative_advancement_traversal_node.h"
#include "fcl/traversal/distance/shape_bvh_distance_traversal_node.h"
#include "fcl/traversal/distance/shape_distance_traversal_node.h"
#include "fcl/traversal/distance/shape_conservative_advancement_traversal_node.h"
#include "fcl/traversal/distance/shape_mesh_distance_traversal_node.h"
#include "fcl/traversal/distance/shape_mesh_conservative_advancement_traversal_node.h"

#if FCL_HAVE_OCTOMAP

#include "fcl/traversal/octree/octree_solver.h"

#include "fcl/traversal/octree/collision/mesh_octree_collision_traversal_node.h"
#include "fcl/traversal/octree/collision/octree_collision_traversal_node.h"
#include "fcl/traversal/octree/collision/octree_mesh_collision_traversal_node.h"
#include "fcl/traversal/octree/collision/octree_shape_collision_traversal_node.h"
#include "fcl/traversal/octree/collision/shape_octree_collision_traversal_node.h"

#include "fcl/traversal/octree/distance/mesh_octree_distance_traversal_node.h"
#include "fcl/traversal/octree/distance/octree_distance_traversal_node.h"
#include "fcl/traversal/octree/distance/octree_mesh_distance_traversal_node.h"
#include "fcl/traversal/octree/distance/octree_shape_distance_traversal_node.h"
#include "fcl/traversal/octree/distance/shape_octree_distance_traversal_node.h"

#endif // FCL_HAVE_OCTOMAP

#endif // FCL_TRAVERSAL_TRAVERSALNODES_H
