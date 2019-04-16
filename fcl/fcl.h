/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Open Source Robotics Foundation
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

/** @author Jeongseok Lee <jslee02@gmail.com> */

// Automatically generated file by cmake

#ifndef FCL_FCL_H
#define FCL_FCL_H

#include "fcl/config.h"
#include "fcl/broadphase/broadphase_SSaP.h"
#include "fcl/broadphase/broadphase_SaP.h"
#include "fcl/broadphase/broadphase_bruteforce.h"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include "fcl/broadphase/broadphase_continuous_collision_manager.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree_array.h"
#include "fcl/broadphase/broadphase_interval_tree.h"
#include "fcl/broadphase/broadphase_spatialhash.h"
#include "fcl/common/exception.h"
#include "fcl/common/profiler.h"
#include "fcl/common/time.h"
#include "fcl/common/types.h"
#include "fcl/common/unused.h"
#include "fcl/common/warning.h"
#include "fcl/geometry/bvh/BVH_internal.h"
#include "fcl/geometry/bvh/BVH_model.h"
#include "fcl/geometry/bvh/BVH_utility.h"
#include "fcl/geometry/bvh/BV_node.h"
#include "fcl/geometry/bvh/BV_node_base.h"
#include "fcl/geometry/collision_geometry.h"
#include "fcl/geometry/geometric_shape_to_BVH_model.h"
#include "fcl/geometry/octree/octree.h"
#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/shape/capsule.h"
#include "fcl/geometry/shape/cone.h"
#include "fcl/geometry/shape/convex.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/shape/ellipsoid.h"
#include "fcl/geometry/shape/halfspace.h"
#include "fcl/geometry/shape/plane.h"
#include "fcl/geometry/shape/shape_base.h"
#include "fcl/geometry/shape/sphere.h"
#include "fcl/geometry/shape/triangle_p.h"
#include "fcl/geometry/shape/utility.h"
#include "fcl/math/bv/AABB.h"
#include "fcl/math/bv/OBB.h"
#include "fcl/math/bv/OBBRSS.h"
#include "fcl/math/bv/RSS.h"
#include "fcl/math/bv/kDOP.h"
#include "fcl/math/bv/kIOS.h"
#include "fcl/math/bv/utility.h"
#include "fcl/math/constants.h"
#include "fcl/math/geometry.h"
#include "fcl/math/motion/bv_motion_bound_visitor.h"
#include "fcl/math/motion/interp_motion.h"
#include "fcl/math/motion/motion_base.h"
#include "fcl/math/motion/screw_motion.h"
#include "fcl/math/motion/spline_motion.h"
#include "fcl/math/motion/taylor_model/interval.h"
#include "fcl/math/motion/taylor_model/interval_matrix.h"
#include "fcl/math/motion/taylor_model/interval_vector.h"
#include "fcl/math/motion/taylor_model/taylor_matrix.h"
#include "fcl/math/motion/taylor_model/taylor_model.h"
#include "fcl/math/motion/taylor_model/taylor_vector.h"
#include "fcl/math/motion/taylor_model/time_interval.h"
#include "fcl/math/motion/tbv_motion_bound_visitor.h"
#include "fcl/math/motion/translation_motion.h"
#include "fcl/math/motion/triangle_motion_bound_visitor.h"
#include "fcl/math/rng.h"
#include "fcl/math/sampler/sampler_base.h"
#include "fcl/math/sampler/sampler_r.h"
#include "fcl/math/sampler/sampler_se2.h"
#include "fcl/math/sampler/sampler_se2_disk.h"
#include "fcl/math/sampler/sampler_se3_euler.h"
#include "fcl/math/sampler/sampler_se3_euler_ball.h"
#include "fcl/math/sampler/sampler_se3_quat.h"
#include "fcl/math/sampler/sampler_se3_quat_ball.h"
#include "fcl/math/triangle.h"
#include "fcl/math/variance3.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/collision_request.h"
#include "fcl/narrowphase/collision_result.h"
#include "fcl/narrowphase/contact.h"
#include "fcl/narrowphase/contact_point.h"
#include "fcl/narrowphase/continuous_collision.h"
#include "fcl/narrowphase/continuous_collision_object.h"
#include "fcl/narrowphase/continuous_collision_request.h"
#include "fcl/narrowphase/continuous_collision_result.h"
#include "fcl/narrowphase/cost_source.h"
#include "fcl/narrowphase/distance.h"
#include "fcl/narrowphase/distance_request.h"
#include "fcl/narrowphase/distance_result.h"
#include "fcl/narrowphase/gjk_solver_type.h"


#endif
