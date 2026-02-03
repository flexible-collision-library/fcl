# -*- mode: cmake -*-
# vi: set ft=cmake :

# Copyright (c) 2012, Willow Garage, Inc.
# Copyright (c) 2017, Toyota Research Institute, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

set(GENERATED_FILE_MARKER "GENERATED FILE DO NOT EDIT")

configure_file(config.h.in config.h @ONLY)

generate_export_header(${PROJECT_NAME}
  EXPORT_FILE_NAME export.h
)

set(HEADERS_GEN
  fcl/config.h
  fcl/export.h
)

set(HEADERS
  fcl/broadphase/broadphase_SSaP-inl.h
  fcl/broadphase/broadphase_SSaP.h
  fcl/broadphase/broadphase_SaP-inl.h
  fcl/broadphase/broadphase_SaP.h
  fcl/broadphase/broadphase_bruteforce-inl.h
  fcl/broadphase/broadphase_bruteforce.h
  fcl/broadphase/broadphase_collision_manager-inl.h
  fcl/broadphase/broadphase_collision_manager.h
  fcl/broadphase/broadphase_continuous_collision_manager-inl.h
  fcl/broadphase/broadphase_continuous_collision_manager.h
  fcl/broadphase/broadphase_dynamic_AABB_tree-inl.h
  fcl/broadphase/broadphase_dynamic_AABB_tree.h
  fcl/broadphase/broadphase_dynamic_AABB_tree_array-inl.h
  fcl/broadphase/broadphase_dynamic_AABB_tree_array.h
  fcl/broadphase/broadphase_interval_tree-inl.h
  fcl/broadphase/broadphase_interval_tree.h
  fcl/broadphase/broadphase_spatialhash-inl.h
  fcl/broadphase/broadphase_spatialhash.h
  fcl/broadphase/default_broadphase_callbacks.h
  fcl/broadphase/detail/hierarchy_tree-inl.h
  fcl/broadphase/detail/hierarchy_tree.h
  fcl/broadphase/detail/hierarchy_tree_array-inl.h
  fcl/broadphase/detail/hierarchy_tree_array.h
  fcl/broadphase/detail/interval_tree-inl.h
  fcl/broadphase/detail/interval_tree.h
  fcl/broadphase/detail/interval_tree_node-inl.h
  fcl/broadphase/detail/interval_tree_node.h
  fcl/broadphase/detail/morton-inl.h
  fcl/broadphase/detail/morton.h
  fcl/broadphase/detail/node_base-inl.h
  fcl/broadphase/detail/node_base.h
  fcl/broadphase/detail/node_base_array-inl.h
  fcl/broadphase/detail/node_base_array.h
  fcl/broadphase/detail/simple_hash_table-inl.h
  fcl/broadphase/detail/simple_hash_table.h
  fcl/broadphase/detail/simple_interval-inl.h
  fcl/broadphase/detail/simple_interval.h
  fcl/broadphase/detail/sparse_hash_table-inl.h
  fcl/broadphase/detail/sparse_hash_table.h
  fcl/broadphase/detail/spatial_hash-inl.h
  fcl/broadphase/detail/spatial_hash.h
  fcl/common/detail/profiler.h
  fcl/common/exception.h
  fcl/common/profiler.h
  fcl/common/time.h
  fcl/common/types.h
  fcl/common/unused.h
  fcl/common/warning.h
  fcl/geometry/bvh/BVH_internal.h
  fcl/geometry/bvh/BVH_model-inl.h
  fcl/geometry/bvh/BVH_model.h
  fcl/geometry/bvh/BVH_utility-inl.h
  fcl/geometry/bvh/BVH_utility.h
  fcl/geometry/bvh/BV_node-inl.h
  fcl/geometry/bvh/BV_node.h
  fcl/geometry/bvh/BV_node_base.h
  fcl/geometry/bvh/detail/BVH_front.h
  fcl/geometry/bvh/detail/BV_fitter-inl.h
  fcl/geometry/bvh/detail/BV_fitter.h
  fcl/geometry/bvh/detail/BV_fitter_base.h
  fcl/geometry/bvh/detail/BV_splitter-inl.h
  fcl/geometry/bvh/detail/BV_splitter.h
  fcl/geometry/bvh/detail/BV_splitter_base.h
  fcl/geometry/collision_geometry-inl.h
  fcl/geometry/collision_geometry.h
  fcl/geometry/geometric_shape_to_BVH_model-inl.h
  fcl/geometry/geometric_shape_to_BVH_model.h
  fcl/geometry/octree/octree-inl.h
  fcl/geometry/octree/octree.h
  fcl/geometry/shape/box-inl.h
  fcl/geometry/shape/box.h
  fcl/geometry/shape/capsule-inl.h
  fcl/geometry/shape/capsule.h
  fcl/geometry/shape/cone-inl.h
  fcl/geometry/shape/cone.h
  fcl/geometry/shape/convex-inl.h
  fcl/geometry/shape/convex.h
  fcl/geometry/shape/cylinder-inl.h
  fcl/geometry/shape/cylinder.h
  fcl/geometry/shape/ellipsoid-inl.h
  fcl/geometry/shape/ellipsoid.h
  fcl/geometry/shape/halfspace-inl.h
  fcl/geometry/shape/halfspace.h
  fcl/geometry/shape/plane-inl.h
  fcl/geometry/shape/plane.h
  fcl/geometry/shape/shape_base-inl.h
  fcl/geometry/shape/shape_base.h
  fcl/geometry/shape/sphere-inl.h
  fcl/geometry/shape/sphere.h
  fcl/geometry/shape/triangle_p-inl.h
  fcl/geometry/shape/triangle_p.h
  fcl/geometry/shape/utility-inl.h
  fcl/geometry/shape/utility.h
  fcl/math/bv/AABB-inl.h
  fcl/math/bv/AABB.h
  fcl/math/bv/OBB-inl.h
  fcl/math/bv/OBB.h
  fcl/math/bv/OBBRSS-inl.h
  fcl/math/bv/OBBRSS.h
  fcl/math/bv/RSS-inl.h
  fcl/math/bv/RSS.h
  fcl/math/bv/kDOP-inl.h
  fcl/math/bv/kDOP.h
  fcl/math/bv/kIOS-inl.h
  fcl/math/bv/kIOS.h
  fcl/math/bv/utility-inl.h
  fcl/math/bv/utility.h
  fcl/math/constants.h
  fcl/math/detail/polysolver-inl.h
  fcl/math/detail/polysolver.h
  fcl/math/detail/project-inl.h
  fcl/math/detail/project.h
  fcl/math/detail/seed.h
  fcl/math/geometry-inl.h
  fcl/math/geometry.h
  fcl/math/motion/bv_motion_bound_visitor.h
  fcl/math/motion/interp_motion-inl.h
  fcl/math/motion/interp_motion.h
  fcl/math/motion/motion_base-inl.h
  fcl/math/motion/motion_base.h
  fcl/math/motion/screw_motion-inl.h
  fcl/math/motion/screw_motion.h
  fcl/math/motion/spline_motion-inl.h
  fcl/math/motion/spline_motion.h
  fcl/math/motion/taylor_model/interval-inl.h
  fcl/math/motion/taylor_model/interval.h
  fcl/math/motion/taylor_model/interval_matrix-inl.h
  fcl/math/motion/taylor_model/interval_matrix.h
  fcl/math/motion/taylor_model/interval_vector-inl.h
  fcl/math/motion/taylor_model/interval_vector.h
  fcl/math/motion/taylor_model/taylor_matrix-inl.h
  fcl/math/motion/taylor_model/taylor_matrix.h
  fcl/math/motion/taylor_model/taylor_model-inl.h
  fcl/math/motion/taylor_model/taylor_model.h
  fcl/math/motion/taylor_model/taylor_vector-inl.h
  fcl/math/motion/taylor_model/taylor_vector.h
  fcl/math/motion/taylor_model/time_interval-inl.h
  fcl/math/motion/taylor_model/time_interval.h
  fcl/math/motion/tbv_motion_bound_visitor-inl.h
  fcl/math/motion/tbv_motion_bound_visitor.h
  fcl/math/motion/translation_motion-inl.h
  fcl/math/motion/translation_motion.h
  fcl/math/motion/triangle_motion_bound_visitor-inl.h
  fcl/math/motion/triangle_motion_bound_visitor.h
  fcl/math/rng-inl.h
  fcl/math/rng.h
  fcl/math/sampler/sampler_base.h
  fcl/math/sampler/sampler_r-inl.h
  fcl/math/sampler/sampler_r.h
  fcl/math/sampler/sampler_se2-inl.h
  fcl/math/sampler/sampler_se2.h
  fcl/math/sampler/sampler_se2_disk-inl.h
  fcl/math/sampler/sampler_se2_disk.h
  fcl/math/sampler/sampler_se3_euler-inl.h
  fcl/math/sampler/sampler_se3_euler.h
  fcl/math/sampler/sampler_se3_euler_ball-inl.h
  fcl/math/sampler/sampler_se3_euler_ball.h
  fcl/math/sampler/sampler_se3_quat-inl.h
  fcl/math/sampler/sampler_se3_quat.h
  fcl/math/sampler/sampler_se3_quat_ball-inl.h
  fcl/math/sampler/sampler_se3_quat_ball.h
  fcl/math/triangle.h
  fcl/math/variance3-inl.h
  fcl/math/variance3.h
  fcl/narrowphase/collision-inl.h
  fcl/narrowphase/collision.h
  fcl/narrowphase/collision_object-inl.h
  fcl/narrowphase/collision_object.h
  fcl/narrowphase/collision_request-inl.h
  fcl/narrowphase/collision_request.h
  fcl/narrowphase/collision_result-inl.h
  fcl/narrowphase/collision_result.h
  fcl/narrowphase/contact-inl.h
  fcl/narrowphase/contact.h
  fcl/narrowphase/contact_point-inl.h
  fcl/narrowphase/contact_point.h
  fcl/narrowphase/continuous_collision-inl.h
  fcl/narrowphase/continuous_collision.h
  fcl/narrowphase/continuous_collision_object-inl.h
  fcl/narrowphase/continuous_collision_object.h
  fcl/narrowphase/continuous_collision_request-inl.h
  fcl/narrowphase/continuous_collision_request.h
  fcl/narrowphase/continuous_collision_result-inl.h
  fcl/narrowphase/continuous_collision_result.h
  fcl/narrowphase/cost_source-inl.h
  fcl/narrowphase/cost_source.h
  fcl/narrowphase/detail/collision_func_matrix-inl.h
  fcl/narrowphase/detail/collision_func_matrix.h
  fcl/narrowphase/detail/conservative_advancement_func_matrix-inl.h
  fcl/narrowphase/detail/conservative_advancement_func_matrix.h
  fcl/narrowphase/detail/convexity_based_algorithm/alloc.h
  fcl/narrowphase/detail/convexity_based_algorithm/epa-inl.h
  fcl/narrowphase/detail/convexity_based_algorithm/epa.h
  fcl/narrowphase/detail/convexity_based_algorithm/gjk-inl.h
  fcl/narrowphase/detail/convexity_based_algorithm/gjk.h
  fcl/narrowphase/detail/convexity_based_algorithm/gjk_libccd-inl.h
  fcl/narrowphase/detail/convexity_based_algorithm/gjk_libccd.h
  fcl/narrowphase/detail/convexity_based_algorithm/list.h
  fcl/narrowphase/detail/convexity_based_algorithm/minkowski_diff-inl.h
  fcl/narrowphase/detail/convexity_based_algorithm/minkowski_diff.h
  fcl/narrowphase/detail/convexity_based_algorithm/polytope.h
  fcl/narrowphase/detail/convexity_based_algorithm/simplex.h
  fcl/narrowphase/detail/convexity_based_algorithm/support.h
  fcl/narrowphase/detail/distance_func_matrix-inl.h
  fcl/narrowphase/detail/distance_func_matrix.h
  fcl/narrowphase/detail/failed_at_this_configuration.h
  fcl/narrowphase/detail/gjk_solver_indep-inl.h
  fcl/narrowphase/detail/gjk_solver_indep.h
  fcl/narrowphase/detail/gjk_solver_libccd-inl.h
  fcl/narrowphase/detail/gjk_solver_libccd.h
  fcl/narrowphase/detail/primitive_shape_algorithm/box_box-inl.h
  fcl/narrowphase/detail/primitive_shape_algorithm/box_box.h
  fcl/narrowphase/detail/primitive_shape_algorithm/capsule_capsule-inl.h
  fcl/narrowphase/detail/primitive_shape_algorithm/capsule_capsule.h
  fcl/narrowphase/detail/primitive_shape_algorithm/halfspace-inl.h
  fcl/narrowphase/detail/primitive_shape_algorithm/halfspace.h
  fcl/narrowphase/detail/primitive_shape_algorithm/plane-inl.h
  fcl/narrowphase/detail/primitive_shape_algorithm/plane.h
  fcl/narrowphase/detail/primitive_shape_algorithm/sphere_box-inl.h
  fcl/narrowphase/detail/primitive_shape_algorithm/sphere_box.h
  fcl/narrowphase/detail/primitive_shape_algorithm/sphere_capsule-inl.h
  fcl/narrowphase/detail/primitive_shape_algorithm/sphere_capsule.h
  fcl/narrowphase/detail/primitive_shape_algorithm/sphere_cylinder-inl.h
  fcl/narrowphase/detail/primitive_shape_algorithm/sphere_cylinder.h
  fcl/narrowphase/detail/primitive_shape_algorithm/sphere_sphere-inl.h
  fcl/narrowphase/detail/primitive_shape_algorithm/sphere_sphere.h
  fcl/narrowphase/detail/primitive_shape_algorithm/sphere_triangle-inl.h
  fcl/narrowphase/detail/primitive_shape_algorithm/sphere_triangle.h
  fcl/narrowphase/detail/primitive_shape_algorithm/triangle_distance-inl.h
  fcl/narrowphase/detail/primitive_shape_algorithm/triangle_distance.h
  fcl/narrowphase/detail/traversal/collision/bvh_collision_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/collision/bvh_collision_traversal_node.h
  fcl/narrowphase/detail/traversal/collision/bvh_shape_collision_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/collision/bvh_shape_collision_traversal_node.h
  fcl/narrowphase/detail/traversal/collision/collision_traversal_node_base-inl.h
  fcl/narrowphase/detail/traversal/collision/collision_traversal_node_base.h
  fcl/narrowphase/detail/traversal/collision/intersect-inl.h
  fcl/narrowphase/detail/traversal/collision/intersect.h
  fcl/narrowphase/detail/traversal/collision/mesh_collision_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/collision/mesh_collision_traversal_node.h
  fcl/narrowphase/detail/traversal/collision/mesh_continuous_collision_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/collision/mesh_continuous_collision_traversal_node.h
  fcl/narrowphase/detail/traversal/collision/mesh_shape_collision_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/collision/mesh_shape_collision_traversal_node.h
  fcl/narrowphase/detail/traversal/collision/shape_bvh_collision_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/collision/shape_bvh_collision_traversal_node.h
  fcl/narrowphase/detail/traversal/collision/shape_collision_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/collision/shape_collision_traversal_node.h
  fcl/narrowphase/detail/traversal/collision/shape_mesh_collision_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/collision/shape_mesh_collision_traversal_node.h
  fcl/narrowphase/detail/traversal/collision_node-inl.h
  fcl/narrowphase/detail/traversal/collision_node.h
  fcl/narrowphase/detail/traversal/distance/bvh_distance_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/distance/bvh_distance_traversal_node.h
  fcl/narrowphase/detail/traversal/distance/bvh_shape_distance_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/distance/bvh_shape_distance_traversal_node.h
  fcl/narrowphase/detail/traversal/distance/conservative_advancement_stack_data-inl.h
  fcl/narrowphase/detail/traversal/distance/conservative_advancement_stack_data.h
  fcl/narrowphase/detail/traversal/distance/distance_traversal_node_base-inl.h
  fcl/narrowphase/detail/traversal/distance/distance_traversal_node_base.h
  fcl/narrowphase/detail/traversal/distance/mesh_conservative_advancement_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/distance/mesh_conservative_advancement_traversal_node.h
  fcl/narrowphase/detail/traversal/distance/mesh_distance_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/distance/mesh_distance_traversal_node.h
  fcl/narrowphase/detail/traversal/distance/mesh_shape_conservative_advancement_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/distance/mesh_shape_conservative_advancement_traversal_node.h
  fcl/narrowphase/detail/traversal/distance/mesh_shape_distance_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/distance/mesh_shape_distance_traversal_node.h
  fcl/narrowphase/detail/traversal/distance/shape_bvh_distance_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/distance/shape_bvh_distance_traversal_node.h
  fcl/narrowphase/detail/traversal/distance/shape_conservative_advancement_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/distance/shape_conservative_advancement_traversal_node.h
  fcl/narrowphase/detail/traversal/distance/shape_distance_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/distance/shape_distance_traversal_node.h
  fcl/narrowphase/detail/traversal/distance/shape_mesh_conservative_advancement_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/distance/shape_mesh_conservative_advancement_traversal_node.h
  fcl/narrowphase/detail/traversal/distance/shape_mesh_distance_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/distance/shape_mesh_distance_traversal_node.h
  fcl/narrowphase/detail/traversal/octree/collision/mesh_octree_collision_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/octree/collision/mesh_octree_collision_traversal_node.h
  fcl/narrowphase/detail/traversal/octree/collision/octree_collision_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/octree/collision/octree_collision_traversal_node.h
  fcl/narrowphase/detail/traversal/octree/collision/octree_mesh_collision_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/octree/collision/octree_mesh_collision_traversal_node.h
  fcl/narrowphase/detail/traversal/octree/collision/octree_shape_collision_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/octree/collision/octree_shape_collision_traversal_node.h
  fcl/narrowphase/detail/traversal/octree/collision/shape_octree_collision_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/octree/collision/shape_octree_collision_traversal_node.h
  fcl/narrowphase/detail/traversal/octree/distance/mesh_octree_distance_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/octree/distance/mesh_octree_distance_traversal_node.h
  fcl/narrowphase/detail/traversal/octree/distance/octree_distance_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/octree/distance/octree_distance_traversal_node.h
  fcl/narrowphase/detail/traversal/octree/distance/octree_mesh_distance_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/octree/distance/octree_mesh_distance_traversal_node.h
  fcl/narrowphase/detail/traversal/octree/distance/octree_shape_distance_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/octree/distance/octree_shape_distance_traversal_node.h
  fcl/narrowphase/detail/traversal/octree/distance/shape_octree_distance_traversal_node-inl.h
  fcl/narrowphase/detail/traversal/octree/distance/shape_octree_distance_traversal_node.h
  fcl/narrowphase/detail/traversal/octree/octree_solver-inl.h
  fcl/narrowphase/detail/traversal/octree/octree_solver.h
  fcl/narrowphase/detail/traversal/traversal_node_base-inl.h
  fcl/narrowphase/detail/traversal/traversal_node_base.h
  fcl/narrowphase/detail/traversal/traversal_recurse-inl.h
  fcl/narrowphase/detail/traversal/traversal_recurse.h
  fcl/narrowphase/distance-inl.h
  fcl/narrowphase/distance.h
  fcl/narrowphase/distance_request-inl.h
  fcl/narrowphase/distance_request.h
  fcl/narrowphase/distance_result-inl.h
  fcl/narrowphase/distance_result.h
  fcl/narrowphase/gjk_solver_type.h
)

foreach(HEADER ${HEADERS_GEN} ${HEADERS})
  if(NOT HEADER MATCHES "(/detail/|-inl.h$)")
    set(INCLUDE_DIRECTIVES
      "${INCLUDE_DIRECTIVES}\n#include \"${HEADER}\""
    )
  endif()
endforeach()

string(STRIP "${INCLUDE_DIRECTIVES}" INCLUDE_DIRECTIVES)
configure_file(fcl.h.in fcl.h @ONLY)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/fcl
  COMPONENT Development
  FILES_MATCHING PATTERN "*.h"
  PATTERN "CMakeFiles" EXCLUDE
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
    ${CMAKE_CURRENT_BINARY_DIR}/export.h
    ${CMAKE_CURRENT_BINARY_DIR}/fcl.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/fcl
  COMPONENT Development
)

if(FCL_COVERALLS)
  get_filename_component(PARENT_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}"
    DIRECTORY
  )

  foreach(HEADER ${HEADERS_GEN})
    set(FCL_HEADERS "${PARENT_BINARY_DIR}/${HEADERS_GEN}")
  endforeach()

  foreach(HEADER ${HEADERS})
    set(FCL_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/${HEADER}")
  endforeach()

  set(FCL_HEADERS ${FCL_HEADERS} PARENT_SCOPE)
endif()
