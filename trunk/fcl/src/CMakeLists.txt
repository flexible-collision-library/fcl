add_library(${PROJECT_NAME} SHARED
  BV/AABB.cpp
  BV/OBB.cpp
  BV/RSS.cpp
  BV/kIOS.cpp
  BV/OBBRSS.cpp
  BV/kDOP.cpp
  traversal/traversal_node_base.cpp
  traversal/traversal_node_bvhs.cpp
  intersect.cpp
  ccd/motion.cpp
  BVH/BV_fitter.cpp
  BVH/BV_splitter.cpp
  BVH/BVH_model.cpp
  BVH/BVH_utility.cpp
  math/transform.cpp
  traversal/traversal_node_setup.cpp
  shape/geometric_shapes.cpp
  shape/geometric_shapes_utility.cpp
  collision_node.cpp
  traversal/traversal_recurse.cpp
  broadphase/broadphase_bruteforce.cpp
  broadphase/broadphase_spatialhash.cpp
  broadphase/broadphase_SaP.cpp
  broadphase/broadphase_SSaP.cpp
  broadphase/broadphase_interval_tree.cpp
  broadphase/broadphase_dynamic_AABB_tree.cpp
  broadphase/broadphase_dynamic_AABB_tree_array.cpp 
  collision.cpp
  collision_func_matrix.cpp
  broadphase/interval_tree.cpp
  ccd/conservative_advancement.cpp
  ccd/interval.cpp
  ccd/interval_vector.cpp
  ccd/interval_matrix.cpp
  ccd/taylor_model.cpp
  ccd/taylor_vector.cpp
  ccd/taylor_matrix.cpp
  distance_func_matrix.cpp
  distance.cpp
  narrowphase/gjk.cpp
  narrowphase/gjk_libccd.cpp
  narrowphase/narrowphase.cpp
  broadphase/hierarchy_tree.cpp
  profile.cpp
  collision_data.cpp)

target_link_libraries(${PROJECT_NAME} ${CCD_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION lib/ LIBRARY DESTINATION lib/)
