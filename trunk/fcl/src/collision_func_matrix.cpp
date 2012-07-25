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


#include "fcl/collision_func_matrix.h"

#include "fcl/simple_setup.h"
#include "fcl/collision_node.h"
#include "fcl/narrowphase/narrowphase.h"


namespace fcl
{

template<typename T_SH>
static inline int OcTreeShapeContactCollection(const std::vector<Contact>& pairs, const OcTree* obj1, const T_SH* obj2,
                                               const CollisionRequest& request, CollisionResult& result)
{
  int num_contacts = pairs.size();
  if(num_contacts > 0)
  {
    if((!request.exhaustive) && (num_contacts > request.num_max_contacts)) num_contacts = request.num_max_contacts;
    std::vector<Contact>& contacts = result.contacts;
    contacts.resize(num_contacts);
    for(int i = 0; i < num_contacts; ++i)
      contacts[i] = pairs[i];
  }
  
  return num_contacts;
}

template<typename T_SH, typename NarrowPhaseSolver>
int ShapeOcTreeCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2,
                       const NarrowPhaseSolver* nsolver,
                       const CollisionRequest& request, CollisionResult& result)
{
  ShapeOcTreeCollisionTraversalNode<T_SH, NarrowPhaseSolver> node;
  const T_SH* obj1 = static_cast<const T_SH*>(o1);
  const OcTree* obj2 = static_cast<const OcTree*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  CollisionResult local_result;
  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, local_result);
  collide(&node);
  int num_contacts = OcTreeShapeContactCollection(local_result.contacts, obj2, obj1, request, result);

  return num_contacts;
}

template<typename T_SH, typename NarrowPhaseSolver>
int OcTreeShapeCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2,
                       const NarrowPhaseSolver* nsolver,
                       const CollisionRequest& request, CollisionResult& result)
{
  OcTreeShapeCollisionTraversalNode<T_SH, NarrowPhaseSolver> node;
  const OcTree* obj1 = static_cast<const OcTree*>(o1);
  const T_SH* obj2 = static_cast<const T_SH*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  CollisionResult local_result;
  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, local_result);
  collide(&node);
  int num_contacts = OcTreeShapeContactCollection(local_result.contacts, obj1, obj2, request, result);

  return num_contacts;
}

static inline int OcTreeContactCollection(const std::vector<Contact>& pairs, const OcTree* obj1, const OcTree* obj2,
                                          const CollisionRequest& request, CollisionResult& result)
{
  int num_contacts = pairs.size();
  if(num_contacts > 0)
  {
    if((!request.exhaustive) && (num_contacts > request.num_max_contacts)) num_contacts = request.num_max_contacts;
    std::vector<Contact>& contacts = result.contacts;
    contacts.resize(num_contacts);
    for(int i = 0; i < num_contacts; ++i)
      contacts[i] = pairs[i];
  }
  
  return num_contacts;
}

template<typename NarrowPhaseSolver>
int OcTreeCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2,
                  const NarrowPhaseSolver* nsolver,
                  const CollisionRequest& request, CollisionResult& result)
{
  OcTreeCollisionTraversalNode<NarrowPhaseSolver> node;
  const OcTree* obj1 = static_cast<const OcTree*>(o1);
  const OcTree* obj2 = static_cast<const OcTree*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  CollisionResult local_result;
  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, local_result);
  collide(&node);
  int num_contacts = OcTreeContactCollection(local_result.contacts, obj1, obj2, request, result);
  return num_contacts;
}


template<typename T_BVH>
static inline int OcTreeBVHContactCollection(const std::vector<Contact>& pairs, const OcTree* obj1, const BVHModel<T_BVH>* obj2,
                                             const CollisionRequest& request, CollisionResult& result)
{
  int num_contacts = pairs.size();
  if(num_contacts > 0)
  {
    if((!request.exhaustive) && (num_contacts > request.num_max_contacts)) num_contacts = request.num_max_contacts;
    std::vector<Contact>& contacts = result.contacts;
    contacts.resize(num_contacts);
    for(int i = 0; i < num_contacts; ++i)
      contacts[i] = pairs[i];
  }
  
  return num_contacts;
}

template<typename T_BVH, typename NarrowPhaseSolver>
int OcTreeBVHCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2,
                     const NarrowPhaseSolver* nsolver,
                     const CollisionRequest& request, CollisionResult& result)
{
  OcTreeMeshCollisionTraversalNode<T_BVH, NarrowPhaseSolver> node;
  const OcTree* obj1 = static_cast<const OcTree*>(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  CollisionResult local_result;
  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, local_result);
  collide(&node);
  int num_contacts = OcTreeBVHContactCollection(local_result.contacts, obj1, obj2, request, result);
  return num_contacts;
}

template<typename T_BVH, typename NarrowPhaseSolver>
int BVHOcTreeCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2,
                     const NarrowPhaseSolver* nsolver,
                     const CollisionRequest& request, CollisionResult& result)
{
  MeshOcTreeCollisionTraversalNode<T_BVH, NarrowPhaseSolver> node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>*>(o1);
  const OcTree* obj2 = static_cast<const OcTree*>(o2);
  OcTreeSolver<NarrowPhaseSolver> otsolver(nsolver);

  CollisionResult local_result;
  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, local_result);
  collide(&node);
  int num_contacts = OcTreeBVHContactCollection(local_result.contacts, obj2, obj1, request, result);
  return num_contacts;
}


template<typename T_SH1, typename T_SH2, typename NarrowPhaseSolver>
int ShapeShapeCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, 
                      const NarrowPhaseSolver* nsolver,
                      const CollisionRequest& request, CollisionResult& result)
{
  ShapeCollisionTraversalNode<T_SH1, T_SH2, NarrowPhaseSolver> node;
  const T_SH1* obj1 = static_cast<const T_SH1*>(o1);
  const T_SH2* obj2 = static_cast<const T_SH2*>(o2);

  CollisionResult local_result;
  initialize(node, *obj1, tf1, *obj2, tf2, nsolver, request, local_result);
  collide(&node);
  if(local_result.contacts.size() == 0) return 0;
  result.contacts.resize(1);                                                   
  result.contacts[0] = local_result.contacts[0];
  return 1;
}



template<typename T_BVH, typename T_SH>
static inline int BVHShapeContactCollection(const std::vector<Contact>& pairs, const BVHModel<T_BVH>* obj1, const T_SH* obj2,
                                            const CollisionRequest& request, CollisionResult& result)
{
  int num_contacts = pairs.size();
  if(num_contacts > 0)
  {
    if((!request.exhaustive) && (num_contacts > request.num_max_contacts)) num_contacts = request.num_max_contacts;
    std::vector<Contact>& contacts = result.contacts;
    contacts.resize(num_contacts);
    for(int i = 0; i < num_contacts; ++i)
      contacts[i] = pairs[i];
  }

  return num_contacts;
}


template<typename T_BVH, typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeCollider
{
  static int collide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, 
                     const NarrowPhaseSolver* nsolver,
                     const CollisionRequest& request, CollisionResult& result)
  {
    MeshShapeCollisionTraversalNode<T_BVH, T_SH, NarrowPhaseSolver> node;
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
    BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
    SimpleTransform tf1_tmp = tf1;
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    CollisionResult local_result;
    initialize(node, *obj1_tmp, tf1_tmp, *obj2, tf2, nsolver, request, local_result);
    fcl::collide(&node);

    int num_contacts = BVHShapeContactCollection(local_result.contacts, obj1, obj2, request, result);

    delete obj1_tmp;
    return num_contacts;
  }
};


template<typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeCollider<OBB, T_SH, NarrowPhaseSolver>
{
  static int collide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, 
                     const NarrowPhaseSolver* nsolver,
                     const CollisionRequest& request, CollisionResult& result)
  {
    MeshShapeCollisionTraversalNodeOBB<T_SH, NarrowPhaseSolver> node;
    const BVHModel<OBB>* obj1 = static_cast<const BVHModel<OBB>* >(o1);
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    CollisionResult local_result;
    initialize(node, *obj1, tf1, *obj2, tf2, nsolver, request, local_result);
    fcl::collide(&node);

    return BVHShapeContactCollection(local_result.contacts, obj1, obj2, request, result);
  } 
};


template<typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeCollider<RSS, T_SH, NarrowPhaseSolver>
{
  static int collide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, 
                     const NarrowPhaseSolver* nsolver,
                     const CollisionRequest& request, CollisionResult& result)
  {
    MeshShapeCollisionTraversalNodeRSS<T_SH, NarrowPhaseSolver> node;
    const BVHModel<RSS>* obj1 = static_cast<const BVHModel<RSS>* >(o1);
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    CollisionResult local_result;
    initialize(node, *obj1, tf1, *obj2, tf2, nsolver, request, local_result);
    fcl::collide(&node);

    return BVHShapeContactCollection(local_result.contacts, obj1, obj2, request, result);
  } 
};


template<typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeCollider<kIOS, T_SH, NarrowPhaseSolver>
{
  static int collide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, 
                     const NarrowPhaseSolver* nsolver,
                     const CollisionRequest& request, CollisionResult& result)
  {
    MeshShapeCollisionTraversalNodekIOS<T_SH, NarrowPhaseSolver> node;
    const BVHModel<kIOS>* obj1 = static_cast<const BVHModel<kIOS>* >(o1);
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    CollisionResult local_result;
    initialize(node, *obj1, tf1, *obj2, tf2, nsolver, request, local_result);
    fcl::collide(&node);

    return BVHShapeContactCollection(local_result.contacts, obj1, obj2, request, result);
  } 
};


template<typename T_SH, typename NarrowPhaseSolver>
struct BVHShapeCollider<OBBRSS, T_SH, NarrowPhaseSolver>
{
  static int collide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, 
                     const NarrowPhaseSolver* nsolver,
                     const CollisionRequest& request, CollisionResult& result)
  {
    MeshShapeCollisionTraversalNodeOBBRSS<T_SH, NarrowPhaseSolver> node;
    const BVHModel<OBBRSS>* obj1 = static_cast<const BVHModel<OBBRSS>* >(o1);
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    CollisionResult local_result;
    initialize(node, *obj1, tf1, *obj2, tf2, nsolver, request, local_result);
    fcl::collide(&node);

    return BVHShapeContactCollection(local_result.contacts, obj1, obj2, request, result);
  } 
};


template<typename T_BVH>
static inline int BVHContactCollection(const std::vector<Contact>& pairs, const BVHModel<T_BVH>* obj1, const BVHModel<T_BVH>* obj2, const CollisionRequest& request, CollisionResult& result)
{
  int num_contacts = pairs.size();
  if(num_contacts > 0)
  {
    if((!request.exhaustive) && (num_contacts > request.num_max_contacts)) num_contacts = request.num_max_contacts;
    std::vector<Contact>& contacts = result.contacts;
    contacts.resize(num_contacts);
    for(int i = 0; i < num_contacts; ++i)
      contacts[i] = pairs[i];
  }

  return num_contacts;
}


template<typename T_BVH>
int BVHCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, const CollisionRequest& request, CollisionResult& result)
{
  MeshCollisionTraversalNode<T_BVH> node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>* >(o2);
  BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
  SimpleTransform tf1_tmp = tf1;
  BVHModel<T_BVH>* obj2_tmp = new BVHModel<T_BVH>(*obj2);
  SimpleTransform tf2_tmp = tf2;
  
  CollisionResult local_result;
  initialize(node, *obj1_tmp, tf1_tmp, *obj2_tmp, tf2_tmp, request, local_result);
  collide(&node);
  int num_contacts = BVHContactCollection(local_result.contacts, obj1, obj2, request, result);

  delete obj1_tmp;
  delete obj2_tmp;
  return num_contacts;
}

template<>
int BVHCollide<OBB>(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, const CollisionRequest& request, CollisionResult& result)
{
  MeshCollisionTraversalNodeOBB node;
  const BVHModel<OBB>* obj1 = static_cast<const BVHModel<OBB>* >(o1);
  const BVHModel<OBB>* obj2 = static_cast<const BVHModel<OBB>* >(o2);

  CollisionResult local_result;
  initialize(node, *obj1, tf1, *obj2, tf2, request, local_result);
  collide(&node);

  return BVHContactCollection(local_result.contacts, obj1, obj2, request, result);
}

template<>
int BVHCollide<OBBRSS>(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, const CollisionRequest& request, CollisionResult& result)
{
  MeshCollisionTraversalNodeOBBRSS node;
  const BVHModel<OBBRSS>* obj1 = static_cast<const BVHModel<OBBRSS>* >(o1);
  const BVHModel<OBBRSS>* obj2 = static_cast<const BVHModel<OBBRSS>* >(o2);

  CollisionResult local_result;
  initialize(node, *obj1, tf1, *obj2, tf2, request, local_result);
  collide(&node);

  return BVHContactCollection(local_result.contacts, obj1, obj2, request, result);
}


template<>
int BVHCollide<kIOS>(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, const CollisionRequest& request, CollisionResult& result)
{
  MeshCollisionTraversalNodekIOS node;
  const BVHModel<kIOS>* obj1 = static_cast<const BVHModel<kIOS>* >(o1);
  const BVHModel<kIOS>* obj2 = static_cast<const BVHModel<kIOS>* >(o2);

  CollisionResult local_result;
  initialize(node, *obj1, tf1, *obj2, tf2, request, local_result);
  collide(&node);

  return BVHContactCollection(local_result.contacts, obj1, obj2, request, result);
}


template<typename T_BVH, typename NarrowPhaseSolver>
int BVHCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, 
               const NarrowPhaseSolver* nsolver,
               const CollisionRequest& request, CollisionResult& result)
{
  return BVHCollide<T_BVH>(o1, tf1, o2, tf2, request, result);
}


template<typename NarrowPhaseSolver>
CollisionFunctionMatrix<NarrowPhaseSolver>::CollisionFunctionMatrix()
{
  for(int i = 0; i < NODE_COUNT; ++i)
  {
    for(int j = 0; j < NODE_COUNT; ++j)
      collision_matrix[i][j] = NULL;
  }

  collision_matrix[GEOM_BOX][GEOM_BOX] = &ShapeShapeCollide<Box, Box, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_SPHERE] = &ShapeShapeCollide<Box, Sphere, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_CAPSULE] = &ShapeShapeCollide<Box, Capsule, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_CONE] = &ShapeShapeCollide<Box, Cone, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_CYLINDER] = &ShapeShapeCollide<Box, Cylinder, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_CONVEX] = &ShapeShapeCollide<Box, Convex, NarrowPhaseSolver>;
  collision_matrix[GEOM_BOX][GEOM_PLANE] = &ShapeShapeCollide<Box, Plane, NarrowPhaseSolver>;

  collision_matrix[GEOM_SPHERE][GEOM_BOX] = &ShapeShapeCollide<Sphere, Box, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_SPHERE] = &ShapeShapeCollide<Sphere, Sphere, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_CAPSULE] = &ShapeShapeCollide<Sphere, Capsule, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_CONE] = &ShapeShapeCollide<Sphere, Cone, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_CYLINDER] = &ShapeShapeCollide<Sphere, Cylinder, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_CONVEX] = &ShapeShapeCollide<Sphere, Convex, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_PLANE] = &ShapeShapeCollide<Sphere, Plane, NarrowPhaseSolver>;

  collision_matrix[GEOM_CAPSULE][GEOM_BOX] = &ShapeShapeCollide<Capsule, Box, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_SPHERE] = &ShapeShapeCollide<Capsule, Sphere, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_CAPSULE] = &ShapeShapeCollide<Capsule, Capsule, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_CONE] = &ShapeShapeCollide<Capsule, Cone, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_CYLINDER] = &ShapeShapeCollide<Capsule, Cylinder, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_CONVEX] = &ShapeShapeCollide<Capsule, Convex, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_PLANE] = &ShapeShapeCollide<Capsule, Plane, NarrowPhaseSolver>;

  collision_matrix[GEOM_CONE][GEOM_BOX] = &ShapeShapeCollide<Cone, Box, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_SPHERE] = &ShapeShapeCollide<Cone, Sphere, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_CAPSULE] = &ShapeShapeCollide<Cone, Capsule, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_CONE] = &ShapeShapeCollide<Cone, Cone, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_CYLINDER] = &ShapeShapeCollide<Cone, Cylinder, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_CONVEX] = &ShapeShapeCollide<Cone, Convex, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_PLANE] = &ShapeShapeCollide<Cone, Plane, NarrowPhaseSolver>;

  collision_matrix[GEOM_CYLINDER][GEOM_BOX] = &ShapeShapeCollide<Cylinder, Box, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_SPHERE] = &ShapeShapeCollide<Cylinder, Sphere, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_CAPSULE] = &ShapeShapeCollide<Cylinder, Capsule, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_CONE] = &ShapeShapeCollide<Cylinder, Cone, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_CYLINDER] = &ShapeShapeCollide<Cylinder, Cylinder, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_CONVEX] = &ShapeShapeCollide<Cylinder, Convex, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_PLANE] = &ShapeShapeCollide<Cylinder, Plane, NarrowPhaseSolver>;

  collision_matrix[GEOM_CONVEX][GEOM_BOX] = &ShapeShapeCollide<Convex, Box, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_SPHERE] = &ShapeShapeCollide<Convex, Sphere, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_CAPSULE] = &ShapeShapeCollide<Convex, Capsule, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_CONE] = &ShapeShapeCollide<Convex, Cone, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_CYLINDER] = &ShapeShapeCollide<Convex, Cylinder, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_CONVEX] = &ShapeShapeCollide<Convex, Convex, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_PLANE] = &ShapeShapeCollide<Convex, Plane, NarrowPhaseSolver>;

  collision_matrix[GEOM_PLANE][GEOM_BOX] = &ShapeShapeCollide<Plane, Box, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_SPHERE] = &ShapeShapeCollide<Plane, Sphere, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_CAPSULE] = &ShapeShapeCollide<Plane, Capsule, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_CONE] = &ShapeShapeCollide<Plane, Cone, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_CYLINDER] = &ShapeShapeCollide<Plane, Cylinder, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_CONVEX] = &ShapeShapeCollide<Plane, Convex, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_PLANE] = &ShapeShapeCollide<Plane, Plane, NarrowPhaseSolver>;

  collision_matrix[BV_AABB][GEOM_BOX] = &BVHShapeCollider<AABB, Box, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_SPHERE] = &BVHShapeCollider<AABB, Sphere, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_CAPSULE] = &BVHShapeCollider<AABB, Capsule, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_CONE] = &BVHShapeCollider<AABB, Cone, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_CYLINDER] = &BVHShapeCollider<AABB, Cylinder, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_CONVEX] = &BVHShapeCollider<AABB, Convex, NarrowPhaseSolver>::collide;
  collision_matrix[BV_AABB][GEOM_PLANE] = &BVHShapeCollider<AABB, Plane, NarrowPhaseSolver>::collide;

  collision_matrix[BV_OBB][GEOM_BOX] = &BVHShapeCollider<OBB, Box, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_SPHERE] = &BVHShapeCollider<OBB, Sphere, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_CAPSULE] = &BVHShapeCollider<OBB, Capsule, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_CONE] = &BVHShapeCollider<OBB, Cone, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_CYLINDER] = &BVHShapeCollider<OBB, Cylinder, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_CONVEX] = &BVHShapeCollider<OBB, Convex, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBB][GEOM_PLANE] = &BVHShapeCollider<OBB, Plane, NarrowPhaseSolver>::collide;

  collision_matrix[BV_RSS][GEOM_BOX] = &BVHShapeCollider<RSS, Box, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_SPHERE] = &BVHShapeCollider<RSS, Sphere, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_CAPSULE] = &BVHShapeCollider<RSS, Capsule, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_CONE] = &BVHShapeCollider<RSS, Cone, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_CYLINDER] = &BVHShapeCollider<RSS, Cylinder, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_CONVEX] = &BVHShapeCollider<RSS, Convex, NarrowPhaseSolver>::collide;
  collision_matrix[BV_RSS][GEOM_PLANE] = &BVHShapeCollider<RSS, Plane, NarrowPhaseSolver>::collide;

  collision_matrix[BV_KDOP16][GEOM_BOX] = &BVHShapeCollider<KDOP<16>, Box, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_SPHERE] = &BVHShapeCollider<KDOP<16>, Sphere, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_CAPSULE] = &BVHShapeCollider<KDOP<16>, Capsule, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_CONE] = &BVHShapeCollider<KDOP<16>, Cone, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_CYLINDER] = &BVHShapeCollider<KDOP<16>, Cylinder, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_CONVEX] = &BVHShapeCollider<KDOP<16>, Convex, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP16][GEOM_PLANE] = &BVHShapeCollider<KDOP<16>, Plane, NarrowPhaseSolver>::collide;

  collision_matrix[BV_KDOP18][GEOM_BOX] = &BVHShapeCollider<KDOP<18>, Box, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_SPHERE] = &BVHShapeCollider<KDOP<18>, Sphere, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_CAPSULE] = &BVHShapeCollider<KDOP<18>, Capsule, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_CONE] = &BVHShapeCollider<KDOP<18>, Cone, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_CYLINDER] = &BVHShapeCollider<KDOP<18>, Cylinder, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_CONVEX] = &BVHShapeCollider<KDOP<18>, Convex, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP18][GEOM_PLANE] = &BVHShapeCollider<KDOP<18>, Plane, NarrowPhaseSolver>::collide;

  collision_matrix[BV_KDOP24][GEOM_BOX] = &BVHShapeCollider<KDOP<24>, Box, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_SPHERE] = &BVHShapeCollider<KDOP<24>, Sphere, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_CAPSULE] = &BVHShapeCollider<KDOP<24>, Capsule, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_CONE] = &BVHShapeCollider<KDOP<24>, Cone, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_CYLINDER] = &BVHShapeCollider<KDOP<24>, Cylinder, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_CONVEX] = &BVHShapeCollider<KDOP<24>, Convex, NarrowPhaseSolver>::collide;
  collision_matrix[BV_KDOP24][GEOM_PLANE] = &BVHShapeCollider<KDOP<24>, Plane, NarrowPhaseSolver>::collide;

  collision_matrix[BV_kIOS][GEOM_BOX] = &BVHShapeCollider<kIOS, Box, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_SPHERE] = &BVHShapeCollider<kIOS, Sphere, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_CAPSULE] = &BVHShapeCollider<kIOS, Capsule, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_CONE] = &BVHShapeCollider<kIOS, Cone, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_CYLINDER] = &BVHShapeCollider<kIOS, Cylinder, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_CONVEX] = &BVHShapeCollider<kIOS, Convex, NarrowPhaseSolver>::collide;
  collision_matrix[BV_kIOS][GEOM_PLANE] = &BVHShapeCollider<kIOS, Plane, NarrowPhaseSolver>::collide;

  collision_matrix[BV_OBBRSS][GEOM_BOX] = &BVHShapeCollider<OBBRSS, Box, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_SPHERE] = &BVHShapeCollider<OBBRSS, Sphere, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CAPSULE] = &BVHShapeCollider<OBBRSS, Capsule, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CONE] = &BVHShapeCollider<OBBRSS, Cone, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CYLINDER] = &BVHShapeCollider<OBBRSS, Cylinder, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CONVEX] = &BVHShapeCollider<OBBRSS, Convex, NarrowPhaseSolver>::collide;
  collision_matrix[BV_OBBRSS][GEOM_PLANE] = &BVHShapeCollider<OBBRSS, Plane, NarrowPhaseSolver>::collide;

  collision_matrix[BV_AABB][BV_AABB] = &BVHCollide<AABB, NarrowPhaseSolver>;
  collision_matrix[BV_OBB][BV_OBB] = &BVHCollide<OBB, NarrowPhaseSolver>;
  collision_matrix[BV_RSS][BV_RSS] = &BVHCollide<RSS, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP16][BV_KDOP16] = &BVHCollide<KDOP<16>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP18][BV_KDOP18] = &BVHCollide<KDOP<18>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP24][BV_KDOP24] = &BVHCollide<KDOP<24>, NarrowPhaseSolver>;
  collision_matrix[BV_kIOS][BV_kIOS] = &BVHCollide<kIOS, NarrowPhaseSolver>;
  collision_matrix[BV_OBBRSS][BV_OBBRSS] = &BVHCollide<OBBRSS, NarrowPhaseSolver>;

  collision_matrix[GEOM_OCTREE][GEOM_BOX] = &OcTreeShapeCollide<Box, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_SPHERE] = &OcTreeShapeCollide<Sphere, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_CAPSULE] = &OcTreeShapeCollide<Capsule, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_CONE] = &OcTreeShapeCollide<Cone, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_CYLINDER] = &OcTreeShapeCollide<Cylinder, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_CONVEX] = &OcTreeShapeCollide<Convex, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][GEOM_PLANE] = &OcTreeShapeCollide<Plane, NarrowPhaseSolver>;

  collision_matrix[GEOM_BOX][GEOM_OCTREE] = &ShapeOcTreeCollide<Box, NarrowPhaseSolver>;
  collision_matrix[GEOM_SPHERE][GEOM_OCTREE] = &ShapeOcTreeCollide<Sphere, NarrowPhaseSolver>;
  collision_matrix[GEOM_CAPSULE][GEOM_OCTREE] = &ShapeOcTreeCollide<Capsule, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONE][GEOM_OCTREE] = &ShapeOcTreeCollide<Cone, NarrowPhaseSolver>;
  collision_matrix[GEOM_CYLINDER][GEOM_OCTREE] = &ShapeOcTreeCollide<Cylinder, NarrowPhaseSolver>;
  collision_matrix[GEOM_CONVEX][GEOM_OCTREE] = &ShapeOcTreeCollide<Convex, NarrowPhaseSolver>;
  collision_matrix[GEOM_PLANE][GEOM_OCTREE] = &ShapeOcTreeCollide<Plane, NarrowPhaseSolver>;

  collision_matrix[GEOM_OCTREE][GEOM_OCTREE] = &OcTreeCollide<NarrowPhaseSolver>;

  collision_matrix[GEOM_OCTREE][BV_AABB] = &OcTreeBVHCollide<AABB, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_OBB] = &OcTreeBVHCollide<OBB, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_RSS] = &OcTreeBVHCollide<RSS, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_OBBRSS] = &OcTreeBVHCollide<OBBRSS, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_kIOS] = &OcTreeBVHCollide<kIOS, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_KDOP16] = &OcTreeBVHCollide<KDOP<16>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_KDOP18] = &OcTreeBVHCollide<KDOP<18>, NarrowPhaseSolver>;
  collision_matrix[GEOM_OCTREE][BV_KDOP24] = &OcTreeBVHCollide<KDOP<24>, NarrowPhaseSolver>;

  collision_matrix[BV_AABB][GEOM_OCTREE] = &BVHOcTreeCollide<AABB, NarrowPhaseSolver>;
  collision_matrix[BV_OBB][GEOM_OCTREE] = &BVHOcTreeCollide<OBB, NarrowPhaseSolver>;
  collision_matrix[BV_RSS][GEOM_OCTREE] = &BVHOcTreeCollide<RSS, NarrowPhaseSolver>;
  collision_matrix[BV_OBBRSS][GEOM_OCTREE] = &BVHOcTreeCollide<OBBRSS, NarrowPhaseSolver>;
  collision_matrix[BV_kIOS][GEOM_OCTREE] = &BVHOcTreeCollide<kIOS, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP16][GEOM_OCTREE] = &BVHOcTreeCollide<KDOP<16>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP18][GEOM_OCTREE] = &BVHOcTreeCollide<KDOP<18>, NarrowPhaseSolver>;
  collision_matrix[BV_KDOP24][GEOM_OCTREE] = &BVHOcTreeCollide<KDOP<24>, NarrowPhaseSolver>;

}


template struct CollisionFunctionMatrix<GJKSolver_libccd>;
template struct CollisionFunctionMatrix<GJKSolver_indep>;


}
