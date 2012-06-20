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

#include "fcl/collision.h"
#include "fcl/simple_setup.h"
#include "fcl/geometric_shapes.h"
#include "fcl/BVH_model.h"
#include "fcl/collision_node.h"
#include "fcl/narrowphase/narrowphase.h"


namespace fcl
{

template<typename T_SH1, typename T_SH2>
int ShapeShapeCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<T_SH1, T_SH2> node;
  const T_SH1* obj1 = static_cast<const T_SH1*>(o1);
  const T_SH2* obj2 = static_cast<const T_SH2*>(o2);
  initialize(node, *obj1, tf1, *obj2, tf2, enable_contact);
  collide(&node);
  if(!node.is_collision) return 0;
  contacts.resize(1);                                                   
  if(!enable_contact) contacts[0] = Contact(o1, o2, 0, 0);
  else contacts[0] = Contact(o1, o2, 0, 0, node.contact_point, node.normal, node.penetration_depth);
  return 1;
}



template<typename T_BVH, typename T_SH>
static inline int BVHShapeContactCollection(const std::vector<BVHShapeCollisionPair>& pairs, const BVHModel<T_BVH>* obj1, const T_SH* obj2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  int num_contacts = pairs.size();
  if(num_contacts > 0)
  {
    if((!exhaustive) && (num_contacts > num_max_contacts)) num_contacts = num_max_contacts;
    contacts.resize(num_contacts);
    if(!enable_contact)
    {
      for(int i = 0; i < num_contacts; ++i)
        contacts[i] = Contact(obj1, obj2, pairs[i].id, 0);
    }
    else
    {
      for(int i = 0; i < num_contacts; ++i)
        contacts[i] = Contact(obj1, obj2, pairs[i].id, 0, pairs[i].contact_point, pairs[i].normal, pairs[i].penetration_depth);
    }
  }

  return num_contacts;
}


template<typename T_BVH, typename T_SH>
struct BVHShapeCollider
{
  static int collide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
  {
    MeshShapeCollisionTraversalNode<T_BVH, T_SH> node;
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
    BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
    SimpleTransform tf1_tmp = tf1;
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    initialize(node, *obj1_tmp, tf1_tmp, *obj2, tf2, num_max_contacts, exhaustive, enable_contact);
    fcl::collide(&node);

    int num_contacts = BVHShapeContactCollection(node.pairs, obj1, obj2, num_max_contacts, exhaustive, enable_contact, contacts);

    delete obj1_tmp;
    return num_contacts;
  }
};


template<typename T_SH>
struct BVHShapeCollider<OBB, T_SH>
{
  static int collide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
  {
    MeshShapeCollisionTraversalNodeOBB<T_SH> node;
    const BVHModel<OBB>* obj1 = static_cast<const BVHModel<OBB>* >(o1);
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    initialize(node, *obj1, tf1, *obj2, tf2, num_max_contacts, exhaustive, enable_contact);
    fcl::collide(&node);

    return BVHShapeContactCollection(node.pairs, obj1, obj2, num_max_contacts, exhaustive, enable_contact, contacts);
  } 
};

/*
template<typename T_SH>
struct BVHShapeCollider<RSS, T_SH>
{
  static int collide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
  {
    MeshShapeCollisionTraversalNodeRSS<T_SH> node;
    const BVHModel<RSS>* obj1 = static_cast<const BVHModel<RSS>* >(o1);
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    initialize(node, *obj1, tf1, *obj2, tf2, num_max_contacts, exhaustive, enable_contact);
    fcl::collide(&node);

    return BVHShapeContactCollection(node.pairs, obj1, obj2, num_max_contacts, exhaustive, enable_contact, contacts);
  } 
};


template<typename T_SH>
struct BVHShapeCollider<kIOS, T_SH>
{
  static int collide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
  {
    MeshShapeCollisionTraversalNodekIOS<T_SH> node;
    const BVHModel<kIOS>* obj1 = static_cast<const BVHModel<kIOS>* >(o1);
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    initialize(node, *obj1, tf1, *obj2, tf2, num_max_contacts, exhaustive, enable_contact);
    fcl::collide(&node);

    return BVHShapeContactCollection(node.pairs, obj1, obj2, num_max_contacts, exhaustive, enable_contact, contacts);
  } 
};

*/

template<typename T_BVH>
static inline int BVHContactCollection(const std::vector<BVHCollisionPair>& pairs, const BVHModel<T_BVH>* obj1, const BVHModel<T_BVH>* obj2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  int num_contacts = pairs.size();
  if(num_contacts > 0)
  {
    if((!exhaustive) && (num_contacts > num_max_contacts)) num_contacts = num_max_contacts;
    contacts.resize(num_contacts);
    if(!enable_contact)
    {
      for(int i = 0; i < num_contacts; ++i)
        contacts[i] = Contact(obj1, obj2, pairs[i].id1, pairs[i].id2);
    }
    else
    {
      for(int i = 0; i < num_contacts; ++i)
        contacts[i] = Contact(obj1, obj2, pairs[i].id1, pairs[i].id2, pairs[i].contact_point, pairs[i].normal, pairs[i].penetration_depth);
    }
  }

  return num_contacts;
}

// for OBB-alike contact 
template<typename T_BVH>
static inline int BVHContactCollection2(const std::vector<BVHCollisionPair>& pairs, const SimpleTransform& tf1, const BVHModel<T_BVH>* obj1, const BVHModel<T_BVH>* obj2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  int num_contacts = pairs.size();
  if(num_contacts > 0)
  {
    if((!exhaustive) && (num_contacts > num_max_contacts)) num_contacts = num_max_contacts;
    contacts.resize(num_contacts);
    if(!enable_contact)
    {
      for(int i = 0; i < num_contacts; ++i)
        contacts[i] = Contact(obj1, obj2, pairs[i].id1, pairs[i].id2);
    }
    else
    {
      for(int i = 0; i < num_contacts; ++i)
      {
        Vec3f normal = tf1.getRotation() * pairs[i].normal;
        Vec3f contact_point = tf1.transform(pairs[i].contact_point);
        contacts[i] = Contact(obj1, obj2, pairs[i].id1, pairs[i].id2, contact_point, normal, pairs[i].penetration_depth);
      }
    }
  }

  return num_contacts;
}



template<typename T_BVH>
int BVHCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshCollisionTraversalNode<T_BVH> node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>* >(o2);
  BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
  SimpleTransform tf1_tmp = tf1;
  BVHModel<T_BVH>* obj2_tmp = new BVHModel<T_BVH>(*obj2);
  SimpleTransform tf2_tmp = tf2;
  
  initialize(node, *obj1_tmp, tf1_tmp, *obj2_tmp, tf2_tmp, num_max_contacts, exhaustive, enable_contact);
  collide(&node);
  int num_contacts = BVHContactCollection(node.pairs, obj1, obj2, num_max_contacts, exhaustive, enable_contact, contacts);

  delete obj1_tmp;
  delete obj2_tmp;
  return num_contacts;
}

template<>
int BVHCollide<OBB>(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshCollisionTraversalNodeOBB node;
  const BVHModel<OBB>* obj1 = static_cast<const BVHModel<OBB>* >(o1);
  const BVHModel<OBB>* obj2 = static_cast<const BVHModel<OBB>* >(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, num_max_contacts, exhaustive, enable_contact);
  collide(&node);

  return BVHContactCollection2(node.pairs, tf1, obj1, obj2, num_max_contacts, exhaustive, enable_contact, contacts);
}

template<>
int BVHCollide<OBBRSS>(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshCollisionTraversalNodeOBBRSS node;
  const BVHModel<OBBRSS>* obj1 = static_cast<const BVHModel<OBBRSS>* >(o1);
  const BVHModel<OBBRSS>* obj2 = static_cast<const BVHModel<OBBRSS>* >(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, num_max_contacts, exhaustive, enable_contact);
  collide(&node);

  return BVHContactCollection2(node.pairs, tf1, obj1, obj2, num_max_contacts, exhaustive, enable_contact, contacts);
}


template<>
int BVHCollide<kIOS>(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshCollisionTraversalNodekIOS node;
  const BVHModel<kIOS>* obj1 = static_cast<const BVHModel<kIOS>* >(o1);
  const BVHModel<kIOS>* obj2 = static_cast<const BVHModel<kIOS>* >(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, num_max_contacts, exhaustive, enable_contact);
  collide(&node);

  return BVHContactCollection2(node.pairs, tf1,  obj1, obj2, num_max_contacts, exhaustive, enable_contact, contacts);
}


CollisionFunctionMatrix::CollisionFunctionMatrix()
{
  for(int i = 0; i < 16; ++i)
  {
    for(int j = 0; j < 16; ++j)
      collision_matrix[i][j] = NULL;
  }

  collision_matrix[GEOM_BOX][GEOM_BOX] = &ShapeShapeCollide<Box, Box>;
  collision_matrix[GEOM_BOX][GEOM_SPHERE] = &ShapeShapeCollide<Box, Sphere>;
  collision_matrix[GEOM_BOX][GEOM_CAPSULE] = &ShapeShapeCollide<Box, Capsule>;
  collision_matrix[GEOM_BOX][GEOM_CONE] = &ShapeShapeCollide<Box, Cone>;
  collision_matrix[GEOM_BOX][GEOM_CYLINDER] = &ShapeShapeCollide<Box, Cylinder>;
  collision_matrix[GEOM_BOX][GEOM_CONVEX] = &ShapeShapeCollide<Box, Convex>;
  collision_matrix[GEOM_BOX][GEOM_PLANE] = &ShapeShapeCollide<Box, Plane>;

  collision_matrix[GEOM_SPHERE][GEOM_BOX] = &ShapeShapeCollide<Sphere, Box>;
  collision_matrix[GEOM_SPHERE][GEOM_SPHERE] = &ShapeShapeCollide<Sphere, Sphere>;
  collision_matrix[GEOM_SPHERE][GEOM_CAPSULE] = &ShapeShapeCollide<Sphere, Capsule>;
  collision_matrix[GEOM_SPHERE][GEOM_CONE] = &ShapeShapeCollide<Sphere, Cone>;
  collision_matrix[GEOM_SPHERE][GEOM_CYLINDER] = &ShapeShapeCollide<Sphere, Cylinder>;
  collision_matrix[GEOM_SPHERE][GEOM_CONVEX] = &ShapeShapeCollide<Sphere, Convex>;
  collision_matrix[GEOM_SPHERE][GEOM_PLANE] = &ShapeShapeCollide<Sphere, Plane>;

  collision_matrix[GEOM_CAPSULE][GEOM_BOX] = &ShapeShapeCollide<Capsule, Box>;
  collision_matrix[GEOM_CAPSULE][GEOM_SPHERE] = &ShapeShapeCollide<Capsule, Sphere>;
  collision_matrix[GEOM_CAPSULE][GEOM_CAPSULE] = &ShapeShapeCollide<Capsule, Capsule>;
  collision_matrix[GEOM_CAPSULE][GEOM_CONE] = &ShapeShapeCollide<Capsule, Cone>;
  collision_matrix[GEOM_CAPSULE][GEOM_CYLINDER] = &ShapeShapeCollide<Capsule, Cylinder>;
  collision_matrix[GEOM_CAPSULE][GEOM_CONVEX] = &ShapeShapeCollide<Capsule, Convex>;
  collision_matrix[GEOM_CAPSULE][GEOM_PLANE] = &ShapeShapeCollide<Capsule, Plane>;

  collision_matrix[GEOM_CONE][GEOM_BOX] = &ShapeShapeCollide<Cone, Box>;
  collision_matrix[GEOM_CONE][GEOM_SPHERE] = &ShapeShapeCollide<Cone, Sphere>;
  collision_matrix[GEOM_CONE][GEOM_CAPSULE] = &ShapeShapeCollide<Cone, Capsule>;
  collision_matrix[GEOM_CONE][GEOM_CONE] = &ShapeShapeCollide<Cone, Cone>;
  collision_matrix[GEOM_CONE][GEOM_CYLINDER] = &ShapeShapeCollide<Cone, Cylinder>;
  collision_matrix[GEOM_CONE][GEOM_CONVEX] = &ShapeShapeCollide<Cone, Convex>;
  collision_matrix[GEOM_CONE][GEOM_PLANE] = &ShapeShapeCollide<Cone, Plane>;

  collision_matrix[GEOM_CYLINDER][GEOM_BOX] = &ShapeShapeCollide<Cylinder, Box>;
  collision_matrix[GEOM_CYLINDER][GEOM_SPHERE] = &ShapeShapeCollide<Cylinder, Sphere>;
  collision_matrix[GEOM_CYLINDER][GEOM_CAPSULE] = &ShapeShapeCollide<Cylinder, Capsule>;
  collision_matrix[GEOM_CYLINDER][GEOM_CONE] = &ShapeShapeCollide<Cylinder, Cone>;
  collision_matrix[GEOM_CYLINDER][GEOM_CYLINDER] = &ShapeShapeCollide<Cylinder, Cylinder>;
  collision_matrix[GEOM_CYLINDER][GEOM_CONVEX] = &ShapeShapeCollide<Cylinder, Convex>;
  collision_matrix[GEOM_CYLINDER][GEOM_PLANE] = &ShapeShapeCollide<Cylinder, Plane>;

  collision_matrix[GEOM_CONVEX][GEOM_BOX] = &ShapeShapeCollide<Convex, Box>;
  collision_matrix[GEOM_CONVEX][GEOM_SPHERE] = &ShapeShapeCollide<Convex, Sphere>;
  collision_matrix[GEOM_CONVEX][GEOM_CAPSULE] = &ShapeShapeCollide<Convex, Capsule>;
  collision_matrix[GEOM_CONVEX][GEOM_CONE] = &ShapeShapeCollide<Convex, Cone>;
  collision_matrix[GEOM_CONVEX][GEOM_CYLINDER] = &ShapeShapeCollide<Convex, Cylinder>;
  collision_matrix[GEOM_CONVEX][GEOM_CONVEX] = &ShapeShapeCollide<Convex, Convex>;
  collision_matrix[GEOM_CONVEX][GEOM_PLANE] = &ShapeShapeCollide<Convex, Plane>;

  collision_matrix[GEOM_PLANE][GEOM_BOX] = &ShapeShapeCollide<Plane, Box>;
  collision_matrix[GEOM_PLANE][GEOM_SPHERE] = &ShapeShapeCollide<Plane, Sphere>;
  collision_matrix[GEOM_PLANE][GEOM_CAPSULE] = &ShapeShapeCollide<Plane, Capsule>;
  collision_matrix[GEOM_PLANE][GEOM_CONE] = &ShapeShapeCollide<Plane, Cone>;
  collision_matrix[GEOM_PLANE][GEOM_CYLINDER] = &ShapeShapeCollide<Plane, Cylinder>;
  collision_matrix[GEOM_PLANE][GEOM_CONVEX] = &ShapeShapeCollide<Plane, Convex>;
  collision_matrix[GEOM_PLANE][GEOM_PLANE] = &ShapeShapeCollide<Plane, Plane>;

  collision_matrix[BV_AABB][GEOM_BOX] = &BVHShapeCollider<AABB, Box>::collide;
  collision_matrix[BV_AABB][GEOM_SPHERE] = &BVHShapeCollider<AABB, Sphere>::collide;
  collision_matrix[BV_AABB][GEOM_CAPSULE] = &BVHShapeCollider<AABB, Capsule>::collide;
  collision_matrix[BV_AABB][GEOM_CONE] = &BVHShapeCollider<AABB, Cone>::collide;
  collision_matrix[BV_AABB][GEOM_CYLINDER] = &BVHShapeCollider<AABB, Cylinder>::collide;
  collision_matrix[BV_AABB][GEOM_CONVEX] = &BVHShapeCollider<AABB, Convex>::collide;
  collision_matrix[BV_AABB][GEOM_PLANE] = &BVHShapeCollider<AABB, Plane>::collide;

  collision_matrix[BV_OBB][GEOM_BOX] = &BVHShapeCollider<OBB, Box>::collide;
  collision_matrix[BV_OBB][GEOM_SPHERE] = &BVHShapeCollider<OBB, Sphere>::collide;
  collision_matrix[BV_OBB][GEOM_CAPSULE] = &BVHShapeCollider<OBB, Capsule>::collide;
  collision_matrix[BV_OBB][GEOM_CONE] = &BVHShapeCollider<OBB, Cone>::collide;
  collision_matrix[BV_OBB][GEOM_CYLINDER] = &BVHShapeCollider<OBB, Cylinder>::collide;
  collision_matrix[BV_OBB][GEOM_CONVEX] = &BVHShapeCollider<OBB, Convex>::collide;
  collision_matrix[BV_OBB][GEOM_PLANE] = &BVHShapeCollider<OBB, Plane>::collide;

  collision_matrix[BV_RSS][GEOM_BOX] = &BVHShapeCollider<RSS, Box>::collide;
  collision_matrix[BV_RSS][GEOM_SPHERE] = &BVHShapeCollider<RSS, Sphere>::collide;
  collision_matrix[BV_RSS][GEOM_CAPSULE] = &BVHShapeCollider<RSS, Capsule>::collide;
  collision_matrix[BV_RSS][GEOM_CONE] = &BVHShapeCollider<RSS, Cone>::collide;
  collision_matrix[BV_RSS][GEOM_CYLINDER] = &BVHShapeCollider<RSS, Cylinder>::collide;
  collision_matrix[BV_RSS][GEOM_CONVEX] = &BVHShapeCollider<RSS, Convex>::collide;
  collision_matrix[BV_RSS][GEOM_PLANE] = &BVHShapeCollider<RSS, Plane>::collide;

  collision_matrix[BV_KDOP16][GEOM_BOX] = &BVHShapeCollider<KDOP<16>, Box>::collide;
  collision_matrix[BV_KDOP16][GEOM_SPHERE] = &BVHShapeCollider<KDOP<16>, Sphere>::collide;
  collision_matrix[BV_KDOP16][GEOM_CAPSULE] = &BVHShapeCollider<KDOP<16>, Capsule>::collide;
  collision_matrix[BV_KDOP16][GEOM_CONE] = &BVHShapeCollider<KDOP<16>, Cone>::collide;
  collision_matrix[BV_KDOP16][GEOM_CYLINDER] = &BVHShapeCollider<KDOP<16>, Cylinder>::collide;
  collision_matrix[BV_KDOP16][GEOM_CONVEX] = &BVHShapeCollider<KDOP<16>, Convex>::collide;
  collision_matrix[BV_KDOP16][GEOM_PLANE] = &BVHShapeCollider<KDOP<16>, Plane>::collide;

  collision_matrix[BV_KDOP18][GEOM_BOX] = &BVHShapeCollider<KDOP<18>, Box>::collide;
  collision_matrix[BV_KDOP18][GEOM_SPHERE] = &BVHShapeCollider<KDOP<18>, Sphere>::collide;
  collision_matrix[BV_KDOP18][GEOM_CAPSULE] = &BVHShapeCollider<KDOP<18>, Capsule>::collide;
  collision_matrix[BV_KDOP18][GEOM_CONE] = &BVHShapeCollider<KDOP<18>, Cone>::collide;
  collision_matrix[BV_KDOP18][GEOM_CYLINDER] = &BVHShapeCollider<KDOP<18>, Cylinder>::collide;
  collision_matrix[BV_KDOP18][GEOM_CONVEX] = &BVHShapeCollider<KDOP<18>, Convex>::collide;
  collision_matrix[BV_KDOP18][GEOM_PLANE] = &BVHShapeCollider<KDOP<18>, Plane>::collide;

  collision_matrix[BV_KDOP24][GEOM_BOX] = &BVHShapeCollider<KDOP<24>, Box>::collide;
  collision_matrix[BV_KDOP24][GEOM_SPHERE] = &BVHShapeCollider<KDOP<24>, Sphere>::collide;
  collision_matrix[BV_KDOP24][GEOM_CAPSULE] = &BVHShapeCollider<KDOP<24>, Capsule>::collide;
  collision_matrix[BV_KDOP24][GEOM_CONE] = &BVHShapeCollider<KDOP<24>, Cone>::collide;
  collision_matrix[BV_KDOP24][GEOM_CYLINDER] = &BVHShapeCollider<KDOP<24>, Cylinder>::collide;
  collision_matrix[BV_KDOP24][GEOM_CONVEX] = &BVHShapeCollider<KDOP<24>, Convex>::collide;
  collision_matrix[BV_KDOP24][GEOM_PLANE] = &BVHShapeCollider<KDOP<24>, Plane>::collide;

  collision_matrix[BV_kIOS][GEOM_BOX] = &BVHShapeCollider<kIOS, Box>::collide;
  collision_matrix[BV_kIOS][GEOM_SPHERE] = &BVHShapeCollider<kIOS, Sphere>::collide;
  collision_matrix[BV_kIOS][GEOM_CAPSULE] = &BVHShapeCollider<kIOS, Capsule>::collide;
  collision_matrix[BV_kIOS][GEOM_CONE] = &BVHShapeCollider<kIOS, Cone>::collide;
  collision_matrix[BV_kIOS][GEOM_CYLINDER] = &BVHShapeCollider<kIOS, Cylinder>::collide;
  collision_matrix[BV_kIOS][GEOM_CONVEX] = &BVHShapeCollider<kIOS, Convex>::collide;
  collision_matrix[BV_kIOS][GEOM_PLANE] = &BVHShapeCollider<kIOS, Plane>::collide;

  collision_matrix[BV_OBBRSS][GEOM_BOX] = &BVHShapeCollider<OBBRSS, Box>::collide;
  collision_matrix[BV_OBBRSS][GEOM_SPHERE] = &BVHShapeCollider<OBBRSS, Sphere>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CAPSULE] = &BVHShapeCollider<OBBRSS, Capsule>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CONE] = &BVHShapeCollider<OBBRSS, Cone>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CYLINDER] = &BVHShapeCollider<OBBRSS, Cylinder>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CONVEX] = &BVHShapeCollider<OBBRSS, Convex>::collide;
  collision_matrix[BV_OBBRSS][GEOM_PLANE] = &BVHShapeCollider<OBBRSS, Plane>::collide;

  collision_matrix[BV_AABB][BV_AABB] = &BVHCollide<AABB>;
  collision_matrix[BV_OBB][BV_OBB] = &BVHCollide<OBB>;
  collision_matrix[BV_RSS][BV_RSS] = &BVHCollide<RSS>;
  collision_matrix[BV_KDOP16][BV_KDOP16] = &BVHCollide<KDOP<16> >;
  collision_matrix[BV_KDOP18][BV_KDOP18] = &BVHCollide<KDOP<18> >;
  collision_matrix[BV_KDOP24][BV_KDOP24] = &BVHCollide<KDOP<24> >;
  collision_matrix[BV_kIOS][BV_kIOS] = &BVHCollide<kIOS>;
  collision_matrix[BV_OBBRSS][BV_OBBRSS] = &BVHCollide<OBBRSS>;
}

}
