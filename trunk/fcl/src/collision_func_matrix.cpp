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
#include "fcl/geometric_shapes_intersect.h"


namespace fcl
{

/** \brief Hey, I know it is ugly... but it is the best way I can find now... */

#define SHAPESHAPE_COMMON_CODE() do{ \
                                     initialize(node, *obj1, tf1, *obj2, tf2, enable_contact); \
                                     collide(&node); \
                                     if(!node.is_collision) return 0; \
                                     contacts.resize(1); \
                                     if(!enable_contact) contacts[0] = Contact(o1, o2, 0, 0); \
                                     else contacts[0] = Contact(o1, o2, 0, 0, node.contact_point, node.normal, node.penetration_depth); \
                                     return 1; \
                                     } while(0)


#define MESHSHAPE_COMMON_CODE() do{ \
                                    initialize(node, *obj1_tmp, tf1_tmp, *obj2, tf2, num_max_contacts, exhaustive, enable_contact); \
                                    collide(&node); \
                                    int num_contacts = node.pairs.size(); \
                                    if(num_contacts > 0) \
                                    { \
                                      if((!exhaustive) && (num_contacts > num_max_contacts)) num_contacts = num_max_contacts; \
                                      contacts.resize(num_contacts); \
                                      if(!enable_contact) \
                                      { \
                                        for(int i = 0; i < num_contacts; ++i) \
                                          contacts[i] = Contact(obj1, obj2, node.pairs[i].id, 0); \
                                      } \
                                      else \
                                      { \
                                        for(int i = 0; i < num_contacts; ++i) \
                                          contacts[i] = Contact(obj1, obj2, node.pairs[i].id, 0, node.pairs[i].contact_point, node.pairs[i].normal, node.pairs[i].penetration_depth); \
                                      } \
                                    } \
                                    delete obj1_tmp; \
                                    obj1_tmp = NULL; \
                                    return num_contacts; \
                                   } while(0)


#define SHAPEMESH_COMMON_CODE() do{ \
                                    initialize(node, *obj1, tf1, *obj2_tmp, tf2_tmp, num_max_contacts, exhaustive, enable_contact); \
                                    collide(&node); \
                                    int num_contacts = node.pairs.size(); \
                                    if(num_contacts > 0) \
                                    { \
                                      if((!exhaustive) && (num_contacts > num_max_contacts)) num_contacts = num_max_contacts; \
                                      contacts.resize(num_contacts); \
                                      if(!enable_contact) \
                                      { \
                                        for(int i = 0; i < num_contacts; ++i) \
                                          contacts[i] = Contact(obj1, obj2, node.pairs[i].id, 0); \
                                      } \
                                      else \
                                      { \
                                        for(int i = 0; i < num_contacts; ++i) \
                                          contacts[i] = Contact(obj1, obj2, node.pairs[i].id, 0, node.pairs[i].contact_point, node.pairs[i].normal, node.pairs[i].penetration_depth); \
                                      } \
                                    } \
                                    delete obj2_tmp; \
                                    obj2_tmp = NULL; \
                                    return num_contacts; \
                                   } while(0)

#define MESHSHAPEOBBRSS_COMMON_CODE() do{ \
                                    initialize(node, *obj1, tf1, *obj2, tf2, num_max_contacts, exhaustive, enable_contact); \
                                    collide(&node); \
                                    int num_contacts = node.pairs.size(); \
                                    if(num_contacts > 0) \
                                    { \
                                      if((!exhaustive) && (num_contacts > num_max_contacts)) num_contacts = num_max_contacts; \
                                      contacts.resize(num_contacts); \
                                      if(!enable_contact) \
                                      { \
                                        for(int i = 0; i < num_contacts; ++i) \
                                          contacts[i] = Contact(obj1, obj2, node.pairs[i].id, 0); \
                                      } \
                                      else \
                                      { \
                                        for(int i = 0; i < num_contacts; ++i) \
                                          contacts[i] = Contact(obj1, obj2, node.pairs[i].id, 0, node.pairs[i].contact_point, node.pairs[i].normal, node.pairs[i].penetration_depth); \
                                      } \
                                    } \
                                    return num_contacts; \
                                   } while(0)

#define MESHMESH_COMMON_CODE() do{ \
                                    initialize(node, *obj1_tmp, tf1_tmp, *obj2_tmp, tf2_tmp, num_max_contacts, exhaustive, enable_contact); \
                                    collide(&node); \
                                    int num_contacts = node.pairs.size(); \
                                    if(num_contacts > 0) \
                                    { \
                                      if((!exhaustive) && (num_contacts > num_max_contacts)) num_contacts = num_max_contacts; \
                                      contacts.resize(num_contacts); \
                                      if(!enable_contact) \
                                      { \
                                        for(int i = 0; i < num_contacts; ++i) \
                                          contacts[i] = Contact(obj1, obj2, node.pairs[i].id1, node.pairs[i].id2); \
                                      } \
                                      else \
                                      { \
                                        for(int i = 0; i < num_contacts; ++i) \
                                          contacts[i] = Contact(obj1, obj2, node.pairs[i].id1, node.pairs[i].id2, node.pairs[i].contact_point, node.pairs[i].normal, node.pairs[i].penetration_depth); \
                                      } \
                                    } \
                                    delete obj1_tmp; \
                                    obj1_tmp = NULL; \
                                    delete obj2_tmp; \
                                    obj2_tmp = NULL; \
                                    return num_contacts; \
                                   } while(0)


#define MESHMESHOBBRSS_COMMON_CODE() do{ \
                                         initialize(node, *obj1, tf1, *obj2, tf2, num_max_contacts, exhaustive, enable_contact); \
                                         collide(&node); \
                                         int num_contacts = node.pairs.size(); \
                                         if(num_contacts > 0) \
                                         { \
                                           if((!exhaustive) && (num_contacts > num_max_contacts)) num_contacts = num_max_contacts; \
                                           contacts.resize(num_contacts); \
                                           if(!enable_contact) \
                                           { \
                                             for(int i = 0; i < num_contacts; ++i) \
                                             contacts[i] = Contact(obj1, obj2, node.pairs[i].id1, node.pairs[i].id2); \
                                           } \
                                           else \
                                           { \
                                             for(int i = 0; i < num_contacts; ++i) \
                                             { \
                                               Vec3f normal = tf1.getRotation() * node.pairs[i].normal; \
                                               Vec3f contact_point = tf1.transform(node.pairs[i].contact_point); \
                                               contacts[i] = Contact(obj1, obj2, node.pairs[i].id1, node.pairs[i].id2, contact_point, normal, node.pairs[i].penetration_depth); \
                                             } \
                                           } \
                                         } \
                                         return num_contacts; \
                                       } while(0)



int BoxBoxCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Box, Box> node;
  const Box* obj1 = (Box*)o1;
  const Box* obj2 = (Box*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int BoxSphereCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Box, Sphere> node;
  const Box* obj1 = (Box*)o1;
  const Sphere* obj2 = (Sphere*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int BoxCapCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Box, Capsule> node;
  const Box* obj1 = (Box*)o1;
  const Capsule* obj2 = (Capsule*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int BoxConeCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Box, Cone> node;
  const Box* obj1 = (Box*)o1;
  const Cone* obj2 = (Cone*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int BoxCylinderCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Box, Cylinder> node;
  const Box* obj1 = (Box*)o1;
  const Cylinder* obj2 = (Cylinder*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int BoxConvexCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Box, Convex> node;
  const Box* obj1 = (Box*)o1;
  const Convex* obj2 = (Convex*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int BoxPlaneCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Box, Plane> node;
  const Box* obj1 = (Box*)o1;
  const Plane* obj2 = (Plane*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int SphereBoxCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Sphere, Box> node;
  const Sphere* obj1 = (Sphere*)o1;
  const Box* obj2 = (Box*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int SphereSphereCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Sphere, Sphere> node;
  const Sphere* obj1 = (Sphere*)o1;
  const Sphere* obj2 = (Sphere*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int SphereCapCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Sphere, Capsule> node;
  const Sphere* obj1 = (Sphere*)o1;
  const Capsule* obj2 = (Capsule*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int SphereConeCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Sphere, Cone> node;
  const Sphere* obj1 = (Sphere*)o1;
  const Cone* obj2 = (Cone*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int SphereCylinderCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Sphere, Cylinder> node;
  const Sphere* obj1 = (Sphere*)o1;
  const Cylinder* obj2 = (Cylinder*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int SphereConvexCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Sphere, Convex> node;
  const Sphere* obj1 = (Sphere*)o1;
  const Convex* obj2 = (Convex*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int SpherePlaneCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Sphere, Plane> node;
  const Sphere* obj1 = (Sphere*)o1;
  const Plane* obj2 = (Plane*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CapBoxCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Capsule, Box> node;
  const Capsule* obj1 = (Capsule*)o1;
  const Box* obj2 = (Box*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CapSphereCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Capsule, Sphere> node;
  const Capsule* obj1 = (Capsule*)o1;
  const Sphere* obj2 = (Sphere*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CapCapCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Capsule, Capsule> node;
  const Capsule* obj1 = (Capsule*)o1;
  const Capsule* obj2 = (Capsule*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CapConeCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Capsule, Cone> node;
  const Capsule* obj1 = (Capsule*)o1;
  const Cone* obj2 = (Cone*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CapCylinderCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Capsule, Cylinder> node;
  const Capsule* obj1 = (Capsule*)o1;
  const Cylinder* obj2 = (Cylinder*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CapConvexCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Capsule, Convex> node;
  const Capsule* obj1 = (Capsule*)o1;
  const Convex* obj2 = (Convex*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CapPlaneCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Capsule, Plane> node;
  const Capsule* obj1 = (Capsule*)o1;
  const Plane* obj2 = (Plane*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConeBoxCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cone, Box> node;
  const Cone* obj1 = (Cone*)o1;
  const Box* obj2 = (Box*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConeSphereCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cone, Sphere> node;
  const Cone* obj1 = (Cone*)o1;
  const Sphere* obj2 = (Sphere*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConeCapCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cone, Capsule> node;
  const Cone* obj1 = (Cone*)o1;
  const Capsule* obj2 = (Capsule*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConeConeCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cone, Cone> node;
  const Cone* obj1 = (Cone*)o1;
  const Cone* obj2 = (Cone*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConeCylinderCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cone, Cylinder> node;
  const Cone* obj1 = (Cone*)o1;
  const Cylinder* obj2 = (Cylinder*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConeConvexCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cone, Convex> node;
  const Cone* obj1 = (Cone*)o1;
  const Convex* obj2 = (Convex*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConePlaneCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cone, Plane> node;
  const Cone* obj1 = (Cone*)o1;
  const Plane* obj2 = (Plane*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CylinderBoxCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cylinder, Box> node;
  const Cylinder* obj1 = (Cylinder*)o1;
  const Box* obj2 = (Box*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CylinderSphereCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cylinder, Sphere> node;
  const Cylinder* obj1 = (Cylinder*)o1;
  const Sphere* obj2 = (Sphere*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CylinderCapCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cylinder, Capsule> node;
  const Cylinder* obj1 = (Cylinder*)o1;
  const Capsule* obj2 = (Capsule*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CylinderConeCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cylinder, Cone> node;
  const Cylinder* obj1 = (Cylinder*)o1;
  const Cone* obj2 = (Cone*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CylinderCylinderCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cylinder, Cylinder> node;
  const Cylinder* obj1 = (Cylinder*)o1;
  const Cylinder* obj2 = (Cylinder*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CylinderConvexCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cylinder, Convex> node;
  const Cylinder* obj1 = (Cylinder*)o1;
  const Convex* obj2 = (Convex*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CylinderPlaneCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cylinder, Plane> node;
  const Cylinder* obj1 = (Cylinder*)o1;
  const Plane* obj2 = (Plane*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConvexBoxCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Convex, Box> node;
  const Convex* obj1 = (Convex*)o1;
  const Box* obj2 = (Box*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConvexSphereCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Convex, Sphere> node;
  const Convex* obj1 = (Convex*)o1;
  const Sphere* obj2 = (Sphere*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConvexCapCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Convex, Capsule> node;
  const Convex* obj1 = (Convex*)o1;
  const Capsule* obj2 = (Capsule*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConvexConeCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Convex, Cone> node;
  const Convex* obj1 = (Convex*)o1;
  const Cone* obj2 = (Cone*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConvexCylinderCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Convex, Cylinder> node;
  const Convex* obj1 = (Convex*)o1;
  const Cylinder* obj2 = (Cylinder*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConvexConvexCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Convex, Convex> node;
  const Convex* obj1 = (Convex*)o1;
  const Convex* obj2 = (Convex*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConvexPlaneCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Convex, Plane> node;
  const Convex* obj1 = (Convex*)o1;
  const Plane* obj2 = (Plane*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int PlaneBoxCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Plane, Box> node;
  const Plane* obj1 = (Plane*)o1;
  const Box* obj2 = (Box*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int PlaneSphereCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Plane, Sphere> node;
  const Plane* obj1 = (Plane*)o1;
  const Sphere* obj2 = (Sphere*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int PlaneCapCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Plane, Capsule> node;
  const Plane* obj1 = (Plane*)o1;
  const Capsule* obj2 = (Capsule*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int PlaneConeCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Plane, Cone> node;
  const Plane* obj1 = (Plane*)o1;
  const Cone* obj2 = (Cone*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int PlaneCylinderCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Plane, Cylinder> node;
  const Plane* obj1 = (Plane*)o1;
  const Cylinder* obj2 = (Cylinder*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int PlaneConvexCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Plane, Convex> node;
  const Plane* obj1 = (Plane*)o1;
  const Convex* obj2 = (Convex*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int PlanePlaneCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Plane, Plane> node;
  const Plane* obj1 = (Plane*)o1;
  const Plane* obj2 = (Plane*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int AABBBoxCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<AABB, Box> node;
  const BVHModel<AABB>* obj1 = (BVHModel<AABB>*)o1;
  BVHModel<AABB>* obj1_tmp = new BVHModel<AABB>(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Box* obj2 = (Box*)o2;
  MESHSHAPE_COMMON_CODE();
}

int AABBSphereCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<AABB, Sphere> node;
  const BVHModel<AABB>* obj1 = (BVHModel<AABB>*)o1;
  BVHModel<AABB>* obj1_tmp = new BVHModel<AABB>(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Sphere* obj2 = (Sphere*)o2;
  MESHSHAPE_COMMON_CODE();
}

int AABBCapCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<AABB, Capsule> node;
  const BVHModel<AABB>* obj1 = (BVHModel<AABB>*)o1;
  BVHModel<AABB>* obj1_tmp = new BVHModel<AABB>(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Capsule* obj2 = (Capsule*)o2;
  MESHSHAPE_COMMON_CODE();
}

int AABBConeCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<AABB, Cone> node;
  const BVHModel<AABB>* obj1 = (BVHModel<AABB>*)o1;
  BVHModel<AABB>* obj1_tmp = new BVHModel<AABB>(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Cone* obj2 = (Cone*)o2;
  MESHSHAPE_COMMON_CODE();
}

int AABBCylinderCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<AABB, Cylinder> node;
  const BVHModel<AABB>* obj1 = (BVHModel<AABB>*)o1;
  BVHModel<AABB>* obj1_tmp = new BVHModel<AABB>(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Cylinder* obj2 = (Cylinder*)o2;
  MESHSHAPE_COMMON_CODE();
}

int AABBConvexCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<AABB, Convex> node;
  const BVHModel<AABB>* obj1 = (BVHModel<AABB>*)o1;
  BVHModel<AABB>* obj1_tmp = new BVHModel<AABB>(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Convex* obj2 = (Convex*)o2;
  MESHSHAPE_COMMON_CODE();
}

int AABBPlaneCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<AABB, Plane> node;
  const BVHModel<AABB>* obj1 = (BVHModel<AABB>*)o1;
  BVHModel<AABB>* obj1_tmp = new BVHModel<AABB>(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Plane* obj2 = (Plane*)o2;
  MESHSHAPE_COMMON_CODE();
}

int OBBBoxCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNodeOBB<Box> node;
  const BVHModel<OBB>* obj1 = (BVHModel<OBB>*)o1;
  const Box* obj2 = (Box*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int OBBSphereCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNodeOBB<Sphere> node;
  const BVHModel<OBB>* obj1 = (BVHModel<OBB>*)o1;
  const Sphere* obj2 = (Sphere*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int OBBCapCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNodeOBB<Capsule> node;
  const BVHModel<OBB>* obj1 = (BVHModel<OBB>*)o1;
  const Capsule* obj2 = (Capsule*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int OBBConeCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNodeOBB<Cone> node;
  const BVHModel<OBB>* obj1 = (BVHModel<OBB>*)o1;
  const Cone* obj2 = (Cone*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int OBBCylinderCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNodeOBB<Cylinder> node;
  const BVHModel<OBB>* obj1 = (BVHModel<OBB>*)o1;
  const Cylinder* obj2 = (Cylinder*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int OBBConvexCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNodeOBB<Convex> node;
  const BVHModel<OBB>* obj1 = (BVHModel<OBB>*)o1;
  const Convex* obj2 = (Convex*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int OBBPlaneCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNodeOBB<Plane> node;
  const BVHModel<OBB>* obj1 = (BVHModel<OBB>*)o1;
  const Plane* obj2 = (Plane*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int RSSBoxCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<RSS, Box> node;
  const BVHModel<RSS>* obj1 = (BVHModel<RSS>*)o1;
  BVHModel<RSS>* obj1_tmp = new BVHModel<RSS>(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Box* obj2 = (Box*)o2;
  MESHSHAPE_COMMON_CODE();
}

int RSSSphereCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<RSS, Sphere> node;
  const BVHModel<RSS>* obj1 = (BVHModel<RSS>*)o1;
  BVHModel<RSS>* obj1_tmp = new BVHModel<RSS>(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Sphere* obj2 = (Sphere*)o2;
  MESHSHAPE_COMMON_CODE();
}

int RSSCapCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<RSS, Capsule> node;
  const BVHModel<RSS>* obj1 = (BVHModel<RSS>*)o1;
  BVHModel<RSS>* obj1_tmp = new BVHModel<RSS>(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Capsule* obj2 = (Capsule*)o2;
  MESHSHAPE_COMMON_CODE();
}

int RSSConeCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<RSS, Cone> node;
  const BVHModel<RSS>* obj1 = (BVHModel<RSS>*)o1;
  BVHModel<RSS>* obj1_tmp = new BVHModel<RSS>(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Cone* obj2 = (Cone*)o2;
  MESHSHAPE_COMMON_CODE();
}

int RSSCylinderCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<RSS, Cylinder> node;
  const BVHModel<RSS>* obj1 = (BVHModel<RSS>*)o1;
  BVHModel<RSS>* obj1_tmp = new BVHModel<RSS>(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Cylinder* obj2 = (Cylinder*)o2;
  MESHSHAPE_COMMON_CODE();
}

int RSSConvexCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<RSS, Convex> node;
  const BVHModel<RSS>* obj1 = (BVHModel<RSS>*)o1;
  BVHModel<RSS>* obj1_tmp = new BVHModel<RSS>(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Convex* obj2 = (Convex*)o2;
  MESHSHAPE_COMMON_CODE();
}

int RSSPlaneCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<RSS, Plane> node;
  const BVHModel<RSS>* obj1 = (BVHModel<RSS>*)o1;
  BVHModel<RSS>* obj1_tmp = new BVHModel<RSS>(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Plane* obj2 = (Plane*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP16BoxCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<16> , Box> node;
  const BVHModel<KDOP<16> >* obj1 = (BVHModel<KDOP<16> >*)o1;
  BVHModel<KDOP<16> >* obj1_tmp = new BVHModel<KDOP<16> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Box* obj2 = (Box*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP16SphereCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<16> , Sphere> node;
  const BVHModel<KDOP<16> >* obj1 = (BVHModel<KDOP<16> >*)o1;
  BVHModel<KDOP<16> >* obj1_tmp = new BVHModel<KDOP<16> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Sphere* obj2 = (Sphere*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP16CapCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<16> , Capsule> node;
  const BVHModel<KDOP<16> >* obj1 = (BVHModel<KDOP<16> >*)o1;
  BVHModel<KDOP<16> >* obj1_tmp = new BVHModel<KDOP<16> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Capsule* obj2 = (Capsule*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP16ConeCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<16> , Cone> node;
  const BVHModel<KDOP<16> >* obj1 = (BVHModel<KDOP<16> >*)o1;
  BVHModel<KDOP<16> >* obj1_tmp = new BVHModel<KDOP<16> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Cone* obj2 = (Cone*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP16CylinderCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<16> , Cylinder> node;
  const BVHModel<KDOP<16> >* obj1 = (BVHModel<KDOP<16> >*)o1;
  BVHModel<KDOP<16> >* obj1_tmp = new BVHModel<KDOP<16> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Cylinder* obj2 = (Cylinder*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP16ConvexCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<16> , Convex> node;
  const BVHModel<KDOP<16> >* obj1 = (BVHModel<KDOP<16> >*)o1;
  BVHModel<KDOP<16> >* obj1_tmp = new BVHModel<KDOP<16> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Convex* obj2 = (Convex*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP16PlaneCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<16> , Plane> node;
  const BVHModel<KDOP<16> >* obj1 = (BVHModel<KDOP<16> >*)o1;
  BVHModel<KDOP<16> >* obj1_tmp = new BVHModel<KDOP<16> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Plane* obj2 = (Plane*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP18BoxCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<18> , Box> node;
  const BVHModel<KDOP<18> >* obj1 = (BVHModel<KDOP<18> >*)o1;
  BVHModel<KDOP<18> >* obj1_tmp = new BVHModel<KDOP<18> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Box* obj2 = (Box*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP18SphereCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<18> , Sphere> node;
  const BVHModel<KDOP<18> >* obj1 = (BVHModel<KDOP<18> >*)o1;
  BVHModel<KDOP<18> >* obj1_tmp = new BVHModel<KDOP<18> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Sphere* obj2 = (Sphere*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP18CapCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<18> , Capsule> node;
  const BVHModel<KDOP<18> >* obj1 = (BVHModel<KDOP<18> >*)o1;
  BVHModel<KDOP<18> >* obj1_tmp = new BVHModel<KDOP<18> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Capsule* obj2 = (Capsule*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP18ConeCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<18> , Cone> node;
  const BVHModel<KDOP<18> >* obj1 = (BVHModel<KDOP<18> >*)o1;
  BVHModel<KDOP<18> >* obj1_tmp = new BVHModel<KDOP<18> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Cone* obj2 = (Cone*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP18CylinderCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<18> , Cylinder> node;
  const BVHModel<KDOP<18> >* obj1 = (BVHModel<KDOP<18> >*)o1;
  BVHModel<KDOP<18> >* obj1_tmp = new BVHModel<KDOP<18> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Cylinder* obj2 = (Cylinder*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP18ConvexCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<18> , Convex> node;
  const BVHModel<KDOP<18> >* obj1 = (BVHModel<KDOP<18> >*)o1;
  BVHModel<KDOP<18> >* obj1_tmp = new BVHModel<KDOP<18> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Convex* obj2 = (Convex*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP18PlaneCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<18> , Plane> node;
  const BVHModel<KDOP<18> >* obj1 = (BVHModel<KDOP<18> >*)o1;
  BVHModel<KDOP<18> >* obj1_tmp = new BVHModel<KDOP<18> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Plane* obj2 = (Plane*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP24BoxCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<24> , Box> node;
  const BVHModel<KDOP<24> >* obj1 = (BVHModel<KDOP<24> >*)o1;
  BVHModel<KDOP<24> >* obj1_tmp = new BVHModel<KDOP<24> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Box* obj2 = (Box*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP24SphereCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<24> , Sphere> node;
  const BVHModel<KDOP<24> >* obj1 = (BVHModel<KDOP<24> >*)o1;
  BVHModel<KDOP<24> >* obj1_tmp = new BVHModel<KDOP<24> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Sphere* obj2 = (Sphere*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP24CapCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<24> , Capsule> node;
  const BVHModel<KDOP<24> >* obj1 = (BVHModel<KDOP<24> >*)o1;
  BVHModel<KDOP<24> >* obj1_tmp = new BVHModel<KDOP<24> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Capsule* obj2 = (Capsule*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP24ConeCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<24> , Cone> node;
  const BVHModel<KDOP<24> >* obj1 = (BVHModel<KDOP<24> >*)o1;
  BVHModel<KDOP<24> >* obj1_tmp = new BVHModel<KDOP<24> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Cone* obj2 = (Cone*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP24CylinderCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<24> , Cylinder> node;
  const BVHModel<KDOP<24> >* obj1 = (BVHModel<KDOP<24> >*)o1;
  BVHModel<KDOP<24> >* obj1_tmp = new BVHModel<KDOP<24> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Cylinder* obj2 = (Cylinder*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP24ConvexCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<24> , Convex> node;
  const BVHModel<KDOP<24> >* obj1 = (BVHModel<KDOP<24> >*)o1;
  BVHModel<KDOP<24> >* obj1_tmp = new BVHModel<KDOP<24> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Convex* obj2 = (Convex*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP24PlaneCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<24> , Plane> node;
  const BVHModel<KDOP<24> >* obj1 = (BVHModel<KDOP<24> >*)o1;
  BVHModel<KDOP<24> >* obj1_tmp = new BVHModel<KDOP<24> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  const Plane* obj2 = (Plane*)o2;
  MESHSHAPE_COMMON_CODE();
}


int AABBAABBCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshCollisionTraversalNode<AABB> node;
  const BVHModel<AABB>* obj1 = (BVHModel<AABB>*)o1;
  const BVHModel<AABB>* obj2 = (BVHModel<AABB>*)o2;
  BVHModel<AABB>* obj1_tmp = new BVHModel<AABB>(*obj1);
  SimpleTransform tf1_tmp = tf1;
  BVHModel<AABB>* obj2_tmp = new BVHModel<AABB>(*obj2);
  SimpleTransform tf2_tmp = tf2;
  MESHMESH_COMMON_CODE();
}

int OBBOBBCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshCollisionTraversalNodeOBB node;
  const BVHModel<OBB>* obj1 = (BVHModel<OBB>*)o1;
  const BVHModel<OBB>* obj2 = (BVHModel<OBB>*)o2;
  MESHMESHOBBRSS_COMMON_CODE();
}

int RSSRSSCollide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshCollisionTraversalNodeRSS node;
  const BVHModel<RSS>* obj1 = (BVHModel<RSS>*)o1;
  const BVHModel<RSS>* obj2 = (BVHModel<RSS>*)o2;
  MESHMESHOBBRSS_COMMON_CODE();
}

int KDOP16KDOP16Collide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshCollisionTraversalNode<KDOP<16> > node;
  const BVHModel<KDOP<16> >* obj1 = (BVHModel<KDOP<16> >*)o1;
  const BVHModel<KDOP<16> >* obj2 = (BVHModel<KDOP<16> >*)o2;
  BVHModel<KDOP<16> >* obj1_tmp = new BVHModel<KDOP<16> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  BVHModel<KDOP<16> >* obj2_tmp = new BVHModel<KDOP<16> >(*obj2);
  SimpleTransform tf2_tmp = tf2;
  MESHMESH_COMMON_CODE();
}

int KDOP18KDOP18Collide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshCollisionTraversalNode<KDOP<18> > node;
  const BVHModel<KDOP<18> >* obj1 = (BVHModel<KDOP<18> >*)o1;
  const BVHModel<KDOP<18> >* obj2 = (BVHModel<KDOP<18> >*)o2;
  BVHModel<KDOP<18> >* obj1_tmp = new BVHModel<KDOP<18> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  BVHModel<KDOP<18> >* obj2_tmp = new BVHModel<KDOP<18> >(*obj2);
  SimpleTransform tf2_tmp = tf2;
  MESHMESH_COMMON_CODE();
}

int KDOP24KDOP24Collide(const CollisionGeometry* o1, const SimpleTransform& tf1, const CollisionGeometry* o2, const SimpleTransform& tf2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshCollisionTraversalNode<KDOP<24> > node;
  const BVHModel<KDOP<24> >* obj1 = (BVHModel<KDOP<24> >*)o1;
  const BVHModel<KDOP<24> >* obj2 = (BVHModel<KDOP<24> >*)o2;
  BVHModel<KDOP<24> >* obj1_tmp = new BVHModel<KDOP<24> >(*obj1);
  SimpleTransform tf1_tmp = tf1;
  BVHModel<KDOP<24> >* obj2_tmp = new BVHModel<KDOP<24> >(*obj2);
  SimpleTransform tf2_tmp = tf2;
  MESHMESH_COMMON_CODE();
}


CollisionFunctionMatrix::CollisionFunctionMatrix()
{
  for(int i = 0; i < 14; ++i)
  {
    for(int j = 0; j < 14; ++j)
      collision_matrix[i][j] = NULL;
  }

  collision_matrix[GEOM_BOX][GEOM_BOX] = BoxBoxCollide;
  collision_matrix[GEOM_BOX][GEOM_SPHERE] = BoxSphereCollide;
  collision_matrix[GEOM_BOX][GEOM_CAPSULE] = BoxCapCollide;
  collision_matrix[GEOM_BOX][GEOM_CONE] = BoxConeCollide;
  collision_matrix[GEOM_BOX][GEOM_CYLINDER] = BoxCylinderCollide;
  collision_matrix[GEOM_BOX][GEOM_CONVEX] = BoxConvexCollide;
  collision_matrix[GEOM_BOX][GEOM_PLANE] = BoxPlaneCollide;

  collision_matrix[GEOM_SPHERE][GEOM_BOX] = SphereBoxCollide;
  collision_matrix[GEOM_SPHERE][GEOM_SPHERE] = SphereSphereCollide;
  collision_matrix[GEOM_SPHERE][GEOM_CAPSULE] = SphereCapCollide;
  collision_matrix[GEOM_SPHERE][GEOM_CONE] = SphereConeCollide;
  collision_matrix[GEOM_SPHERE][GEOM_CYLINDER] = SphereCylinderCollide;
  collision_matrix[GEOM_SPHERE][GEOM_CONVEX] = SphereConvexCollide;
  collision_matrix[GEOM_SPHERE][GEOM_PLANE] = SpherePlaneCollide;

  collision_matrix[GEOM_CAPSULE][GEOM_BOX] = CapBoxCollide;
  collision_matrix[GEOM_CAPSULE][GEOM_SPHERE] = CapSphereCollide;
  collision_matrix[GEOM_CAPSULE][GEOM_CAPSULE] = CapCapCollide;
  collision_matrix[GEOM_CAPSULE][GEOM_CONE] = CapConeCollide;
  collision_matrix[GEOM_CAPSULE][GEOM_CYLINDER] = CapCylinderCollide;
  collision_matrix[GEOM_CAPSULE][GEOM_CONVEX] = CapConvexCollide;
  collision_matrix[GEOM_CAPSULE][GEOM_PLANE] = CapPlaneCollide;

  collision_matrix[GEOM_CONE][GEOM_BOX] = ConeBoxCollide;
  collision_matrix[GEOM_CONE][GEOM_SPHERE] = ConeSphereCollide;
  collision_matrix[GEOM_CONE][GEOM_CAPSULE] = ConeCapCollide;
  collision_matrix[GEOM_CONE][GEOM_CONE] = ConeConeCollide;
  collision_matrix[GEOM_CONE][GEOM_CYLINDER] = ConeCylinderCollide;
  collision_matrix[GEOM_CONE][GEOM_CONVEX] = ConeConvexCollide;
  collision_matrix[GEOM_CONE][GEOM_PLANE] = ConePlaneCollide;

  collision_matrix[GEOM_CYLINDER][GEOM_BOX] = CylinderBoxCollide;
  collision_matrix[GEOM_CYLINDER][GEOM_SPHERE] = CylinderSphereCollide;
  collision_matrix[GEOM_CYLINDER][GEOM_CAPSULE] = CylinderCapCollide;
  collision_matrix[GEOM_CYLINDER][GEOM_CONE] = CylinderConeCollide;
  collision_matrix[GEOM_CYLINDER][GEOM_CYLINDER] = CylinderCylinderCollide;
  collision_matrix[GEOM_CYLINDER][GEOM_CONVEX] = CylinderConvexCollide;
  collision_matrix[GEOM_CYLINDER][GEOM_PLANE] = CylinderPlaneCollide;

  collision_matrix[GEOM_CONVEX][GEOM_BOX] = ConvexBoxCollide;
  collision_matrix[GEOM_CONVEX][GEOM_SPHERE] = ConvexSphereCollide;
  collision_matrix[GEOM_CONVEX][GEOM_CAPSULE] = ConvexCapCollide;
  collision_matrix[GEOM_CONVEX][GEOM_CONE] = ConvexConeCollide;
  collision_matrix[GEOM_CONVEX][GEOM_CYLINDER] = ConvexCylinderCollide;
  collision_matrix[GEOM_CONVEX][GEOM_CONVEX] = ConvexConvexCollide;
  collision_matrix[GEOM_CONVEX][GEOM_PLANE] = ConvexPlaneCollide;

  collision_matrix[GEOM_PLANE][GEOM_BOX] = PlaneBoxCollide;
  collision_matrix[GEOM_PLANE][GEOM_SPHERE] = PlaneSphereCollide;
  collision_matrix[GEOM_PLANE][GEOM_CAPSULE] = PlaneCapCollide;
  collision_matrix[GEOM_PLANE][GEOM_CONE] = PlaneConeCollide;
  collision_matrix[GEOM_PLANE][GEOM_CYLINDER] = PlaneCylinderCollide;
  collision_matrix[GEOM_PLANE][GEOM_CONVEX] = PlaneConvexCollide;
  collision_matrix[GEOM_PLANE][GEOM_PLANE] = PlanePlaneCollide;

  collision_matrix[BV_AABB][GEOM_BOX] = AABBBoxCollide;
  collision_matrix[BV_AABB][GEOM_SPHERE] = AABBSphereCollide;
  collision_matrix[BV_AABB][GEOM_CAPSULE] = AABBCapCollide;
  collision_matrix[BV_AABB][GEOM_CONE] = AABBConeCollide;
  collision_matrix[BV_AABB][GEOM_CYLINDER] = AABBCylinderCollide;
  collision_matrix[BV_AABB][GEOM_CONVEX] = AABBConvexCollide;
  collision_matrix[BV_AABB][GEOM_PLANE] = AABBPlaneCollide;

  collision_matrix[BV_OBB][GEOM_BOX] = OBBBoxCollide;
  collision_matrix[BV_OBB][GEOM_SPHERE] = OBBSphereCollide;
  collision_matrix[BV_OBB][GEOM_CAPSULE] = OBBCapCollide;
  collision_matrix[BV_OBB][GEOM_CONE] = OBBConeCollide;
  collision_matrix[BV_OBB][GEOM_CYLINDER] = OBBCylinderCollide;
  collision_matrix[BV_OBB][GEOM_CONVEX] = OBBConvexCollide;
  collision_matrix[BV_OBB][GEOM_PLANE] = OBBPlaneCollide;

  collision_matrix[BV_RSS][GEOM_BOX] = RSSBoxCollide;
  collision_matrix[BV_RSS][GEOM_SPHERE] = RSSSphereCollide;
  collision_matrix[BV_RSS][GEOM_CAPSULE] = RSSCapCollide;
  collision_matrix[BV_RSS][GEOM_CONE] = RSSConeCollide;
  collision_matrix[BV_RSS][GEOM_CYLINDER] = RSSCylinderCollide;
  collision_matrix[BV_RSS][GEOM_CONVEX] = RSSConvexCollide;
  collision_matrix[BV_RSS][GEOM_PLANE] = RSSPlaneCollide;

  collision_matrix[BV_KDOP16][GEOM_BOX] = KDOP16BoxCollide;
  collision_matrix[BV_KDOP16][GEOM_SPHERE] = KDOP16SphereCollide;
  collision_matrix[BV_KDOP16][GEOM_CAPSULE] = KDOP16CapCollide;
  collision_matrix[BV_KDOP16][GEOM_CONE] = KDOP16ConeCollide;
  collision_matrix[BV_KDOP16][GEOM_CYLINDER] = KDOP16CylinderCollide;
  collision_matrix[BV_KDOP16][GEOM_CONVEX] = KDOP16ConvexCollide;
  collision_matrix[BV_KDOP16][GEOM_PLANE] = KDOP16PlaneCollide;

  collision_matrix[BV_KDOP18][GEOM_BOX] = KDOP18BoxCollide;
  collision_matrix[BV_KDOP18][GEOM_SPHERE] = KDOP18SphereCollide;
  collision_matrix[BV_KDOP18][GEOM_CAPSULE] = KDOP18CapCollide;
  collision_matrix[BV_KDOP18][GEOM_CONE] = KDOP18ConeCollide;
  collision_matrix[BV_KDOP18][GEOM_CYLINDER] = KDOP18CylinderCollide;
  collision_matrix[BV_KDOP18][GEOM_CONVEX] = KDOP18ConvexCollide;
  collision_matrix[BV_KDOP18][GEOM_PLANE] = KDOP18PlaneCollide;

  collision_matrix[BV_KDOP24][GEOM_BOX] = KDOP24BoxCollide;
  collision_matrix[BV_KDOP24][GEOM_SPHERE] = KDOP24SphereCollide;
  collision_matrix[BV_KDOP24][GEOM_CAPSULE] = KDOP24CapCollide;
  collision_matrix[BV_KDOP24][GEOM_CONE] = KDOP24ConeCollide;
  collision_matrix[BV_KDOP24][GEOM_CYLINDER] = KDOP24CylinderCollide;
  collision_matrix[BV_KDOP24][GEOM_CONVEX] = KDOP24ConvexCollide;
  collision_matrix[BV_KDOP24][GEOM_PLANE] = KDOP24PlaneCollide;

  /*
  collision_matrix[GEOM_BOX][BV_AABB] = BoxAABBCollide;
  collision_matrix[GEOM_SPHERE][BV_AABB] = SphereAABBCollide;
  collision_matrix[GEOM_CAPSULE][BV_AABB] = CapAABBCollide;
  collision_matrix[GEOM_CONE][BV_AABB] = ConeAABBCollide;
  collision_matrix[GEOM_CYLINDER][BV_AABB] = CylinderAABBCollide;
  collision_matrix[GEOM_CONVEX][BV_AABB] = ConvexAABBCollide;
  collision_matrix[GEOM_PLANE][BV_AABB] = PlaneAABBCollide;

  collision_matrix[GEOM_BOX][BV_OBB] = BoxOBBCollide;
  collision_matrix[GEOM_SPHERE][BV_OBB] = SphereOBBCollide;
  collision_matrix[GEOM_CAPSULE][BV_OBB] = CapOBBCollide;
  collision_matrix[GEOM_CONE][BV_OBB] = ConeOBBCollide;
  collision_matrix[GEOM_CYLINDER][BV_OBB] = CylinderOBBCollide;
  collision_matrix[GEOM_CONVEX][BV_OBB] = ConvexOBBCollide;
  collision_matrix[GEOM_PLANE][BV_OBB] = PlaneOBBCollide;

  collision_matrix[GEOM_BOX][BV_RSS] = BoxRSSCollide;
  collision_matrix[GEOM_SPHERE][BV_RSS] = SphereRSSCollide;
  collision_matrix[GEOM_CAPSULE][BV_RSS] = CapRSSCollide;
  collision_matrix[GEOM_CONE][BV_RSS] = ConeRSSCollide;
  collision_matrix[GEOM_CYLINDER][BV_RSS] = CylinderRSSCollide;
  collision_matrix[GEOM_CONVEX][BV_RSS] = ConvexRSSCollide;
  collision_matrix[GEOM_PLANE][BV_RSS] = PlaneRSSCollide;

  collision_matrix[GEOM_BOX][BV_KDOP16] = BoxKDOP16Collide;
  collision_matrix[GEOM_SPHERE][BV_KDOP16] = SphereKDOP16Collide;
  collision_matrix[GEOM_CAPSULE][BV_KDOP16] = CapKDOP16Collide;
  collision_matrix[GEOM_CONE][BV_KDOP16] = ConeKDOP16Collide;
  collision_matrix[GEOM_CYLINDER][BV_KDOP16] = CylinderKDOP16Collide;
  collision_matrix[GEOM_CONVEX][BV_KDOP16] = ConvexKDOP16Collide;
  collision_matrix[GEOM_PLANE][BV_KDOP16] = PlaneKDOP16Collide;

  collision_matrix[GEOM_BOX][BV_KDOP18] = BoxKDOP18Collide;
  collision_matrix[GEOM_SPHERE][BV_KDOP18] = SphereKDOP18Collide;
  collision_matrix[GEOM_CAPSULE][BV_KDOP18] = CapKDOP18Collide;
  collision_matrix[GEOM_CONE][BV_KDOP18] = ConeKDOP18Collide;
  collision_matrix[GEOM_CYLINDER][BV_KDOP18] = CylinderKDOP18Collide;
  collision_matrix[GEOM_CONVEX][BV_KDOP18] = ConvexKDOP18Collide;
  collision_matrix[GEOM_PLANE][BV_KDOP18] = PlaneKDOP18Collide;

  collision_matrix[GEOM_BOX][BV_KDOP24] = BoxKDOP24Collide;
  collision_matrix[GEOM_SPHERE][BV_KDOP24] = SphereKDOP24Collide;
  collision_matrix[GEOM_CAPSULE][BV_KDOP24] = CapKDOP24Collide;
  collision_matrix[GEOM_CONE][BV_KDOP24] = ConeKDOP24Collide;
  collision_matrix[GEOM_CYLINDER][BV_KDOP24] = CylinderKDOP24Collide;
  collision_matrix[GEOM_CONVEX][BV_KDOP24] = ConvexKDOP24Collide;
  collision_matrix[GEOM_PLANE][BV_KDOP24] = PlaneKDOP24Collide;
  */

  collision_matrix[BV_AABB][BV_AABB] = AABBAABBCollide;
  collision_matrix[BV_OBB][BV_OBB] = OBBOBBCollide;
  collision_matrix[BV_RSS][BV_RSS] = RSSRSSCollide;
  collision_matrix[BV_KDOP16][BV_KDOP16] = KDOP16KDOP16Collide;
  collision_matrix[BV_KDOP18][BV_KDOP18] = KDOP18KDOP18Collide;
  collision_matrix[BV_KDOP24][BV_KDOP24] = KDOP24KDOP24Collide;
}

}
