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
                                     initialize(node, *obj1, *obj2, enable_contact); \
                                     collide(&node); \
                                     if(!node.is_collision) return 0; \
                                     contacts.resize(1); \
                                     if(!enable_contact) contacts[0] = Contact(o1, o2, 0, 0); \
                                     else contacts[0] = Contact(o1, o2, 0, 0, node.contact_point, node.normal, node.penetration_depth); \
                                     return 1; \
                                     } while(0)


#define MESHSHAPE_COMMON_CODE() do{ \
                                    initialize(node, *obj1_tmp, *obj2, num_max_contacts, exhaustive, enable_contact); \
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
                                    initialize(node, *obj1, *obj2_tmp, num_max_contacts, exhaustive, enable_contact); \
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
                                    initialize(node, *obj1, *obj2, num_max_contacts, exhaustive, enable_contact); \
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
                                    initialize(node, *obj1_tmp, *obj2_tmp, num_max_contacts, exhaustive, enable_contact); \
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
                                         initialize(node, *obj1, *obj2, num_max_contacts, exhaustive, enable_contact); \
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
                                               Vec3f normal = matMulVec(obj1->getRotation(), node.pairs[i].normal); \
                                               Vec3f contact_point = matMulVec(obj1->getRotation(), node.pairs[i].contact_point) + obj1->getTranslation(); \
                                               contacts[i] = Contact(obj1, obj2, node.pairs[i].id1, node.pairs[i].id2, contact_point, normal, node.pairs[i].penetration_depth); \
                                             } \
                                           } \
                                         } \
                                         return num_contacts; \
                                       } while(0)



int BoxBoxCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Box, Box> node;
  const Box* obj1 = (Box*)o1;
  const Box* obj2 = (Box*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int BoxSphereCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Box, Sphere> node;
  const Box* obj1 = (Box*)o1;
  const Sphere* obj2 = (Sphere*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int BoxCapCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Box, Capsule> node;
  const Box* obj1 = (Box*)o1;
  const Capsule* obj2 = (Capsule*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int BoxConeCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Box, Cone> node;
  const Box* obj1 = (Box*)o1;
  const Cone* obj2 = (Cone*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int BoxCylinderCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Box, Cylinder> node;
  const Box* obj1 = (Box*)o1;
  const Cylinder* obj2 = (Cylinder*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int BoxConvexCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Box, Convex> node;
  const Box* obj1 = (Box*)o1;
  const Convex* obj2 = (Convex*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int BoxPlaneCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Box, Plane> node;
  const Box* obj1 = (Box*)o1;
  const Plane* obj2 = (Plane*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int SphereBoxCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Sphere, Box> node;
  const Sphere* obj1 = (Sphere*)o1;
  const Box* obj2 = (Box*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int SphereSphereCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Sphere, Sphere> node;
  const Sphere* obj1 = (Sphere*)o1;
  const Sphere* obj2 = (Sphere*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int SphereCapCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Sphere, Capsule> node;
  const Sphere* obj1 = (Sphere*)o1;
  const Capsule* obj2 = (Capsule*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int SphereConeCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Sphere, Cone> node;
  const Sphere* obj1 = (Sphere*)o1;
  const Cone* obj2 = (Cone*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int SphereCylinderCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Sphere, Cylinder> node;
  const Sphere* obj1 = (Sphere*)o1;
  const Cylinder* obj2 = (Cylinder*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int SphereConvexCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Sphere, Convex> node;
  const Sphere* obj1 = (Sphere*)o1;
  const Convex* obj2 = (Convex*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int SpherePlaneCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Sphere, Plane> node;
  const Sphere* obj1 = (Sphere*)o1;
  const Plane* obj2 = (Plane*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CapBoxCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Capsule, Box> node;
  const Capsule* obj1 = (Capsule*)o1;
  const Box* obj2 = (Box*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CapSphereCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Capsule, Sphere> node;
  const Capsule* obj1 = (Capsule*)o1;
  const Sphere* obj2 = (Sphere*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CapCapCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Capsule, Capsule> node;
  const Capsule* obj1 = (Capsule*)o1;
  const Capsule* obj2 = (Capsule*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CapConeCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Capsule, Cone> node;
  const Capsule* obj1 = (Capsule*)o1;
  const Cone* obj2 = (Cone*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CapCylinderCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Capsule, Cylinder> node;
  const Capsule* obj1 = (Capsule*)o1;
  const Cylinder* obj2 = (Cylinder*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CapConvexCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Capsule, Convex> node;
  const Capsule* obj1 = (Capsule*)o1;
  const Convex* obj2 = (Convex*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CapPlaneCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Capsule, Plane> node;
  const Capsule* obj1 = (Capsule*)o1;
  const Plane* obj2 = (Plane*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConeBoxCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cone, Box> node;
  const Cone* obj1 = (Cone*)o1;
  const Box* obj2 = (Box*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConeSphereCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cone, Sphere> node;
  const Cone* obj1 = (Cone*)o1;
  const Sphere* obj2 = (Sphere*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConeCapCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cone, Capsule> node;
  const Cone* obj1 = (Cone*)o1;
  const Capsule* obj2 = (Capsule*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConeConeCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cone, Cone> node;
  const Cone* obj1 = (Cone*)o1;
  const Cone* obj2 = (Cone*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConeCylinderCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cone, Cylinder> node;
  const Cone* obj1 = (Cone*)o1;
  const Cylinder* obj2 = (Cylinder*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConeConvexCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cone, Convex> node;
  const Cone* obj1 = (Cone*)o1;
  const Convex* obj2 = (Convex*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConePlaneCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cone, Plane> node;
  const Cone* obj1 = (Cone*)o1;
  const Plane* obj2 = (Plane*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CylinderBoxCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cylinder, Box> node;
  const Cylinder* obj1 = (Cylinder*)o1;
  const Box* obj2 = (Box*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CylinderSphereCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cylinder, Sphere> node;
  const Cylinder* obj1 = (Cylinder*)o1;
  const Sphere* obj2 = (Sphere*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CylinderCapCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cylinder, Capsule> node;
  const Cylinder* obj1 = (Cylinder*)o1;
  const Capsule* obj2 = (Capsule*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CylinderConeCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cylinder, Cone> node;
  const Cylinder* obj1 = (Cylinder*)o1;
  const Cone* obj2 = (Cone*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CylinderCylinderCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cylinder, Cylinder> node;
  const Cylinder* obj1 = (Cylinder*)o1;
  const Cylinder* obj2 = (Cylinder*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CylinderConvexCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cylinder, Convex> node;
  const Cylinder* obj1 = (Cylinder*)o1;
  const Convex* obj2 = (Convex*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int CylinderPlaneCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Cylinder, Plane> node;
  const Cylinder* obj1 = (Cylinder*)o1;
  const Plane* obj2 = (Plane*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConvexBoxCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Convex, Box> node;
  const Convex* obj1 = (Convex*)o1;
  const Box* obj2 = (Box*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConvexSphereCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Convex, Sphere> node;
  const Convex* obj1 = (Convex*)o1;
  const Sphere* obj2 = (Sphere*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConvexCapCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Convex, Capsule> node;
  const Convex* obj1 = (Convex*)o1;
  const Capsule* obj2 = (Capsule*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConvexConeCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Convex, Cone> node;
  const Convex* obj1 = (Convex*)o1;
  const Cone* obj2 = (Cone*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConvexCylinderCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Convex, Cylinder> node;
  const Convex* obj1 = (Convex*)o1;
  const Cylinder* obj2 = (Cylinder*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConvexConvexCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Convex, Convex> node;
  const Convex* obj1 = (Convex*)o1;
  const Convex* obj2 = (Convex*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int ConvexPlaneCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Convex, Plane> node;
  const Convex* obj1 = (Convex*)o1;
  const Plane* obj2 = (Plane*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int PlaneBoxCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Plane, Box> node;
  const Plane* obj1 = (Plane*)o1;
  const Box* obj2 = (Box*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int PlaneSphereCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Plane, Sphere> node;
  const Plane* obj1 = (Plane*)o1;
  const Sphere* obj2 = (Sphere*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int PlaneCapCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Plane, Capsule> node;
  const Plane* obj1 = (Plane*)o1;
  const Capsule* obj2 = (Capsule*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int PlaneConeCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Plane, Cone> node;
  const Plane* obj1 = (Plane*)o1;
  const Cone* obj2 = (Cone*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int PlaneCylinderCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Plane, Cylinder> node;
  const Plane* obj1 = (Plane*)o1;
  const Cylinder* obj2 = (Cylinder*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int PlaneConvexCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Plane, Convex> node;
  const Plane* obj1 = (Plane*)o1;
  const Convex* obj2 = (Convex*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int PlanePlaneCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeCollisionTraversalNode<Plane, Plane> node;
  const Plane* obj1 = (Plane*)o1;
  const Plane* obj2 = (Plane*)o2;
  SHAPESHAPE_COMMON_CODE();
}

int AABBBoxCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<AABB, Box> node;
  const BVHModel<AABB>* obj1 = (BVHModel<AABB>*)o1;
  BVHModel<AABB>* obj1_tmp = new BVHModel<AABB>(*obj1);
  const Box* obj2 = (Box*)o2;
  MESHSHAPE_COMMON_CODE();
}

int AABBSphereCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<AABB, Sphere> node;
  const BVHModel<AABB>* obj1 = (BVHModel<AABB>*)o1;
  BVHModel<AABB>* obj1_tmp = new BVHModel<AABB>(*obj1);
  const Sphere* obj2 = (Sphere*)o2;
  MESHSHAPE_COMMON_CODE();
}

int AABBCapCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<AABB, Capsule> node;
  const BVHModel<AABB>* obj1 = (BVHModel<AABB>*)o1;
  BVHModel<AABB>* obj1_tmp = new BVHModel<AABB>(*obj1);
  const Capsule* obj2 = (Capsule*)o2;
  MESHSHAPE_COMMON_CODE();
}

int AABBConeCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<AABB, Cone> node;
  const BVHModel<AABB>* obj1 = (BVHModel<AABB>*)o1;
  BVHModel<AABB>* obj1_tmp = new BVHModel<AABB>(*obj1);
  const Cone* obj2 = (Cone*)o2;
  MESHSHAPE_COMMON_CODE();
}

int AABBCylinderCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<AABB, Cylinder> node;
  const BVHModel<AABB>* obj1 = (BVHModel<AABB>*)o1;
  BVHModel<AABB>* obj1_tmp = new BVHModel<AABB>(*obj1);
  const Cylinder* obj2 = (Cylinder*)o2;
  MESHSHAPE_COMMON_CODE();
}

int AABBConvexCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<AABB, Convex> node;
  const BVHModel<AABB>* obj1 = (BVHModel<AABB>*)o1;
  BVHModel<AABB>* obj1_tmp = new BVHModel<AABB>(*obj1);
  const Convex* obj2 = (Convex*)o2;
  MESHSHAPE_COMMON_CODE();
}

int AABBPlaneCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<AABB, Plane> node;
  const BVHModel<AABB>* obj1 = (BVHModel<AABB>*)o1;
  BVHModel<AABB>* obj1_tmp = new BVHModel<AABB>(*obj1);
  const Plane* obj2 = (Plane*)o2;
  MESHSHAPE_COMMON_CODE();
}

int OBBBoxCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNodeOBB<Box> node;
  const BVHModel<OBB>* obj1 = (BVHModel<OBB>*)o1;
  const Box* obj2 = (Box*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int OBBSphereCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNodeOBB<Sphere> node;
  const BVHModel<OBB>* obj1 = (BVHModel<OBB>*)o1;
  const Sphere* obj2 = (Sphere*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int OBBCapCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNodeOBB<Capsule> node;
  const BVHModel<OBB>* obj1 = (BVHModel<OBB>*)o1;
  const Capsule* obj2 = (Capsule*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int OBBConeCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNodeOBB<Cone> node;
  const BVHModel<OBB>* obj1 = (BVHModel<OBB>*)o1;
  const Cone* obj2 = (Cone*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int OBBCylinderCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNodeOBB<Cylinder> node;
  const BVHModel<OBB>* obj1 = (BVHModel<OBB>*)o1;
  const Cylinder* obj2 = (Cylinder*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int OBBConvexCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNodeOBB<Convex> node;
  const BVHModel<OBB>* obj1 = (BVHModel<OBB>*)o1;
  const Convex* obj2 = (Convex*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int OBBPlaneCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNodeOBB<Plane> node;
  const BVHModel<OBB>* obj1 = (BVHModel<OBB>*)o1;
  const Plane* obj2 = (Plane*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int RSSBoxCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<RSS, Box> node;
  const BVHModel<RSS>* obj1 = (BVHModel<RSS>*)o1;
  BVHModel<RSS>* obj1_tmp = new BVHModel<RSS>(*obj1);
  const Box* obj2 = (Box*)o2;
  MESHSHAPE_COMMON_CODE();
}

int RSSSphereCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<RSS, Sphere> node;
  const BVHModel<RSS>* obj1 = (BVHModel<RSS>*)o1;
  BVHModel<RSS>* obj1_tmp = new BVHModel<RSS>(*obj1);
  const Sphere* obj2 = (Sphere*)o2;
  MESHSHAPE_COMMON_CODE();
}

int RSSCapCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<RSS, Capsule> node;
  const BVHModel<RSS>* obj1 = (BVHModel<RSS>*)o1;
  BVHModel<RSS>* obj1_tmp = new BVHModel<RSS>(*obj1);
  const Capsule* obj2 = (Capsule*)o2;
  MESHSHAPE_COMMON_CODE();
}

int RSSConeCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<RSS, Cone> node;
  const BVHModel<RSS>* obj1 = (BVHModel<RSS>*)o1;
  BVHModel<RSS>* obj1_tmp = new BVHModel<RSS>(*obj1);
  const Cone* obj2 = (Cone*)o2;
  MESHSHAPE_COMMON_CODE();
}

int RSSCylinderCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<RSS, Cylinder> node;
  const BVHModel<RSS>* obj1 = (BVHModel<RSS>*)o1;
  BVHModel<RSS>* obj1_tmp = new BVHModel<RSS>(*obj1);
  const Cylinder* obj2 = (Cylinder*)o2;
  MESHSHAPE_COMMON_CODE();
}

int RSSConvexCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<RSS, Convex> node;
  const BVHModel<RSS>* obj1 = (BVHModel<RSS>*)o1;
  BVHModel<RSS>* obj1_tmp = new BVHModel<RSS>(*obj1);
  const Convex* obj2 = (Convex*)o2;
  MESHSHAPE_COMMON_CODE();
}

int RSSPlaneCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<RSS, Plane> node;
  const BVHModel<RSS>* obj1 = (BVHModel<RSS>*)o1;
  BVHModel<RSS>* obj1_tmp = new BVHModel<RSS>(*obj1);
  const Plane* obj2 = (Plane*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP16BoxCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<16> , Box> node;
  const BVHModel<KDOP<16> >* obj1 = (BVHModel<KDOP<16> >*)o1;
  BVHModel<KDOP<16> >* obj1_tmp = new BVHModel<KDOP<16> >(*obj1);
  const Box* obj2 = (Box*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP16SphereCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<16> , Sphere> node;
  const BVHModel<KDOP<16> >* obj1 = (BVHModel<KDOP<16> >*)o1;
  BVHModel<KDOP<16> >* obj1_tmp = new BVHModel<KDOP<16> >(*obj1);
  const Sphere* obj2 = (Sphere*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP16CapCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<16> , Capsule> node;
  const BVHModel<KDOP<16> >* obj1 = (BVHModel<KDOP<16> >*)o1;
  BVHModel<KDOP<16> >* obj1_tmp = new BVHModel<KDOP<16> >(*obj1);
  const Capsule* obj2 = (Capsule*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP16ConeCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<16> , Cone> node;
  const BVHModel<KDOP<16> >* obj1 = (BVHModel<KDOP<16> >*)o1;
  BVHModel<KDOP<16> >* obj1_tmp = new BVHModel<KDOP<16> >(*obj1);
  const Cone* obj2 = (Cone*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP16CylinderCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<16> , Cylinder> node;
  const BVHModel<KDOP<16> >* obj1 = (BVHModel<KDOP<16> >*)o1;
  BVHModel<KDOP<16> >* obj1_tmp = new BVHModel<KDOP<16> >(*obj1);
  const Cylinder* obj2 = (Cylinder*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP16ConvexCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<16> , Convex> node;
  const BVHModel<KDOP<16> >* obj1 = (BVHModel<KDOP<16> >*)o1;
  BVHModel<KDOP<16> >* obj1_tmp = new BVHModel<KDOP<16> >(*obj1);
  const Convex* obj2 = (Convex*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP16PlaneCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<16> , Plane> node;
  const BVHModel<KDOP<16> >* obj1 = (BVHModel<KDOP<16> >*)o1;
  BVHModel<KDOP<16> >* obj1_tmp = new BVHModel<KDOP<16> >(*obj1);
  const Plane* obj2 = (Plane*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP18BoxCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<18> , Box> node;
  const BVHModel<KDOP<18> >* obj1 = (BVHModel<KDOP<18> >*)o1;
  BVHModel<KDOP<18> >* obj1_tmp = new BVHModel<KDOP<18> >(*obj1);
  const Box* obj2 = (Box*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP18SphereCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<18> , Sphere> node;
  const BVHModel<KDOP<18> >* obj1 = (BVHModel<KDOP<18> >*)o1;
  BVHModel<KDOP<18> >* obj1_tmp = new BVHModel<KDOP<18> >(*obj1);
  const Sphere* obj2 = (Sphere*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP18CapCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<18> , Capsule> node;
  const BVHModel<KDOP<18> >* obj1 = (BVHModel<KDOP<18> >*)o1;
  BVHModel<KDOP<18> >* obj1_tmp = new BVHModel<KDOP<18> >(*obj1);
  const Capsule* obj2 = (Capsule*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP18ConeCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<18> , Cone> node;
  const BVHModel<KDOP<18> >* obj1 = (BVHModel<KDOP<18> >*)o1;
  BVHModel<KDOP<18> >* obj1_tmp = new BVHModel<KDOP<18> >(*obj1);
  const Cone* obj2 = (Cone*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP18CylinderCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<18> , Cylinder> node;
  const BVHModel<KDOP<18> >* obj1 = (BVHModel<KDOP<18> >*)o1;
  BVHModel<KDOP<18> >* obj1_tmp = new BVHModel<KDOP<18> >(*obj1);
  const Cylinder* obj2 = (Cylinder*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP18ConvexCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<18> , Convex> node;
  const BVHModel<KDOP<18> >* obj1 = (BVHModel<KDOP<18> >*)o1;
  BVHModel<KDOP<18> >* obj1_tmp = new BVHModel<KDOP<18> >(*obj1);
  const Convex* obj2 = (Convex*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP18PlaneCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<18> , Plane> node;
  const BVHModel<KDOP<18> >* obj1 = (BVHModel<KDOP<18> >*)o1;
  BVHModel<KDOP<18> >* obj1_tmp = new BVHModel<KDOP<18> >(*obj1);
  const Plane* obj2 = (Plane*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP24BoxCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<24> , Box> node;
  const BVHModel<KDOP<24> >* obj1 = (BVHModel<KDOP<24> >*)o1;
  BVHModel<KDOP<24> >* obj1_tmp = new BVHModel<KDOP<24> >(*obj1);
  const Box* obj2 = (Box*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP24SphereCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<24> , Sphere> node;
  const BVHModel<KDOP<24> >* obj1 = (BVHModel<KDOP<24> >*)o1;
  BVHModel<KDOP<24> >* obj1_tmp = new BVHModel<KDOP<24> >(*obj1);
  const Sphere* obj2 = (Sphere*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP24CapCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<24> , Capsule> node;
  const BVHModel<KDOP<24> >* obj1 = (BVHModel<KDOP<24> >*)o1;
  BVHModel<KDOP<24> >* obj1_tmp = new BVHModel<KDOP<24> >(*obj1);
  const Capsule* obj2 = (Capsule*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP24ConeCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<24> , Cone> node;
  const BVHModel<KDOP<24> >* obj1 = (BVHModel<KDOP<24> >*)o1;
  BVHModel<KDOP<24> >* obj1_tmp = new BVHModel<KDOP<24> >(*obj1);
  const Cone* obj2 = (Cone*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP24CylinderCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<24> , Cylinder> node;
  const BVHModel<KDOP<24> >* obj1 = (BVHModel<KDOP<24> >*)o1;
  BVHModel<KDOP<24> >* obj1_tmp = new BVHModel<KDOP<24> >(*obj1);
  const Cylinder* obj2 = (Cylinder*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP24ConvexCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<24> , Convex> node;
  const BVHModel<KDOP<24> >* obj1 = (BVHModel<KDOP<24> >*)o1;
  BVHModel<KDOP<24> >* obj1_tmp = new BVHModel<KDOP<24> >(*obj1);
  const Convex* obj2 = (Convex*)o2;
  MESHSHAPE_COMMON_CODE();
}

int KDOP24PlaneCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshShapeCollisionTraversalNode<KDOP<24> , Plane> node;
  const BVHModel<KDOP<24> >* obj1 = (BVHModel<KDOP<24> >*)o1;
  BVHModel<KDOP<24> >* obj1_tmp = new BVHModel<KDOP<24> >(*obj1);
  const Plane* obj2 = (Plane*)o2;
  MESHSHAPE_COMMON_CODE();
}

// use MESH SHAPE
/*
int BoxAABBCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Box, AABB> node;
  const Box* obj1 = (Box*)o1;
  const BVHModel<AABB>* obj2 = (BVHModel<AABB>*)o2;
  BVHModel<AABB>* obj2_tmp = new BVHModel<AABB>(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int SphereAABBCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Sphere, AABB> node;
  const Sphere* obj1 = (Sphere*)o1;
  const BVHModel<AABB>* obj2 = (BVHModel<AABB>*)o2;
  BVHModel<AABB>* obj2_tmp = new BVHModel<AABB>(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int CapAABBCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Capsule, AABB> node;
  const Capsule* obj1 = (Capsule*)o1;
  const BVHModel<AABB>* obj2 = (BVHModel<AABB>*)o2;
  BVHModel<AABB>* obj2_tmp = new BVHModel<AABB>(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int ConeAABBCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Cone, AABB> node;
  const Cone* obj1 = (Cone*)o1;
  const BVHModel<AABB>* obj2 = (BVHModel<AABB>*)o2;
  BVHModel<AABB>* obj2_tmp = new BVHModel<AABB>(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int CylinderAABBCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Cylinder, AABB> node;
  const Cylinder* obj1 = (Cylinder*)o1;
  const BVHModel<AABB>* obj2 = (BVHModel<AABB>*)o2;
  BVHModel<AABB>* obj2_tmp = new BVHModel<AABB>(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int ConvexAABBCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Convex, AABB> node;
  const Convex* obj1 = (Convex*)o1;
  const BVHModel<AABB>* obj2 = (BVHModel<AABB>*)o2;
  BVHModel<AABB>* obj2_tmp = new BVHModel<AABB>(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int PlaneAABBCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Plane, AABB> node;
  const Plane* obj1 = (Plane*)o1;
  const BVHModel<AABB>* obj2 = (BVHModel<AABB>*)o2;
  BVHModel<AABB>* obj2_tmp = new BVHModel<AABB>(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int BoxOBBCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNodeOBB<Box> node;
  const Box* obj1 = (Box*)o1;
  const BVHModel<OBB>* obj2 = (BVHModel<OBB>*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int SphereOBBCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNodeOBB<Sphere> node;
  const Sphere* obj1 = (Sphere*)o1;
  const BVHModel<OBB>* obj2 = (BVHModel<OBB>*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int CapOBBCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNodeOBB<Capsule> node;
  const Capsule* obj1 = (Capsule*)o1;
  const BVHModel<OBB>* obj2 = (BVHModel<OBB>*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int ConeOBBCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNodeOBB<Cone> node;
  const Cone* obj1 = (Cone*)o1;
  const BVHModel<OBB>* obj2 = (BVHModel<OBB>*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int CylinderOBBCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNodeOBB<Cylinder> node;
  const Cylinder* obj1 = (Cylinder*)o1;
  const BVHModel<OBB>* obj2 = (BVHModel<OBB>*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int ConvexOBBCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNodeOBB<Convex> node;
  const Convex* obj1 = (Convex*)o1;
  const BVHModel<OBB>* obj2 = (BVHModel<OBB>*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int PlaneOBBCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNodeOBB<Plane> node;
  const Plane* obj1 = (Plane*)o1;
  const BVHModel<OBB>* obj2 = (BVHModel<OBB>*)o2;
  MESHSHAPEOBBRSS_COMMON_CODE();
}

int BoxRSSCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Box, RSS> node;
  const Box* obj1 = (Box*)o1;
  const BVHModel<RSS>* obj2 = (BVHModel<RSS>*)o2;
  BVHModel<RSS>* obj2_tmp = new BVHModel<RSS>(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int SphereRSSCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Sphere, RSS> node;
  const Sphere* obj1 = (Sphere*)o1;
  const BVHModel<RSS>* obj2 = (BVHModel<RSS>*)o2;
  BVHModel<RSS>* obj2_tmp = new BVHModel<RSS>(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int CapRSSCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Capsule, RSS> node;
  const Capsule* obj1 = (Capsule*)o1;
  const BVHModel<RSS>* obj2 = (BVHModel<RSS>*)o2;
  BVHModel<RSS>* obj2_tmp = new BVHModel<RSS>(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int ConeRSSCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Cone, RSS> node;
  const Cone* obj1 = (Cone*)o1;
  const BVHModel<RSS>* obj2 = (BVHModel<RSS>*)o2;
  BVHModel<RSS>* obj2_tmp = new BVHModel<RSS>(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int CylinderRSSCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Cylinder, RSS> node;
  const Cylinder* obj1 = (Cylinder*)o1;
  const BVHModel<RSS>* obj2 = (BVHModel<RSS>*)o2;
  BVHModel<RSS>* obj2_tmp = new BVHModel<RSS>(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int ConvexRSSCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Convex, RSS> node;
  const Convex* obj1 = (Convex*)o1;
  const BVHModel<RSS>* obj2 = (BVHModel<RSS>*)o2;
  BVHModel<RSS>* obj2_tmp = new BVHModel<RSS>(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int PlaneRSSCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Plane, RSS> node;
  const Plane* obj1 = (Plane*)o1;
  const BVHModel<RSS>* obj2 = (BVHModel<RSS>*)o2;
  BVHModel<RSS>* obj2_tmp = new BVHModel<RSS>(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int BoxKDOP16Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Box, KDOP<16> > node;
  const Box* obj1 = (Box*)o1;
  const BVHModel<KDOP<16> >* obj2 = (BVHModel<KDOP<16> >*)o2;
  BVHModel<KDOP<16> >* obj2_tmp = new BVHModel<KDOP<16> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int SphereKDOP16Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Sphere, KDOP<16> > node;
  const Sphere* obj1 = (Sphere*)o1;
  const BVHModel<KDOP<16> >* obj2 = (BVHModel<KDOP<16> >*)o2;
  BVHModel<KDOP<16> >* obj2_tmp = new BVHModel<KDOP<16> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int CapKDOP16Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Capsule, KDOP<16> > node;
  const Capsule* obj1 = (Capsule*)o1;
  const BVHModel<KDOP<16> >* obj2 = (BVHModel<KDOP<16> >*)o2;
  BVHModel<KDOP<16> >* obj2_tmp = new BVHModel<KDOP<16> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int ConeKDOP16Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Cone, KDOP<16> > node;
  const Cone* obj1 = (Cone*)o1;
  const BVHModel<KDOP<16> >* obj2 = (BVHModel<KDOP<16> >*)o2;
  BVHModel<KDOP<16> >* obj2_tmp = new BVHModel<KDOP<16> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int CylinderKDOP16Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Cylinder, KDOP<16> > node;
  const Cylinder* obj1 = (Cylinder*)o1;
  const BVHModel<KDOP<16> >* obj2 = (BVHModel<KDOP<16> >*)o2;
  BVHModel<KDOP<16> >* obj2_tmp = new BVHModel<KDOP<16> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int ConvexKDOP16Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Convex, KDOP<16> > node;
  const Convex* obj1 = (Convex*)o1;
  const BVHModel<KDOP<16> >* obj2 = (BVHModel<KDOP<16> >*)o2;
  BVHModel<KDOP<16> >* obj2_tmp = new BVHModel<KDOP<16> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int PlaneKDOP16Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Plane, KDOP<16> > node;
  const Plane* obj1 = (Plane*)o1;
  const BVHModel<KDOP<16> >* obj2 = (BVHModel<KDOP<16> >*)o2;
  BVHModel<KDOP<16> >* obj2_tmp = new BVHModel<KDOP<16> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int BoxKDOP18Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Box, KDOP<18> > node;
  const Box* obj1 = (Box*)o1;
  const BVHModel<KDOP<18> >* obj2 = (BVHModel<KDOP<18> >*)o2;
  BVHModel<KDOP<18> >* obj2_tmp = new BVHModel<KDOP<18> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int SphereKDOP18Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Sphere, KDOP<18> > node;
  const Sphere* obj1 = (Sphere*)o1;
  const BVHModel<KDOP<18> >* obj2 = (BVHModel<KDOP<18> >*)o2;
  BVHModel<KDOP<18> >* obj2_tmp = new BVHModel<KDOP<18> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int CapKDOP18Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Capsule, KDOP<18> > node;
  const Capsule* obj1 = (Capsule*)o1;
  const BVHModel<KDOP<18> >* obj2 = (BVHModel<KDOP<18> >*)o2;
  BVHModel<KDOP<18> >* obj2_tmp = new BVHModel<KDOP<18> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int ConeKDOP18Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Cone, KDOP<18> > node;
  const Cone* obj1 = (Cone*)o1;
  const BVHModel<KDOP<18> >* obj2 = (BVHModel<KDOP<18> >*)o2;
  BVHModel<KDOP<18> >* obj2_tmp = new BVHModel<KDOP<18> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int CylinderKDOP18Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Cylinder, KDOP<18> > node;
  const Cylinder* obj1 = (Cylinder*)o1;
  const BVHModel<KDOP<18> >* obj2 = (BVHModel<KDOP<18> >*)o2;
  BVHModel<KDOP<18> >* obj2_tmp = new BVHModel<KDOP<18> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int ConvexKDOP18Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Convex, KDOP<18> > node;
  const Convex* obj1 = (Convex*)o1;
  const BVHModel<KDOP<18> >* obj2 = (BVHModel<KDOP<18> >*)o2;
  BVHModel<KDOP<18> >* obj2_tmp = new BVHModel<KDOP<18> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int PlaneKDOP18Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Plane, KDOP<18> > node;
  const Plane* obj1 = (Plane*)o1;
  const BVHModel<KDOP<18> >* obj2 = (BVHModel<KDOP<18> >*)o2;
  BVHModel<KDOP<18> >* obj2_tmp = new BVHModel<KDOP<18> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int BoxKDOP24Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Box, KDOP<24> > node;
  const Box* obj1 = (Box*)o1;
  const BVHModel<KDOP<24> >* obj2 = (BVHModel<KDOP<24> >*)o2;
  BVHModel<KDOP<24> >* obj2_tmp = new BVHModel<KDOP<24> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int SphereKDOP24Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Sphere, KDOP<24> > node;
  const Sphere* obj1 = (Sphere*)o1;
  const BVHModel<KDOP<24> >* obj2 = (BVHModel<KDOP<24> >*)o2;
  BVHModel<KDOP<24> >* obj2_tmp = new BVHModel<KDOP<24> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int CapKDOP24Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Capsule, KDOP<24> > node;
  const Capsule* obj1 = (Capsule*)o1;
  const BVHModel<KDOP<24> >* obj2 = (BVHModel<KDOP<24> >*)o2;
  BVHModel<KDOP<24> >* obj2_tmp = new BVHModel<KDOP<24> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int ConeKDOP24Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Cone, KDOP<24> > node;
  const Cone* obj1 = (Cone*)o1;
  const BVHModel<KDOP<24> >* obj2 = (BVHModel<KDOP<24> >*)o2;
  BVHModel<KDOP<24> >* obj2_tmp = new BVHModel<KDOP<24> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int CylinderKDOP24Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Cylinder, KDOP<24> > node;
  const Cylinder* obj1 = (Cylinder*)o1;
  const BVHModel<KDOP<24> >* obj2 = (BVHModel<KDOP<24> >*)o2;
  BVHModel<KDOP<24> >* obj2_tmp = new BVHModel<KDOP<24> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int ConvexKDOP24Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Convex, KDOP<24> > node;
  const Convex* obj1 = (Convex*)o1;
  const BVHModel<KDOP<24> >* obj2 = (BVHModel<KDOP<24> >*)o2;
  BVHModel<KDOP<24> >* obj2_tmp = new BVHModel<KDOP<24> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

int PlaneKDOP24Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  ShapeMeshCollisionTraversalNode<Plane, KDOP<24> > node;
  const Plane* obj1 = (Plane*)o1;
  const BVHModel<KDOP<24> >* obj2 = (BVHModel<KDOP<24> >*)o2;
  BVHModel<KDOP<24> >* obj2_tmp = new BVHModel<KDOP<24> >(*obj2);
  SHAPEMESH_COMMON_CODE();
}

*/


int AABBAABBCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshCollisionTraversalNode<AABB> node;
  const BVHModel<AABB>* obj1 = (BVHModel<AABB>*)o1;
  const BVHModel<AABB>* obj2 = (BVHModel<AABB>*)o2;
  BVHModel<AABB>* obj1_tmp = new BVHModel<AABB>(*obj1);
  BVHModel<AABB>* obj2_tmp = new BVHModel<AABB>(*obj2);
  MESHMESH_COMMON_CODE();
}

int OBBOBBCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshCollisionTraversalNodeOBB node;
  const BVHModel<OBB>* obj1 = (BVHModel<OBB>*)o1;
  const BVHModel<OBB>* obj2 = (BVHModel<OBB>*)o2;
  MESHMESHOBBRSS_COMMON_CODE();
}

int RSSRSSCollide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshCollisionTraversalNodeRSS node;
  const BVHModel<RSS>* obj1 = (BVHModel<RSS>*)o1;
  const BVHModel<RSS>* obj2 = (BVHModel<RSS>*)o2;
  MESHMESHOBBRSS_COMMON_CODE();
}

int KDOP16KDOP16Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshCollisionTraversalNode<KDOP<16> > node;
  const BVHModel<KDOP<16> >* obj1 = (BVHModel<KDOP<16> >*)o1;
  const BVHModel<KDOP<16> >* obj2 = (BVHModel<KDOP<16> >*)o2;
  BVHModel<KDOP<16> >* obj1_tmp = new BVHModel<KDOP<16> >(*obj1);
  BVHModel<KDOP<16> >* obj2_tmp = new BVHModel<KDOP<16> >(*obj2);
  MESHMESH_COMMON_CODE();
}

int KDOP18KDOP18Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshCollisionTraversalNode<KDOP<18> > node;
  const BVHModel<KDOP<18> >* obj1 = (BVHModel<KDOP<18> >*)o1;
  const BVHModel<KDOP<18> >* obj2 = (BVHModel<KDOP<18> >*)o2;
  BVHModel<KDOP<18> >* obj1_tmp = new BVHModel<KDOP<18> >(*obj1);
  BVHModel<KDOP<18> >* obj2_tmp = new BVHModel<KDOP<18> >(*obj2);
  MESHMESH_COMMON_CODE();
}

int KDOP24KDOP24Collide(const CollisionObject* o1, const CollisionObject* o2, int num_max_contacts, bool exhaustive, bool enable_contact, std::vector<Contact>& contacts)
{
  MeshCollisionTraversalNode<KDOP<24> > node;
  const BVHModel<KDOP<24> >* obj1 = (BVHModel<KDOP<24> >*)o1;
  const BVHModel<KDOP<24> >* obj2 = (BVHModel<KDOP<24> >*)o2;
  BVHModel<KDOP<24> >* obj1_tmp = new BVHModel<KDOP<24> >(*obj1);
  BVHModel<KDOP<24> >* obj2_tmp = new BVHModel<KDOP<24> >(*obj2);
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
