#ifndef FCL_COLLISION_DATA_H
#define FCL_COLLISION_DATA_H

#include "fcl/collision_object.h"
#include "fcl/vec_3f.h"
#include <vector>


namespace fcl
{

struct Contact
{
  BVH_REAL penetration_depth;
  Vec3f normal;
  Vec3f pos;
  const CollisionObject* o1;
  const CollisionObject* o2;
  int b1;
  int b2;

  Contact()
  {
    o1 = NULL;
    o2 = NULL;
    b1 = -1;
    b2 = -1;
  }

  Contact(const CollisionObject* o1_, const CollisionObject* o2_, int b1_, int b2_)
  {
    o1 = o1_;
    o2 = o2_;
    b1 = b1_;
    b2 = b2_;
  }

  Contact(const CollisionObject* o1_, const CollisionObject* o2_, int b1_, int b2_,
          const Vec3f& pos_, const Vec3f& normal_, BVH_REAL depth_)
  {
    o1 = o1_;
    o2 = o2_;
    b1 = b1_;
    b2 = b2_;
    normal = normal_;
    pos = pos_;
    penetration_depth = depth_;
  }
};

struct CollisionData
{
  CollisionData()
  {
    done = false;
    is_collision = false;
    num_max_contacts = 1;
    enable_contact = false;
    exhaustive = false;
  }

  bool done;
  bool is_collision;
  bool exhaustive;
  unsigned int num_max_contacts;
  bool enable_contact;

  std::vector<Contact> contacts;
};



}

#endif
