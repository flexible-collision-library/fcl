#ifndef FCL_COLLISION_DATA_H
#define FCL_COLLISION_DATA_H

#include "fcl/collision_object.h"
#include "fcl/vec_3f.h"
#include <vector>
#include <limits>


namespace fcl
{

struct Contact
{
  FCL_REAL penetration_depth;
  Vec3f normal;
  Vec3f pos;
  const CollisionGeometry* o1;
  const CollisionGeometry* o2;
  int b1;
  int b2;

  Contact()
  {
    o1 = NULL;
    o2 = NULL;
    b1 = -1;
    b2 = -1;
  }

  Contact(const CollisionGeometry* o1_, const CollisionGeometry* o2_, int b1_, int b2_)
  {
    o1 = o1_;
    o2 = o2_;
    b1 = b1_;
    b2 = b2_;
  }

  Contact(const CollisionGeometry* o1_, const CollisionGeometry* o2_, int b1_, int b2_,
          const Vec3f& pos_, const Vec3f& normal_, FCL_REAL depth_)
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

struct CostSource
{
  Vec3f aabb_min;
  Vec3f aabb_max;
  FCL_REAL cost_density;

  CostSource(const Vec3f& aabb_min_, const Vec3f& aabb_max_, FCL_REAL cost_density_) : aabb_min(aabb_min_),
                                                                                       aabb_max(aabb_max_),
                                                                                       cost_density(cost_density_)
  {
  }

  CostSource() {}
};

struct CollisionRequest
{
  bool exhaustive;
  size_t num_max_contacts;
  bool enable_contact;
  size_t num_max_cost_sources;
  bool enable_cost;

  CollisionRequest(bool exhaustive_ = false,
                   size_t num_max_contacts_ = 1,
                   bool enable_contact_ = false,
                   size_t num_max_cost_sources_ = 1,
                   bool enable_cost_ = false) : exhaustive(exhaustive_),
                                                num_max_contacts(num_max_contacts_),
                                                enable_contact(enable_contact_),
                                                num_max_cost_sources(num_max_cost_sources_),
                                                enable_cost(enable_cost_)
  {
  }

};


struct CollisionResult
{
  std::vector<Contact> contacts;
  std::vector<CostSource> cost_sources;

  bool is_collision;

  CollisionResult() : is_collision(false)
  {
  }

  void clear()
  {
    contacts.clear();
    cost_sources.clear();
    is_collision = false;
  }
};


struct CollisionData
{
  CollisionData()
  {
    done = false;
  }

  CollisionRequest request;
  CollisionResult result;
  bool done;
};


struct DistanceRequest
{
  bool enable_nearest_points;
  DistanceRequest() : enable_nearest_points(false)
  {
  }
};

struct DistanceResult
{
  FCL_REAL min_distance;

  Vec3f nearest_points[2];
  
  DistanceResult() : min_distance(std::numeric_limits<FCL_REAL>::max())
  {
  }

  void clear()
  {
    min_distance = std::numeric_limits<FCL_REAL>::max();
  }
};

struct DistanceData
{
  DistanceData()
  {
    done = false;
  }

  bool done;

  DistanceRequest request;
  DistanceResult result;
};



}

#endif
