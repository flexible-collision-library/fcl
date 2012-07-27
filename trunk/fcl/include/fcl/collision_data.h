#ifndef FCL_COLLISION_DATA_H
#define FCL_COLLISION_DATA_H

#include "fcl/collision_object.h"
#include "fcl/vec_3f.h"
#include <vector>
#include <set>
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
  
  static const int NONE = -1;

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

  bool operator < (const Contact& other) const
  {
    if(b1 == other.b1)
      return b2 < other.b2;
    return b1 < other.b1;
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

  CostSource(const AABB& aabb, FCL_REAL cost_density_) : aabb_min(aabb.min_),
                                                         aabb_max(aabb.max_),
                                                         cost_density(cost_density_)
  {
  }

  CostSource() {}

  bool operator < (const CostSource& other) const
  {
    return cost_density < other.cost_density;
  }
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
private:
  std::vector<Contact> contacts;

  std::set<CostSource> cost_sources;

  const CollisionRequest* request;

public:
  CollisionResult()
  {
    request = NULL;
  }

  void setRequest(const CollisionRequest& request_)
  {
    request = &request_;
  }

  inline void addContact(const Contact& c) 
  {
    contacts.push_back(c);
  }

  inline void addCostSource(const CostSource& c)
  {
    if(request)
    {
      cost_sources.insert(c);
      if(cost_sources.size() > request->num_max_cost_sources)
        cost_sources.erase(cost_sources.begin());        
    }
    else
    {
      cost_sources.insert(c);
    }
  }

  bool isCollision() const
  {
    return contacts.size() > 0;
  }

  size_t numContacts() const
  {
    return contacts.size();
  }

  size_t numCostSources() const
  {
    return cost_sources.size();
  }

  const Contact& getContact(size_t i) const
  {
    if(i < contacts.size()) 
      return contacts[i];
    else
      return contacts.back();
  }

  void getContacts(std::vector<Contact>& contacts_)
  {
    contacts_.resize(contacts.size());
    std::copy(contacts.begin(), contacts.end(), contacts_.begin());
  }

  void getCostSources(std::vector<CostSource>& cost_sources_)
  {
    cost_sources_.resize(cost_sources.size());
    std::copy(cost_sources.begin(), cost_sources.end(), cost_sources_.begin());
  }

  void clear()
  {
    contacts.clear();
    cost_sources.clear();
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

  DistanceRequest(bool enable_nearest_points_ = false) : enable_nearest_points(enable_nearest_points_)
  {
  }
};

struct DistanceResult
{
private:
  const DistanceRequest* request;

public:

  FCL_REAL min_distance;

  Vec3f nearest_points[2];

  const CollisionGeometry* o1;
  const CollisionGeometry* o2;
  int b1;
  int b2;

  static const int NONE = -1;
  
  DistanceResult(FCL_REAL min_distance_ = std::numeric_limits<FCL_REAL>::max()) : min_distance(min_distance_), 
                                                                                  o1(NULL),
                                                                                  o2(NULL),
                                                                                  b1(-1),
                                                                                  b2(-1)
  {
    request = NULL;
  }

  void setRequest(const DistanceRequest& request_)
  {
    request = &request_;
  }

  void update(FCL_REAL distance, const CollisionGeometry* o1_, const CollisionGeometry* o2_, int b1_, int b2_)
  {
    if(min_distance > distance)
    {
      min_distance = distance;
      o1 = o1_;
      o2 = o2_;
      b1 = b1_;
      b2 = b2_;
    }
  }

  void update(FCL_REAL distance, const CollisionGeometry* o1_, const CollisionGeometry* o2_, int b1_, int b2_, const Vec3f& p1, const Vec3f& p2)
  {
    if(min_distance > distance)
    {
      min_distance = distance;
      o1 = o1_;
      o2 = o2_;
      b1 = b1_;
      b2 = b2_;
      nearest_points[0] = p1;
      nearest_points[1] = p2;
    }
  }

  void update(const DistanceResult& other_result)
  {
    if(min_distance > other_result.min_distance)
    {
      min_distance = other_result.min_distance;
      o1 = other_result.o1;
      o2 = other_result.o2;
      b1 = other_result.b1;
      b2 = other_result.b2;
      nearest_points[0] = other_result.nearest_points[0];
      nearest_points[1] = other_result.nearest_points[1];
    }
  }

  void clear()
  {
    min_distance = std::numeric_limits<FCL_REAL>::max();
    o1 = NULL;
    o2 = NULL;
    b1 = -1;
    b2 = -1;
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
