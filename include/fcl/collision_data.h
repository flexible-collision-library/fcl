/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
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

/** \author Jia Pan */


#ifndef FCL_COLLISION_DATA_H
#define FCL_COLLISION_DATA_H

#include "fcl/collision_object.h"
#include "fcl/learning/classifier.h"

#include <vector>
#include <set>
#include <limits>
#include <functional>

namespace fcl
{

/// @brief Type of narrow phase GJK solver
enum GJKSolverType {GST_LIBCCD, GST_INDEP};

/// @brief Minimal contact information returned by collision
template <typename Scalar>
struct ContactPoint
{
  /// @brief Contact normal, pointing from o1 to o2
  Vector3<Scalar> normal;

  /// @brief Contact position, in world space
  Vector3<Scalar> pos;

  /// @brief Penetration depth
  Scalar penetration_depth;

  /// @brief Constructor
  ContactPoint();

  /// @brief Constructor
  ContactPoint(const Vector3<Scalar>& n_, const Vector3<Scalar>& p_, Scalar d_);
};

using ContactPointf = ContactPoint<float>;
using ContactPointd = ContactPoint<double>;

template <typename Scalar>
void flipNormal(std::vector<ContactPoint<Scalar>>& contacts)
{
  for (auto& contact : contacts)
    contact.normal *= -1.0;
}

/// @brief Return true if _cp1's penentration depth is less than _cp2's.
template <typename Scalar>
bool comparePenDepth(
    const ContactPoint<Scalar>& _cp1, const ContactPoint<Scalar>& _cp2);

/// @brief Contact information returned by collision
template <typename Scalar>
struct Contact
{
  /// @brief collision object 1
  const CollisionGeometry<Scalar>* o1;

  /// @brief collision object 2
  const CollisionGeometry<Scalar>* o2;

  /// @brief contact primitive in object 1
  /// if object 1 is mesh or point cloud, it is the triangle or point id
  /// if object 1 is geometry shape, it is NONE (-1),
  /// if object 1 is octree, it is the id of the cell
  int b1;

  /// @brief contact primitive in object 2
  /// if object 2 is mesh or point cloud, it is the triangle or point id
  /// if object 2 is geometry shape, it is NONE (-1),
  /// if object 2 is octree, it is the id of the cell
  int b2;
 
  /// @brief contact normal, pointing from o1 to o2
  Vector3<Scalar> normal;

  /// @brief contact position, in world space
  Vector3<Scalar> pos;

  /// @brief penetration depth
  Scalar penetration_depth;

 
  /// @brief invalid contact primitive information
  static const int NONE = -1;

  Contact();

  Contact(const CollisionGeometry<Scalar>* o1_, const CollisionGeometry<Scalar>* o2_, int b1_, int b2_);

  Contact(const CollisionGeometry<Scalar>* o1_, const CollisionGeometry<Scalar>* o2_, int b1_, int b2_,
          const Vector3<Scalar>& pos_, const Vector3<Scalar>& normal_, Scalar depth_);

  bool operator < (const Contact& other) const;
};

using Contactf = Contact<float>;
using Contactd = Contact<double>;

/// @brief Cost source describes an area with a cost. The area is described by an AABBd region.
template <typename Scalar>
struct CostSource
{
  /// @brief aabb lower bound
  Vector3<Scalar> aabb_min;

  /// @brief aabb upper bound
  Vector3<Scalar> aabb_max;

  /// @brief cost density in the AABBd region
  Scalar cost_density;

  Scalar total_cost;

  CostSource(const Vector3<Scalar>& aabb_min_, const Vector3<Scalar>& aabb_max_, Scalar cost_density_);

  CostSource(const AABBd& aabb, Scalar cost_density_);

  CostSource();

  bool operator < (const CostSource& other) const;
};

using CostSourcef = CostSource<float>;
using CostSourced = CostSource<double>;

template <typename Scalar>
struct CollisionResult;

/// @brief request to the collision algorithm
template <typename Scalar>
struct CollisionRequest
{  
  /// @brief The maximum number of contacts will return
  size_t num_max_contacts;

  /// @brief whether the contact information (normal, penetration depth and contact position) will return
  bool enable_contact;

  /// @brief The maximum number of cost sources will return
  size_t num_max_cost_sources;

  /// @brief whether the cost sources will be computed
  bool enable_cost;

  /// @brief whether the cost computation is approximated
  bool use_approximate_cost;

  /// @brief narrow phase solver
  GJKSolverType gjk_solver_type;

  /// @brief whether enable gjk intial guess
  bool enable_cached_gjk_guess;
  
  /// @brief the gjk intial guess set by user
  Vector3<Scalar> cached_gjk_guess;

  CollisionRequest(size_t num_max_contacts_ = 1,
                   bool enable_contact_ = false,
                   size_t num_max_cost_sources_ = 1,
                   bool enable_cost_ = false,
                   bool use_approximate_cost_ = true,
                   GJKSolverType gjk_solver_type_ = GST_LIBCCD);

  bool isSatisfied(const CollisionResult<Scalar>& result) const;
};

using CollisionRequestf = CollisionRequest<float>;
using CollisionRequestd = CollisionRequest<double>;

/// @brief collision result
template <typename Scalar>
struct CollisionResult
{
private:
  /// @brief contact information
  std::vector<Contact<Scalar>> contacts;

  /// @brief cost sources
  std::set<CostSource<Scalar>> cost_sources;

public:
  Vector3<Scalar> cached_gjk_guess;

public:
  CollisionResult();

  /// @brief add one contact into result structure
  void addContact(const Contact<Scalar>& c);

  /// @brief add one cost source into result structure
  void addCostSource(const CostSource<Scalar>& c, std::size_t num_max_cost_sources);

  /// @brief return binary collision result
  bool isCollision() const;

  /// @brief number of contacts found
  size_t numContacts() const;

  /// @brief number of cost sources found
  size_t numCostSources() const;

  /// @brief get the i-th contact calculated
  const Contact<Scalar>& getContact(size_t i) const;

  /// @brief get all the contacts
  void getContacts(std::vector<Contact<Scalar>>& contacts_);

  /// @brief get all the cost sources 
  void getCostSources(std::vector<CostSource<Scalar>>& cost_sources_);

  /// @brief clear the results obtained
  void clear();
};

using CollisionResultf = CollisionResult<float>;
using CollisionResultd = CollisionResult<double>;

template <typename Scalar>
struct DistanceResult;

/// @brief request to the distance computation
template <typename Scalar>
struct DistanceRequest
{
  /// @brief whether to return the nearest points
  bool enable_nearest_points;

  /// @brief error threshold for approximate distance
  Scalar rel_err; // relative error, between 0 and 1
  Scalar abs_err; // absoluate error

  /// @brief narrow phase solver type
  GJKSolverType gjk_solver_type;

  DistanceRequest(bool enable_nearest_points_ = false,
                  Scalar rel_err_ = 0.0,
                  Scalar abs_err_ = 0.0,
                  GJKSolverType gjk_solver_type_ = GST_LIBCCD);

  bool isSatisfied(const DistanceResult<Scalar>& result) const;
};

using DistanceRequestf = DistanceRequest<float>;
using DistanceRequestd = DistanceRequest<double>;

/// @brief distance result
template <typename Scalar>
struct DistanceResult
{

public:

  /// @brief minimum distance between two objects. if two objects are in collision, min_distance <= 0.
  Scalar min_distance;

  /// @brief nearest points
  Vector3<Scalar> nearest_points[2];

  /// @brief collision object 1
  const CollisionGeometry<Scalar>* o1;

  /// @brief collision object 2
  const CollisionGeometry<Scalar>* o2;

  /// @brief information about the nearest point in object 1
  /// if object 1 is mesh or point cloud, it is the triangle or point id
  /// if object 1 is geometry shape, it is NONE (-1),
  /// if object 1 is octree, it is the id of the cell
  int b1;

  /// @brief information about the nearest point in object 2
  /// if object 2 is mesh or point cloud, it is the triangle or point id
  /// if object 2 is geometry shape, it is NONE (-1),
  /// if object 2 is octree, it is the id of the cell
  int b2;

  /// @brief invalid contact primitive information
  static const int NONE = -1;
  
  DistanceResult(Scalar min_distance_ = std::numeric_limits<Scalar>::max());

  /// @brief add distance information into the result
  void update(Scalar distance, const CollisionGeometry<Scalar>* o1_, const CollisionGeometry<Scalar>* o2_, int b1_, int b2_);

  /// @brief add distance information into the result
  void update(Scalar distance, const CollisionGeometry<Scalar>* o1_, const CollisionGeometry<Scalar>* o2_, int b1_, int b2_, const Vector3<Scalar>& p1, const Vector3<Scalar>& p2);

  /// @brief add distance information into the result
  void update(const DistanceResult& other_result);

  /// @brief clear the result
  void clear();
};

using DistanceResultf = DistanceResult<float>;
using DistanceResultd = DistanceResult<double>;

enum CCDMotionType {CCDM_TRANS, CCDM_LINEAR, CCDM_SCREW, CCDM_SPLINE};
enum CCDSolverType {CCDC_NAIVE, CCDC_CONSERVATIVE_ADVANCEMENT, CCDC_RAY_SHOOTING, CCDC_POLYNOMIAL_SOLVER};

template <typename Scalar>
struct ContinuousCollisionRequest
{
  /// @brief maximum num of iterations
  std::size_t num_max_iterations;

  /// @brief error in first contact time
  Scalar toc_err;

  /// @brief ccd motion type
  CCDMotionType ccd_motion_type;

  /// @brief gjk solver type
  GJKSolverType gjk_solver_type;

  /// @brief ccd solver type
  CCDSolverType ccd_solver_type;
  
  ContinuousCollisionRequest(std::size_t num_max_iterations_ = 10,
                             Scalar toc_err_ = 0.0001,
                             CCDMotionType ccd_motion_type_ = CCDM_TRANS,
                             GJKSolverType gjk_solver_type_ = GST_LIBCCD,
                             CCDSolverType ccd_solver_type_ = CCDC_NAIVE);
  
};

using ContinuousCollisionRequestf = ContinuousCollisionRequest<float>;
using ContinuousCollisionRequestd = ContinuousCollisionRequest<double>;

/// @brief continuous collision result
template <typename Scalar>
struct ContinuousCollisionResult
{
  /// @brief collision or not
  bool is_collide;
  
  /// @brief time of contact in [0, 1]
  Scalar time_of_contact;

  Transform3<Scalar> contact_tf1, contact_tf2;
  
  ContinuousCollisionResult();
};

using ContinuousCollisionResultf = ContinuousCollisionResult<float>;
using ContinuousCollisionResultd = ContinuousCollisionResult<double>;

enum PenetrationDepthType {PDT_TRANSLATIONAL, PDT_GENERAL_EULER, PDT_GENERAL_QUAT, PDT_GENERAL_EULER_BALL, PDT_GENERAL_QUAT_BALL};

template <typename Scalar>
struct PenetrationDepthRequest
{
  void* classifier;

  /// @brief PD algorithm type
  PenetrationDepthType pd_type;

  /// @brief gjk solver type
  GJKSolverType gjk_solver_type;

  std::vector<Transform3<Scalar>> contact_vectors;

  PenetrationDepthRequest(void* classifier_,
                          PenetrationDepthType pd_type_ = PDT_TRANSLATIONAL,
                          GJKSolverType gjk_solver_type_ = GST_LIBCCD);
};

template <typename Scalar>
struct PenetrationDepthResult
{
  /// @brief penetration depth value
  Scalar pd_value;

  /// @brief the transform where the collision is resolved
  Transform3<Scalar> resolved_tf;
};

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
template <typename Scalar>
ContactPoint<Scalar>::ContactPoint()
  : normal(Vector3<Scalar>::Zero()),
    pos(Vector3<Scalar>::Zero()),
    penetration_depth(0.0)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
ContactPoint<Scalar>::ContactPoint(
    const Vector3<Scalar>& n_, const Vector3<Scalar>& p_, Scalar d_)
  : normal(n_), pos(p_), penetration_depth(d_)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
bool comparePenDepth(const ContactPoint<Scalar>& _cp1, const ContactPoint<Scalar>& _cp2)
{
  return _cp1.penetration_depth < _cp2.penetration_depth;
}

//==============================================================================
template <typename Scalar>
Contact<Scalar>::Contact()
  : o1(NULL),
    o2(NULL),
    b1(NONE),
    b2(NONE)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
Contact<Scalar>::Contact(const CollisionGeometry<Scalar>* o1_, const CollisionGeometry<Scalar>* o2_, int b1_, int b2_) : o1(o1_),
  o2(o2_),
  b1(b1_),
  b2(b2_)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
Contact<Scalar>::Contact(const CollisionGeometry<Scalar>* o1_, const CollisionGeometry<Scalar>* o2_, int b1_, int b2_, const Vector3<Scalar>& pos_, const Vector3<Scalar>& normal_, Scalar depth_) : o1(o1_),
  o2(o2_),
  b1(b1_),
  b2(b2_),
  normal(normal_),
  pos(pos_),
  penetration_depth(depth_)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
bool Contact<Scalar>::operator <(const Contact& other) const
{
  if(b1 == other.b1)
    return b2 < other.b2;
  return b1 < other.b1;
}

//==============================================================================
template <typename Scalar>
CostSource<Scalar>::CostSource(const Vector3<Scalar>& aabb_min_, const Vector3<Scalar>& aabb_max_, Scalar cost_density_) : aabb_min(aabb_min_),
  aabb_max(aabb_max_),
  cost_density(cost_density_)
{
  total_cost = cost_density * (aabb_max[0] - aabb_min[0]) * (aabb_max[1] - aabb_min[1]) * (aabb_max[2] - aabb_min[2]);
}

//==============================================================================
template <typename Scalar>
CostSource<Scalar>::CostSource(const AABBd& aabb, Scalar cost_density_) : aabb_min(aabb.min_),
  aabb_max(aabb.max_),
  cost_density(cost_density_)
{
  total_cost = cost_density * (aabb_max[0] - aabb_min[0]) * (aabb_max[1] - aabb_min[1]) * (aabb_max[2] - aabb_min[2]);
}

//==============================================================================
template <typename Scalar>
CostSource<Scalar>::CostSource()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
bool CostSource<Scalar>::operator <(const CostSource& other) const
{
  if(total_cost < other.total_cost)
    return false;
  if(total_cost > other.total_cost)
    return true;

  if(cost_density < other.cost_density)
    return false;
  if(cost_density > other.cost_density)
    return true;

  for(size_t i = 0; i < 3; ++i)
    if(aabb_min[i] != other.aabb_min[i])
      return aabb_min[i] < other.aabb_min[i];

  return false;
}

//==============================================================================
template <typename Scalar>
CollisionRequest<Scalar>::CollisionRequest(size_t num_max_contacts_, bool enable_contact_, size_t num_max_cost_sources_, bool enable_cost_, bool use_approximate_cost_, GJKSolverType gjk_solver_type_) : num_max_contacts(num_max_contacts_),
  enable_contact(enable_contact_),
  num_max_cost_sources(num_max_cost_sources_),
  enable_cost(enable_cost_),
  use_approximate_cost(use_approximate_cost_),
  gjk_solver_type(gjk_solver_type_)
{
  enable_cached_gjk_guess = false;
  cached_gjk_guess = Vector3<Scalar>(1, 0, 0);
}

//==============================================================================
template <typename Scalar>
bool CollisionRequest<Scalar>::isSatisfied(
    const CollisionResult<Scalar>& result) const
{
  return (!enable_cost)
      && result.isCollision()
      && (num_max_contacts <= result.numContacts());
}

//==============================================================================
template <typename Scalar>
CollisionResult<Scalar>::CollisionResult()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
void CollisionResult<Scalar>::addContact(const Contact<Scalar>& c)
{
  contacts.push_back(c);
}

//==============================================================================
template <typename Scalar>
void CollisionResult<Scalar>::addCostSource(
    const CostSource<Scalar>& c, std::size_t num_max_cost_sources)
{
  cost_sources.insert(c);
  while (cost_sources.size() > num_max_cost_sources)
    cost_sources.erase(--cost_sources.end());
}

//==============================================================================
template <typename Scalar>
bool CollisionResult<Scalar>::isCollision() const
{
  return contacts.size() > 0;
}

//==============================================================================
template <typename Scalar>
size_t CollisionResult<Scalar>::numContacts() const
{
  return contacts.size();
}

//==============================================================================
template <typename Scalar>
size_t CollisionResult<Scalar>::numCostSources() const
{
  return cost_sources.size();
}

//==============================================================================
template <typename Scalar>
const Contact<Scalar>& CollisionResult<Scalar>::getContact(size_t i) const
{
  if(i < contacts.size())
    return contacts[i];
  else
    return contacts.back();
}

//==============================================================================
template <typename Scalar>
void CollisionResult<Scalar>::getContacts(
    std::vector<Contact<Scalar>>& contacts_)
{
  contacts_.resize(contacts.size());
  std::copy(contacts.begin(), contacts.end(), contacts_.begin());
}

//==============================================================================
template <typename Scalar>
void CollisionResult<Scalar>::getCostSources(
    std::vector<CostSource<Scalar>>& cost_sources_)
{
  cost_sources_.resize(cost_sources.size());
  std::copy(cost_sources.begin(), cost_sources.end(), cost_sources_.begin());
}

//==============================================================================
template <typename Scalar>
void CollisionResult<Scalar>::clear()
{
  contacts.clear();
  cost_sources.clear();
}

//==============================================================================
template <typename Scalar>
DistanceRequest<Scalar>::DistanceRequest(bool enable_nearest_points_, Scalar rel_err_, Scalar abs_err_, GJKSolverType gjk_solver_type_) : enable_nearest_points(enable_nearest_points_),
  rel_err(rel_err_),
  abs_err(abs_err_),
  gjk_solver_type(gjk_solver_type_)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
bool DistanceRequest<Scalar>::isSatisfied(
    const DistanceResult<Scalar>& result) const
{
  return (result.min_distance <= 0);
}

//==============================================================================
template <typename Scalar>
DistanceResult<Scalar>::DistanceResult(Scalar min_distance_)
  : min_distance(min_distance_),
    o1(NULL),
    o2(NULL),
    b1(NONE),
    b2(NONE)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
void DistanceResult<Scalar>::update(Scalar distance, const CollisionGeometry<Scalar>* o1_, const CollisionGeometry<Scalar>* o2_, int b1_, int b2_)
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

//==============================================================================
template <typename Scalar>
void DistanceResult<Scalar>::update(Scalar distance, const CollisionGeometry<Scalar>* o1_, const CollisionGeometry<Scalar>* o2_, int b1_, int b2_, const Vector3<Scalar>& p1, const Vector3<Scalar>& p2)
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

//==============================================================================
template <typename Scalar>
void DistanceResult<Scalar>::update(const DistanceResult& other_result)
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

//==============================================================================
template <typename Scalar>
void DistanceResult<Scalar>::clear()
{
  min_distance = std::numeric_limits<Scalar>::max();
  o1 = NULL;
  o2 = NULL;
  b1 = NONE;
  b2 = NONE;
}

//==============================================================================
template <typename Scalar>
ContinuousCollisionRequest<Scalar>::ContinuousCollisionRequest(std::size_t num_max_iterations_, Scalar toc_err_, CCDMotionType ccd_motion_type_, GJKSolverType gjk_solver_type_, CCDSolverType ccd_solver_type_) : num_max_iterations(num_max_iterations_),
  toc_err(toc_err_),
  ccd_motion_type(ccd_motion_type_),
  gjk_solver_type(gjk_solver_type_),
  ccd_solver_type(ccd_solver_type_)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
ContinuousCollisionResult<Scalar>::ContinuousCollisionResult() : is_collide(false), time_of_contact(1.0)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
PenetrationDepthRequest<Scalar>::PenetrationDepthRequest(void* classifier_, PenetrationDepthType pd_type_, GJKSolverType gjk_solver_type_) : classifier(classifier_),
  pd_type(pd_type_),
  gjk_solver_type(gjk_solver_type_)
{
}

} // namespace fcl

#endif
