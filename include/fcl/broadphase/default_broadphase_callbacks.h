/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Toyota Research Institute
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
 *   * Neither the name of the copyright holder nor the names of its
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

/** @author Sean Curtis (sean@tri.global) */

#ifndef FCL_BROADPHASE_DEFAULTBROADPHASECALLBACKS_H
#define FCL_BROADPHASE_DEFAULTBROADPHASECALLBACKS_H

#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_request.h"
#include "fcl/narrowphase/collision_result.h"
#include "fcl/narrowphase/continuous_collision.h"
#include "fcl/narrowphase/continuous_collision_request.h"
#include "fcl/narrowphase/continuous_collision_result.h"
#include "fcl/narrowphase/distance.h"
#include "fcl/narrowphase/distance_request.h"
#include "fcl/narrowphase/distance_result.h"

namespace fcl {

/// @brief Collision data for use with the DefaultCollisionFunction. It stores
/// the collision request and the result given by collision algorithm (and
/// stores the conclusion of whether further evaluation of the broadphase
/// collision manager has been deemed unnecessary).
template <typename S>
struct DefaultCollisionData {
  CollisionRequest<S> request;
  CollisionResult<S> result;

  /// If `true`, requests that the broadphase evaluation stop.
  bool done{false};
};

/// @brief Provides a simple callback for the collision query in the
/// BroadPhaseCollisionManager. It assumes the `data` parameter is non-null and
/// points to an instance of DefaultCollisionData. It simply invokes the
/// `collide()` method on the culled pair of geometries and stores the results
/// in the data's CollisionResult instance.
///
/// This callback will cause the broadphase evaluation to stop if:
///   - the collision requests _disables_ cost _and_
///   - the collide() reports a collision for the culled pair, _and_
///   - we've reported the number of contacts requested in the CollisionRequest.
///
/// For a given instance of DefaultCollisionData, if broadphase evaluation has
/// already terminated (i.e., DefaultCollisionFunction() returned `true`),
/// subsequent invocations with the same instance of DefaultCollisionData will
/// return immediately, requesting termination of broadphase evaluation (i.e.,
/// return `true`).
///
/// @param o1   The first object in the culled pair.
/// @param o2   The second object in the culled pair.
/// @param data A non-null pointer to a DefaultCollisionData instance.
/// @return `true` if the broadphase evaluation should stop.
/// @tparam S   The scalar type with which the computation will be performed.
template <typename S>
bool DefaultCollisionFunction(CollisionObject<S>* o1, CollisionObject<S>* o2,
                              void* data) {
  assert(data != nullptr);
  auto* collision_data = static_cast<DefaultCollisionData<S>*>(data);
  const CollisionRequest<S>& request = collision_data->request;
  CollisionResult<S>& result = collision_data->result;

  if(collision_data->done) return true;

  collide(o1, o2, request, result);

  if (!request.enable_cost && result.isCollision() &&
      result.numContacts() >= request.num_max_contacts) {
    collision_data->done = true;
  }

  return collision_data->done;
}


/// @brief Collision data for use with the DefaultContinuousCollisionFunction.
/// It stores the collision request and the result given by the collision
/// algorithm (and stores the conclusion of whether further evaluation of the
/// broadphase collision manager has been deemed unnecessary).
template <typename S>
struct DefaultContinuousCollisionData {
  ContinuousCollisionRequest<S> request;
  ContinuousCollisionResult<S> result;

  /// If `true`, requests that the broadphase evaluation stop.
  bool done{false};
};


/// @brief Provides a simple callback for the continuous collision query in the
/// BroadPhaseCollisionManager. It assumes the `data` parameter is non-null and
/// points to an instance of DefaultContinuousCollisionData. It simply invokes
/// the `collide()` method on the culled pair of geometries and stores the
/// results in the data's ContinuousCollisionResult instance.
///
/// This callback will never cause the broadphase evaluation to terminate early.
/// However, if the `done` member of the DefaultContinuousCollisionData is set
/// to true, this method will simply return without doing any computation.
///
/// For a given instance of DefaultContinuousCollisionData, if broadphase
/// evaluation has already terminated (i.e.,
/// DefaultContinuousCollisionFunction() returned `true`), subsequent
/// invocations with the same instance of DefaultCollisionData will return
/// immediately, requesting termination of broadphase evaluation (i.e., return
/// `true`).
///
/// @param o1   The first object in the culled pair.
/// @param o2   The second object in the culled pair.
/// @param data A non-null pointer to a DefaultCollisionData instance.
/// @return True if the broadphase evaluation should stop.
/// @tparam S   The scalar type with which the computation will be performed.
template <typename S>
bool DefaultContinuousCollisionFunction(ContinuousCollisionObject<S>* o1,
                                        ContinuousCollisionObject<S>* o2,
                                        void* data) {
  assert(data != nullptr);
  auto* cdata = static_cast<DefaultContinuousCollisionData<S>*>(data);

  if (cdata->done) return true;

  const ContinuousCollisionRequest<S>& request = cdata->request;
  ContinuousCollisionResult<S>& result = cdata->result;
  collide(o1, o2, request, result);

  return cdata->done;
}

/// @brief Distance data for use with the DefaultDistanceFunction. It stores
/// the distance request and the result given by distance algorithm (and
/// stores the conclusion of whether further evaluation of the broadphase
/// collision manager has been deemed unnecessary).
template <typename S>
struct DefaultDistanceData {
  DistanceRequest<S> request;
  DistanceResult<S> result;

  /// If `true`, requests that the broadphase evaluation stop.
  bool done{false};
};

/// @brief Provides a simple callback for the distance query in the
/// BroadPhaseCollisionManager. It assumes the `data` parameter is non-null and
/// points to an instance of DefaultDistanceData. It simply invokes the
/// `distance()` method on the culled pair of geometries and stores the results
/// in the data's DistanceResult instance.
///
/// This callback will cause the broadphase evaluation to stop if:
///   - The distance is less than or equal to zero (i.e., the pair is in
///     contact).
///
/// For a given instance of DefaultDistanceData, if broadphase evaluation has
/// already terminated (i.e., DefaultDistanceFunction() returned `true`),
/// subsequent invocations with the same instance of DefaultDistanceData will
/// simply report the previously reported minimum distance and request
/// termination of broadphase evaluation (i.e., return `true`).
///
/// @param o1     The first object in the culled pair.
/// @param o2     The second object in the culled pair.
/// @param data   A non-null pointer to a DefaultDistanceData instance.
/// @param dist   The distance computed by distance().
/// @return `true` if the broadphase evaluation should stop.
/// @tparam S   The scalar type with which the computation will be performed.
template <typename S>
bool DefaultDistanceFunction(CollisionObject<S>* o1, CollisionObject<S>* o2,
                             void* data, S& dist) {
  assert(data != nullptr);
  auto* cdata = static_cast<DefaultDistanceData<S>*>(data);
  const DistanceRequest<S>& request = cdata->request;
  DistanceResult<S>& result = cdata->result;

  if (cdata->done) {
    dist = result.min_distance;
    return true;
  }

  distance(o1, o2, request, result);

  dist = result.min_distance;

  if (dist <= 0) return true;  // in collision or in touch

  return cdata->done;
}

}  // namespace fcl

#endif  // FCL_BROADPHASE_DEFAULTBROADPHASECALLBACKS_H
