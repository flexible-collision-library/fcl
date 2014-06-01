/*
 *  Software License Agreement (BSD License)
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

/** \author Karsten Knese <Karsten.Knese@googlemail.com> */

#define BOOST_TEST_MODULE "FCL_CAPSULE_CAPSULE"
#define CHECK_CLOSE_TO_0(x, eps) BOOST_CHECK_CLOSE ((x + 1.0), (1.0), (eps))

#include <boost/test/unit_test.hpp>

#include <cmath>
#include <fcl/distance.h>
#include <fcl/math/transform.h>
#include <fcl/collision.h>
#include <fcl/collision_object.h>
#include <fcl/shape/geometric_shapes.h>

using namespace fcl;
typedef boost::shared_ptr <CollisionGeometry> CollisionGeometryPtr_t;

BOOST_AUTO_TEST_CASE(distance_capsulecapsule_origin)
{
  CollisionGeometryPtr_t s1 (new Capsule (5, 10));
  CollisionGeometryPtr_t s2 (new Capsule (5, 10));

  Transform3f tf1;
  Transform3f tf2 (Vec3f(20.1, 0,0));

  CollisionObject o1 (s1, tf1);
  CollisionObject o2 (s2, tf2);

  // Enable computation of nearest points
  DistanceRequest distanceRequest (true);
  DistanceResult distanceResult;

  distance (&o1, &o2, distanceRequest, distanceResult);

  std::cerr << "Applied translation on two capsules";
  std::cerr << " T1 = " << tf1.getTranslation()
	    << ", T2 = " << tf2.getTranslation() << std::endl;
  std::cerr << "Closest points: p1 = " << distanceResult.nearest_points [0]
	    << ", p2 = " << distanceResult.nearest_points [1]
	    << ", distance = " << distanceResult.min_distance << std::endl;

  BOOST_CHECK_CLOSE(distanceResult.min_distance, 10.1, 1e-6);
}

BOOST_AUTO_TEST_CASE(distance_capsulecapsule_transformXY)
{
  CollisionGeometryPtr_t s1 (new Capsule (5, 10));
  CollisionGeometryPtr_t s2 (new Capsule (5, 10));

  Transform3f tf1;
  Transform3f tf2 (Vec3f(20, 20,0));

  CollisionObject o1 (s1, tf1);
  CollisionObject o2 (s2, tf2);

  // Enable computation of nearest points
  DistanceRequest distanceRequest (true);
  DistanceResult distanceResult;

  distance (&o1, &o2, distanceRequest, distanceResult);

  std::cerr << "Applied translation on two capsules";
  std::cerr << " T1 = " << tf1.getTranslation()
	    << ", T2 = " << tf2.getTranslation() << std::endl;
  std::cerr << "Closest points: p1 = " << distanceResult.nearest_points [0]
	    << ", p2 = " << distanceResult.nearest_points [1]
	    << ", distance = " << distanceResult.min_distance << std::endl;

  FCL_REAL expected = sqrt(800) - 10;
  BOOST_CHECK_CLOSE(distanceResult.min_distance, expected, 1e-6);
}

BOOST_AUTO_TEST_CASE(distance_capsulecapsule_transformZ)
{
  CollisionGeometryPtr_t s1 (new Capsule (5, 10));
  CollisionGeometryPtr_t s2 (new Capsule (5, 10));

  Transform3f tf1;
  Transform3f tf2 (Vec3f(0,0,20.1));

  CollisionObject o1 (s1, tf1);
  CollisionObject o2 (s2, tf2);

  // Enable computation of nearest points
  DistanceRequest distanceRequest (true);
  DistanceResult distanceResult;

  distance (&o1, &o2, distanceRequest, distanceResult);

  std::cerr << "Applied translation on two capsules";
  std::cerr << " T1 = " << tf1.getTranslation()
	    << ", T2 = " << tf2.getTranslation() << std::endl;
  std::cerr << "Closest points: p1 = " << distanceResult.nearest_points [0]
	    << ", p2 = " << distanceResult.nearest_points [1]
	    << ", distance = " << distanceResult.min_distance << std::endl;

  BOOST_CHECK_CLOSE(distanceResult.min_distance, 0.1, 1e-6);
}


BOOST_AUTO_TEST_CASE(distance_capsulecapsule_transformZ2)
{
  CollisionGeometryPtr_t s1 (new Capsule (5, 10));
  CollisionGeometryPtr_t s2 (new Capsule (5, 10));

  Transform3f tf1;
  Transform3f tf2 (Quaternion3f (sqrt (2)/2, 0, sqrt (2)/2, 0),
		   Vec3f(0,0,25.1));

  CollisionObject o1 (s1, tf1);
  CollisionObject o2 (s2, tf2);

  // Enable computation of nearest points
  DistanceRequest distanceRequest (true);
  DistanceResult distanceResult;

  distance (&o1, &o2, distanceRequest, distanceResult);

  std::cerr << "Applied rotation and translation on two capsules" << std::endl;
  std::cerr << "R1 = " << tf1.getRotation ()
	    << ", T1 = " << tf1.getTranslation() << std::endl
	    << "R2 = " << tf2.getRotation ()
	    << ", T2 = " << tf2.getTranslation() << std::endl;
  std::cerr << "Closest points: p1 = " << distanceResult.nearest_points [0]
	    << ", p2 = " << distanceResult.nearest_points [1]
	    << ", distance = " << distanceResult.min_distance << std::endl;

  const Vec3f& p1 = distanceResult.nearest_points [0];
  const Vec3f& p2 = distanceResult.nearest_points [1];

  BOOST_CHECK_CLOSE(distanceResult.min_distance, 10.1, 1e-6);
  CHECK_CLOSE_TO_0 (p1 [0], 1e-4);
  CHECK_CLOSE_TO_0 (p1 [1], 1e-4);
  BOOST_CHECK_CLOSE (p1 [2], 10, 1e-4);
  CHECK_CLOSE_TO_0 (p2 [0], 1e-4);
  CHECK_CLOSE_TO_0 (p2 [1], 1e-4);
  BOOST_CHECK_CLOSE (p2 [2], 20.1, 1e-4);
}
