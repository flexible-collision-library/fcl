/*
 *  Software License Agreement (BSD License)
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

/** \author Karsten Knese <Karsten.Knese@googlemail.com> */

#define BOOST_TEST_MODULE "FCL_CAPSULE_CAPSULE"
#include <boost/test/unit_test.hpp>

#include "fcl/math/constants.h"
#include "fcl/collision.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"

#include <cmath>
using namespace fcl;

BOOST_AUTO_TEST_CASE(distance_capsulecapsule_origin)
{

  GJKSolver_indep solver;
  Capsule s1(5, 10);
  Capsule s2(5, 10);

  Vec3f closest_p1, closest_p2;

  Transform3f transform;
  Transform3f transform2;
  transform2 = Transform3f(Vec3f(20.1, 0,0));

  bool res;
  FCL_REAL dist;

  res = solver.shapeDistance<Capsule, Capsule>(s1, transform, s2, transform2, &dist, &closest_p1, &closest_p2);
  std::cerr << "applied transformation of two caps: " << transform.getTranslation() << " & " << transform2.getTranslation() << std::endl;
  std::cerr << "computed points in caps to caps" << closest_p1 << " & " << closest_p2 << "with dist: " << dist << std::endl;

  BOOST_CHECK(std::abs(dist - 10.1) < 0.001);
  BOOST_CHECK(res);

}


BOOST_AUTO_TEST_CASE(distance_capsulecapsule_transformXY)
{

  GJKSolver_indep solver;
  Capsule s1(5, 10);
  Capsule s2(5, 10);

  Vec3f closest_p1, closest_p2;

  Transform3f transform;
  Transform3f transform2;
  transform2 = Transform3f(Vec3f(20, 20,0));

  bool res;
  FCL_REAL dist;

  res = solver.shapeDistance<Capsule, Capsule>(s1, transform, s2, transform2, &dist, &closest_p1, &closest_p2);
  std::cerr << "applied transformation of two caps: " << transform.getTranslation() << " & " << transform2.getTranslation() << std::endl;
  std::cerr << "computed points in caps to caps" << closest_p1 << " & " << closest_p2 << "with dist: " << dist << std::endl;

  FCL_REAL expected = std::sqrt(FCL_REAL(800)) - 10;
  BOOST_CHECK(std::abs(expected-dist) < 0.01);
  BOOST_CHECK(res);

}

BOOST_AUTO_TEST_CASE(distance_capsulecapsule_transformZ)
{

  GJKSolver_indep solver;
  Capsule s1(5, 10);
  Capsule s2(5, 10);

  Vec3f closest_p1, closest_p2;

  Transform3f transform;
  Transform3f transform2;
  transform2 = Transform3f(Vec3f(0,0,20.1));

  bool res;
  FCL_REAL dist;

  res = solver.shapeDistance<Capsule, Capsule>(s1, transform, s2, transform2, &dist, &closest_p1, &closest_p2);
  std::cerr << "applied transformation of two caps: " << transform.getTranslation() << " & " << transform2.getTranslation() << std::endl;
  std::cerr << "computed points in caps to caps" << closest_p1 << " & " << closest_p2 << "with dist: " << dist << std::endl;

  BOOST_CHECK(std::abs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

}


BOOST_AUTO_TEST_CASE(distance_capsulecapsule_transformZ2)
{
  const FCL_REAL Pi = constants::pi;

  GJKSolver_indep solver;
  Capsule s1(5, 10);
  Capsule s2(5, 10);

  Vec3f closest_p1, closest_p2;

  Transform3f transform;
  Transform3f transform2;
  transform2 = Transform3f(Vec3f(0,0,25.1));
  Matrix3f rot2;

  rot2.setEulerZYX(0,Pi/2,0);
  transform2.setRotation(rot2);

  bool res;
  FCL_REAL dist;

  res = solver.shapeDistance<Capsule, Capsule>(s1, transform, s2, transform2, &dist, &closest_p1, &closest_p2);
  std::cerr << "applied transformation of two caps: " << transform.getTranslation() << " & " << transform2.getTranslation() << std::endl;
  std::cerr << "applied transformation of two caps: " << transform.getRotation() << " & " << transform2.getRotation() << std::endl;
  std::cerr << "computed points in caps to caps" << closest_p1 << " & " << closest_p2 << "with dist: " << dist << std::endl;

  BOOST_CHECK(std::abs(dist - 5.1) < 0.001);
  BOOST_CHECK(res);

}
