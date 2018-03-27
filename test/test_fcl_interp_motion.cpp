/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  Copyright (c) 2016, Toyota Research Institute
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

#include <gtest/gtest.h>

#include "fcl/config.h"
#include "fcl/math/bv/AABB.h"
#include "fcl/math/motion/interp_motion.h"

using namespace fcl;

template <typename S>
void test_interp_motion_empty()
{
  Transform3<S> from;
  from.setIdentity();
  Transform3<S> to;
  to.setIdentity();

  InterpMotion<S> interp_motion;

  Vector3<S> linear_axis = interp_motion.getLinearAxis();
  EXPECT_TRUE(linear_axis == Vector3<S>(0.0, 0.0, 0.0));
  S linear_vel = interp_motion.getLinearVelocity();
  EXPECT_TRUE(linear_vel == 0);

  Vector3<S> angular_axis = interp_motion.getAngularAxis();
  EXPECT_TRUE(angular_axis == Vector3<S>(0.0, 0.0, 0.0));
  S angular_vel = interp_motion.getAngularVelocity();
  EXPECT_TRUE(angular_vel == 0);

  Vector3<S> ref_p = interp_motion.getReferencePoint();
  EXPECT_TRUE(ref_p == Vector3<S>(0.0, 0.0, 0.0));

  Transform3<S> itf; itf.setIdentity();
  Transform3<S> ctf;

  interp_motion.integrate(0);
  interp_motion.getCurrentTransform(ctf);
  EXPECT_TRUE(itf.translation() == ctf.translation());
  EXPECT_TRUE(itf.rotation() == ctf.rotation());

  interp_motion.integrate(0.4);
  interp_motion.getCurrentTransform(ctf);
  EXPECT_TRUE(itf.translation() == ctf.translation());
  EXPECT_TRUE(itf.rotation() == ctf.rotation());

  interp_motion.integrate(1);
  interp_motion.getCurrentTransform(ctf);
  EXPECT_TRUE(itf.translation() == ctf.translation());
  EXPECT_TRUE(itf.rotation() == ctf.rotation());

  // NOTE: There is a limit for dt > 1, but not for dt < 0
  // interp_motion.integrate(-.5);
  // interp_motion.getCurrentTransform(ctf);
  // EXPECT_TRUE(itf.translation() == ctf.translation());
  // EXPECT_TRUE(itf.rotation() == ctf.rotation());

  interp_motion.integrate(2.25);
  interp_motion.getCurrentTransform(ctf);
  EXPECT_TRUE(itf.translation() == ctf.translation());
  EXPECT_TRUE(itf.rotation() == ctf.rotation());

  // TODO computeMotionBound
  // TODO getTaylorModel
}


template <typename S>
void test_interp_motion_only_translation()
{
  Transform3<S> from;
  from.setIdentity();
  Transform3<S> to;
  to.setIdentity();
  to.translation()[0] = 3;
  to.translation()[1] = 4;
  to.translation()[2] = 0;

  InterpMotion<S> interp_motion(from, to);

  Vector3<S> linear_axis = interp_motion.getLinearAxis();
  S linear_vel = interp_motion.getLinearVelocity();
  EXPECT_TRUE(linear_axis == Vector3<S>(0.6, 0.8, 0.0));
  EXPECT_TRUE(linear_vel == 5);

  Vector3<S> angular_axis = interp_motion.getAngularAxis();
  S angular_vel = interp_motion.getAngularVelocity();
  EXPECT_TRUE(angular_axis == Vector3<S>(1.0, 0.0, 0.0));
  EXPECT_TRUE(angular_vel == 0);

  Vector3<S> ref_p = interp_motion.getReferencePoint();
  EXPECT_TRUE(ref_p == Vector3<S>(0.0, 0.0, 0.0));

  Transform3<S> ctf;

  interp_motion.integrate(0);
  interp_motion.getCurrentTransform(ctf);
  EXPECT_TRUE(ctf.translation() == from.translation());
  EXPECT_TRUE(ctf.rotation() == from.rotation());

  interp_motion.integrate(0.25);
  interp_motion.getCurrentTransform(ctf);
  EXPECT_TRUE(ctf.translation() == Vector3<S>(0.75,1,0.0));
  EXPECT_TRUE(ctf.rotation() == from.rotation());

  interp_motion.integrate(1);
  interp_motion.getCurrentTransform(ctf);
  EXPECT_TRUE(ctf.translation() == to.translation());
  EXPECT_TRUE(ctf.rotation() == from.rotation());

  // NOTE: There is a limit for dt > 1, but not for dt < 0
  // interp_motion.integrate(-.5);
  // interp_motion.getCurrentTransform(ctf);
  // EXPECT_TRUE(ctf.translation() == from.translation());
  // EXPECT_TRUE(ctf.rotation() == from.rotation());

  interp_motion.integrate(2.25);
  interp_motion.getCurrentTransform(ctf);
  EXPECT_TRUE(ctf.translation() == to.translation());
  EXPECT_TRUE(ctf.rotation() == to.rotation());

  // TODO computeMotionBound
  // TODO getTaylorModel
}

template <typename S>
void test_interp_motion_only_rotation()
{
  Transform3<S> from;
  from.setIdentity();
  Vector3<S> up(0.0, 0.0, 1.0);
  AngleAxis<S> aapi(M_PI, up);
  AngleAxis<S> aapih(M_PI/2, up);
  Transform3<S> to(aapi);

  InterpMotion<S> interp_motion(from, to);

  Vector3<S> linear_axis = interp_motion.getLinearAxis();
  S linear_vel = interp_motion.getLinearVelocity();
  EXPECT_TRUE(linear_axis == Vector3<S>(0.0, 0.0, 0.0));
  EXPECT_TRUE(linear_vel == 0);

  Vector3<S> angular_axis = interp_motion.getAngularAxis();
  S angular_vel = interp_motion.getAngularVelocity();
  EXPECT_TRUE(angular_axis == up);
  EXPECT_TRUE(angular_vel == M_PI);

  Vector3<S> ref_p = interp_motion.getReferencePoint();
  EXPECT_TRUE(ref_p == Vector3<S>(0.0, 0.0, 0.0));

  Transform3<S> ctf;

  interp_motion.integrate(0);
  interp_motion.getCurrentTransform(ctf);
  EXPECT_TRUE(ctf.translation() == from.translation());
  EXPECT_TRUE(ctf.rotation() == from.rotation());

  interp_motion.integrate(0.5);
  interp_motion.getCurrentTransform(ctf);
  EXPECT_TRUE(ctf.translation() == from.translation());
  Quaternion<S> ctfq; ctfq = ctf.rotation(); ctfq.normalize();
  Quaternion<S> mpihq; mpihq = aapih; mpihq.normalize();
  EXPECT_TRUE(ctfq.isApprox(mpihq));

  interp_motion.integrate(1);
  interp_motion.getCurrentTransform(ctf);
  EXPECT_TRUE(ctf.translation() == to.translation());
  EXPECT_TRUE(ctf.rotation() == to.rotation());

  // NOTE: There is a limit for dt > 1, but not for dt < 0
  // interp_motion.integrate(-.5);
  // interp_motion.getCurrentTransform(ctf);
  // EXPECT_TRUE(ctf.translation() == from.translation());
  // EXPECT_TRUE(ctf.rotation() == from.rotation());

  interp_motion.integrate(2.25);
  interp_motion.getCurrentTransform(ctf);
  EXPECT_TRUE(ctf.translation() == to.translation());
  EXPECT_TRUE(ctf.rotation() == to.rotation());

  // TODO computeMotionBound
  // TODO getTaylorModel
}

template <typename S>
void test_interp_motion_both()
{
  Transform3<S> from;
  from.setIdentity();
  Vector3<S> up(0.0, 0.0, 1.0);
  AngleAxis<S> aapi(M_PI, up);
  AngleAxis<S> aapih(M_PI/2, up);
  Transform3<S> to(aapi);
  to.translation()[0] = 3;
  to.translation()[1] = 4;
  to.translation()[2] = 0;

  InterpMotion<S> interp_motion(from, to);

  Vector3<S> linear_axis = interp_motion.getLinearAxis();
  S linear_vel = interp_motion.getLinearVelocity();
  EXPECT_TRUE(linear_axis == Vector3<S>(0.6, 0.8, 0.0));
  EXPECT_TRUE(linear_vel == 5);

  Vector3<S> angular_axis = interp_motion.getAngularAxis();
  S angular_vel = interp_motion.getAngularVelocity();
  EXPECT_TRUE(angular_axis == up);
  EXPECT_TRUE(angular_vel == M_PI);

  Vector3<S> ref_p = interp_motion.getReferencePoint();
  EXPECT_TRUE(ref_p == Vector3<S>(0.0, 0.0, 0.0));

  Transform3<S> ctf;

  interp_motion.integrate(0);
  interp_motion.getCurrentTransform(ctf);
  EXPECT_TRUE(ctf.translation() == from.translation());
  EXPECT_TRUE(ctf.rotation() == from.rotation());

  interp_motion.integrate(0.5);
  interp_motion.getCurrentTransform(ctf);
  EXPECT_TRUE(ctf.translation() == Vector3<S>(1.5,2.0,0.0));
  Quaternion<S> ctfq; ctfq = ctf.rotation(); ctfq.normalize();
  Quaternion<S> mpihq; mpihq = aapih; mpihq.normalize();
  EXPECT_TRUE(ctfq.isApprox(mpihq));

  interp_motion.integrate(1);
  interp_motion.getCurrentTransform(ctf);
  EXPECT_TRUE(ctf.translation() == to.translation());
  EXPECT_TRUE(ctf.rotation() == to.rotation());

  // NOTE: There is a limit for dt > 1, but not for dt < 0
  // interp_motion.integrate(-.5);
  // interp_motion.getCurrentTransform(ctf);
  // EXPECT_TRUE(ctf.translation() == from.translation());
  // EXPECT_TRUE(ctf.rotation() == from.rotation());

  interp_motion.integrate(2.25);
  interp_motion.getCurrentTransform(ctf);
  EXPECT_TRUE(ctf.translation() == to.translation());
  EXPECT_TRUE(ctf.rotation() == to.rotation());

  // TODO computeMotionBound
  // TODO getTaylorModel
}


// TODO: The other constructors

GTEST_TEST(FCL_MATH, interp_motion_test_motion)
{
  // test_interp_motion_empty<float>();
  test_interp_motion_empty<double>();

  // test_interp_motion_only_translation<float>();
  test_interp_motion_only_translation<double>();

  // test_interp_motion_only_rotation<float>();
  test_interp_motion_only_rotation<double>();

  // test_interp_motion_both<float>();
  test_interp_motion_both<double>();
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
