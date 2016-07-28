/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2014, Willow Garage, Inc.
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

#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shapes_utility.h>
#include <fcl/narrowphase/narrowphase.h>
#include <iostream>
#include <fcl/collision.h>

using namespace std;
using namespace fcl;

int main(int argc, char** argv) 
{
  std::shared_ptr<Box> box0(new Box(1,1,1));
  std::shared_ptr<Box> box1(new Box(1,1,1));
//  GJKSolver_indep solver;
  GJKSolver_libccd solver;
  Vec3f contact_points;
  FCL_REAL penetration_depth;
  Vec3f normal;

  Transform3f tf0, tf1;
  tf0.setIdentity();
  tf0.setTranslation(Vec3f(.9,0,0));
  tf0.setQuatRotation(Quaternion3f(.6, .8, 0, 0));
  tf1.setIdentity();



  bool res = solver.shapeIntersect(*box0, tf0, *box1, tf1, &contact_points, &penetration_depth, &normal);

  cout << "contact points: " << contact_points << endl;
  cout << "pen depth: " << penetration_depth << endl;
  cout << "normal: " << normal << endl;
  cout << "result: " << res << endl;
  
  static const int num_max_contacts = std::numeric_limits<int>::max();
  static const bool enable_contact = true;
  fcl::CollisionResult result;
  fcl::CollisionRequest request(num_max_contacts,
                                enable_contact);


  CollisionObject co0(box0, tf0);
  CollisionObject co1(box1, tf1);

  fcl::collide(&co0, &co1, request, result);
  vector<Contact> contacts;
  result.getContacts(contacts);

  cout << contacts.size() << " contacts found" << endl;
  for(const Contact &contact : contacts) {
    cout << "position: " << contact.pos << endl;
  }
}
