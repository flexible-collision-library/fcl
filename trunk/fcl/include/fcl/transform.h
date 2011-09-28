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


#ifndef FCL_TRANSFORM_H
#define FCL_TRANSFORM_H

#include "fcl/vec_3f.h"

namespace fcl
{

/** \brief Quaternion used locally by InterpMotion */
class SimpleQuaternion
{
public:
  /** \brief Default quaternion is identity rotation */
  SimpleQuaternion()
  {
    data[0] = 1;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
  }

  SimpleQuaternion(BVH_REAL a, BVH_REAL b, BVH_REAL c, BVH_REAL d)
  {
    data[0] = a; // w
    data[1] = b; // x
    data[2] = c; // y
    data[3] = d; // z
  }

  /** \brief Matrix to quaternion */
  void fromRotation(const Vec3f R[3]);

  /** \brief Quaternion to matrix */
  void toRotation(Vec3f R[3]) const;

  /** \brief Axes to quaternion */
  void fromAxes(const Vec3f axis[3]);

  /** \brief Axes to matrix */
  void toAxes(Vec3f axis[3]) const;

  /** \brief Axis and angle to quaternion */
  void fromAxisAngle(const Vec3f& axis, BVH_REAL angle);

  /** \brief Quaternion to axis and angle */
  void toAxisAngle(Vec3f& axis, BVH_REAL& angle) const;

  /** \brief Dot product between quaternions */
  BVH_REAL dot(const SimpleQuaternion& other) const;

  /** \brief addition */
  SimpleQuaternion operator + (const SimpleQuaternion& other) const;

  /** \brief minus */
  SimpleQuaternion operator - (const SimpleQuaternion& other) const;

  /** \brief multiplication */
  SimpleQuaternion operator * (const SimpleQuaternion& other) const;

  /** \brief division */
  SimpleQuaternion operator - () const;

  /** \brief scalar multiplication */
  SimpleQuaternion operator * (BVH_REAL t) const;

  /** \brief conjugate */
  SimpleQuaternion conj() const;

  /** \brief inverse */
  SimpleQuaternion inverse() const;

  /** \brief rotate a vector */
  Vec3f transform(const Vec3f& v) const;

  inline const BVH_REAL& getW() const { return data[0]; }
  inline const BVH_REAL& getX() const { return data[1]; }
  inline const BVH_REAL& getY() const { return data[2]; }
  inline const BVH_REAL& getZ() const { return data[3]; }

  inline BVH_REAL& getW() { return data[0]; }
  inline BVH_REAL& getX() { return data[1]; }
  inline BVH_REAL& getY() { return data[2]; }
  inline BVH_REAL& getZ() { return data[3]; }

private:

  BVH_REAL data[4];
};

/** \brief Simple transform class used locally by InterpMotion */
class SimpleTransform
{
  /** \brief Rotation matrix and translation vector */
  Vec3f R[3];
  Vec3f T;

  /** \brief Quaternion representation for R */
  SimpleQuaternion q;

public:

  /** \brief Default transform is no movement */
  SimpleTransform()
  {
    R[0][0] = 1; R[1][1] = 1; R[2][2] = 1;
  }

  SimpleTransform(const Vec3f R_[3], const Vec3f& T_)
  {
    for(int i = 0; i < 3; ++i)
      R[i] = R_[i];
    T = T_;

    q.fromRotation(R_);
  }

  inline const Vec3f& getTranslation() const
  {
    return T;
  }

  inline const Vec3f* getRotation() const
  {
    return R;
  }

  inline const SimpleQuaternion& getQuatRotation() const
  {
    return q;
  }

  inline void setTransform(const Vec3f R_[3], const Vec3f& T_)
  {
    for(int i = 0; i < 3; ++i)
      R[i] = R_[i];
    T = T_;

    q.fromRotation(R_);
  }

  inline void setTransform(const SimpleQuaternion& q_, const Vec3f& T_)
  {
    q = q_;
    T = T_;
    q.toRotation(R);
  }

  inline void setRotation(const Vec3f R_[3])
  {
    for(int i = 0; i < 3; ++i)
      R[i] = R_[i];
    q.fromRotation(R_);
  }

  inline void setTranslation(const Vec3f& T_)
  {
    T = T_;
  }

  inline void setQuatRotation(const SimpleQuaternion& q_)
  {
    q = q_;
    q.toRotation(R);
  }

  Vec3f transform(const Vec3f& v) const
  {
    return q.transform(v) + T;
  }

  bool isIdentity() const
  {
    return (R[0][0] == 1) && (R[0][1] == 0) && (R[0][2] == 0) && (R[1][0] == 0) && (R[1][1] == 1) && (R[1][2] == 0) && (R[2][0] == 0) && (R[2][1] == 0) && (R[2][2] == 1)
        && (T[0] == 0) && (T[1] == 0) && (T[2] == 0);
  }

  void setIdentity()
  {
    R[0] = Vec3f(1, 0, 0);
    R[1] = Vec3f(0, 1, 0);
    R[2] = Vec3f(0, 0, 1);
    T = Vec3f();
    q = SimpleQuaternion();
  }

};


}

#endif
