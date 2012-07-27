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
#include "fcl/matrix_3f.h"

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

  SimpleQuaternion(FCL_REAL a, FCL_REAL b, FCL_REAL c, FCL_REAL d)
  {
    data[0] = a;
    data[1] = b;
    data[2] = c;
    data[3] = d;
  }

  bool isIdentity() const
  {
    return (data[0] == 1) && (data[1] == 0) && (data[2] == 0) && (data[3] == 0);
  }

  /** \brief Matrix to quaternion */
  void fromRotation(const Matrix3f& R);

  /** \brief Quaternion to matrix */
  void toRotation(Matrix3f& R) const;

  /** \brief Axes to quaternion */
  void fromAxes(const Vec3f axis[3]);

  /** \brief Axes to matrix */
  void toAxes(Vec3f axis[3]) const;

  /** \brief Axis and angle to quaternion */
  void fromAxisAngle(const Vec3f& axis, FCL_REAL angle);

  /** \brief Quaternion to axis and angle */
  void toAxisAngle(Vec3f& axis, FCL_REAL& angle) const;

  /** \brief Dot product between quaternions */
  FCL_REAL dot(const SimpleQuaternion& other) const;

  /** \brief addition */
  SimpleQuaternion operator + (const SimpleQuaternion& other) const;
  const SimpleQuaternion& operator += (const SimpleQuaternion& other);

  /** \brief minus */
  SimpleQuaternion operator - (const SimpleQuaternion& other) const;
  const SimpleQuaternion& operator -= (const SimpleQuaternion& other);

  /** \brief multiplication */
  SimpleQuaternion operator * (const SimpleQuaternion& other) const;
  const SimpleQuaternion& operator *= (const SimpleQuaternion& other);

  /** \brief division */
  SimpleQuaternion operator - () const;

  /** \brief scalar multiplication */
  SimpleQuaternion operator * (FCL_REAL t) const;
  const SimpleQuaternion& operator *= (FCL_REAL t);

  /** \brief conjugate */
  SimpleQuaternion& conj();

  /** \brief inverse */
  SimpleQuaternion& inverse();

  /** \brief rotate a vector */
  Vec3f transform(const Vec3f& v) const;

  inline const FCL_REAL& getW() const { return data[0]; }
  inline const FCL_REAL& getX() const { return data[1]; }
  inline const FCL_REAL& getY() const { return data[2]; }
  inline const FCL_REAL& getZ() const { return data[3]; }

  inline FCL_REAL& getW() { return data[0]; }
  inline FCL_REAL& getX() { return data[1]; }
  inline FCL_REAL& getY() { return data[2]; }
  inline FCL_REAL& getZ() { return data[3]; }

private:

  FCL_REAL data[4];
};

SimpleQuaternion conj(const SimpleQuaternion& q);
SimpleQuaternion inverse(const SimpleQuaternion& q);

/** \brief Simple transform class used locally by InterpMotion */
class SimpleTransform
{
  /** \brief Whether matrix cache is set */
  mutable bool matrix_set;
  /** \brief Matrix cache */
  mutable Matrix3f R;


  /** \brief Tranlation vector */
  Vec3f T;

  /** \brief Rotation*/
  SimpleQuaternion q;

public:

  /** \brief Default transform is no movement */
  SimpleTransform()
  {
    setIdentity();
  }

  SimpleTransform(const Matrix3f& R_, const Vec3f& T_)
  {
    R = R_;
    T = T_;

    q.fromRotation(R_);

    matrix_set = true;
  }

  SimpleTransform(const SimpleQuaternion& q_, const Vec3f& T_)
  {
    // q_.toRotation(R);
    T = T_;
    q = q_;

    matrix_set = false;
  }

  SimpleTransform(const Matrix3f& R_)
  {
    R = R_;
    q.fromRotation(R_);
    
    matrix_set = true;
  }

  SimpleTransform(const SimpleQuaternion& q_)
  {
    // q_.toRotation(R);
    q = q_;
    
    matrix_set = false;
  }

  SimpleTransform(const Vec3f& T_)
  {
    T = T_;
    R.setIdentity();
  }

  inline const Vec3f& getTranslation() const
  {
    return T;
  }

  inline const Matrix3f& getRotation() const
  {
    if(!matrix_set)
    {
      q.toRotation(R);
      matrix_set = true;
    }
    
    return R;
  }

  inline const SimpleQuaternion& getQuatRotation() const
  {
    return q;
  }

  inline void setTransform(const Matrix3f& R_, const Vec3f& T_)
  {
    matrix_set = true;
    R = R_;
    T = T_;

    q.fromRotation(R_);
  }

  inline void setTransform(const SimpleQuaternion& q_, const Vec3f& T_)
  {
    matrix_set = false;
    q = q_;
    T = T_;
    // q.toRotation(R);
  }

  inline void setRotation(const Matrix3f& R_)
  {
    matrix_set = true;
    R = R_;
    q.fromRotation(R_);
  }

  inline void setTranslation(const Vec3f& T_)
  {
    T = T_;
  }

  inline void setQuatRotation(const SimpleQuaternion& q_)
  {
    matrix_set = false;
    q = q_;
    // q.toRotation(R);
  }

  Vec3f transform(const Vec3f& v) const
  {
    return q.transform(v) + T;
  }

  SimpleTransform& inverse()
  {
    matrix_set = false;
    q.conj();
    // R.transpose();
    T = q.transform(-T);
    return *this;
  }

  SimpleTransform inverseTimes(const SimpleTransform& other) const
  {
    const SimpleQuaternion& q_inv = fcl::conj(q);
    return SimpleTransform(q_inv * other.q, q_inv.transform(other.T - T));
  }

  const SimpleTransform& operator *= (const SimpleTransform& other)
  {
    matrix_set = false;
    T = q.transform(other.T) + T;
    q *= other.q;
    // q.toRotation(R);
    return *this;
  }

  SimpleTransform operator * (const SimpleTransform& other) const
  {
    SimpleQuaternion q_new = q * other.q;
    return SimpleTransform(q_new, q.transform(other.T) + T);
  }

  bool isIdentity() const
  {
    return q.isIdentity() && T.isZero();
  }

  void setIdentity()
  {
    R.setIdentity();
    T.setValue(0);
    q = SimpleQuaternion();
    matrix_set = true;
  }

};

SimpleTransform inverse(const SimpleTransform& tf);

void relativeTransform(const SimpleTransform& tf1, const SimpleTransform& tf2,
                       SimpleTransform& tf);

}

#endif
