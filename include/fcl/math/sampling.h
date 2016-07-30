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

#ifndef FCL_MATH_SAMPLING_H
#define FCL_MATH_SAMPLING_H

#include <cassert>
#include "fcl/math/constants.h"
#include "fcl/math/rng.h"

namespace fcl
{

template <typename Scalar>
class SamplerBase
{
public:
  mutable RNG<Scalar> rng;
};

template <typename Scalar, std::size_t N>
class SamplerR : public SamplerBase<Scalar>
{
public:
  SamplerR() {}

  SamplerR(const VectorN<Scalar, N>& lower_bound_,
           const VectorN<Scalar, N>& upper_bound_)
    : lower_bound(lower_bound_), upper_bound(upper_bound_)
  {
  }

  void setBound(const VectorN<Scalar, N>& lower_bound_,
                const VectorN<Scalar, N>& upper_bound_)
  {
    lower_bound = lower_bound_;
    upper_bound = upper_bound_;
  }

  void getBound(VectorN<Scalar, N>& lower_bound_,
                VectorN<Scalar, N>& upper_bound_) const
  {
    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;
  }

  VectorN<Scalar, N> sample() const
  {
    VectorN<Scalar, N> q;

    for(std::size_t i = 0; i < N; ++i)
    {
      q[i] = this->rng.uniformReal(lower_bound[i], upper_bound[i]);
    }

    return q;
  }

private:
  VectorN<Scalar, N> lower_bound;
  VectorN<Scalar, N> upper_bound;

};

template <std::size_t N>
using SamplerRf = SamplerR<float, N>;
template <std::size_t N>
using SamplerRd = SamplerR<double, N>;

template <typename Scalar>
class SamplerSE2 : public SamplerBase<Scalar>
{
public:
  SamplerSE2() {}

  SamplerSE2(const VectorN<Scalar, 2>& lower_bound_,
             const VectorN<Scalar, 2>& upper_bound_) : lower_bound(lower_bound_),
                                             upper_bound(upper_bound_)
  {}

  SamplerSE2(Scalar x_min, Scalar x_max,
             Scalar y_min, Scalar y_max) : lower_bound(VectorN<Scalar, 2>(x_min, y_min)),
                                               upper_bound(VectorN<Scalar, 2>(x_max, y_max))
                                               
  {}


  void setBound(const VectorN<Scalar, 2>& lower_bound_,
                const VectorN<Scalar, 2>& upper_bound_)
  {
    lower_bound = lower_bound_;
    upper_bound = upper_bound_;
  }

  void getBound(VectorN<Scalar, 2>& lower_bound_,
                VectorN<Scalar, 2>& upper_bound_) const
  {
    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;
  }


  VectorN<Scalar, 3> sample() const
  {
    VectorN<Scalar, 3> q;
    q[0] = this->rng.uniformReal(lower_bound[0], lower_bound[1]);
    q[1] = this->rng.uniformReal(lower_bound[1], lower_bound[2]);
    q[2] = this->rng.uniformReal(-constants::pi, constants::pi);

    return q;
  }

protected:
  VectorN<Scalar, 2> lower_bound;
  VectorN<Scalar, 2> upper_bound;
};

using SamplerSE2f = SamplerSE2<float>;
using SamplerSE2d = SamplerSE2<double>;

template <typename Scalar>
class SamplerSE2_disk : public SamplerBase<Scalar>
{
public:
  SamplerSE2_disk() {}

  SamplerSE2_disk(Scalar cx, Scalar cy,
                  Scalar r1, Scalar r2,
                  Scalar crefx, Scalar crefy)
  {
    setBound(cx, cy, r1, r2, crefx, crefy);
  }

  void setBound(Scalar cx, Scalar cy,
                Scalar r1, Scalar r2,
                Scalar crefx, Scalar crefy)
  {
    c[0] = cx; c[1] = cy;
    cref[0] = crefx; cref[1] = crefy;
    r_min = r1;
    r_max = r2;
  }

  VectorN<Scalar, 3> sample() const
  {
    VectorN<Scalar, 3> q;
    Scalar x, y;
    this->rng.disk(r_min, r_max, x, y);
    q[0] = x + c[0] - cref[0];
    q[1] = y + c[1] - cref[1];
    q[2] = this->rng.uniformReal(-constants::pi, constants::pi);

    return q;
  }

protected:
  Scalar c[2];
  Scalar cref[2];
  Scalar r_min, r_max;
};

using SamplerSE2_diskf = SamplerSE2_disk<float>;
using SamplerSE2_diskd = SamplerSE2_disk<double>;

template <typename Scalar>
class SamplerSE3Euler : public SamplerBase<Scalar>
{
public:
  SamplerSE3Euler() {}

  SamplerSE3Euler(const VectorN<Scalar, 3>& lower_bound_,
                  const VectorN<Scalar, 3>& upper_bound_) : lower_bound(lower_bound_),
                                                  upper_bound(upper_bound_)
  {}

  void setBound(const VectorN<Scalar, 3>& lower_bound_,
                const VectorN<Scalar, 3>& upper_bound_)

  {
    lower_bound = lower_bound_;
    upper_bound = upper_bound_;
  }

  void getBound(VectorN<Scalar, 3>& lower_bound_,
                VectorN<Scalar, 3>& upper_bound_) const
  {
    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;
  }

  VectorN<Scalar, 6> sample() const
  {
    VectorN<Scalar, 6> q;
    q[0] = this->rng.uniformReal(lower_bound[0], upper_bound[0]);
    q[1] = this->rng.uniformReal(lower_bound[1], upper_bound[1]);
    q[2] = this->rng.uniformReal(lower_bound[2], upper_bound[2]);

    Scalar s[4];
    this->rng.quaternion(s);

    Quaternion3<Scalar> quat(s[0], s[1], s[2], s[3]);
    Vector3<Scalar> angles = quat.toRotationMatrix().eulerAngles(0, 1, 2);

    q[3] = angles[0];
    q[4] = angles[1];
    q[5] = angles[2];

    return q;
  }

protected:
  VectorN<Scalar, 3> lower_bound;
  VectorN<Scalar, 3> upper_bound;
  
};

using SamplerSE3Eulerf = SamplerSE3Euler<float>;
using SamplerSE3Eulerd = SamplerSE3Euler<double>;

template <typename Scalar>
class SamplerSE3Quat : public SamplerBase<Scalar>
{
public:
  SamplerSE3Quat() {}

  SamplerSE3Quat(const VectorN<Scalar, 3>& lower_bound_,
                 const VectorN<Scalar, 3>& upper_bound_) : lower_bound(lower_bound_),
                                                 upper_bound(upper_bound_)
  {}

  void setBound(const VectorN<Scalar, 3>& lower_bound_,
                const VectorN<Scalar, 3>& upper_bound_)

  {
    lower_bound = lower_bound_;
    upper_bound = upper_bound_;
  }

  void getBound(VectorN<Scalar, 3>& lower_bound_,
                VectorN<Scalar, 3>& upper_bound_) const
  {
    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;
  }

  VectorN<Scalar, 7> sample() const
  {
    VectorN<Scalar, 7> q;
    q[0] = this->rng.uniformReal(lower_bound[0], upper_bound[0]);
    q[1] = this->rng.uniformReal(lower_bound[1], upper_bound[1]);
    q[2] = this->rng.uniformReal(lower_bound[2], upper_bound[2]);

    Scalar s[4];
    this->rng.quaternion(s);

    q[3] = s[0];
    q[4] = s[1];
    q[5] = s[2];
    q[6] = s[3];
    return q;
  }

protected:
  VectorN<Scalar, 3> lower_bound;
  VectorN<Scalar, 3> upper_bound;
};

using SamplerSE3Quatf = SamplerSE3Quat<float>;
using SamplerSE3Quatd = SamplerSE3Quat<double>;

template <typename Scalar>
class SamplerSE3Euler_ball : public SamplerBase<Scalar>
{
public:
  SamplerSE3Euler_ball() {}

  SamplerSE3Euler_ball(Scalar r_) : r(r_)
  {
  }

  void setBound(const Scalar& r_)
  {
    r = r_;
  }
  
  void getBound(Scalar& r_) const
  {
    r_ = r;
  }

  VectorN<Scalar, 6> sample() const
  {
    VectorN<Scalar, 6> q;
    Scalar x, y, z;
    this->rng.ball(0, r, x, y, z);
    q[0] = x;
    q[1] = y;
    q[2] = z;

    Scalar s[4];
    this->rng.quaternion(s);

    Quaternion3<Scalar> quat(s[0], s[1], s[2], s[3]);
    Vector3<Scalar> angles = quat.toRotationMatrix().eulerAngles(0, 1, 2);
    q[3] = angles[0];
    q[4] = angles[1];
    q[5] = angles[2];
    
    return q;
  }

protected:
  Scalar r;

};

using SamplerSE3Euler_ballf = SamplerSE3Euler_ball<float>;
using SamplerSE3Euler_balld = SamplerSE3Euler_ball<double>;

template <typename Scalar>
class SamplerSE3Quat_ball : public SamplerBase<Scalar>
{
public:
  SamplerSE3Quat_ball() {}

  SamplerSE3Quat_ball(Scalar r_) : r(r_)
  {}

  void setBound(const Scalar& r_)
  {
    r = r_;
  }

  void getBound(Scalar& r_) const
  {
    r_ = r;
  }

  VectorN<Scalar, 7> sample() const
  {
    VectorN<Scalar, 7> q;
    Scalar x, y, z;
    this->rng.ball(0, r, x, y, z);
    q[0] = x;
    q[1] = y;
    q[2] = z;

    Scalar s[4];
    this->rng.quaternion(s);

    q[3] = s[0];
    q[4] = s[1];
    q[5] = s[2];
    q[6] = s[3];
    return q;
  }

protected:
  Scalar r;
};

using SamplerSE3Quat_ballf = SamplerSE3Quat_ball<float>;
using SamplerSE3Quat_balld = SamplerSE3Quat_ball<double>;

}

#endif
