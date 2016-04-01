/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
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

#ifndef FCL_VEC_3F_H
#define FCL_VEC_3F_H

#include "fcl/config.h"
#include "fcl/data_types.h"

#if FCL_HAVE_EIGEN
#  include "fcl/eigen/fcl_matrix.h"
#elif FCL_HAVE_SSE
#  include "fcl/simd/math_simd_details.h"
#  include "fcl/math/vec_3fx.h"
#else
#  include "fcl/math/math_details.h"
#  include "fcl/math/vec_3fx.h"
#endif

#include <cmath>
#include <iostream>
#include <limits>


namespace fcl
{


#if FCL_HAVE_EIGEN
  typedef Eigen::FclMatrix<FCL_REAL, 1> Vec3f;
#elif FCL_HAVE_SSE
  typedef Vec3fX<details::sse_meta_f4> Vec3f;
#else
  typedef Vec3fX<details::Vec3Data<FCL_REAL> > Vec3f;
#endif

static inline std::ostream& operator << (std::ostream& o, const Vec3f& v)
{
  o << "(" << v[0] << " " << v[1] << " " << v[2] << ")";
  return o;
}

}


#endif
