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

#ifndef FCL_SIMD_INTERSECT
#define FCL_SIMD_INTERSECT

#include "fcl/vec_3f.h"

namespace fcl
{

/** \brief four spheres versus four spheres SIMD test */
bool four_spheres_intersect_and(const Vec3f& o1, BVH_REAL r1,
                                const Vec3f& o2, BVH_REAL r2,
                                const Vec3f& o3, BVH_REAL r3,
                                const Vec3f& o4, BVH_REAL r4,
                                const Vec3f& o5, BVH_REAL r5,
                                const Vec3f& o6, BVH_REAL r6,
                                const Vec3f& o7, BVH_REAL r7,
                                const Vec3f& o8, BVH_REAL r8);

bool four_spheres_intersect_or(const Vec3f& o1, BVH_REAL r1,
                               const Vec3f& o2, BVH_REAL r2,
                               const Vec3f& o3, BVH_REAL r3,
                               const Vec3f& o4, BVH_REAL r4,
                               const Vec3f& o5, BVH_REAL r5,
                               const Vec3f& o6, BVH_REAL r6,
                               const Vec3f& o7, BVH_REAL r7,
                               const Vec3f& o8, BVH_REAL r8);

/** \brief four spheres versus four AABBs SIMD test */
bool four_spheres_four_AABBs_intersect_and(const Vec3f& o1, BVH_REAL r1,
                                           const Vec3f& o2, BVH_REAL r2,
                                           const Vec3f& o3, BVH_REAL r3,
                                           const Vec3f& o4, BVH_REAL r4,
                                           const Vec3f& min1, const Vec3f& max1,
                                           const Vec3f& min2, const Vec3f& max2,
                                           const Vec3f& min3, const Vec3f& max3,
                                           const Vec3f& min4, const Vec3f& max4);

bool four_spheres_four_AABBs_intersect_or(const Vec3f& o1, BVH_REAL r1,
                                          const Vec3f& o2, BVH_REAL r2,
                                          const Vec3f& o3, BVH_REAL r3,
                                          const Vec3f& o4, BVH_REAL r4,
                                          const Vec3f& min1, const Vec3f& max1,
                                          const Vec3f& min2, const Vec3f& max2,
                                          const Vec3f& min3, const Vec3f& max3,
                                          const Vec3f& min4, const Vec3f& max4);

/** \brief four AABBs versus four AABBs SIMD test */
bool four_AABBs_intersect_and(const Vec3f& min1, const Vec3f& max1,
                              const Vec3f& min2, const Vec3f& max2,
                              const Vec3f& min3, const Vec3f& max3,
                              const Vec3f& min4, const Vec3f& max4,
                              const Vec3f& min5, const Vec3f& max5,
                              const Vec3f& min6, const Vec3f& max6,
                              const Vec3f& min7, const Vec3f& max7,
                              const Vec3f& min8, const Vec3f& max8);

bool four_AABBs_intersect_or(const Vec3f& min1, const Vec3f& max1,
                             const Vec3f& min2, const Vec3f& max2,
                             const Vec3f& min3, const Vec3f& max3,
                             const Vec3f& min4, const Vec3f& max4,
                             const Vec3f& min5, const Vec3f& max5,
                             const Vec3f& min6, const Vec3f& max6,
                             const Vec3f& min7, const Vec3f& max7,
                             const Vec3f& min8, const Vec3f& max8);

}

#endif
