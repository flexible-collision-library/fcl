/*
 * Software License Agreement (BSD License)
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

/** \author Jia Pan */

#ifndef FCL_MULTIPLE_INTERSECT
#define FCL_MULTIPLE_INTERSECT

#include <xmmintrin.h>
#include <pmmintrin.h>


namespace fcl
{


static inline __m128 sse_four_spheres_intersect(const Vector3d& o1, FCL_REAL r1,
                                                const Vector3d& o2, FCL_REAL r2,
                                                const Vector3d& o3, FCL_REAL r3,
                                                const Vector3d& o4, FCL_REAL r4,
                                                const Vector3d& o5, FCL_REAL r5,
                                                const Vector3d& o6, FCL_REAL r6,
                                                const Vector3d& o7, FCL_REAL r7,
                                                const Vector3d& o8, FCL_REAL r8)
{
  __m128 PX, PY, PZ, PR, QX, QY, QZ, QR;
  PX = _mm_set_ps(o1[0], o2[0], o3[0], o4[0]);
  PY = _mm_set_ps(o1[1], o2[1], o3[1], o4[1]);
  PZ = _mm_set_ps(o1[2], o2[2], o3[2], o4[2]);
  PR = _mm_set_ps(r1, r2, r3, r4);
  QX = _mm_set_ps(o5[0], o6[0], o7[0], o8[0]);
  QY = _mm_set_ps(o5[1], o6[1], o7[1], o8[1]);
  QZ = _mm_set_ps(o5[2], o6[2], o7[2], o8[2]);
  QR = _mm_set_ps(r5, r6, r7, r8);

  __m128 T1 = _mm_sub_ps(PX, QX);
  __m128 T2 = _mm_sub_ps(PY, QY);
  __m128 T3 = _mm_sub_ps(PZ, QZ);
  __m128 T4 = _mm_add_ps(PR, QR);
  T1 = _mm_mul_ps(T1, T1);
  T2 = _mm_mul_ps(T2, T2);
  T3 = _mm_mul_ps(T3, T3);
  T4 = _mm_mul_ps(T4, T4);
  T1 = _mm_add_ps(T1, T2);
  T2 = _mm_sub_ps(R2, T3);
  return _mm_cmple_ps(T1, T2);
}


static inline __m128 sse_four_spheres_four_AABBs_intersect(const Vector3d& o1, FCL_REAL r1,
                                                           const Vector3d& o2, FCL_REAL r2,
                                                           const Vector3d& o3, FCL_REAL r3,
                                                           const Vector3d& o4, FCL_REAL r4,
                                                           const Vector3d& min1, const Vector3d& max1,
                                                           const Vector3d& min2, const Vector3d& max2,
                                                           const Vector3d& min3, const Vector3d& max3,
                                                           const Vector3d& min4, const Vector3d& max4)
{
  __m128 MINX, MINY, MINZ, MAXX, MAXX, MAXY, MAXZ, SX, SY, SZ, SR;
  MINX = _mm_set_ps(min1[0], min2[0], min3[0], min4[0]);
  MAXX = _mm_set_ps(max1[0], max2[0], max3[0], max4[0]);
  MINY = _mm_set_ps(min1[1], min2[1], min3[1], min4[1]);
  MAXY = _mm_set_ps(max1[1], max2[1], max3[1], max4[1]);
  MINZ = _mm_set_ps(min1[2], min2[2], min3[2], min4[2]);
  MAXZ = _mm_set_ps(max1[2], max2[2], max3[2], max4[2]);
  SX = _mm_set_ps(o1[0], o2[0], o3[0], o4[0]);
  SY = _mm_set_ps(o1[1], o2[1], o3[1], o4[1]);
  SZ = _mm_set_ps(o1[2], o2[2], o3[2], o4[2]);
  SR = _mm_set_ps(r1, r2, r3, r4);


  __m128 TX = _mm_max_ps(SX, MINX);
  __m128 TY = _mm_max_ps(SY, MINY);
  __m128 TZ = _mm_max_ps(SZ, MINZ);
  TX = _mm_min_ps(TX, MAXX);
  TY = _mm_min_ps(TY, MAXY);
  TZ = _mm_min_ps(TZ, MAXZ);
  __m128 DX = _mm_sub_ps(SX, TX);
  __m128 DY = _mm_sub_ps(SY, TY);
  __m128 DZ = _mm_sub_ps(SZ, TZ);
  __m128 R2 = _mm_mul_ps(SR, SR);
  DX = _mm_mul_ps(DX, DX);
  DY = _mm_mul_ps(DY, DY);
  DZ = _mm_mul_ps(DZ, DZ);
  __m128 T1 = _mm_add_ps(DX, DY);
  __m128 T2 = _mm_sub_ps(R2, DZ);
  return _mm_cmple_ps(T1, T2);  
}


static inline __m128 sse_four_AABBs_intersect(const Vector3d& min1, const Vector3d& max1,
                                              const Vector3d& min2, const Vector3d& max2,
                                              const Vector3d& min3, const Vector3d& max3,
                                              const Vector3d& min4, const Vector3d& max4,
                                              const Vector3d& min5, const Vector3d& max5,
                                              const Vector3d& min6, const Vector3d& max6,
                                              const Vector3d& min7, const Vector3d& max7,
                                              const Vector3d& min8, const Vector3d& max8)
{
  __m128 MIN1X, MIN1Y, MIN1Z, MAX1X, MAX1Y, MAX1Z, MIN2X, MIN2Y, MIN2Z, MAX2X, MAX2Y, MAX2Z;
  MIN1X = _mm_set_ps(min1[0], min2[0], min3[0], min4[0]);
  MAX1X = _mm_set_ps(max1[0], max2[0], max3[0], max4[0]);
  MIN1Y = _mm_set_ps(min1[1], min2[1], min3[1], min4[1]);
  MAX1Y = _mm_set_ps(max1[1], max2[1], max3[1], max4[1]);
  MIN1Z = _mm_set_ps(min1[2], min2[2], min3[2], min4[2]);
  MAX1Z = _mm_set_ps(max1[2], max2[2], max3[2], max4[2]);
  MIN2X = _mm_set_ps(min5[0], min6[0], min7[0], min8[0]);
  MAX2X = _mm_set_ps(max5[0], max6[0], max7[0], max8[0]);
  MIN2Y = _mm_set_ps(min5[1], min6[1], min7[1], min8[1]);
  MAX2Y = _mm_set_ps(max5[1], max6[1], max7[1], max8[1]);
  MIN2Z = _mm_set_ps(min5[2], min6[2], min7[2], min8[2]);
  MAX2Z = _mm_set_ps(max5[2], max6[2], max7[2], max8[2]);

  __m128 AX = _mm_max_ps(MIN1X, MIN2X);
  __m128 BX = _mm_min_ps(MAX1X, MAX2X);
  __m128 AY = _mm_max_ps(MIN1Y, MIN2Y);
  __m128 BY = _mm_min_ps(MAX1Y, MAX2Y);
  __m128 AZ = _mm_max_ps(MIN1Z, MIN2Z);
  __m128 BZ = _mm_min_ps(MAX1Z, MAX2Z);
  __m128 T1 = _mm_cmple_ps(AX, BX);
  __m128 T2 = _mm_cmple_ps(AY, BY);
  __m128 T3 = _mm_cmple_ps(AZ, BZ);
  __m128 T4 = _mm_and_ps(T1, T2);
  return _mm_and_ps(T3, T4);
}

static bool four_spheres_intersect_and(const Vector3d& o1, FCL_REAL r1,
                                       const Vector3d& o2, FCL_REAL r2,
                                       const Vector3d& o3, FCL_REAL r3,
                                       const Vector3d& o4, FCL_REAL r4,
                                       const Vector3d& o5, FCL_REAL r5,
                                       const Vector3d& o6, FCL_REAL r6,
                                       const Vector3d& o7, FCL_REAL r7,
                                       const Vector3d& o8, FCL_REAL r8)
{
  __m128 CMP = four_spheres_intersect(o1, r1, o2, r2, o3, r3, o4, r4, o5, r5, o6, r6, o7, r7, o8, r8);
  return _mm_movemask_ps(CMP) == 15.f;
}

static bool four_spheres_intersect_or(const Vector3d& o1, FCL_REAL r1,
                                      const Vector3d& o2, FCL_REAL r2,
                                      const Vector3d& o3, FCL_REAL r3,
                                      const Vector3d& o4, FCL_REAL r4,
                                      const Vector3d& o5, FCL_REAL r5,
                                      const Vector3d& o6, FCL_REAL r6,
                                      const Vector3d& o7, FCL_REAL r7,
                                      const Vector3d& o8, FCL_REAL r8)
{
  __m128 CMP = four_spheres_intersect(o1, r1, o2, r2, o3, r3, o4, r4, o5, r5, o6, r6, o7, r7, o8, r8);
  return __mm_movemask_ps(CMP) > 0;
}

/** @brief four spheres versus four AABBs SIMD test */
static bool four_spheres_four_AABBs_intersect_and(const Vector3d& o1, FCL_REAL r1,
                                                  const Vector3d& o2, FCL_REAL r2,
                                                  const Vector3d& o3, FCL_REAL r3,
                                                  const Vector3d& o4, FCL_REAL r4,
                                                  const Vector3d& min1, const Vector3d& max1,
                                                  const Vector3d& min2, const Vector3d& max2,
                                                  const Vector3d& min3, const Vector3d& max3,
                                                  const Vector3d& min4, const Vector3d& max4)
{
  __m128 CMP = four_spheres_four_AABBs_intersect(o1, r1, o2, r2, o3, r3, o4, r4, min1, max1, min2, max2, min3, max3, min4, max4);
  return _mm_movemask_ps(CMP) == 15.f;
}

static bool four_spheres_four_AABBs_intersect_or(const Vector3d& o1, FCL_REAL r1,
                                                 const Vector3d& o2, FCL_REAL r2,
                                                 const Vector3d& o3, FCL_REAL r3,
                                                 const Vector3d& o4, FCL_REAL r4,
                                                 const Vector3d& min1, const Vector3d& max1,
                                                 const Vector3d& min2, const Vector3d& max2,
                                                 const Vector3d& min3, const Vector3d& max3,
                                                 const Vector3d& min4, const Vector3d& max4)
{
  __m128 CMP = four_spheres_four_AABBs_intersect(o1, r1, o2, r2, o3, r3, o4, r4, min1, max1, min2, max2, min3, max3, min4, max4);
  return _mm_movemask_ps(CMP) > 0;
}

/** @brief four AABBs versus four AABBs SIMD test */
static bool four_AABBs_intersect_and(const Vector3d& min1, const Vector3d& max1,
                                     const Vector3d& min2, const Vector3d& max2,
                                     const Vector3d& min3, const Vector3d& max3,
                                     const Vector3d& min4, const Vector3d& max4,
                                     const Vector3d& min5, const Vector3d& max5,
                                     const Vector3d& min6, const Vector3d& max6,
                                     const Vector3d& min7, const Vector3d& max7,
                                     const Vector3d& min8, const Vector3d& max8)
{
  __m128 CMP = four_AABBs_intersect(min1, max1, min2, max2, min3, max3, min4, max4, min5, max5, min6, max6, min7, max7, min8, max8);
  return _mm_movemask_ps(CMP) == 15.f;
}

static bool four_AABBs_intersect_or(const Vector3d& min1, const Vector3d& max1,
                                    const Vector3d& min2, const Vector3d& max2,
                                    const Vector3d& min3, const Vector3d& max3,
                                    const Vector3d& min4, const Vector3d& max4,
                                    const Vector3d& min5, const Vector3d& max5,
                                    const Vector3d& min6, const Vector3d& max6,
                                    const Vector3d& min7, const Vector3d& max7,
                                    const Vector3d& min8, const Vector3d& max8)
{
  __m128 CMP = four_AABBs_intersect(min1, max1, min2, max2, min3, max3, min4, max4, min5, max5, min6, max6, min7, max7, min8, max8);
  return _mm_movemask_ps(CMP) > 0;
}

}

#endif
