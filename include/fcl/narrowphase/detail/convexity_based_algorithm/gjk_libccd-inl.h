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

/** @author Jia Pan */

#ifndef FCL_NARROWPHASE_DETAIL_GJKLIBCCD_INL_H
#define FCL_NARROWPHASE_DETAIL_GJKLIBCCD_INL_H

#include "fcl/narrowphase/detail/convexity_based_algorithm/gjk_libccd.h"
#include "fcl/narrowphase/detail/failed_at_this_configuration.h"

#include <array>
#include <unordered_map>
#include <unordered_set>

#include "fcl/common/unused.h"
#include "fcl/common/warning.h"

namespace fcl
{

namespace detail
{

//==============================================================================
extern template
class FCL_EXPORT GJKInitializer<double, Cylinder<double>>;

//==============================================================================
extern template
class FCL_EXPORT GJKInitializer<double, Sphere<double>>;

//==============================================================================
extern template
class FCL_EXPORT GJKInitializer<double, Ellipsoid<double>>;

//==============================================================================
extern template
class FCL_EXPORT GJKInitializer<double, Box<double>>;

//==============================================================================
extern template
class FCL_EXPORT GJKInitializer<double, Capsule<double>>;

//==============================================================================
extern template
class FCL_EXPORT GJKInitializer<double, Cone<double>>;

//==============================================================================
extern template
class FCL_EXPORT GJKInitializer<double, Convex<double>>;

//==============================================================================
extern template
void* triCreateGJKObject(
    const Vector3d& P1, const Vector3d& P2, const Vector3d& P3);

//==============================================================================
extern template
void* triCreateGJKObject(
    const Vector3d& P1,
    const Vector3d& P2,
    const Vector3d& P3,
    const Transform3d& tf);

//==============================================================================
extern template
bool GJKCollide(
    void* obj1,
    ccd_support_fn supp1,
    ccd_center_fn cen1,
    void* obj2,
    ccd_support_fn supp2,
    ccd_center_fn cen2,
    unsigned int max_iterations,
    double tolerance,
    Vector3d* contact_points,
    double* penetration_depth,
    Vector3d* normal);

//==============================================================================
extern template
bool GJKDistance(
    void* obj1,
    ccd_support_fn supp1,
    void* obj2,
    ccd_support_fn supp2,
    unsigned int max_iterations,
    double tolerance,
    double* dist,
    Vector3d* p1,
    Vector3d* p2);

extern template
bool GJKSignedDistance(
    void* obj1,
    ccd_support_fn supp1,
    void* obj2,
    ccd_support_fn supp2,
    unsigned int max_iterations,
    double tolerance,
    double* dist,
    Vector3d* p1,
    Vector3d* p2);

struct ccd_obj_t
{
  ccd_vec3_t pos;
  ccd_quat_t rot, rot_inv;
};

struct ccd_box_t : public ccd_obj_t
{
  ccd_real_t dim[3];
};

struct ccd_cap_t : public ccd_obj_t
{
  ccd_real_t radius, height;
};

struct ccd_cyl_t : public ccd_obj_t
{
  ccd_real_t radius, height;
};

struct ccd_cone_t : public ccd_obj_t
{
  ccd_real_t radius, height;
};

struct ccd_sphere_t : public ccd_obj_t
{
  ccd_real_t radius;
};

struct ccd_ellipsoid_t : public ccd_obj_t
{
  ccd_real_t radii[3];
};

template <typename S>
struct ccd_convex_t : public ccd_obj_t
{
  const Convex<S>* convex;
};

struct ccd_triangle_t : public ccd_obj_t
{
  ccd_vec3_t p[3];
  ccd_vec3_t c;
};

namespace libccd_extension
{

static ccd_real_t simplexReduceToTriangle(ccd_simplex_t *simplex,
                                          ccd_real_t dist,
                                          ccd_vec3_t *best_witness)
{
  ccd_real_t newdist;
  ccd_vec3_t witness;
  int best = -1;
  int i;

  // try the fourth point in all three positions
  for (i = 0; i < 3; i++){
    newdist = ccdVec3PointTriDist2(ccd_vec3_origin,
                                   &ccdSimplexPoint(simplex, (i == 0 ? 3 : 0))->v,
                                   &ccdSimplexPoint(simplex, (i == 1 ? 3 : 1))->v,
                                   &ccdSimplexPoint(simplex, (i == 2 ? 3 : 2))->v,
                                   &witness);
    newdist = CCD_SQRT(newdist);

    // record the best triangle
    if (newdist < dist){
      dist = newdist;
      best = i;
      ccdVec3Copy(best_witness, &witness);
    }
  }

  if (best >= 0){
    ccdSimplexSet(simplex, best, ccdSimplexPoint(simplex, 3));
  }
  ccdSimplexSetSize(simplex, 3);

  return dist;
}

_ccd_inline void tripleCross(const ccd_vec3_t *a, const ccd_vec3_t *b,
                             const ccd_vec3_t *c, ccd_vec3_t *d)
{
  ccd_vec3_t e;
  ccdVec3Cross(&e, a, b);
  ccdVec3Cross(d, &e, c);
}

/* This is *not* an implementation of the general function: what's the nearest
 point on the line segment AB to the origin O? It is not intended to be.
 This is a limited, special case which exploits the known (or at least
 expected) construction history of AB. The history is as follows:

   1. We originally started with the Minkowski support point B (p_OB), which
      was *not* the origin.
   2. We define a support direction as p_BO = -p_OB and use that to get the
      Minkowski support point A.
   3. We confirm that O is not strictly beyond A in the direction p_BO
      (confirming separation).
   4. Then, and only then, do we invoke this method.

 The method will do one of two things:

   - determine if the origin lies within the simplex (i.e. lies on the line
     segment, confirming non-separation) and reports if this is the case,
   - otherwise it computes a new support direction: a vector pointing to the
     origin from the nearest point on the segment AB. The direction is
     guaranteed; the only guarantee about the magnitude is that it is
     numerically viable (i.e. greater than epsilon).

 The algorithm exploits the construction history as outlined below. Without
 loss of generality, we place B some non-zero distance away from the origin
 along the î direction (all other orientations can be rigidly transformed to
 this canonical example). The diagram below shows the origin O and the point
 B. It also shows three regions: 1, 2, and 3.

                     ĵ
              1      ⯅    2        3
                     │░░░░░░░░░░┆▒▒▒▒▒
                     │░░░░░░░░░░┆▒▒▒▒▒
                     │░░░░░░░░░░┆▒▒▒▒▒
                     │░░░░░░░░░░┆▒▒▒▒▒
                     │░░░░░░░░░░┆▒▒▒▒▒
          ───────────O──────────B────⯈  î
                     │░░░░░░░░░░┆▒▒▒▒▒
                     │░░░░░░░░░░┆▒▒▒▒▒
                     │░░░░░░░░░░┆▒▒▒▒▒
                     │░░░░░░░░░░┆▒▒▒▒▒
                     │░░░░░░░░░░┆▒▒▒▒▒

 The point A cannot lie in region 3.

   - B is a support point of the Minkowski difference. p_BO defines the
     support vector that produces the support point A. Vertex A must lie at
     least as far in the p_BO as B otherwise A is not actually a valid support
     point for that direction. It could lie on the boundary of regions 2 & 3
     and still be a valid support point.

 The point A cannot lie in region 2 (or on the boundary between 2 & 3).
   - We confirm that O is not strictly beyond A in the direction p_BO. For all
     A in region 2, O lies beyond A (when projected onto the p_BO vector).

 The point A _must_ lie in region 1 (including the boundary between regions 1 &
 2) by process of elimination.

 The implication of this is that the O must project onto the _interior_ of the
 line segment AB (with one notable exception). If A = O, then the projection of
 O is at the end point and, is in fact, itself.

 Therefore, this function can only have two possible outcomes:

   1. In the case where p_BA = k⋅p_BO (i.e., they are co-linear), we know the
      origin lies "in" the simplex. If A = O, it lies on the simplex's surface
      and the objects are touching, otherwise, the objects are penetrating.
      Either way, we can report that they are definitely *not* separated.
   2. p_BA ≠ k⋅p_BO, we define the new support direction as perpendicular to the
      line segment AB, pointing to O from the nearest point on the segment to O.

 Return value indicates concrete knowledge that the origin lies "in" the
 2-simplex (encoded as a 1), or indication that computation should continue (0).

 @note: doSimplex2 should _only_ be called with the construction history
 outlined above: promotion of a 1-simplex. doSimplex2() is only invoked by
 doSimplex(). This follows the computation of A and the promotion of the
 simplex. Therefore, the history is always valid. Even though doSimplex3() can
 demote itself to a 2-simplex, that 2-simplex immediately gets promoted back to
 a 3-simplex via the same construction process. Therefore, as long as
 doSimplex2() is only called from doSimplex() its region 1 assumption _should_
 remain valid.
*/
static int doSimplex2(ccd_simplex_t *simplex, ccd_vec3_t *dir) {
  // Used to define numerical thresholds near zero; typically scaled to the size
  // of the quantities being tested.
  constexpr ccd_real_t eps = constants<ccd_real_t>::eps();

  const Vector3<ccd_real_t> p_OA(simplex->ps[simplex->last].v.v);
  const Vector3<ccd_real_t> p_OB(simplex->ps[0].v.v);

  // Confirm that A is in region 1. Given that A may be very near to the origin,
  // we must avoid normalizing p_OA. So, we use this instead.
  //  let A' be the projection of A onto the line defined by O and B.
  //  |A'B| >= |OB| iff A is in region 1.
  // Numerically, we can express it as follows (allowing for |A'B| to be ever
  // so slightly *less* than |OB|):
  //             p_AB ⋅ phat_OB >= |p_OB| - |p_OB| * ε = |p_OB| * (1 - ε)
  //    p_AB ⋅ phat_OB ⋅ |p_OB| >= |p_OB|⋅|p_OB| * (1 - ε)
  //                p_AB ⋅ p_OB >= |p_OB|² * (1 - ε)
  //       (p_OB - p_OA) ⋅ p_OB >= |p_OB|² * (1 - ε)
  //  p_OB ⋅ p_OB - p_OA ⋅ p_OB >= |p_OB|² * (1 - ε)
  //      |p_OB|² - p_OA ⋅ p_OB >= |p_OB|² * (1 - ε)
  //               -p_OA ⋅ p_OB >= -|p_OB|²ε
  //                p_OA ⋅ p_OB <= |p_OB|²ε
  assert(p_OA.dot(p_OB) <= p_OB.squaredNorm() * eps && "A is not in region 1");

  // Test for co-linearity. Given A is in region 1, co-linearity --> O is "in"
  // the simplex.
  // We'll use the angle between two vectors to determine co-linearity: p_AB
  // and p_OB. If they are co-linear, then the angle between them (θ) is zero.
  // Similarly, sin(θ) is zero. Ideally, it can be expressed as:
  //   |p_AB × p_OB| = |p_AB||p_OB||sin(θ)| = 0
  // Numerically, we allow θ (and sin(θ)) to be slightly larger than zero
  // leading to a numerical formulation as:
  //   |p_AB × p_OB| = |p_AB||p_OB||sin(θ)| < |p_AB||p_OB|ε
  // Finally, to reduce the computational cost, we eliminate the square roots by
  // evaluating the equivalently discriminating test:
  //   |p_AB × p_OB|² < |p_AB|²|p_OB|²ε²
  //
  // In addition to providing a measure of co-linearity, the cross product gives
  // us the normal to the plane on which points A, B, and O lie (which we will
  // use later to compute a new support direction, as necessary).
  const Vector3<ccd_real_t> p_AB = p_OB - p_OA;
  const Vector3<ccd_real_t> plane_normal = p_OB.cross(p_AB);
  if (plane_normal.squaredNorm() <
      p_AB.squaredNorm() * p_OB.squaredNorm() * eps * eps) {
    return 1;
  }

  // O is not co-linear with AB, so dist(O, AB) > ε. Define `dir` as the
  // direction to O from the nearest point on AB.
  // Note: We use the normalized `plane_normal` (n̂) because we've already
  // concluded that the origin is farther from AB than ε. We want to make sure
  // `dir` likewise has a magnitude larger than ε. With normalization, we know
  // |dir| = |n̂ × AB| = |AB| > dist(O, AB) > ε.
  // Without normalizing, if |OA| and |OB| were smaller than ³√ε but
  // sufficiently larger than ε, dist(O, AB) > ε, but |dir| < ε.
  const Vector3<ccd_real_t> new_dir = plane_normal.normalized().cross(p_AB);
  ccdVec3Set(dir, new_dir(0), new_dir(1), new_dir(2));
  return 0;
}

// Compares the given `value` against a _squared epsilon_. This is particularly
// important when testing some quantity (e.g., distance) to see if it
// is _functionally_ zero but using its _squared_ value in the test. Comparing
// _squared distance_ directly against epsilon is equivalent to comparing
// distance to sqrt(epsilon) -- we classify the distance as zero or not using
// only half the available precision.
static bool isAbsValueLessThanEpsSquared(ccd_real_t val) {
    return std::abs(val) < std::numeric_limits<ccd_real_t>::epsilon() *
                           std::numeric_limits<ccd_real_t>::epsilon();
}

// TODO(SeanCurtis-TRI): Define the return value:
//   1: (like doSimplex2) --> origin is "in" the simplex.
//   0:
//  -1: If the 3-simplex is degenerate. How is this intepreted?
static int doSimplex3(ccd_simplex_t *simplex, ccd_vec3_t *dir)
{
  const ccd_support_t *A, *B, *C;
  ccd_vec3_t AO, AB, AC, ABC, tmp;
  ccd_real_t dot;

  // get last added as A
  A = ccdSimplexLast(simplex);
  // get the other points
  B = ccdSimplexPoint(simplex, 1);
  C = ccdSimplexPoint(simplex, 0);

  // check touching contact
  // Compute origin_projection as well. Without computing the origin projection,
  // libccd could give inaccurate result. See
  // https://github.com/danfis/libccd/issues/55.
  ccd_vec3_t origin_projection_unused;

  const ccd_real_t dist_squared = ccdVec3PointTriDist2(
      ccd_vec3_origin, &A->v, &B->v, &C->v, &origin_projection_unused);
  if (isAbsValueLessThanEpsSquared(dist_squared)) {
    return 1;
  }

  // check if triangle is really triangle (has area > 0)
  // if not simplex can't be expanded and thus no intersection is found
  // TODO(SeanCurtis-TRI): Coincident points is sufficient but not necessary
  // for a zero-area triangle. What about co-linearity? Can we guarantee that
  // co-linearity can't happen?  See the `triangle_area_is_zero()` method in
  // this same file.
  if (ccdVec3Eq(&A->v, &B->v) || ccdVec3Eq(&A->v, &C->v)){
    // TODO(SeanCurtis-TRI): Why do we simply quit if the simplex is degenerate?
    return -1;
  }

  // compute AO vector
  ccdVec3Copy(&AO, &A->v);
  ccdVec3Scale(&AO, -CCD_ONE);

  // compute AB and AC segments and ABC vector (perpendircular to triangle)
  ccdVec3Sub2(&AB, &B->v, &A->v);
  ccdVec3Sub2(&AC, &C->v, &A->v);
  ccdVec3Cross(&ABC, &AB, &AC);

  ccdVec3Cross(&tmp, &ABC, &AC);
  dot = ccdVec3Dot(&tmp, &AO);
  if (ccdIsZero(dot) || dot > CCD_ZERO){
    dot = ccdVec3Dot(&AC, &AO);
    if (ccdIsZero(dot) || dot > CCD_ZERO){
      // C is already in place
      ccdSimplexSet(simplex, 1, A);
      ccdSimplexSetSize(simplex, 2);
      tripleCross(&AC, &AO, &AC, dir);
    }else{
    ccd_do_simplex3_45:
      dot = ccdVec3Dot(&AB, &AO);
      if (ccdIsZero(dot) || dot > CCD_ZERO){
        ccdSimplexSet(simplex, 0, B);
        ccdSimplexSet(simplex, 1, A);
        ccdSimplexSetSize(simplex, 2);
        tripleCross(&AB, &AO, &AB, dir);
      }else{
        ccdSimplexSet(simplex, 0, A);
        ccdSimplexSetSize(simplex, 1);
        ccdVec3Copy(dir, &AO);
      }
    }
  }else{
    ccdVec3Cross(&tmp, &AB, &ABC);
    dot = ccdVec3Dot(&tmp, &AO);
    if (ccdIsZero(dot) || dot > CCD_ZERO){
      goto ccd_do_simplex3_45;
    }else{
      dot = ccdVec3Dot(&ABC, &AO);
      if (ccdIsZero(dot) || dot > CCD_ZERO){
        ccdVec3Copy(dir, &ABC);
      }else{
        ccd_support_t Ctmp;
        ccdSupportCopy(&Ctmp, C);
        ccdSimplexSet(simplex, 0, B);
        ccdSimplexSet(simplex, 1, &Ctmp);

        ccdVec3Copy(dir, &ABC);
        ccdVec3Scale(dir, -CCD_ONE);
      }
    }
  }

  return 0;
}

static int doSimplex4(ccd_simplex_t *simplex, ccd_vec3_t *dir)
{
  const ccd_support_t *A, *B, *C, *D;
  ccd_vec3_t AO, AB, AC, AD, ABC, ACD, ADB;
  int B_on_ACD, C_on_ADB, D_on_ABC;
  int AB_O, AC_O, AD_O;

  // get last added as A
  A = ccdSimplexLast(simplex);
  // get the other points
  B = ccdSimplexPoint(simplex, 2);
  C = ccdSimplexPoint(simplex, 1);
  D = ccdSimplexPoint(simplex, 0);

  // check if tetrahedron is really tetrahedron (has volume > 0)
  // if it is not simplex can't be expanded and thus no intersection is
  // found.
  // point_projection_on_triangle_unused is not used. We ask
  // ccdVec3PointTriDist2 to compute this witness point, so as to get a
  // numerical robust dist_squared. See
  // https://github.com/danfis/libccd/issues/55 for an explanation.
  ccd_vec3_t point_projection_on_triangle_unused;
  ccd_real_t dist_squared = ccdVec3PointTriDist2(
      &A->v, &B->v, &C->v, &D->v, &point_projection_on_triangle_unused);
  if (isAbsValueLessThanEpsSquared(dist_squared)) {
    return -1;
  }

  // check if origin lies on some of tetrahedron's face - if so objects
  // intersect
  dist_squared = ccdVec3PointTriDist2(ccd_vec3_origin, &A->v, &B->v, &C->v,
                                      &point_projection_on_triangle_unused);
  if (isAbsValueLessThanEpsSquared((dist_squared))) return 1;
  dist_squared = ccdVec3PointTriDist2(ccd_vec3_origin, &A->v, &C->v, &D->v,
                                      &point_projection_on_triangle_unused);
  if (isAbsValueLessThanEpsSquared((dist_squared))) return 1;
  dist_squared = ccdVec3PointTriDist2(ccd_vec3_origin, &A->v, &B->v, &D->v,
                                      &point_projection_on_triangle_unused);
  if (isAbsValueLessThanEpsSquared(dist_squared)) return 1;
  dist_squared = ccdVec3PointTriDist2(ccd_vec3_origin, &B->v, &C->v, &D->v,
                                      &point_projection_on_triangle_unused);
  if (isAbsValueLessThanEpsSquared(dist_squared)) return 1;

  // compute AO, AB, AC, AD segments and ABC, ACD, ADB normal vectors
  ccdVec3Copy(&AO, &A->v);
  ccdVec3Scale(&AO, -CCD_ONE);
  ccdVec3Sub2(&AB, &B->v, &A->v);
  ccdVec3Sub2(&AC, &C->v, &A->v);
  ccdVec3Sub2(&AD, &D->v, &A->v);
  ccdVec3Cross(&ABC, &AB, &AC);
  ccdVec3Cross(&ACD, &AC, &AD);
  ccdVec3Cross(&ADB, &AD, &AB);

  // side (positive or negative) of B, C, D relative to planes ACD, ADB
  // and ABC respectively
  B_on_ACD = ccdSign(ccdVec3Dot(&ACD, &AB));
  C_on_ADB = ccdSign(ccdVec3Dot(&ADB, &AC));
  D_on_ABC = ccdSign(ccdVec3Dot(&ABC, &AD));

  // whether origin is on same side of ACD, ADB, ABC as B, C, D
  // respectively
  AB_O = ccdSign(ccdVec3Dot(&ACD, &AO)) == B_on_ACD;
  AC_O = ccdSign(ccdVec3Dot(&ADB, &AO)) == C_on_ADB;
  AD_O = ccdSign(ccdVec3Dot(&ABC, &AO)) == D_on_ABC;

  if (AB_O && AC_O && AD_O){
    // origin is in tetrahedron
    return 1;

    // rearrange simplex to triangle and call doSimplex3()
  }else if (!AB_O){
    // B is farthest from the origin among all of the tetrahedron's
    // points, so remove it from the list and go on with the triangle
    // case

    // D and C are in place
    ccdSimplexSet(simplex, 2, A);
    ccdSimplexSetSize(simplex, 3);
  }else if (!AC_O){
    // C is farthest
    ccdSimplexSet(simplex, 1, D);
    ccdSimplexSet(simplex, 0, B);
    ccdSimplexSet(simplex, 2, A);
    ccdSimplexSetSize(simplex, 3);
  }else{ // (!AD_O)
    ccdSimplexSet(simplex, 0, C);
    ccdSimplexSet(simplex, 1, B);
    ccdSimplexSet(simplex, 2, A);
    ccdSimplexSetSize(simplex, 3);
  }

  return doSimplex3(simplex, dir);
}

static int doSimplex(ccd_simplex_t *simplex, ccd_vec3_t *dir)
{
  if (ccdSimplexSize(simplex) == 2){
    // simplex contains segment only one segment
    return doSimplex2(simplex, dir);
  }else if (ccdSimplexSize(simplex) == 3){
    // simplex contains triangle
    return doSimplex3(simplex, dir);
  }else{ // ccdSimplexSize(simplex) == 4
    // tetrahedron - this is the only shape which can encapsule origin
    // so doSimplex4() also contains test on it
    return doSimplex4(simplex, dir);
  }
}

/** Transforms simplex to polytope, two vertices required */
static int simplexToPolytope2(const void *obj1, const void *obj2,
                              const ccd_t *ccd,
                              const ccd_simplex_t *simplex,
                              ccd_pt_t *pt, ccd_pt_el_t **nearest)
{
    const ccd_support_t *a, *b;
    ccd_vec3_t ab, ac, dir;
    ccd_support_t supp[4];
    ccd_pt_vertex_t *v[6];
    ccd_pt_edge_t *e[12];
    size_t i;
    int found;

    a = ccdSimplexPoint(simplex, 0);
    b = ccdSimplexPoint(simplex, 1);

    // This situation is a bit tricky. If only one segment comes from
    // previous run of GJK - it means that either this segment is on
    // minkowski edge (and thus we have touch contact) or it it isn't and
    // therefore segment is somewhere *inside* minkowski sum and it *must*
    // be possible to fully enclose this segment with polyhedron formed by
    // at least 8 triangle faces.

    // get first support point (any)
    found = 0;
    for (i = 0; i < ccd_points_on_sphere_len; i++){
        __ccdSupport(obj1, obj2, &ccd_points_on_sphere[i], ccd, &supp[0]);
        if (!ccdVec3Eq(&a->v, &supp[0].v) && !ccdVec3Eq(&b->v, &supp[0].v)){
            found = 1;
            break;
        }
    }
    if (!found)
        goto simplexToPolytope2_touching_contact;

    // get second support point in opposite direction than supp[0]
    ccdVec3Copy(&dir, &supp[0].v);
    ccdVec3Scale(&dir, -CCD_ONE);
    __ccdSupport(obj1, obj2, &dir, ccd, &supp[1]);
    if (ccdVec3Eq(&a->v, &supp[1].v) || ccdVec3Eq(&b->v, &supp[1].v))
        goto simplexToPolytope2_touching_contact;

    // next will be in direction of normal of triangle a,supp[0],supp[1]
    ccdVec3Sub2(&ab, &supp[0].v, &a->v);
    ccdVec3Sub2(&ac, &supp[1].v, &a->v);
    ccdVec3Cross(&dir, &ab, &ac);
    __ccdSupport(obj1, obj2, &dir, ccd, &supp[2]);
    if (ccdVec3Eq(&a->v, &supp[2].v) || ccdVec3Eq(&b->v, &supp[2].v))
        goto simplexToPolytope2_touching_contact;

    // and last one will be in opposite direction
    ccdVec3Scale(&dir, -CCD_ONE);
    __ccdSupport(obj1, obj2, &dir, ccd, &supp[3]);
    if (ccdVec3Eq(&a->v, &supp[3].v) || ccdVec3Eq(&b->v, &supp[3].v))
        goto simplexToPolytope2_touching_contact;

    goto simplexToPolytope2_not_touching_contact;
simplexToPolytope2_touching_contact:
    v[0] = ccdPtAddVertex(pt, a);
    v[1] = ccdPtAddVertex(pt, b);
    *nearest = (ccd_pt_el_t *)ccdPtAddEdge(pt, v[0], v[1]);
    if (*nearest == NULL)
        return -2;

    return -1;

simplexToPolytope2_not_touching_contact:
    // form polyhedron
    v[0] = ccdPtAddVertex(pt, a);
    v[1] = ccdPtAddVertex(pt, &supp[0]);
    v[2] = ccdPtAddVertex(pt, b);
    v[3] = ccdPtAddVertex(pt, &supp[1]);
    v[4] = ccdPtAddVertex(pt, &supp[2]);
    v[5] = ccdPtAddVertex(pt, &supp[3]);

    e[0] = ccdPtAddEdge(pt, v[0], v[1]);
    e[1] = ccdPtAddEdge(pt, v[1], v[2]);
    e[2] = ccdPtAddEdge(pt, v[2], v[3]);
    e[3] = ccdPtAddEdge(pt, v[3], v[0]);

    e[4] = ccdPtAddEdge(pt, v[4], v[0]);
    e[5] = ccdPtAddEdge(pt, v[4], v[1]);
    e[6] = ccdPtAddEdge(pt, v[4], v[2]);
    e[7] = ccdPtAddEdge(pt, v[4], v[3]);

    e[8]  = ccdPtAddEdge(pt, v[5], v[0]);
    e[9]  = ccdPtAddEdge(pt, v[5], v[1]);
    e[10] = ccdPtAddEdge(pt, v[5], v[2]);
    e[11] = ccdPtAddEdge(pt, v[5], v[3]);

    if (ccdPtAddFace(pt, e[4], e[5], e[0]) == NULL
            || ccdPtAddFace(pt, e[5], e[6], e[1]) == NULL
            || ccdPtAddFace(pt, e[6], e[7], e[2]) == NULL
            || ccdPtAddFace(pt, e[7], e[4], e[3]) == NULL

            || ccdPtAddFace(pt, e[8],  e[9],  e[0]) == NULL
            || ccdPtAddFace(pt, e[9],  e[10], e[1]) == NULL
            || ccdPtAddFace(pt, e[10], e[11], e[2]) == NULL
            || ccdPtAddFace(pt, e[11], e[8],  e[3]) == NULL){
        return -2;
    }

    return 0;
}

#ifndef NDEBUG
static bool isPolytopeEmpty(const ccd_pt_t& polytope) {
  ccd_pt_vertex_t* v = nullptr;
  ccdListForEachEntry(&polytope.vertices, v, ccd_pt_vertex_t, list) {
    if (v) {
      return false;
    }
  }
  ccd_pt_edge_t* e = nullptr;
  ccdListForEachEntry(&polytope.edges, e, ccd_pt_edge_t, list) {
    if (e) {
      return false;
    }
  }
  ccd_pt_face_t* f = nullptr;
  ccdListForEachEntry(&polytope.faces, f, ccd_pt_face_t, list) {
    if (f) {
      return false;
    }
  }
  return true;
}
#endif

/** Transforms a 2-simplex (triangle) to a polytope (tetrahedron), three
 * vertices required.
 * Both the simplex and the transformed polytope contain the origin. The simplex
 * vertices lie on the surface of the Minkowski difference obj1 ⊖ obj2.
 * @param[in] obj1 object 1 on which the distance is queried.
 * @param[in] obj2 object 2 on which the distance is queried.
 * @param[in] ccd The ccd solver.
 * @param[in] simplex The simplex (with three vertices) that contains the
 * origin.
 * @param[out] polytope The polytope (tetrahedron) that also contains the origin
 * on one of its faces. At input, the polytope should be empty.
 * @param[out] nearest If the function detects that obj1 and obj2 are touching,
 * then set *nearest to be the nearest points on obj1 and obj2 respectively;
 * otherwise set *nearest to NULL. @note nearest cannot be NULL.
 * @retval status return 0 on success, -1 if touching contact is detected, and
 * -2 on non-recoverable failure (mostly due to memory allocation bug).
 */
static int convert2SimplexToTetrahedron(const void* obj1, const void* obj2,
                              const ccd_t* ccd, const ccd_simplex_t* simplex,
                              ccd_pt_t* polytope, ccd_pt_el_t** nearest) {
  assert(nearest);
  assert(isPolytopeEmpty(*polytope));
  assert(simplex->last == 2); // a 2-simplex.
  const ccd_support_t *a, *b, *c;
  ccd_support_t d, d2;
  ccd_vec3_t ab, ac, dir;
  ccd_pt_vertex_t* v[4];
  ccd_pt_edge_t* e[6];

  *nearest = NULL;

  a = ccdSimplexPoint(simplex, 0);
  b = ccdSimplexPoint(simplex, 1);
  c = ccdSimplexPoint(simplex, 2);

  // The 2-simplex is just a triangle containing the origin. We will expand this
  // triangle to a tetrahedron, by adding the support point along the normal
  // direction of the triangle.
  ccdVec3Sub2(&ab, &b->v, &a->v);
  ccdVec3Sub2(&ac, &c->v, &a->v);
  ccdVec3Cross(&dir, &ab, &ac);
  __ccdSupport(obj1, obj2, &dir, ccd, &d);
  ccd_vec3_t point_projection_on_triangle_unused;
  const ccd_real_t dist_squared = ccdVec3PointTriDist2(
      &d.v, &a->v, &b->v, &c->v, &point_projection_on_triangle_unused);

  // and second one take in opposite direction
  ccdVec3Scale(&dir, -CCD_ONE);
  __ccdSupport(obj1, obj2, &dir, ccd, &d2);
  const ccd_real_t dist_squared_opposite = ccdVec3PointTriDist2(
      &d2.v, &a->v, &b->v, &c->v, &point_projection_on_triangle_unused);

  // check if face isn't already on edge of minkowski sum and thus we
  // have touching contact
  if (ccdIsZero(dist_squared) || ccdIsZero(dist_squared_opposite)) {
    v[0] = ccdPtAddVertex(polytope, a);
    v[1] = ccdPtAddVertex(polytope, b);
    v[2] = ccdPtAddVertex(polytope, c);
    e[0] = ccdPtAddEdge(polytope, v[0], v[1]);
    e[1] = ccdPtAddEdge(polytope, v[1], v[2]);
    e[2] = ccdPtAddEdge(polytope, v[2], v[0]);
    *nearest = (ccd_pt_el_t*)ccdPtAddFace(polytope, e[0], e[1], e[2]);
    if (*nearest == NULL) return -2;

    return -1;
  }
  // Form a tetrahedron with abc as one face, pick either d or d2, based
  // on which one has larger distance to the face abc. We pick the larger
  // distance because it gives a tetrahedron with larger volume, so potentially
  // more "expanded" than the one with the smaller volume.
  auto FormTetrahedron = [polytope, a, b, c, &v,
                          &e](const ccd_support_t& new_support) -> int {
    v[0] = ccdPtAddVertex(polytope, a);
    v[1] = ccdPtAddVertex(polytope, b);
    v[2] = ccdPtAddVertex(polytope, c);
    v[3] = ccdPtAddVertex(polytope, &new_support);

    e[0] = ccdPtAddEdge(polytope, v[0], v[1]);
    e[1] = ccdPtAddEdge(polytope, v[1], v[2]);
    e[2] = ccdPtAddEdge(polytope, v[2], v[0]);
    e[3] = ccdPtAddEdge(polytope, v[0], v[3]);
    e[4] = ccdPtAddEdge(polytope, v[1], v[3]);
    e[5] = ccdPtAddEdge(polytope, v[2], v[3]);

    // ccdPtAdd*() functions return NULL either if the memory allocation
    // failed of if any of the input pointers are NULL, so the bad
    // allocation can be checked by the last calls of ccdPtAddFace()
    // because the rest of the bad allocations eventually "bubble up" here
    // Note, there is no requirement on the winding of the face, namely we do
    // not guarantee if all f.e(0).cross(f.e(1)) points outward (or inward) for
    // all the faces added below.
    if (ccdPtAddFace(polytope, e[0], e[1], e[2]) == NULL ||
        ccdPtAddFace(polytope, e[3], e[4], e[0]) == NULL ||
        ccdPtAddFace(polytope, e[4], e[5], e[1]) == NULL ||
        ccdPtAddFace(polytope, e[5], e[3], e[2]) == NULL) {
      return -2;
    }
    return 0;
  };

  if (std::abs(dist_squared) > std::abs(dist_squared_opposite)) {
    return FormTetrahedron(d);
  } else {
    return FormTetrahedron(d2);
  }
}

/** Transforms simplex to polytope. It is assumed that simplex has 4
 *  vertices! */
static int simplexToPolytope4(const void* obj1, const void* obj2,
                              const ccd_t* ccd, ccd_simplex_t* simplex,
                              ccd_pt_t* pt, ccd_pt_el_t** nearest) {
  const ccd_support_t *a, *b, *c, *d;
  bool use_polytope3{false};
  ccd_pt_vertex_t* v[4];
  ccd_pt_edge_t* e[6];
  size_t i;

  a = ccdSimplexPoint(simplex, 0);
  b = ccdSimplexPoint(simplex, 1);
  c = ccdSimplexPoint(simplex, 2);
  d = ccdSimplexPoint(simplex, 3);

  // The origin can lie on any of the tetrahedra faces. In fact, for a
  // degenerate tetrahedron, it could be considered to lie on multiple faces
  // simultaneously. If it lies on any face, we can simply reduce the dimension
  // of the simplex to that face and then attempt to construct the polytope from
  // the resulting face. We simply take the first face which exhibited the
  // trait.
  ccd_real_t dist_squared =
      ccdVec3PointTriDist2(ccd_vec3_origin, &a->v, &b->v, &c->v, NULL);
  if (isAbsValueLessThanEpsSquared(dist_squared)) {
    use_polytope3 = true;
  }
  if (!use_polytope3) {
    dist_squared =
        ccdVec3PointTriDist2(ccd_vec3_origin, &a->v, &c->v, &d->v, NULL);
    if (isAbsValueLessThanEpsSquared(dist_squared)) {
      use_polytope3 = true;
      ccdSimplexSet(simplex, 1, c);
      ccdSimplexSet(simplex, 2, d);
    }
  }
  if (!use_polytope3) {
    dist_squared =
        ccdVec3PointTriDist2(ccd_vec3_origin, &a->v, &b->v, &d->v, NULL);
    if (isAbsValueLessThanEpsSquared(dist_squared)) {
      use_polytope3 = true;
      ccdSimplexSet(simplex, 2, d);
    }
  }
  if (!use_polytope3) {
    dist_squared =
        ccdVec3PointTriDist2(ccd_vec3_origin, &b->v, &c->v, &d->v, NULL);
    if (isAbsValueLessThanEpsSquared(dist_squared)) {
      use_polytope3 = true;
      ccdSimplexSet(simplex, 0, b);
      ccdSimplexSet(simplex, 1, c);
      ccdSimplexSet(simplex, 2, d);
    }
  }

  if (use_polytope3) {
    ccdSimplexSetSize(simplex, 3);
    return convert2SimplexToTetrahedron(obj1, obj2, ccd, simplex, pt, nearest);
  }

  // no touching contact - simply create tetrahedron
  for (i = 0; i < 4; i++) {
    v[i] = ccdPtAddVertex(pt, ccdSimplexPoint(simplex, i));
  }

  e[0] = ccdPtAddEdge(pt, v[0], v[1]);
  e[1] = ccdPtAddEdge(pt, v[1], v[2]);
  e[2] = ccdPtAddEdge(pt, v[2], v[0]);
  e[3] = ccdPtAddEdge(pt, v[3], v[0]);
  e[4] = ccdPtAddEdge(pt, v[3], v[1]);
  e[5] = ccdPtAddEdge(pt, v[3], v[2]);

  // ccdPtAdd*() functions return NULL either if the memory allocation
  // failed of if any of the input pointers are NULL, so the bad
  // allocation can be checked by the last calls of ccdPtAddFace()
  // because the rest of the bad allocations eventually "bubble up" here
  if (ccdPtAddFace(pt, e[0], e[1], e[2]) == NULL ||
      ccdPtAddFace(pt, e[3], e[4], e[0]) == NULL ||
      ccdPtAddFace(pt, e[4], e[5], e[1]) == NULL ||
      ccdPtAddFace(pt, e[5], e[3], e[2]) == NULL) {
    return -2;
  }

  return 0;
}

/** Reports true if p and q are coincident. */
static bool are_coincident(const ccd_vec3_t& p, const ccd_vec3_t& q) {
  // This uses a scale-dependent basis for determining coincidence. It examines
  // each axis independently, and only, if all three axes are sufficiently
  // close (relative to its own scale), are the two points considered
  // coincident.
  //
  // For dimension i, two values are considered the same if:
  //   |pᵢ - qᵢ| <= ε·max(1, |pᵢ|, |qᵢ|)
  // And the points are coincident if the previous condition holds for all
  // `i ∈ {0, 1, 2}` (i.e. the x-, y-, *and* z-dimensions).
  using std::abs;
  using std::max;

  constexpr ccd_real_t eps = constants<ccd_real_t>::eps();
  // NOTE: Wrapping "1.0" with ccd_real_t accounts for mac problems where ccd
  // is actually float based.
  for (int i = 0; i < 3; ++i) {
    const ccd_real_t tolerance =
        max({ccd_real_t{1}, abs(p.v[i]), abs(q.v[i])}) * eps;
    const ccd_real_t delta = abs(p.v[i] - q.v[i]);
    if (delta > tolerance) return false;
  }
  return true;
}

/** Determines if the the triangle defined by the three vertices has zero area.
 Area can be zero for one of two reasons:
   - the triangle is so small that the vertices are functionally coincident, or
   - the vertices are co-linear.
 Both conditions are computed with respect to machine precision.
 @returns true if the area is zero.  */
static bool triangle_area_is_zero(const ccd_vec3_t& a, const ccd_vec3_t& b,
                                  const ccd_vec3_t& c) {
  // First coincidence condition. This doesn't *explicitly* test for b and c
  // being coincident. That will be captured in the subsequent co-linearity
  // test. If b and c *were* coincident, it would be cheaper to perform the
  // coincidence test than the co-linearity test.
  // However, the expectation is that typically the triangle will not have zero
  // area. In that case, we want to minimize the number of tests performed on
  // the average, so we prefer to eliminate one coincidence test.
  if (are_coincident(a, b) || are_coincident(a, c)) return true;

  // We're going to compute the *sine* of the angle θ between edges (given that
  // the vertices are *not* coincident). If the sin(θ) < ε, the edges are
  // co-linear.
  ccd_vec3_t AB, AC, n;
  ccdVec3Sub2(&AB, &b, &a);
  ccdVec3Sub2(&AC, &c, &a);
  ccdVec3Normalize(&AB);
  ccdVec3Normalize(&AC);
  ccdVec3Cross(&n, &AB, &AC);
  constexpr ccd_real_t eps = constants<ccd_real_t>::eps();
  // Second co-linearity condition.
  if (ccdVec3Len2(&n) < eps * eps) return true;
  return false;
}

/**
 * Computes the normal vector of a triangular face on a polytope, and the normal
 * vector points outward from the polytope. Notice we assume that the origin
 * lives within the polytope, and the normal vector may not have unit length.
 * @param[in] polytope The polytope on which the face lives. We assume that the
 * origin also lives inside the polytope.
 * @param[in] face The face for which the normal vector is computed.
 * @retval dir The vector normal to the face, and points outward from the
 * polytope. `dir` is unnormalized, that it does not necessarily have a unit
 * length.
 * @throws UnexpectedConfigurationException if built in debug mode _and_ the
 * triangle has zero area (either by being too small, or being co-linear).
 */
static ccd_vec3_t faceNormalPointingOutward(const ccd_pt_t* polytope,
                                            const ccd_pt_face_t* face) {
  // This doesn't necessarily define a triangle; I don't know that the third
  // vertex added here is unique from the other two.
#ifndef NDEBUG
  // quick test for degeneracy
  const ccd_vec3_t& a = face->edge[0]->vertex[1]->v.v;
  const ccd_vec3_t& b = face->edge[0]->vertex[0]->v.v;
  const ccd_vec3_t& test_v = face->edge[1]->vertex[0]->v.v;
  const ccd_vec3_t& c = are_coincident(test_v, a) || are_coincident(test_v, b) ?
                        face->edge[1]->vertex[1]->v.v : test_v;
  if (triangle_area_is_zero(a, b, c)) {
    FCL_THROW_FAILED_AT_THIS_CONFIGURATION(
        "Cannot compute face normal for a degenerate (zero-area) triangle");
  }
#endif
  // We find two edges of the triangle as e1 and e2, and the normal vector
  // of the face is e1.cross(e2).
  ccd_vec3_t e1, e2;
  ccdVec3Sub2(&e1, &(face->edge[0]->vertex[1]->v.v),
              &(face->edge[0]->vertex[0]->v.v));
  ccdVec3Sub2(&e2, &(face->edge[1]->vertex[1]->v.v),
              &(face->edge[1]->vertex[0]->v.v));
  ccd_vec3_t dir;
  // TODO(hongkai.dai): we ignore the degeneracy here, namely we assume e1 and
  // e2 are not colinear. We should check if e1 and e2 are colinear, and handle
  // this corner case.
  ccdVec3Cross(&dir, &e1, &e2);
  const ccd_real_t dir_norm = std::sqrt(ccdVec3Len2(&dir));
  ccd_vec3_t unit_dir = dir;
  ccdVec3Scale(&unit_dir, 1.0 / dir_norm);
  // The winding of the triangle is *not* guaranteed. The normal `n = e₁ × e₂`
  // may point inside or outside. We rely on the fact that the origin lies
  // within the polytope to resolve this ambiguity. A vector from the origin to
  // any point on the triangle must point in the "same" direction as the normal
  // (positive dot product).

  // However, the distance to the origin may be too small for the origin to
  // serve as a reliable witness of inside-ness. In that case, we examine the
  // polytope's *other* vertices; they should all lie on the "inside" of the
  // current triangle. If at least one is a reliable distance, then that is
  // considered to be the inside. If all vertices are "too close" (like the
  // origin), then "inside" is defined as the side of the triangle that had the
  // most distant vertex.

  // For these tests, we use the arbitrary distance of 0.01 unit length as a
  // "reliable" distance for both the origin and other vertices. Even if it
  // seems large, the fall through case of comparing the maximum distance will
  // always guarantee correctness.
  const ccd_real_t dist_tol = 0.01;
  // origin_distance_to_plane computes the signed distance from the origin to
  // the plane nᵀ * (x - v) = 0 coinciding with the triangle,  where v is a
  // point on the triangle.
  const ccd_real_t origin_distance_to_plane =
      ccdVec3Dot(&unit_dir, &(face->edge[0]->vertex[0]->v.v));
  if (origin_distance_to_plane < -dist_tol) {
    // Origin is more than `dist_tol` away from the plane, but the negative
    // value implies that the normal vector is pointing in the wrong direction;
    // flip it.
    ccdVec3Scale(&dir, ccd_real_t(-1));
  } else if (-dist_tol <= origin_distance_to_plane &&
             origin_distance_to_plane <= dist_tol) {
    // The origin is close to the plane of the face. Pick another vertex to test
    // the normal direction.
    ccd_real_t max_distance_to_plane = -CCD_REAL_MAX;
    ccd_real_t min_distance_to_plane = CCD_REAL_MAX;
    ccd_pt_vertex_t* v;
    // If the magnitude of the distance_to_plane is larger than dist_tol,
    // then it means one of the vertices is at least `dist_tol` away from the
    // plane coinciding with the face.
    ccdListForEachEntry(&polytope->vertices, v, ccd_pt_vertex_t, list) {
      // distance_to_plane is the signed distance from the
      // vertex v->v.v to the face, i.e., distance_to_plane = nᵀ *
      // (v->v.v - face_point). Note that origin_distance_to_plane = nᵀ *
      // face_point.
      const ccd_real_t distance_to_plane =
          ccdVec3Dot(&unit_dir, &(v->v.v)) - origin_distance_to_plane;
      if (distance_to_plane > dist_tol) {
        // The vertex is at least dist_tol away from the face plane, on the same
        // direction of `dir`. So we flip dir to point it outward from the
        // polytope.
        ccdVec3Scale(&dir, ccd_real_t(-1));
        return dir;
      } else if (distance_to_plane < -dist_tol) {
        // The vertex is at least `dist_tol` away from the face plane, on the
        // opposite direction of `dir`. So `dir` points outward already.
        return dir;
      } else {
        max_distance_to_plane =
            std::max(max_distance_to_plane, distance_to_plane);
        min_distance_to_plane =
            std::min(min_distance_to_plane, distance_to_plane);
      }
    }
    // If max_distance_to_plane > |min_distance_to_plane|, it means that the
    // vertices that are on the positive side of the plane, has a larger maximal
    // distance than the vertices on the negative side of the plane. Thus we
    // regard that `dir` points into the polytope. Hence we flip `dir`.
    if (max_distance_to_plane > std::abs(min_distance_to_plane)) {
      ccdVec3Scale(&dir, ccd_real_t(-1));
    }
  }
  return dir;
}

// Return true if the point `pt` is on the outward side of the half-plane, on
// which the triangle `f1 lives. Notice if the point `pt` is exactly on the
// half-plane, the return is false.
// @param f A triangle on a polytope.
// @param pt A point.
static bool isOutsidePolytopeFace(const ccd_pt_t* polytope,
                                const ccd_pt_face_t* f, const ccd_vec3_t* pt) {
  ccd_vec3_t n = faceNormalPointingOutward(polytope, f);
  // r_VP is the vector from a vertex V on the face `f`, to the point P `pt`.
  ccd_vec3_t r_VP;
  ccdVec3Sub2(&r_VP, pt, &(f->edge[0]->vertex[0]->v.v));
  return ccdVec3Dot(&n, &r_VP) > 0;
}

#ifndef NDEBUG
// The function ComputeVisiblePatchRecursiveSanityCheck() is only called in the
// debug mode. In the release mode, this function is declared/defined but not
// used. Without this NDEBUG macro, the function will cause a -Wunused-function
// error on CI's release builds.
/**
 * Reports true if the visible patch is valid.
 * The invariant for computing the visible patch is that for each edge in the
 * polytope, if both neighbouring faces are visible, then the edge is an
 * internal edge; if only one neighbouring face is visible, then the edge
 * is a border edge.
 * For each face, if one of its edges is an internal edge, then the face is
 * visible.
 */
static bool ComputeVisiblePatchRecursiveSanityCheck(
    const ccd_pt_t& polytope,
    const std::unordered_set<ccd_pt_edge_t*>& border_edges,
    const std::unordered_set<ccd_pt_face_t*>& visible_faces,
    const std::unordered_set<ccd_pt_edge_t*>& internal_edges) {
  ccd_pt_face_t* f;
  // TODO(SeanCurtis-TRI): Instead of returning false, have this throw the
  //  exception itself so that it can provide additional information.
  ccdListForEachEntry(&polytope.faces, f, ccd_pt_face_t, list) {
    bool has_edge_internal = false;
    for (int i = 0; i < 3; ++i) {
      // Since internal_edges is a set, internal_edges.count(e) means that e
      // is contained in the set internal_edges.
      if (internal_edges.count(f->edge[i]) > 0) {
        has_edge_internal = true;
        break;
      }
    }
    if (has_edge_internal) {
      if (visible_faces.count(f) == 0) {
        return false;
      }
    }
  }
  ccd_pt_edge_t* e;
  ccdListForEachEntry(&polytope.edges, e, ccd_pt_edge_t, list) {
    if (visible_faces.count(e->faces[0]) > 0 &&
        visible_faces.count(e->faces[1]) > 0) {
      if (internal_edges.count(e) == 0) {
        return false;
      }
    } else if (visible_faces.count(e->faces[0]) +
                   visible_faces.count(e->faces[1]) ==
               1) {
      if (border_edges.count(e) == 0) {
        return false;
      }
    }
  }
  for (const auto b : border_edges) {
    if (internal_edges.count(b) > 0) {
      return false;
    }
  }
  return true;
}
#endif

/** Attempts to classify the given `edge` as a border edge, confirming it hasn't
 already been classified as an internal edge.
 @param edge              The edge to classify.
 @param border_edges      The set of edges already classified as border.
 @param internal_edges    The set of edges already classified as internal.
 @throws ThrowFailedAtThisConfiguration if the edge is being re-classified.
 */
static void ClassifyBorderEdge(ccd_pt_edge_t* edge,
                        std::unordered_set<ccd_pt_edge_t*>* border_edges,
                        std::unordered_set<ccd_pt_edge_t*>* internal_edges) {
  border_edges->insert(edge);
  if (internal_edges->count(edge) > 0) {
    FCL_THROW_FAILED_AT_THIS_CONFIGURATION(
        "An edge is being classified as border that has already been "
        "classifed as internal");
  }
}

/** Attempts to classify the given `edge` as an internal edge, confirming it
 hasn't already been classified as a border edge.
 @param edge              The edge to classify.
 @param border_edges      The set of edges already classified as border.
 @param internal_edges    The set of edges already classified as internal.
 @throws ThrowFailedAtThisConfiguration if the edge is being re-classified.
 */
static void ClassifyInternalEdge(ccd_pt_edge_t* edge,
                          std::unordered_set<ccd_pt_edge_t*>* border_edges,
                          std::unordered_set<ccd_pt_edge_t*>* internal_edges) {
  internal_edges->insert(edge);
  if (border_edges->count(edge) > 0) {
    FCL_THROW_FAILED_AT_THIS_CONFIGURATION(
        "An edge is being classified as internal that has already been "
        "classified as border");
  }
}

/**
 * This function contains the implementation detail of ComputeVisiblePatch()
 * function. It should not be called by any function other than
 * ComputeVisiblePatch().
 * The parameters are documented as in ComputeVisiblePatch() but has the
 * additional parameter: hidden_faces. Every visited face will be categorized
 * explicitly as either visible or hidden.
 * By design, whatever classification is _first_ given to a face (visible or
 * hidden) it can't change. This doesn't purport that the first classification
 * is more correct than any subsequent code that might otherwise classify it
 * differently. This simply precludes the possibility of classifying edges
 * multiple ways.
 */
static void ComputeVisiblePatchRecursive(
    const ccd_pt_t& polytope, ccd_pt_face_t& f, int edge_index,
    const ccd_vec3_t& query_point,
    std::unordered_set<ccd_pt_edge_t*>* border_edges,
    std::unordered_set<ccd_pt_face_t*>* visible_faces,
    std::unordered_set<ccd_pt_face_t*>* hidden_faces,
    std::unordered_set<ccd_pt_edge_t*>* internal_edges) {
  ccd_pt_edge_t* edge = f.edge[edge_index];

  ccd_pt_face_t* g = edge->faces[0] == &f ? edge->faces[1] : edge->faces[0];
  assert(g != nullptr);

  bool is_visible = visible_faces->count(g) > 0;
  bool is_hidden = hidden_faces->count(g) > 0;
  assert(!(is_visible && is_hidden));

  // First check to see if face `g` has been classifed already.
  if (is_visible) {
    // Face f is visible (prerequisite), and g has been previously
    // marked visible (via a different path); their shared edge should be
    // marked internal.
    ClassifyInternalEdge(edge, border_edges, internal_edges);
    return;
  }

  if (is_hidden) {
    // Face *has* already been classified as hidden; we just need to classify
    // the edge.
    ClassifyBorderEdge(edge, border_edges, internal_edges);
    return;
  }

  // Face `g` has not been classified yet. Try to classify it as visible (i.e.,
  // the `query_point` lies outside of this face).
  is_visible = isOutsidePolytopeFace(&polytope, g, &query_point);
  if (!is_visible) {
    // Face `g` is *apparently* not visible from the query point. However, it
    // might _still_ be considered visible. The query point could be co-linear
    // with `edge` which, by definition means that `g` *is* visible and
    // numerical error led to considering it hidden. We can detect this case
    // if the triangle formed by edge and query point has zero area.
    //
    // It may be that the previous designation that `f` was visible was a
    // mistake and now face `g` is inheriting that visible status. This is a
    // conservative resolution that will prevent us from creating co-planar
    // faces (at the cost of a longer border).
    is_visible = triangle_area_is_zero(query_point, edge->vertex[0]->v.v,
                                       edge->vertex[1]->v.v);
  }

  if (is_visible) {
    visible_faces->insert(g);
    ClassifyInternalEdge(edge, border_edges, internal_edges);
    for (int i = 0; i < 3; ++i) {
      if (g->edge[i] != edge) {
        // One of the neighbouring face is `f`, so do not need to visit again.
        ComputeVisiblePatchRecursive(polytope, *g, i, query_point, border_edges,
                                     visible_faces, hidden_faces,
                                     internal_edges);
      }
    }
  } else {
    // No logic could classify the face as visible, so mark it hidden and
    // mark the edge as a border edge.
    ClassifyBorderEdge(edge, border_edges, internal_edges);
    hidden_faces->insert(g);
  }
}

/** Defines the "visible" patch on the given convex `polytope` (with respect to
 * the given `query_vertex` which is a point outside the polytope.)
 *
 * A patch is characterized by:
 *   - a contiguous set of visible faces
 *   - internal edges (the edges for which *both* adjacent faces are in the
 * patch)
 *   - border edges (edges for which only *one* adjacent face is in the patch)
 *
 * A face `f` (with vertices `(v₀, v₁, v₂)` and outward pointing normal `n̂`) is
 * "visible" and included in the patch if, for query vertex `q`:
 *
 *    `n̂ ⋅ (q - v₀) > 0`
 *
 * Namely the query vertex `q` is on the "outer" side of the face `f`.
 *
 * @param polytope             The polytope to evaluate.
 * @param f                    A face known to be visible to the query point.
 * @param query_point          The point from which visibility is evaluated.
 * @param[out] border_edges    The collection of patch border edges.
 * @param[out] visible_faces   The collection of patch faces.
 * @param[out] internal_edges  The collection of internal edges.
 * @throws UnexpectedConfigurationException in debug builds if the resulting
 * patch is not consistent.
 *
 * @pre The `polytope` is convex.
 * @pre The face `f` is visible from `query_point`.
 * @pre Output parameters are non-null.
 * TODO(hongkai.dai@tri.global) Replace patch computation with patch deletion
 * and return border edges as an optimization.
 * TODO(hongkai.dai@tri.global) Consider caching results of per-face visibility
 * status to prevent redundant recalculation -- or by associating the face
 * normal with the face.
 */
static void ComputeVisiblePatch(
    const ccd_pt_t& polytope, ccd_pt_face_t& f,
    const ccd_vec3_t& query_point,
    std::unordered_set<ccd_pt_edge_t*>* border_edges,
    std::unordered_set<ccd_pt_face_t*>* visible_faces,
    std::unordered_set<ccd_pt_edge_t*>* internal_edges) {
  assert(border_edges);
  assert(visible_faces);
  assert(internal_edges);
  assert(border_edges->empty());
  assert(visible_faces->empty());
  assert(internal_edges->empty());
  assert(isOutsidePolytopeFace(&polytope, &f, &query_point));
  std::unordered_set<ccd_pt_face_t*> hidden_faces;
  visible_faces->insert(&f);
  for (int edge_index = 0; edge_index < 3; ++edge_index) {
    ComputeVisiblePatchRecursive(polytope, f, edge_index, query_point,
                                 border_edges, visible_faces, &hidden_faces,
                                 internal_edges);
  }
#ifndef NDEBUG
  // TODO(SeanCurtis-TRI): Extend the sanity check to include hidden_faces.
  if (!ComputeVisiblePatchRecursiveSanityCheck(
          polytope, *border_edges, *visible_faces, *internal_edges)) {
    FCL_THROW_FAILED_AT_THIS_CONFIGURATION(
        "The visible patch failed its sanity check");
  }
#endif
}

/** Expands the polytope by adding a new vertex `newv` to the polytope. The
 * new polytope is the convex hull of the new vertex together with the old
 * polytope. This new polytope includes new edges (by connecting the new vertex
 * with existing vertices) and new faces (by connecting the new vertex with
 * existing edges). We only keep the edges and faces that are on the boundary
 * of the new polytope. The edges/faces on the original polytope that would be
 * interior to the new convex hull are discarded.
 * @param[in/out] polytope The polytope.
 * @param[in] el A feature that is visible from the point `newv` and contains
 * the polytope boundary point that is closest to the origin. This feature
 * should be either a face or an edge. A face is visible from a point outside
 * the original polytope, if the point is on the "outer" side of the face. If an
 * edge is visible from that point, at least one of its neighbouring faces is
 * visible. This feature contains the point that is closest to the origin on
 * the boundary of the polytope. If the feature is an edge, and the two
 * neighbouring faces of that edge are not co-planar, then the origin must lie
 * on that edge. The feature should not be a vertex, as that would imply the two
 * objects are in touching contact, causing the algorithm to exit before calling
 * expandPolytope() function.
 * @param[in] newv The new vertex add to the polytope.
 * @retval status Returns 0 on success. Returns -2 otherwise.
 * @throws UnexpectedConfigurationException if expanding is meaningless either
 * because 1) the nearest feature is a vertex, 2) the new vertex lies on
 * an edge of the current polytope, or 3) the visible feature is an edge with
 * one or more adjacent faces with no area.
 */
static int expandPolytope(ccd_pt_t *polytope, ccd_pt_el_t *el,
                          const ccd_support_t *newv)
{
  // The outline of the algorithm is as follows:
  //  1. Compute the visible patch relative to the new vertex (See
  //  ComputeVisiblePatch() for details).
  //  2. Delete the faces and internal edges.
  //  3. Build a new face from each border edge and the new vertex.

  // To remove all faces that can be seen from the new vertex, we start with the
  // face on which the closest point lives, and then do a depth-first search on
  // its neighbouring triangles, until the triangle cannot be seen from the new
  // vertex.
  // TODO(hongkai.dai@tri.global): it is inefficient to store visible
  // faces/edges. A better implementation should remove visible faces and
  // internal edges inside ComputeVisiblePatch() function, when traversing the
  // faces on the polytope. We focus on the correctness in the first place.
  // Later when we make sure that the whole EPA implementation is bug free, we
  // will improve the performance.

  ccd_pt_face_t* start_face = NULL;

  if (el->type == CCD_PT_VERTEX) {
    FCL_THROW_FAILED_AT_THIS_CONFIGURATION(
        "The visible feature is a vertex. This should already have been "
        "identified as a touching contact");
  }
  // Start with the face on which the closest point lives
  if (el->type == CCD_PT_FACE) {
    start_face = reinterpret_cast<ccd_pt_face_t*>(el);
  } else if (el->type == CCD_PT_EDGE) {
    // Check the two neighbouring faces of the edge.
    ccd_pt_face_t* f[2];
    ccdPtEdgeFaces(reinterpret_cast<ccd_pt_edge_t*>(el), &f[0], &f[1]);
    if (isOutsidePolytopeFace(polytope, f[0], &newv->v)) {
      start_face = f[0];
    } else if (isOutsidePolytopeFace(polytope, f[1], &newv->v)) {
      start_face = f[1];
    } else {
      FCL_THROW_FAILED_AT_THIS_CONFIGURATION(
          "Both the nearest point and the new vertex are on an edge, thus the "
          "nearest distance should be 0. This is touching contact, and should "
          "already have been identified");
    }
  }

  std::unordered_set<ccd_pt_face_t*> visible_faces;
  std::unordered_set<ccd_pt_edge_t*> internal_edges;
  std::unordered_set<ccd_pt_edge_t*> border_edges;
  ComputeVisiblePatch(*polytope, *start_face, newv->v, &border_edges,
                      &visible_faces, &internal_edges);

  // Now remove all the obsolete faces.
  // TODO(hongkai.dai@tri.global): currently we need to loop through each face
  // in visible_faces, and then do a linear search in the list pt->faces to
  // delete `face`. It would be better if we only loop through the list
  // polytope->faces for once. Same for the edges.
  for (const auto& f : visible_faces) {
    ccdPtDelFace(polytope, f);
  }

  // Now remove all the obsolete edges.
  for (const auto& e : internal_edges) {
    ccdPtDelEdge(polytope, e);
  }

  // Note: this does not delete any vertices that were on the interior of the
  // deleted patch. There are no longer any faces or edges that reference them.
  // Technically, they are still part of the polytope but have no effect (except
  // for footprint in memory). It's just as simple to leave them, knowing the
  // whole polytope will be destroyed when we're done with the query.

  // A vertex cannot be obsolete, since a vertex is always on the boundary of
  // the Minkowski difference A ⊖ B.
  // TODO(hongkai.dai@tri.global): as a sanity check, we should make sure that
  // all vertices has at least one face/edge invisible from the new vertex
  // `newv`.

  // Now add the new vertex.
  ccd_pt_vertex_t* new_vertex = ccdPtAddVertex(polytope, newv);

  // Now add the new edges and faces, by connecting the new vertex with vertices
  // on border_edges. map_vertex_to_new_edge maps a vertex on the silhouette
  // edges to a new edge, with one end being the new vertex, and the other end
  // being that vertex on the silhouette edges.
  std::unordered_map<ccd_pt_vertex_t*, ccd_pt_edge_t*> map_vertex_to_new_edge;
  for (const auto& border_edge : border_edges) {
    ccd_pt_edge_t* e[2];  // The two new edges added by connecting new_vertex
                          // to the two vertices on border_edge.
    for (int i = 0; i < 2; ++i) {
      auto it = map_vertex_to_new_edge.find(border_edge->vertex[i]);
      if (it == map_vertex_to_new_edge.end()) {
        // This edge has not been added yet.
        e[i] = ccdPtAddEdge(polytope, new_vertex, border_edge->vertex[i]);
        map_vertex_to_new_edge.emplace_hint(it, border_edge->vertex[i],
                                            e[i]);
      } else {
        e[i] = it->second;
      }
    }
    // Now add the face.
    ccdPtAddFace(polytope, border_edge, e[0], e[1]);
  }

  return 0;
}

/** In each iteration of EPA algorithm, given the nearest point on the polytope
 * boundary to the origin, a support direction will be computed, to find the
 * support of the Minkowski difference A ⊖ B along that direction, so as to
 * expand the polytope.
 * @param polytope The polytope contained in A ⊖ B.
 * @param nearest_feature The feature containing the nearest point on the
 * boundary of the polytope to the origin.
 * @retval dir The support direction along which to expand the polytope. Notice
 * that dir is a normalized vector.
 * @throws UnexpectedConfigurationException if the nearest feature is a vertex.
 */
static ccd_vec3_t supportEPADirection(const ccd_pt_t* polytope,
                                      const ccd_pt_el_t* nearest_feature) {
  /*
  If we denote the nearest point as v, when v is not the origin, then the
  support direction is v. If v is the origin, then v should be an interior
  point on a face, and the support direction is the normal of that face,
  pointing outward from the polytope.
  */
  ccd_vec3_t dir;
  if (ccdIsZero(nearest_feature->dist)) {
    // nearest point is the origin.
    switch (nearest_feature->type) {
      case CCD_PT_VERTEX: {
        FCL_THROW_FAILED_AT_THIS_CONFIGURATION(
            "The nearest point to the origin is a vertex of the polytope. This "
            "should be identified as a touching contact");
        break;
      }
      case CCD_PT_EDGE: {
        // When the nearest point is on an edge, the origin must be on that
        // edge. The support direction could be in the range between
        // edge.faces[0].normal and edge.faces[1].normal, where the face normals
        // point outward from the polytope. In this implementation, we
        // arbitrarily choose faces[0] normal.
        const ccd_pt_edge_t* edge =
            reinterpret_cast<const ccd_pt_edge_t*>(nearest_feature);
        dir = faceNormalPointingOutward(polytope, edge->faces[0]);
        break;
      }
      case CCD_PT_FACE: {
        // If origin is an interior point of a face, then choose the normal of
        // that face as the sample direction.
        const ccd_pt_face_t* face =
            reinterpret_cast<const ccd_pt_face_t*>(nearest_feature);
        dir = faceNormalPointingOutward(polytope, face);
        break;
      }
    }
  } else {
    ccdVec3Copy(&dir, &(nearest_feature->witness));
  }
  ccdVec3Normalize(&dir);
  return dir;
}

/** Finds next support point (and stores it in out argument).
 * @param[in] polytope The current polytope contained inside the Minkowski
 * difference A ⊖ B.
 * @param[in] obj1 Geometric object A.
 * @param[in] obj2 Geometric object B.
 * @param[in] ccd The libccd solver.
 * @param[in] el The polytope boundary feature that contains the point nearest
 * to the origin.
 * @param[out] out The next support point.
 * @retval status If the next support point is found, then returns 0; otherwise
 * returns -1. There are several reasons why the next support point is not
 * found:
 * 1. If the nearest point on the boundary of the polytope to the origin is a
 * vertex of the polytope. Then we know the two objects A and B are in touching
 * contact.
 * 2. If the difference between the upper bound and lower bound of the distance
 * is below ccd->epa_tolerance, then we converge to a distance whose
 * difference from the real distance is less than ccd->epa_tolerance.
 */
static int nextSupport(const ccd_pt_t* polytope, const void* obj1,
                       const void* obj2, const ccd_t* ccd,
                       const ccd_pt_el_t* el, ccd_support_t* out) {
  ccd_vec3_t *a, *b, *c;

  if (el->type == CCD_PT_VERTEX) return -1;

  const ccd_vec3_t dir = supportEPADirection(polytope, el);

  __ccdSupport(obj1, obj2, &dir, ccd, out);

  // Compute distance of support point in the support direction, so we can
  // determine whether we expanded a polytope surrounding the origin a bit.
  const ccd_real_t dist = ccdVec3Dot(&out->v, &dir);

  // el->dist is the squared distance from the feature "el" to the origin..
  // dist is an upper bound on the distance from the boundary of the Minkowski
  // difference to the origin, and sqrt(el->dist) is a lower bound of that
  // distance.
  if (dist - std::sqrt(el->dist) < ccd->epa_tolerance) return -1;

  ccd_real_t dist_squared{};
  if (el->type == CCD_PT_EDGE) {
    // fetch end points of edge
    ccdPtEdgeVec3(reinterpret_cast<const ccd_pt_edge_t*>(el), &a, &b);

    // get distance from segment
    dist_squared = ccdVec3PointSegmentDist2(&out->v, a, b, NULL);
  } else {  // el->type == CCD_PT_FACE
    // fetch vertices of triangle face
    ccdPtFaceVec3(reinterpret_cast<const ccd_pt_face_t*>(el), &a, &b, &c);

    // check if new point can significantly expand polytope
    ccd_vec3_t point_projection_on_triangle_unused;
    dist_squared = ccdVec3PointTriDist2(&out->v, a, b, c,
                                        &point_projection_on_triangle_unused);
  }

  if (std::sqrt(dist_squared) < ccd->epa_tolerance) return -1;

  return 0;
}


static int __ccdGJK(const void *obj1, const void *obj2,
                    const ccd_t *ccd, ccd_simplex_t *simplex)
{
  unsigned long iterations;
  ccd_vec3_t dir; // direction vector
  ccd_support_t last; // last support point
  int do_simplex_res;

  // initialize simplex struct
  ccdSimplexInit(simplex);

  // get first direction
  ccd->first_dir(obj1, obj2, &dir);
  // get first support point
  __ccdSupport(obj1, obj2, &dir, ccd, &last);
  // and add this point to simplex as last one
  ccdSimplexAdd(simplex, &last);

  // set up direction vector to as (O - last) which is exactly -last
  ccdVec3Copy(&dir, &last.v);
  ccdVec3Scale(&dir, -CCD_ONE);

  // start iterations
  for (iterations = 0UL; iterations < ccd->max_iterations; ++iterations) {
    // obtain support point
    __ccdSupport(obj1, obj2, &dir, ccd, &last);

    // check if farthest point in Minkowski difference in direction dir
    // isn't somewhere before origin (the test on negative dot product)
    // - because if it is, objects are not intersecting at all.
    if (ccdVec3Dot(&last.v, &dir) < CCD_ZERO){
      return -1; // intersection not found
    }

    // add last support vector to simplex
    ccdSimplexAdd(simplex, &last);

    // if doSimplex returns 1 if objects intersect, -1 if objects don't
    // intersect and 0 if algorithm should continue
    do_simplex_res = doSimplex(simplex, &dir);
    if (do_simplex_res == 1){
      return 0; // intersection found
    }else if (do_simplex_res == -1){
      return -1; // intersection not found
    }

    if (ccdIsZero(ccdVec3Len2(&dir))){
      return -1; // intersection not found
    }
  }

  // intersection wasn't found
  return -1;
}

/**
 * When the nearest feature of a polytope to the origin is an edge, and the
 * origin is inside the polytope, it implies one of the following conditions
 * 1. The origin lies exactly on that edge
 * 2. The two neighbouring faces of that edge are coplanar, and the projection
 * of the origin onto that plane is on the edge.
 * At times libccd incorrectly claims that the nearest feature is an edge.
 * Inside this function, we will verify if one of these two conditions are true.
 * If not, we will modify the nearest feature stored inside @p polytope, such
 * that it stores the correct nearest feature and distance.
 * @note we assume that even if the edge is not the correct nearest feature, the
 * correct one should be one of the neighbouring faces of that edge. Namely the
 * libccd solution is just slightly wrong.
 * @param polytope The polytope whose nearest feature is being verified (and
 * corrected if the edge should not be nearest feature).
 * @note Only call this function in the EPA functions, where the origin should
 * be inside the polytope.
 */
static void validateNearestFeatureOfPolytopeBeingEdge(ccd_pt_t* polytope) {
  assert(polytope->nearest_type == CCD_PT_EDGE);

  // We define epsilon to include an additional bit of noise. The goal is to
  // pick the smallest epsilon possible. This factor of two proved necessary
  // due to unit test behavior on the mac. In the future, as we collect
  // more evidence, it may be necessary to increase to more bits. But the need
  // should always be demonstrable and not purely theoretical.
  constexpr ccd_real_t kEps = 2 * constants<ccd_real_t>::eps();

  // Only verify the feature if the nearest feature is an edge.

  const ccd_pt_edge_t* const nearest_edge =
      reinterpret_cast<ccd_pt_edge_t*>(polytope->nearest);
  // Find the outward normals on the two neighbouring faces of the edge, if
  // the origin is on the "inner" side of these two faces, then we regard the
  // origin to be inside the polytope. Note that these face_normals are
  // normalized.
  std::array<ccd_vec3_t, 2> face_normals;
  std::array<double, 2> origin_to_face_distance;

  // We define the plane equation using vertex[0]. If vertex[0] is far away
  // from the origin, it can magnify rounding error. We scale epsilon to account
  // for this possibility.
  const ccd_real_t v0_dist =
      std::sqrt(ccdVec3Len2(&nearest_edge->vertex[0]->v.v));
  const ccd_real_t plane_threshold =
      kEps * std::max(static_cast<ccd_real_t>(1.0), v0_dist);

  for (int i = 0; i < 2; ++i) {
    face_normals[i] =
        faceNormalPointingOutward(polytope, nearest_edge->faces[i]);
    ccdVec3Normalize(&face_normals[i]);
    // If the origin o is on the "inner" side of the face, then
    // n̂ ⋅ (o - vₑ) ≤ 0 or, with simplification, -n̂ ⋅ vₑ ≤ 0 (since n̂ ⋅ o = 0).
    origin_to_face_distance[i] =
        -ccdVec3Dot(&face_normals[i], &nearest_edge->vertex[0]->v.v);
    // If the origin lies *on* the edge, then it also lies on the two adjacent
    // faces. Rather than failing on strictly *positive* signed distance, we
    // introduce an epsilon threshold. This usage of epsilon is to account for a
    // discrepancy in the signed distance computation. How GJK (and partially
    // EPA) compute the signed distance from origin to face may *not* be exactly
    // the same as done in this test (i.e. for a given set of vertices, the
    // plane equation can be defined various ways. Those ways are
    // *mathematically* equivalent but numerically will differ due to rounding).
    // We account for those differences by allowing a very small positive signed
    // distance to be considered zero. We assume that the GJK/EPA code
    // ultimately classifies inside/outside around *zero* and *not* epsilon.
    if (origin_to_face_distance[i] > plane_threshold) {
      FCL_THROW_FAILED_AT_THIS_CONFIGURATION(
          "The origin is outside of the polytope. This should already have "
          "been identified as separating.");
    }
  }

  // We know the reported squared distance to the edge. If that distance is
  // functionally zero, then the edge must *truly* be the nearest feature.
  // If it isn't, then it must be one of the adjacent faces.
  const bool is_edge_closest_feature = nearest_edge->dist < kEps * kEps;

  if (!is_edge_closest_feature) {
    // We assume that libccd is not crazily wrong. Although the closest
    // feature is not the edge, it is near that edge. Hence we select the
    // neighboring face that is closest to the origin.
    polytope->nearest_type = CCD_PT_FACE;
    // Note origin_to_face_distance is the *signed* distance and it is
    // guaranteed to be negative if we are here, hence sense of this
    // comparison is reversed.
    const int closest_face =
        origin_to_face_distance[0] < origin_to_face_distance[1] ? 1 : 0;
    polytope->nearest =
        reinterpret_cast<ccd_pt_el_t*>(nearest_edge->faces[closest_face]);
    // polytope->nearest_dist stores the SQUARED distance.
    polytope->nearest_dist = pow(origin_to_face_distance[closest_face], 2);
  }
}

static int __ccdEPA(const void *obj1, const void *obj2,
                    const ccd_t *ccd,
                    ccd_simplex_t* simplex,
                    ccd_pt_t *polytope, ccd_pt_el_t **nearest)
{
    ccd_support_t supp; // support point
    int ret, size;


    ret = 0;
    *nearest = NULL;

    // transform simplex to polytope - simplex won't be used anymore
    size = ccdSimplexSize(simplex);
    if (size == 4){
        ret = simplexToPolytope4(obj1, obj2, ccd, simplex, polytope, nearest);
    } else if (size == 3) {
      ret = convert2SimplexToTetrahedron(obj1, obj2, ccd, simplex, polytope,
                                         nearest);
    }else{ // size == 2
        ret = simplexToPolytope2(obj1, obj2, ccd, simplex, polytope, nearest);
    }


    if (ret == -1){
        // touching contact
        return 0;
    }else if (ret == -2){
        // failed memory allocation
        return -2;
    }

    while (1) {
      // get triangle nearest to origin
      *nearest = ccdPtNearest(polytope);
      if (polytope->nearest_type == CCD_PT_EDGE) {
        // When libccd thinks the nearest feature is an edge, that is often
        // wrong, hence we validate the nearest feature by ourselves.
        // TODO remove this validation step when we can reliably compute the
        // nearest feature of a polytope.
        validateNearestFeatureOfPolytopeBeingEdge(polytope);
        *nearest = ccdPtNearest(polytope);
      }

      // get next support point
      if (nextSupport(polytope, obj1, obj2, ccd, *nearest, &supp) != 0) {
        break;
      }

      // expand nearest triangle using new point - supp
      if (expandPolytope(polytope, *nearest, &supp) != 0) return -2;
    }

    return 0;
}

/** Given a single support point, `q`, extract the point `p1` and `p2`, the
 points on object 1 and 2, respectively, in the support data of `q`.  */
static void extractObjectPointsFromPoint(ccd_support_t *q, ccd_vec3_t *p1,
                                         ccd_vec3_t *p2) {
  // TODO(SeanCurtis-TRI): Determine if I should be demanding that p1 and p2
  // are defined.
  // Closest points are the ones stored in the simplex
  if (p1) *p1 = q->v1;
  if (p2) *p2 = q->v2;
}

/** Given two support points which define a line segment (`a` and `b`), and a
 point on that line segment `p`, computes the points `p1` and `p2`, the points
 on object 1 and 2, respectively, in the support data which correspond to `p`.
 @pre `p = a + s(b - a), 0 <= s <= 1`  */
static void extractObjectPointsFromSegment(ccd_support_t *a, ccd_support_t *b,
                                           ccd_vec3_t *p1, ccd_vec3_t *p2,
                                           ccd_vec3_t *p) {
  // Closest points lie on the segment defined by the points in the simplex
  // Let the segment be defined by points A and B. We can write p as
  //
  // p = A + s*AB, 0 <= s <= 1
  // p - A = s*AB
  ccd_vec3_t AB;
  ccdVec3Sub2(&AB, &(b->v), &(a->v));

  // This defines three equations, but we only need one. Taking the i-th
  // component gives
  //
  // p_i - A_i = s*AB_i.
  //
  // Thus, s is given by
  //
  // s = (p_i - A_i)/AB_i.
  //
  // To avoid dividing by an AB_i ≪ 1, we choose i such that |AB_i| is
  // maximized
  ccd_real_t abs_AB_x{std::abs(ccdVec3X(&AB))};
  ccd_real_t abs_AB_y{std::abs(ccdVec3Y(&AB))};
  ccd_real_t abs_AB_z{std::abs(ccdVec3Z(&AB))};

  ccd_real_t A_i, AB_i, p_i;
  if (abs_AB_x >= abs_AB_y && abs_AB_x >= abs_AB_z) {
    A_i = ccdVec3X(&(a->v));
    AB_i = ccdVec3X(&AB);
    p_i = ccdVec3X(p);
  } else if (abs_AB_y >= abs_AB_z) {
    A_i = ccdVec3Y(&(a->v));
    AB_i = ccdVec3Y(&AB);
    p_i = ccdVec3Y(p);
  } else {
    A_i = ccdVec3Z(&(a->v));
    AB_i = ccdVec3Z(&AB);
    p_i = ccdVec3Z(p);
  }

  if (std::abs(AB_i) < constants<ccd_real_t>::eps()) {
    // Points are coincident; treat as a single point.
    extractObjectPointsFromPoint(a, p1, p2);
    return;
  }

  auto calc_p = [](ccd_vec3_t *p_a, ccd_vec3_t *p_b, ccd_vec3_t *p,
                   ccd_real_t s) {
    ccd_vec3_t sAB;
    ccdVec3Sub2(&sAB, p_b, p_a);
    ccdVec3Scale(&sAB, s);
    ccdVec3Copy(p, p_a);
    ccdVec3Add(p, &sAB);
  };

  // TODO(SeanCurtis-TRI): If p1 or p2 is null, there seems little point in
  // calling this method. It seems that both of these being non-null should be
  // a *requirement*. Determine that this is the case and do so.
  ccd_real_t s = (p_i - A_i) / AB_i;

  if (p1) {
    // p1 = A1 + s*A1B1
    calc_p(&(a->v1), &(b->v1), p1, s);
  }
  if (p2) {
    // p2 = A2 + s*A2B2
    calc_p(&(a->v2), &(b->v2), p2, s);
  }
}

/** Returns the points `p1` and `p2` on the original shapes that correspond to
 point `p` in the given simplex.
 @pre simplex size <= 3.
 @pre p lies _on_ the simplex (i.e., within the triangle, line segment, or is
      coincident with the point).  */
static void extractClosestPoints(ccd_simplex_t *simplex, ccd_vec3_t *p1,
                                 ccd_vec3_t *p2, ccd_vec3_t *p) {
  const int simplex_size = ccdSimplexSize(simplex);
  assert(simplex_size <= 3);
  if (simplex_size == 1)
  {
    extractObjectPointsFromPoint(&simplex->ps[0], p1, p2);
  }
  else if (simplex_size == 2)
  {
    extractObjectPointsFromSegment(&simplex->ps[0], &simplex->ps[1], p1, p2, p);
  }
  else // simplex_size == 3
  {
    if (triangle_area_is_zero(simplex->ps[0].v, simplex->ps[1].v,
                              simplex->ps[2].v)) {
      // The triangle is degenerate; compute the nearest point to a line
      // segment. The segment is defined by the most-distant vertex pair.
      int a_index, b_index;
      ccd_vec3_t AB, AC, BC;
      ccdVec3Sub2(&AB, &(simplex->ps[1].v), &(simplex->ps[0].v));
      ccdVec3Sub2(&AC, &(simplex->ps[2].v), &(simplex->ps[0].v));
      ccdVec3Sub2(&BC, &(simplex->ps[2].v), &(simplex->ps[1].v));
      ccd_real_t AB_len2 = ccdVec3Len2(&AB);
      ccd_real_t AC_len2 = ccdVec3Len2(&AC);
      ccd_real_t BC_len2 = ccdVec3Len2(&BC);
      if (AB_len2 >= AC_len2 && AB_len2 >= BC_len2) {
        a_index = 0;
        b_index = 1;
      } else if (AC_len2 >= AB_len2 && AC_len2 >= BC_len2) {
        a_index = 0;
        b_index = 2;
      } else {
        a_index = 1;
        b_index = 2;
      }
      extractObjectPointsFromSegment(&simplex->ps[a_index],
                                     &simplex->ps[b_index], p1, p2, p);
      return;
    }

    // Compute the barycentric coordinates of point p in triangle ABC.
    //
    //             A
    //             ╱╲                p = αA + βB + γC
    //            ╱ |╲
    //           ╱  | ╲              α = 1 - β - γ
    //          ╱ p |  ╲             β = AREA(pAC) / AREA(ABC)
    //         ╱   / \  ╲            γ = AREA(pAB) / AREA(ABC)
    //        ╱__/_____\_╲
    //      B             C          AREA(XYZ) = |r_XY × r_XZ| / 2
    //
    //  Rewrite coordinates in terms of cross products.
    //
    //    β = AREA(pAC) / AREA(ABC) = |r_Ap × r_AC| / |r_AB × r_AC|
    //    γ = AREA(pAB) / AREA(ABC) = |r_AB × r_Ap| / |r_AB × r_AC|
    //
    // NOTE: There are multiple options for the cross products, these have been
    // selected to re-use as many symbols as possible.
    //
    // Solving for β and γ:
    //
    //  β = |r_Ap × r_AC| / |r_AB × r_AC|
    //  β = |r_Ap × r_AC| / |n|                  n ≙ r_AB × r_AC, n̂ = n / |n|
    //  β = n̂·(r_Ap × r_AC) / n̂·n                This step arises from the fact
    //                                           that (r_Ap × r_AC) and n point
    //                                           in the same direction. It
    //                                           allows us to take a single sqrt
    //                                           instead of three.
    //  β = (n/|n|)·(r_Ap × r_AC) / (n/|n|)·n
    //  β = n·(r_Ap × r_AC) / n·n
    //  β = n·(r_Ap × r_AC) / |n|²
    //
    // A similar process to solve for gamma
    //  γ = n·(r_AB × r_Ap) / |n|²

    // Compute n and |n|².
    ccd_vec3_t r_AB, r_AC, n;
    ccdVec3Sub2(&r_AB, &(simplex->ps[1].v), &(simplex->ps[0].v));
    ccdVec3Sub2(&r_AC, &(simplex->ps[2].v), &(simplex->ps[0].v));
    ccdVec3Cross(&n, &r_AB, &r_AC);
    ccd_real_t norm_squared_n{ccdVec3Len2(&n)};

    // Compute r_Ap.
    ccd_vec3_t r_Ap;
    ccdVec3Sub2(&r_Ap, p, &(simplex->ps[0].v));

    // Compute the cross products in the numerators.
    ccd_vec3_t r_Ap_cross_r_AC, r_AB_cross_r_Ap;
    ccdVec3Cross(&r_Ap_cross_r_AC, &r_Ap, &r_AC);  // r_Ap × r_AC
    ccdVec3Cross(&r_AB_cross_r_Ap, &r_AB, &r_Ap);  // r_AB × r_Ap

    // Compute beta and gamma.
    ccd_real_t beta{ccdVec3Dot(&n, &r_Ap_cross_r_AC) / norm_squared_n};
    ccd_real_t gamma{ccdVec3Dot(&n, &r_AB_cross_r_Ap) / norm_squared_n};

    // Evaluate barycentric interpolation (with the locally defined barycentric
    // coordinates).
    auto interpolate = [&beta, &gamma](const ccd_vec3_t& r_WA,
                                       const ccd_vec3_t& r_WB,
                                       const ccd_vec3_t& r_WC,
                                       ccd_vec3_t* r_WP) {
      // r_WP = r_WA + β * r_AB + γ * r_AC
      ccdVec3Copy(r_WP, &r_WA);

      ccd_vec3_t beta_r_AB;
      ccdVec3Sub2(&beta_r_AB, &r_WB, &r_WA);
      ccdVec3Scale(&beta_r_AB, beta);
      ccdVec3Add(r_WP, &beta_r_AB);

      ccd_vec3_t  gamma_r_AC;
      ccdVec3Sub2(&gamma_r_AC, &r_WC, &r_WA);
      ccdVec3Scale(&gamma_r_AC, gamma);
      ccdVec3Add(r_WP, &gamma_r_AC);
    };

    if (p1) {
      interpolate(simplex->ps[0].v1, simplex->ps[1].v1, simplex->ps[2].v1, p1);
    }

    if (p2) {
      interpolate(simplex->ps[0].v2, simplex->ps[1].v2, simplex->ps[2].v2, p2);
    }
  }
}

// Computes the distance between two non-penetrating convex objects, `obj1` and
// `obj2` based on the warm-started witness simplex, returning the distance and
// nearest points on each object.
// @param obj1 The first convex object.
// @param obj2 The second convex object.
// @param ccd The libccd configuration.
// @param simplex A witness to the objects' separation generated by the GJK
// algorithm. NOTE: the simplex is not necessarily sufficiently refined to
// report the actual distance and may be further refined in this method.
// @param p1 If the objects are non-penetrating, the point on the surface of
// obj1 closest to obj2 (expressed in the world frame).
// @param p2 If the objects are non-penetrating, the point on the surface of
// obj2 closest to obj1 (expressed in the world frame).
// @returns The minimum distance between the two objects. If they are
// penetrating, -1 is returned.
static inline ccd_real_t _ccdDist(const void *obj1, const void *obj2,
                                  const ccd_t *ccd,
                                  ccd_simplex_t* simplex,
                                  ccd_vec3_t* p1, ccd_vec3_t* p2)
{
  ccd_real_t last_dist = CCD_REAL_MAX;

  for (unsigned long iterations = 0UL; iterations < ccd->max_iterations;
       ++iterations) {
    ccd_vec3_t closest_p; // The point on the simplex that is closest to the
                          // origin.
    ccd_real_t dist;
    // get a next direction vector
    // we are trying to find out a point on the minkowski difference
    // that is nearest to the origin, so we obtain a point on the
    // simplex that is nearest and try to exapand the simplex towards
    // the origin
    if (ccdSimplexSize(simplex) == 1)
    {
      ccdVec3Copy(&closest_p, &ccdSimplexPoint(simplex, 0)->v);
      dist = ccdVec3Len2(&ccdSimplexPoint(simplex, 0)->v);
      dist = CCD_SQRT(dist);
    }
    else if (ccdSimplexSize(simplex) == 2)
    {
      dist = ccdVec3PointSegmentDist2(ccd_vec3_origin,
                                      &ccdSimplexPoint(simplex, 0)->v,
                                      &ccdSimplexPoint(simplex, 1)->v,
                                      &closest_p);
      dist = CCD_SQRT(dist);
    }
    else if (ccdSimplexSize(simplex) == 3)
    {
      dist = ccdVec3PointTriDist2(ccd_vec3_origin,
                                  &ccdSimplexPoint(simplex, 0)->v,
                                  &ccdSimplexPoint(simplex, 1)->v,
                                  &ccdSimplexPoint(simplex, 2)->v,
                                  &closest_p);
      dist = CCD_SQRT(dist);
    }
    else
    { // ccdSimplexSize(&simplex) == 4
      dist = simplexReduceToTriangle(simplex, last_dist, &closest_p);
    }

    // check whether we improved for at least a minimum tolerance
    if ((last_dist - dist) < ccd->dist_tolerance)
    {
      extractClosestPoints(simplex, p1, p2, &closest_p);
      return dist;
    }

    // point direction towards the origin
    ccd_vec3_t dir; // direction vector
    ccdVec3Copy(&dir, &closest_p);
    ccdVec3Scale(&dir, -CCD_ONE);
    ccdVec3Normalize(&dir);

    // find out support point
    ccd_support_t last; // last support point
    __ccdSupport(obj1, obj2, &dir, ccd, &last);

    // record last distance
    last_dist = dist;

    // check whether we improved for at least a minimum tolerance
    // this is here probably only for a degenerate cases when we got a
    // point that is already in the simplex
    dist = ccdVec3Len2(&last.v);
    dist = CCD_SQRT(dist);
    if (CCD_FABS(last_dist - dist) < ccd->dist_tolerance)
    {
      extractClosestPoints(simplex, p1, p2, &closest_p);
      return last_dist;
    }

    // add a point to simplex
    ccdSimplexAdd(simplex, &last);
  }

  return -CCD_REAL(1.);
}

/**
 * Given the nearest point on the polytope inside the Minkowski difference
 * A ⊖ B, returns the point p1 on geometric object A and p2 on geometric object
 * B, such that p1 is the deepest penetration point on A, and p2 is the deepest
 * penetration point on B.
 * @param[in] nearest The feature with the point that is nearest to the origin
 * on the boundary of the polytope.
 * @param[out] p1 the deepest penetration point on A, measured and expressed in
 * the world frame.
 * @param[out] p2 the deepest penetration point on B, measured and expressed in
 * the world frame.
 * @retval status Return 0 on success, and -1 on failure.
 */
static int penEPAPosClosest(const ccd_pt_el_t* nearest, ccd_vec3_t* p1,
                            ccd_vec3_t* p2) {
  // We reconstruct the simplex on which the nearest point lives, and then
  // compute the deepest penetration point on each geometric objects. Note that
  // the reconstructed simplex has size up to 3 (at most 3 vertices).
  if (nearest->type == CCD_PT_VERTEX) {
    ccd_pt_vertex_t* v = (ccd_pt_vertex_t*)nearest;
    ccdVec3Copy(p1, &v->v.v1);
    ccdVec3Copy(p2, &v->v.v2);
    return 0;
  } else {
    ccd_simplex_t s;
    ccdSimplexInit(&s);
    if (nearest->type == CCD_PT_EDGE) {
      const ccd_pt_edge_t* e = reinterpret_cast<const ccd_pt_edge_t*>(nearest);
      ccdSimplexAdd(&s, &(e->vertex[0]->v));
      ccdSimplexAdd(&s, &(e->vertex[1]->v));
    } else if (nearest->type == CCD_PT_FACE) {
      const ccd_pt_face_t* f = reinterpret_cast<const ccd_pt_face_t*>(nearest);
      // The face triangle has three edges, each edge consists of two end
      // points, so there are 6 end points in total, each vertex of the triangle
      // appears twice among the 6 end points. We need to choose the three
      // distinctive vertices out of these 6 end points.
      // First we pick edge0, the two end points of edge0 are distinct.
      ccdSimplexAdd(&s, &(f->edge[0]->vertex[0]->v));
      ccdSimplexAdd(&s, &(f->edge[0]->vertex[1]->v));
      // Next we pick edge1, one of the two end points on edge1 is distinct from
      // the end points in edge0, we will add this distinct vertex to the
      // simplex.
      for (int i = 0; i < 2; ++i) {
        ccd_pt_vertex_t* third_vertex = f->edge[1]->vertex[i];
        if (third_vertex != f->edge[0]->vertex[0] &&
            third_vertex != f->edge[0]->vertex[1]) {
          ccdSimplexAdd(&s, &(third_vertex->v));
          break;
        }
      }
    } else {
      throw std::logic_error(
          "FCL penEPAPosClosest(): Unsupported feature type. The closest point "
          "should be either a vertex, on an edge, or on a face.");
    }
    // Now compute the closest point in the simplex.
    // TODO(hongkai.dai@tri.global): we do not need to compute the closest point
    // on the simplex, as that is already given in `nearest`. We only need to
    // extract the deepest penetration points on each geometric object.
    // Sean.Curtis@tri.global and I will refactor this code in the future, to
    // avoid calling extractClosestPoints.
    ccd_vec3_t p;
    ccdVec3Copy(&p, &(nearest->witness));
    extractClosestPoints(&s, p1, p2, &p);
    return 0;
  }
}

static inline ccd_real_t ccdGJKSignedDist(const void* obj1, const void* obj2,
                                          const ccd_t* ccd, ccd_vec3_t* p1,
                                          ccd_vec3_t* p2)
{
  ccd_simplex_t simplex;

  if (__ccdGJK(obj1, obj2, ccd, &simplex) == 0) // in collision, then using the EPA
  {
    ccd_pt_t polytope;
    ccd_pt_el_t *nearest;
    ccd_real_t depth;

    ccdPtInit(&polytope);
    int ret = __ccdEPA(obj1, obj2, ccd, &simplex, &polytope, &nearest);
    if (ret == 0 && nearest)
    {
      depth = -CCD_SQRT(nearest->dist);

      ccd_vec3_t pos1, pos2;
      penEPAPosClosest(nearest, &pos1, &pos2);

      if (p1) *p1 = pos1;
      if (p2) *p2 = pos2;

      //ccd_vec3_t dir; // direction vector
      //ccdVec3Copy(&dir, &nearest->witness);
      //std::cout << dir.v[0] << " " << dir.v[1] << " " << dir.v[2] << std::endl;
      //ccd_support_t last;
      //__ccdSupport(obj1, obj2, &dir, ccd, &last);

      //if (p1) *p1 = last.v1;
      //if (p2) *p2 = last.v2;
    }
    else
    {
      depth = -CCD_ONE;
    }

    ccdPtDestroy(&polytope);

    return depth;
  }
  else // not in collision
  {
    return _ccdDist(obj1, obj2, ccd, &simplex, p1, p2);
  }
}


// Computes the distance between two non-penetrating convex objects, `obj1` and
// `obj2`, returning the distance and
// nearest points on each object.
// @param obj1 The first convex object.
// @param obj2 The second convex object.
// @param ccd The libccd configuration.
// @param p1 If the objects are non-penetrating, the point on the surface of
// obj1 closest to obj2 (expressed in the world frame).
// @param p2 If the objects are non-penetrating, the point on the surface of
// obj2 closest to obj1 (expressed in the world frame).
// @returns The minimum distance between the two objects. If they are
// penetrating, -1 is returned.
// @note Unlike _ccdDist function, this function does not need a warm-started
// simplex as the input argument.
static inline ccd_real_t ccdGJKDist2(const void *obj1, const void *obj2,
                                     const ccd_t *ccd, ccd_vec3_t* p1,
                                     ccd_vec3_t* p2)
{
  ccd_simplex_t simplex;
  // first find an intersection
  if (__ccdGJK(obj1, obj2, ccd, &simplex) == 0)
    return -CCD_ONE;

  return _ccdDist(obj1, obj2, ccd, &simplex, p1, p2);
}

} // namespace libccd_extension

/** Basic shape to ccd shape */
template <typename S>
static void shapeToGJK(const ShapeBase<S>& s, const Transform3<S>& tf,
                       ccd_obj_t* o)
{
  FCL_UNUSED(s);

  const Quaternion<S> q(tf.linear());
  const Vector3<S>& T = tf.translation();
  ccdVec3Set(&o->pos, T[0], T[1], T[2]);
  ccdQuatSet(&o->rot, q.x(), q.y(), q.z(), q.w());
  ccdQuatInvert2(&o->rot_inv, &o->rot);
}

template <typename S>
static void boxToGJK(const Box<S>& s, const Transform3<S>& tf, ccd_box_t* box)
{
  shapeToGJK(s, tf, box);
  box->dim[0] = s.side[0] / 2.0;
  box->dim[1] = s.side[1] / 2.0;
  box->dim[2] = s.side[2] / 2.0;
}

template <typename S>
static void capToGJK(const Capsule<S>& s, const Transform3<S>& tf,
                     ccd_cap_t* cap)
{
  shapeToGJK(s, tf, cap);
  cap->radius = s.radius;
  cap->height = s.lz / 2;
}

template <typename S>
static void cylToGJK(const Cylinder<S>& s, const Transform3<S>& tf,
                     ccd_cyl_t* cyl)
{
  shapeToGJK(s, tf, cyl);
  cyl->radius = s.radius;
  cyl->height = s.lz / 2;
}

template <typename S>
static void coneToGJK(const Cone<S>& s, const Transform3<S>& tf,
                      ccd_cone_t* cone)
{
  shapeToGJK(s, tf, cone);
  cone->radius = s.radius;
  cone->height = s.lz / 2;
}

template <typename S>
static void sphereToGJK(const Sphere<S>& s, const Transform3<S>& tf,
                        ccd_sphere_t* sph)
{
  shapeToGJK(s, tf, sph);
  sph->radius = s.radius;
}

template <typename S>
static void ellipsoidToGJK(const Ellipsoid<S>& s, const Transform3<S>& tf,
                           ccd_ellipsoid_t* ellipsoid)
{
  shapeToGJK(s, tf, ellipsoid);
  ellipsoid->radii[0] = s.radii[0];
  ellipsoid->radii[1] = s.radii[1];
  ellipsoid->radii[2] = s.radii[2];
}

template <typename S>
static void convexToGJK(const Convex<S>& s, const Transform3<S>& tf,
                        ccd_convex_t<S>* conv)
{
  shapeToGJK(s, tf, conv);
  conv->convex = &s;
}

/** Support functions */
static inline void supportBox(const void* obj, const ccd_vec3_t* dir_,
                              ccd_vec3_t* v)
{
  // Use a customized sign function, so that the support of the box always
  // appears in one of the box vertices.
  // Picking support vertices on the interior of faces/edges can lead to
  // degenerate triangles in the EPA algorithm and are no more correct than just
  // picking box corners.
  auto sign = [](ccd_real_t x) -> ccd_real_t {
    return x >= 0 ? ccd_real_t(1.0) : ccd_real_t(-1.0);
  };
  const ccd_box_t* o = static_cast<const ccd_box_t*>(obj);
  ccd_vec3_t dir;
  ccdVec3Copy(&dir, dir_);
  ccdQuatRotVec(&dir, &o->rot_inv);
  ccdVec3Set(v, sign(ccdVec3X(&dir)) * o->dim[0],
             sign(ccdVec3Y(&dir)) * o->dim[1],
             sign(ccdVec3Z(&dir)) * o->dim[2]);
  ccdQuatRotVec(v, &o->rot);
  ccdVec3Add(v, &o->pos);
}

static inline void supportCap(const void* obj, const ccd_vec3_t* dir_,
                              ccd_vec3_t* v)
{
  const ccd_cap_t* o = static_cast<const ccd_cap_t*>(obj);
  ccd_vec3_t dir, pos1, pos2;

  ccdVec3Copy(&dir, dir_);
  ccdQuatRotVec(&dir, &o->rot_inv);

  ccdVec3Set(&pos1, CCD_ZERO, CCD_ZERO, o->height);
  ccdVec3Set(&pos2, CCD_ZERO, CCD_ZERO, -o->height);

  ccdVec3Copy(v, &dir);
  ccdVec3Normalize (v);
  ccdVec3Scale(v, o->radius);
  ccdVec3Add(&pos1, v);
  ccdVec3Add(&pos2, v);

  if (ccdVec3Z (&dir) > 0)
    ccdVec3Copy(v, &pos1);
  else
    ccdVec3Copy(v, &pos2);

  // transform support vertex
  ccdQuatRotVec(v, &o->rot);
  ccdVec3Add(v, &o->pos);
}

static inline void supportCyl(const void* obj, const ccd_vec3_t* dir_,
                              ccd_vec3_t* v)
{
  const ccd_cyl_t* cyl = static_cast<const ccd_cyl_t*>(obj);
  ccd_vec3_t dir;
  double zdist, rad;

  ccdVec3Copy(&dir, dir_);
  ccdQuatRotVec(&dir, &cyl->rot_inv);

  zdist = dir.v[0] * dir.v[0] + dir.v[1] * dir.v[1];
  zdist = sqrt(zdist);
  if (ccdIsZero(zdist))
    ccdVec3Set(v, 0., 0., ccdSign(ccdVec3Z(&dir)) * cyl->height);
  else
  {
    rad = cyl->radius / zdist;

    ccdVec3Set(v, rad * ccdVec3X(&dir),
               rad * ccdVec3Y(&dir),
               ccdSign(ccdVec3Z(&dir)) * cyl->height);
  }

  // transform support vertex
  ccdQuatRotVec(v, &cyl->rot);
  ccdVec3Add(v, &cyl->pos);
}

static inline void supportCone(const void* obj, const ccd_vec3_t* dir_,
                               ccd_vec3_t* v)
{
  const ccd_cone_t* cone = static_cast<const ccd_cone_t*>(obj);
  ccd_vec3_t dir;

  ccdVec3Copy(&dir, dir_);
  ccdQuatRotVec(&dir, &cone->rot_inv);

  double zdist, len, rad;
  zdist = dir.v[0] * dir.v[0] + dir.v[1] * dir.v[1];
  len = zdist + dir.v[2] * dir.v[2];
  zdist = sqrt(zdist);
  len = sqrt(len);

  double sin_a = cone->radius / sqrt(cone->radius * cone->radius + 4 * cone->height * cone->height);

  if (dir.v[2] > len * sin_a)
    ccdVec3Set(v, 0., 0., cone->height);
  else if (zdist > 0)
  {
    rad = cone->radius / zdist;
    ccdVec3Set(v, rad * ccdVec3X(&dir), rad * ccdVec3Y(&dir), -cone->height);
  }
  else
    ccdVec3Set(v, 0, 0, -cone->height);

  // transform support vertex
  ccdQuatRotVec(v, &cone->rot);
  ccdVec3Add(v, &cone->pos);
}

static inline void supportSphere(const void* obj, const ccd_vec3_t* dir_,
                                 ccd_vec3_t* v)
{
  const ccd_sphere_t* s = static_cast<const ccd_sphere_t*>(obj);
  ccd_vec3_t dir;

  ccdVec3Copy(&dir, dir_);
  ccdQuatRotVec(&dir, &s->rot_inv);

  ccdVec3Copy(v, &dir);
  ccdVec3Scale(v, s->radius);
  ccdVec3Scale(v, CCD_ONE / CCD_SQRT(ccdVec3Len2(&dir)));

  // transform support vertex
  ccdQuatRotVec(v, &s->rot);
  ccdVec3Add(v, &s->pos);
}

static inline void supportEllipsoid(const void* obj, const ccd_vec3_t* dir_,
                                    ccd_vec3_t* v)
{
  const ccd_ellipsoid_t* s = static_cast<const ccd_ellipsoid_t*>(obj);
  ccd_vec3_t dir;

  ccdVec3Copy(&dir, dir_);
  ccdQuatRotVec(&dir, &s->rot_inv);

  ccd_vec3_t abc2;
  abc2.v[0] = s->radii[0] * s->radii[0];
  abc2.v[1] = s->radii[1] * s->radii[1];
  abc2.v[2] = s->radii[2] * s->radii[2];

  v->v[0] = abc2.v[0] * dir.v[0];
  v->v[1] = abc2.v[1] * dir.v[1];
  v->v[2] = abc2.v[2] * dir.v[2];

  ccdVec3Scale(v, CCD_ONE / CCD_SQRT(ccdVec3Dot(v, &dir)));

  // transform support vertex
  ccdQuatRotVec(v, &s->rot);
  ccdVec3Add(v, &s->pos);
}

template <typename S>
static void supportConvex(const void* obj, const ccd_vec3_t* dir_,
                          ccd_vec3_t* v)
{
  const auto* c = (const ccd_convex_t<S>*)obj;

  // Transform the support direction vector from the query frame Q to the
  // convex mesh's local frame C. I.e., dir_Q -> dir_C.
  ccd_vec3_t dir;
  ccdVec3Copy(&dir, dir_);
  ccdQuatRotVec(&dir, &c->rot_inv);
  // Note: The scalar type ccd_real_t is fixed w.r.t. S. That means if S is
  // float and ccd_real_t is double, this conversion requires narrowing. To
  // avoid warning/errors about implicit narrowing, we explicitly convert.
  Vector3<S> dir_C{S(dir.v[0]), S(dir.v[1]), S(dir.v[2])};

  // The extremal point E measured and expressed in Frame C.
  const Vector3<S>& p_CE = c->convex->findExtremeVertex(dir_C);

  // Now measure and express E in the original query frame Q: p_CE -> p_QE.
  v->v[0] = p_CE(0);
  v->v[1] = p_CE(1);
  v->v[2] = p_CE(2);
  ccdQuatRotVec(v, &c->rot);
  ccdVec3Add(v, &c->pos);
}

static void supportTriangle(const void* obj, const ccd_vec3_t* dir_,
                            ccd_vec3_t* v)
{
  const ccd_triangle_t* tri = static_cast<const ccd_triangle_t*>(obj);
  ccd_vec3_t dir, p;
  ccd_real_t maxdot, dot;
  int i;

  ccdVec3Copy(&dir, dir_);
  ccdQuatRotVec(&dir, &tri->rot_inv);

  maxdot = -CCD_REAL_MAX;

  for (i = 0; i < 3; ++i)
  {
    ccdVec3Set(&p, tri->p[i].v[0] - tri->c.v[0], tri->p[i].v[1] - tri->c.v[1],
        tri->p[i].v[2] - tri->c.v[2]);
    dot = ccdVec3Dot(&dir, &p);
    if (dot > maxdot)
    {
      ccdVec3Copy(v, &tri->p[i]);
      maxdot = dot;
    }
  }

  // transform support vertex
  ccdQuatRotVec(v, &tri->rot);
  ccdVec3Add(v, &tri->pos);
}

static inline void centerShape(const void* obj, ccd_vec3_t* c)
{
  const ccd_obj_t *o = static_cast<const ccd_obj_t*>(obj);
  ccdVec3Copy(c, &o->pos);
}

template <typename S>
static void centerConvex(const void* obj, ccd_vec3_t* c)
{
  const auto *o = static_cast<const ccd_convex_t<S>*>(obj);
  const Vector3<S>& p = o->convex->getInteriorPoint();
  ccdVec3Set(c, p[0], p[1], p[2]);
  ccdQuatRotVec(c, &o->rot);
  ccdVec3Add(c, &o->pos);
}

static void centerTriangle(const void* obj, ccd_vec3_t* c)
{
  const ccd_triangle_t *o = static_cast<const ccd_triangle_t*>(obj);
  ccdVec3Copy(c, &o->c);
  ccdQuatRotVec(c, &o->rot);
  ccdVec3Add(c, &o->pos);
}

template <typename S>
bool GJKCollide(void* obj1, ccd_support_fn supp1, ccd_center_fn cen1,
                void* obj2, ccd_support_fn supp2, ccd_center_fn cen2,
                unsigned int max_iterations, S tolerance,
                Vector3<S>* contact_points, S* penetration_depth,
                Vector3<S>* normal)
{
  ccd_t ccd;
  int res;
  ccd_real_t depth;
  ccd_vec3_t dir, pos;


  CCD_INIT(&ccd);
  ccd.support1 = supp1;
  ccd.support2 = supp2;
  ccd.center1 = cen1;
  ccd.center2 = cen2;
  ccd.max_iterations = max_iterations;
  ccd.mpr_tolerance = tolerance;

  if (!contact_points)
  {
    return ccdMPRIntersect(obj1, obj2, &ccd);
  }


  /// libccd returns dir and pos in world space and dir is pointing from
  /// object 1 to object 2
  res = ccdMPRPenetration(obj1, obj2, &ccd, &depth, &dir, &pos);
  if (res == 0)
  {
    *contact_points << ccdVec3X(&pos), ccdVec3Y(&pos), ccdVec3Z(&pos);
    *penetration_depth = depth;
    *normal << ccdVec3X(&dir), ccdVec3Y(&dir), ccdVec3Z(&dir);

    return true;
  }

  return false;
}

// For two geometric objects, computes the distance between the two objects and
// returns the closest points. The return argument is the distance when the two
// objects are not colliding, thus a non-negative number; it is a negative
// number when the object is colliding, though the meaning of that negative
// number depends on the implementation.
using DistanceFn = std::function<ccd_real_t (
    const void*, const void*, const ccd_t*, ccd_vec3_t*, ccd_vec3_t*)>;

/** Compute the distance between two objects using GJK algorithm.
 * @param[in] obj1 A convex geometric object.
 * @param[in] supp1 A function to compute the support of obj1 along some
 * direction.
 * @param[in] obj2 A convex geometric object.
 * @param[in] supp2 A function to compute the support of obj2 along some
 * direction.
 * @param max_iterations The maximal iterations before the GJK algorithm
 * terminates.
 * @param[in] tolerance The tolerance used in GJK. When the change of distance
 * is smaller than this tolerance, the algorithm terminates.
 * @param[in] distance_func The actual function that computes the distance.
 * Different functions should be passed in, depending on whether the user wants
 * to compute a signed distance (with penetration depth) or not.
 * @param[out] res The distance between the objects. When the two objects are
 * not colliding, this is the actual distance, a positive number. When the two
 * objects are colliding, it is a negative value. The actual meaning of the
 * negative distance is defined by `distance_func`.
 * @param[out] p1 The closest point on object 1 in the world frame.
 * @param[out] p2 The closest point on object 2 in the world frame.
 * @retval is_separated True if the objects are separated, false otherwise.
 */
template <typename S>
bool GJKDistanceImpl(void* obj1, ccd_support_fn supp1, void* obj2,
                     ccd_support_fn supp2, unsigned int max_iterations,
                     S tolerance, detail::DistanceFn distance_func, S* res,
                     Vector3<S>* p1, Vector3<S>* p2) {
  ccd_t ccd;
  ccd_real_t dist;
  CCD_INIT(&ccd);
  ccd.support1 = supp1;
  ccd.support2 = supp2;

  ccd.max_iterations = max_iterations;
  ccd.dist_tolerance = tolerance;
  ccd.epa_tolerance = tolerance;

  ccd_vec3_t p1_, p2_;
  // NOTE(JS): p1_ and p2_ are set to zeros in order to suppress uninitialized
  // warning. It seems the warnings occur since libccd_extension::ccdGJKDist2
  // conditionally set p1_ and p2_. If this wasn't intentional then please
  // remove the initialization of p1_ and p2_, and change the function
  // libccd_extension::ccdGJKDist2(...) to always set p1_ and p2_.
  ccdVec3Set(&p1_, 0.0, 0.0, 0.0);
  ccdVec3Set(&p2_, 0.0, 0.0, 0.0);
  dist = distance_func(obj1, obj2, &ccd, &p1_, &p2_);
  if (p1) *p1 << ccdVec3X(&p1_), ccdVec3Y(&p1_), ccdVec3Z(&p1_);
  if (p2) *p2 << ccdVec3X(&p2_), ccdVec3Y(&p2_), ccdVec3Z(&p2_);
  if (res) *res = dist;
  if (dist < 0)
    return false;
  else
    return true;
}

template <typename S>
bool GJKDistance(void* obj1, ccd_support_fn supp1,
                 void* obj2, ccd_support_fn supp2,
                 unsigned int max_iterations, S tolerance,
                 S* res, Vector3<S>* p1, Vector3<S>* p2) {
  return detail::GJKDistanceImpl(obj1, supp1, obj2, supp2, max_iterations,
                                 tolerance, libccd_extension::ccdGJKDist2, res,
                                 p1, p2);
}

/**
 * Compute the signed distance between two mesh objects. When the objects are
 * separating, the signed distance is > 0. When the objects are touching or
 * penetrating, the signed distance is <= 0.
 * @param tolerance. When the objects are separating, the GJK algorithm
 * terminates when the change of distance between iterations is smaller than
 * this tolerance. Note that this does NOT necessarily mean that the computed
 * distance is within @p tolerance to the actual distance. On the other hand,
 * when the objects are penetrating, the EPA algorithm terminates when the
 * difference between the upper bound and the lower bound of the penetration
 * depth is smaller than @p tolerance. Hence the computed penetration depth is
 * within @p tolerance to the true depth.
 */
template <typename S>
bool GJKSignedDistance(void* obj1, ccd_support_fn supp1,
                       void* obj2, ccd_support_fn supp2,
                       unsigned int max_iterations,
                       S tolerance, S* res, Vector3<S>* p1, Vector3<S>* p2) {
  return detail::GJKDistanceImpl(
      obj1, supp1, obj2, supp2, max_iterations, tolerance,
      libccd_extension::ccdGJKSignedDist, res, p1, p2);
}

template <typename S>
GJKSupportFunction GJKInitializer<S, Cylinder<S>>::getSupportFunction()
{
  return &supportCyl;
}

template <typename S>
GJKCenterFunction GJKInitializer<S, Cylinder<S>>::getCenterFunction()
{
  return &centerShape;
}

template <typename S>
void* GJKInitializer<S, Cylinder<S>>::createGJKObject(const Cylinder<S>& s,
                                                      const Transform3<S>& tf)
{
  ccd_cyl_t* o = new ccd_cyl_t;
  cylToGJK(s, tf, o);
  return o;
}

template <typename S>
void GJKInitializer<S, Cylinder<S>>::deleteGJKObject(void* o_)
{
  ccd_cyl_t* o = static_cast<ccd_cyl_t*>(o_);
  delete o;
}

template <typename S>
GJKSupportFunction GJKInitializer<S, Sphere<S>>::getSupportFunction()
{
  return &supportSphere;
}

template <typename S>
GJKCenterFunction GJKInitializer<S, Sphere<S>>::getCenterFunction()
{
  return &centerShape;
}

template <typename S>
void* GJKInitializer<S, Sphere<S>>::createGJKObject(const Sphere<S>& s,
                                                    const Transform3<S>& tf)
{
  ccd_sphere_t* o = new ccd_sphere_t;
  sphereToGJK(s, tf, o);
  return o;
}

template <typename S>
void GJKInitializer<S, Sphere<S>>::deleteGJKObject(void* o_)
{
  ccd_sphere_t* o = static_cast<ccd_sphere_t*>(o_);
  delete o;
}

template <typename S>
GJKSupportFunction GJKInitializer<S, Ellipsoid<S>>::getSupportFunction()
{
  return &supportEllipsoid;
}

template <typename S>
GJKCenterFunction GJKInitializer<S, Ellipsoid<S>>::getCenterFunction()
{
  return &centerShape;
}

template <typename S>
void* GJKInitializer<S, Ellipsoid<S>>::createGJKObject(const Ellipsoid<S>& s,
                                                       const Transform3<S>& tf)
{
  ccd_ellipsoid_t* o = new ccd_ellipsoid_t;
  ellipsoidToGJK(s, tf, o);
  return o;
}

template <typename S>
void GJKInitializer<S, Ellipsoid<S>>::deleteGJKObject(void* o_)
{
  ccd_ellipsoid_t* o = static_cast<ccd_ellipsoid_t*>(o_);
  delete o;
}

template <typename S>
GJKSupportFunction GJKInitializer<S, Box<S>>::getSupportFunction()
{
  return &supportBox;
}

template <typename S>
GJKCenterFunction GJKInitializer<S, Box<S>>::getCenterFunction()
{
  return &centerShape;
}

template <typename S>
void* GJKInitializer<S, Box<S>>::createGJKObject(const Box<S>& s,
                                                 const Transform3<S>& tf)
{
  ccd_box_t* o = new ccd_box_t;
  boxToGJK(s, tf, o);
  return o;
}

template <typename S>
void GJKInitializer<S, Box<S>>::deleteGJKObject(void* o_)
{
  ccd_box_t* o = static_cast<ccd_box_t*>(o_);
  delete o;
}

template <typename S>
GJKSupportFunction GJKInitializer<S, Capsule<S>>::getSupportFunction()
{
  return &supportCap;
}

template <typename S>
GJKCenterFunction GJKInitializer<S, Capsule<S>>::getCenterFunction()
{
  return &centerShape;
}

template <typename S>
void* GJKInitializer<S, Capsule<S>>::createGJKObject(const Capsule<S>& s,
                                                     const Transform3<S>& tf)
{
  ccd_cap_t* o = new ccd_cap_t;
  capToGJK(s, tf, o);
  return o;
}

template <typename S>
void GJKInitializer<S, Capsule<S>>::deleteGJKObject(void* o_)
{
  ccd_cap_t* o = static_cast<ccd_cap_t*>(o_);
  delete o;
}

template <typename S>
GJKSupportFunction GJKInitializer<S, Cone<S>>::getSupportFunction()
{
  return &supportCone;
}

template <typename S>
GJKCenterFunction GJKInitializer<S, Cone<S>>::getCenterFunction()
{
  return &centerShape;
}

template <typename S>
void* GJKInitializer<S, Cone<S>>::createGJKObject(const Cone<S>& s,
                                                  const Transform3<S>& tf)
{
  ccd_cone_t* o = new ccd_cone_t;
  coneToGJK(s, tf, o);
  return o;
}

template <typename S>
void GJKInitializer<S, Cone<S>>::deleteGJKObject(void* o_)
{
  ccd_cone_t* o = static_cast<ccd_cone_t*>(o_);
  delete o;
}

template <typename S>
GJKSupportFunction GJKInitializer<S, Convex<S>>::getSupportFunction()
{
  return &supportConvex<S>;
}

template <typename S>
GJKCenterFunction GJKInitializer<S, Convex<S>>::getCenterFunction()
{
  return &centerConvex<S>;
}

template <typename S>
void* GJKInitializer<S, Convex<S>>::createGJKObject(const Convex<S>& s,
                                                    const Transform3<S>& tf)
{
  auto* o = new ccd_convex_t<S>;
  convexToGJK(s, tf, o);
  return o;
}

template <typename S>
void GJKInitializer<S, Convex<S>>::deleteGJKObject(void* o_)
{
  auto* o = static_cast<ccd_convex_t<S>*>(o_);
  delete o;
}

inline GJKSupportFunction triGetSupportFunction()
{
  return &supportTriangle;
}

inline GJKCenterFunction triGetCenterFunction()
{
  return &centerTriangle;
}

template <typename S>
void* triCreateGJKObject(const Vector3<S>& P1, const Vector3<S>& P2,
                         const Vector3<S>& P3)
{
  ccd_triangle_t* o = new ccd_triangle_t;
  Vector3<S> center((P1[0] + P2[0] + P3[0]) / 3, (P1[1] + P2[1] + P3[1]) / 3,
      (P1[2] + P2[2] + P3[2]) / 3);

  ccdVec3Set(&o->p[0], P1[0], P1[1], P1[2]);
  ccdVec3Set(&o->p[1], P2[0], P2[1], P2[2]);
  ccdVec3Set(&o->p[2], P3[0], P3[1], P3[2]);
  ccdVec3Set(&o->c, center[0], center[1], center[2]);
  ccdVec3Set(&o->pos, 0., 0., 0.);
  ccdQuatSet(&o->rot, 0., 0., 0., 1.);
  ccdQuatInvert2(&o->rot_inv, &o->rot);

  return o;
}

template <typename S>
void* triCreateGJKObject(const Vector3<S>& P1, const Vector3<S>& P2,
                         const Vector3<S>& P3, const Transform3<S>& tf)
{
  ccd_triangle_t* o = new ccd_triangle_t;
  Vector3<S> center((P1[0] + P2[0] + P3[0]) / 3, (P1[1] + P2[1] + P3[1]) / 3,
      (P1[2] + P2[2] + P3[2]) / 3);

  ccdVec3Set(&o->p[0], P1[0], P1[1], P1[2]);
  ccdVec3Set(&o->p[1], P2[0], P2[1], P2[2]);
  ccdVec3Set(&o->p[2], P3[0], P3[1], P3[2]);
  ccdVec3Set(&o->c, center[0], center[1], center[2]);
  const Quaternion<S> q(tf.linear());
  const Vector3<S>& T = tf.translation();
  ccdVec3Set(&o->pos, T[0], T[1], T[2]);
  ccdQuatSet(&o->rot, q.x(), q.y(), q.z(), q.w());
  ccdQuatInvert2(&o->rot_inv, &o->rot);

  return o;
}

inline void triDeleteGJKObject(void* o_)
{
  ccd_triangle_t* o = static_cast<ccd_triangle_t*>(o_);
  delete o;
}

} // namespace detail
} // namespace fcl

#endif
