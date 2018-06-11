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

#include <unordered_set>
#include <unordered_map>

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

static int doSimplex2(ccd_simplex_t *simplex, ccd_vec3_t *dir)
{
  const ccd_support_t *A, *B;
  ccd_vec3_t AB, AO, tmp;
  ccd_real_t dot;

  // get last added as A
  A = ccdSimplexLast(simplex);
  // get the other point
  B = ccdSimplexPoint(simplex, 0);
  // compute AB oriented segment
  ccdVec3Sub2(&AB, &B->v, &A->v);
  // compute AO vector
  ccdVec3Copy(&AO, &A->v);
  ccdVec3Scale(&AO, -CCD_ONE);

  // dot product AB . AO
  dot = ccdVec3Dot(&AB, &AO);

  // check if origin doesn't lie on AB segment
  ccdVec3Cross(&tmp, &AB, &AO);
  if (ccdIsZero(ccdVec3Len2(&tmp)) && dot > CCD_ZERO){
    return 1;
  }

  // check if origin is in area where AB segment is
  if (ccdIsZero(dot) || dot < CCD_ZERO){
    // origin is in outside are of A
    ccdSimplexSet(simplex, 0, A);
    ccdSimplexSetSize(simplex, 1);
    ccdVec3Copy(dir, &AO);
  }else{
    // origin is in area where AB segment is

    // keep simplex untouched and set direction to
    // AB x AO x AB
    tripleCross(&AB, &AO, &AB, dir);
  }

  return 0;
}

static int doSimplex3(ccd_simplex_t *simplex, ccd_vec3_t *dir)
{
  const ccd_support_t *A, *B, *C;
  ccd_vec3_t AO, AB, AC, ABC, tmp;
  ccd_real_t dot, dist;

  // get last added as A
  A = ccdSimplexLast(simplex);
  // get the other points
  B = ccdSimplexPoint(simplex, 1);
  C = ccdSimplexPoint(simplex, 0);

  // check touching contact
  dist = ccdVec3PointTriDist2(ccd_vec3_origin, &A->v, &B->v, &C->v, nullptr);
  if (ccdIsZero(dist)){
    return 1;
  }

  // check if triangle is really triangle (has area > 0)
  // if not simplex can't be expanded and thus no itersection is found
  if (ccdVec3Eq(&A->v, &B->v) || ccdVec3Eq(&A->v, &C->v)){
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
  ccd_real_t dist;

  // get last added as A
  A = ccdSimplexLast(simplex);
  // get the other points
  B = ccdSimplexPoint(simplex, 2);
  C = ccdSimplexPoint(simplex, 1);
  D = ccdSimplexPoint(simplex, 0);

  // check if tetrahedron is really tetrahedron (has volume > 0)
  // if it is not simplex can't be expanded and thus no intersection is
  // found
  dist = ccdVec3PointTriDist2(&A->v, &B->v, &C->v, &D->v, nullptr);
  if (ccdIsZero(dist)){
    return -1;
  }

  // check if origin lies on some of tetrahedron's face - if so objects
  // intersect
  dist = ccdVec3PointTriDist2(ccd_vec3_origin, &A->v, &B->v, &C->v, nullptr);
  if (ccdIsZero(dist))
    return 1;
  dist = ccdVec3PointTriDist2(ccd_vec3_origin, &A->v, &C->v, &D->v, nullptr);
  if (ccdIsZero(dist))
    return 1;
  dist = ccdVec3PointTriDist2(ccd_vec3_origin, &A->v, &B->v, &D->v, nullptr);
  if (ccdIsZero(dist))
    return 1;
  dist = ccdVec3PointTriDist2(ccd_vec3_origin, &B->v, &C->v, &D->v, nullptr);
  if (ccdIsZero(dist))
    return 1;

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


/** Transforms simplex to polytope, three vertices required */
static int simplexToPolytope3(const void *obj1, const void *obj2,
                              const ccd_t *ccd,
                              const ccd_simplex_t *simplex,
                              ccd_pt_t *pt, ccd_pt_el_t **nearest)
{
    const ccd_support_t *a, *b, *c;
    ccd_support_t d, d2;
    ccd_vec3_t ab, ac, dir;
    ccd_pt_vertex_t *v[5];
    ccd_pt_edge_t *e[9];
    ccd_real_t dist, dist2;

    *nearest = NULL;

    a = ccdSimplexPoint(simplex, 0);
    b = ccdSimplexPoint(simplex, 1);
    c = ccdSimplexPoint(simplex, 2);

    // If only one triangle left from previous GJK run origin lies on this
    // triangle. So it is necessary to expand triangle into two
    // tetrahedrons connected with base (which is exactly abc triangle).

    // get next support point in direction of normal of triangle
    ccdVec3Sub2(&ab, &b->v, &a->v);
    ccdVec3Sub2(&ac, &c->v, &a->v);
    ccdVec3Cross(&dir, &ab, &ac);
    __ccdSupport(obj1, obj2, &dir, ccd, &d);
    dist = ccdVec3PointTriDist2(&d.v, &a->v, &b->v, &c->v, NULL);

    // and second one take in opposite direction
    ccdVec3Scale(&dir, -CCD_ONE);
    __ccdSupport(obj1, obj2, &dir, ccd, &d2);
    dist2 = ccdVec3PointTriDist2(&d2.v, &a->v, &b->v, &c->v, NULL);

    // check if face isn't already on edge of minkowski sum and thus we
    // have touching contact
    if (ccdIsZero(dist) || ccdIsZero(dist2)){
        v[0] = ccdPtAddVertex(pt, a);
        v[1] = ccdPtAddVertex(pt, b);
        v[2] = ccdPtAddVertex(pt, c);
        e[0] = ccdPtAddEdge(pt, v[0], v[1]);
        e[1] = ccdPtAddEdge(pt, v[1], v[2]);
        e[2] = ccdPtAddEdge(pt, v[2], v[0]);
        *nearest = (ccd_pt_el_t *)ccdPtAddFace(pt, e[0], e[1], e[2]);
        if (*nearest == NULL)
            return -2;

        return -1;
    }

    // form polyhedron
    v[0] = ccdPtAddVertex(pt, a);
    v[1] = ccdPtAddVertex(pt, b);
    v[2] = ccdPtAddVertex(pt, c);
    v[3] = ccdPtAddVertex(pt, &d);
    v[4] = ccdPtAddVertex(pt, &d2);

    e[0] = ccdPtAddEdge(pt, v[0], v[1]);
    e[1] = ccdPtAddEdge(pt, v[1], v[2]);
    e[2] = ccdPtAddEdge(pt, v[2], v[0]);

    e[3] = ccdPtAddEdge(pt, v[3], v[0]);
    e[4] = ccdPtAddEdge(pt, v[3], v[1]);
    e[5] = ccdPtAddEdge(pt, v[3], v[2]);

    e[6] = ccdPtAddEdge(pt, v[4], v[0]);
    e[7] = ccdPtAddEdge(pt, v[4], v[1]);
    e[8] = ccdPtAddEdge(pt, v[4], v[2]);

    if (ccdPtAddFace(pt, e[3], e[4], e[0]) == NULL
            || ccdPtAddFace(pt, e[4], e[5], e[1]) == NULL
            || ccdPtAddFace(pt, e[5], e[3], e[2]) == NULL

            || ccdPtAddFace(pt, e[6], e[7], e[0]) == NULL
            || ccdPtAddFace(pt, e[7], e[8], e[1]) == NULL
            || ccdPtAddFace(pt, e[8], e[6], e[2]) == NULL){
        return -2;
    }

    return 0;
}


/** Transforms simplex to polytope. It is assumed that simplex has 4
 *  vertices! */
static int simplexToPolytope4(const void *obj1, const void *obj2,
                              const ccd_t *ccd,
                              ccd_simplex_t *simplex,
                              ccd_pt_t *pt, ccd_pt_el_t **nearest)
{
    const ccd_support_t *a, *b, *c, *d;
    int use_polytope3;
    ccd_real_t dist;
    ccd_pt_vertex_t *v[4];
    ccd_pt_edge_t *e[6];
    size_t i;

    a = ccdSimplexPoint(simplex, 0);
    b = ccdSimplexPoint(simplex, 1);
    c = ccdSimplexPoint(simplex, 2);
    d = ccdSimplexPoint(simplex, 3);

    // check if origin lies on some of tetrahedron's face - if so use
    // simplexToPolytope3()
    use_polytope3 = 0;
    dist = ccdVec3PointTriDist2(ccd_vec3_origin, &a->v, &b->v, &c->v, NULL);
    if (ccdIsZero(dist)){
        use_polytope3 = 1;
    }
    dist = ccdVec3PointTriDist2(ccd_vec3_origin, &a->v, &c->v, &d->v, NULL);
    if (ccdIsZero(dist)){
        use_polytope3 = 1;
        ccdSimplexSet(simplex, 1, c);
        ccdSimplexSet(simplex, 2, d);
    }
    dist = ccdVec3PointTriDist2(ccd_vec3_origin, &a->v, &b->v, &d->v, NULL);
    if (ccdIsZero(dist)){
        use_polytope3 = 1;
        ccdSimplexSet(simplex, 2, d);
    }
    dist = ccdVec3PointTriDist2(ccd_vec3_origin, &b->v, &c->v, &d->v, NULL);
    if (ccdIsZero(dist)){
        use_polytope3 = 1;
        ccdSimplexSet(simplex, 0, b);
        ccdSimplexSet(simplex, 1, c);
        ccdSimplexSet(simplex, 2, d);
    }

    if (use_polytope3){
        ccdSimplexSetSize(simplex, 3);
        return simplexToPolytope3(obj1, obj2, ccd, simplex, pt, nearest);
    }

    // no touching contact - simply create tetrahedron
    for (i = 0; i < 4; i++){
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
    if (ccdPtAddFace(pt, e[0], e[1], e[2]) == NULL
            || ccdPtAddFace(pt, e[3], e[4], e[0]) == NULL
            || ccdPtAddFace(pt, e[4], e[5], e[1]) == NULL
            || ccdPtAddFace(pt, e[5], e[3], e[2]) == NULL){
        return -2;
    }

    return 0;
}

/**
 * Computes the normal vector of a face on a polytope, and the normal vector
 * points outward from the polytope. Notice we assume that the origin lives
 * within the polytope.
 * @param[in] polytope The polytope on which the face lives. We assume that the
 * origin also lives inside the polytope.
 * @param[in] face The face for which the unit length normal vector is computed.
 * @retval dir The unit length vector normal to the face, and points outward
 * from the polytope.
 */
static ccd_vec3_t faceNormalPointingOutward(const ccd_pt_t* polytope,
                                            const ccd_pt_face_t* face) {
  // We find two edges of the triangle as e1 and e2, and the normal vector
  // of the face is e1.cross(e2).
  ccd_vec3_t e1, e2;
  ccdVec3Sub2(&e1, &(face->edge[0]->vertex[1]->v.v),
              &(face->edge[0]->vertex[0]->v.v));
  ccdVec3Sub2(&e2, &(face->edge[1]->vertex[1]->v.v),
              &(face->edge[1]->vertex[0]->v.v));
  ccd_vec3_t dir;
  ccdVec3Cross(&dir, &e1, &e2);
  ccd_real_t projection = ccdVec3Dot(&dir, &(face->edge[0]->vertex[0]->v.v));
  if (projection < 0) {
    // This means dir.dot(origin) = 0 > dir.dot(face.v).
    // The origin is on the outward direction along `dir`. Since origin is
    // within the polytope, this means `dir` points into the polytope, so we
    // should flip the direction.
    ccdVec3Scale(&dir, ccd_real_t(-1));
  } else if (ccdIsZero(projection)) {
    // The origin is on the face. Pick another vertex to test the normal
    // direction. 
    ccd_real_t max_projection = -CCD_REAL_MAX;
    ccd_real_t min_projection = CCD_REAL_MAX;
    ccd_pt_vertex_t* v;
    // If the magnitude of the projection is larger than tolerance, then it
    // means one of the vertex is at least 1cm away from the plane coinciding
    // with the face.
    ccd_real_t tol = 1E-2 / std::sqrt(ccdVec3Len2(&dir));
    ccdListForEachEntry(&polytope->vertices, v, ccd_pt_vertex_t, list) {
      projection = ccdVec3Dot(&dir, &(v->v.v)); 
      if (projection > tol) {
        // The vertex is at least 1cm away from the face plane, on the same
        // direction of `dir`. So we flip dir to point it outward from the
        // polytope.
        ccdVec3Scale(&dir, ccd_real_t(-1));
        break;
      } else if (projection < -tol) {
        // The vertex is at least 1cm away from the face plane, on the opposite
        // direction of `dir`. So `dir` points outward already.
        break;
      } else {
        max_projection = std::max(max_projection, projection);
        min_projection = std::min(min_projection, projection);
      }
    }
    // If max_projection > |min_projection|, then flip dir.
    if (max_projection > std::abs(min_projection)) {
      ccdVec3Scale(&dir, ccd_real_t(-1));
    }
  }
  return dir;
}

// Return true if the point @p pt is on the outward side of the half-plane, on
// which the triangle @p f lives. Notice if the point @p pt is exactly on the
// half-plane, the return if false.
// @param f A triangle on a polytope.
// @param pt A point.
static bool outsidePolytopeFace(const ccd_pt_t* polytope,
                                const ccd_pt_face_t* f, const ccd_vec3_t* pt) {
  ccd_vec3_t n = faceNormalPointingOutward(polytope, f);
  return ccdVec3Dot(&n, pt) > ccdVec3Dot(&n, &(f->edge[0]->vertex[0]->v.v));
}

/**
 * Test if the face neighbouring triangle f, and sharing the common edge
 * f->edge[edge_index] can be seen from the new vertex or not. If the face
 * cannot be seen, then we add the common edge to silhouette_edges; otherwise
 * we add the common edge to obsolete_edges. The face f will be added to 
 * obsolete_faces.
 * We will call this function recursively to traverse all faces that can be seen
 * from the new vertex.
 * @param[in] f A face that can be seen from the new vertex. This face will be
 * deleted at the end of the function.
 * @param[in] edge_index We will check if the face neighbouring f with this
 * common edge f->edge[edge_index] can be seen from the new vertex.
 * @param[in] new_vertex The new vertex to be added to the polytope.
 * @param[in/out] visited_faces If the neighbouring face hasn't been visited,
 * then add it to visited_faces.
 * @param[in/out] silhouette_edges If the neighbouring face cannot be seen from
 * the new vertex, then the common edge will be preserved, after adding the new
 * vertex, and this common edge will be added to silhouette_edges.
 * @param[in/out] obsolete_faces If the neighbouring face can be seen from
 * the new vertex, then add it to obsolete_faces.
 * @param[in/out] obsolete_edges If the neighbouring face can be seen from
 * the new vertex, then add this common edge to obsolete_edges.
 */
static void floodFillSilhouette(
    const ccd_pt_t* polytope, ccd_pt_face_t* f, int edge_index,
    const ccd_vec3_t* new_vertex,
    std::unordered_set<ccd_pt_edge_t*>* silhouette_edges,
    std::unordered_set<ccd_pt_face_t*>* obsolete_faces,
    std::unordered_set<ccd_pt_edge_t*>* obsolete_edges) {
  ccd_pt_face_t* f_neighbour = f->edge[edge_index]->faces[0] == f
                                   ? f->edge[edge_index]->faces[1]
                                   : f->edge[edge_index]->faces[0];
  const auto it = obsolete_faces->find(f_neighbour);
  if (it == obsolete_faces->end()) {
    // f_neighbour is not an obsolete face
    if (!outsidePolytopeFace(polytope, f_neighbour, new_vertex)) {
      // Cannot see the neighbouring face from the new vertex.
      silhouette_edges->insert(f->edge[edge_index]);
      return;
    } else {
      obsolete_faces->insert(f_neighbour);
      obsolete_edges->insert(f->edge[edge_index]);
      for (int i = 0; i < 3; ++i) {
        if (f_neighbour->edge[i] != f->edge[edge_index]) {
          // One of the neighbouring face is `f`, so do not need to visit again.
          floodFillSilhouette(polytope, f_neighbour, i, new_vertex,
                              silhouette_edges, obsolete_faces, obsolete_edges);
        }
      }
    }
  }
}

/** Expands the polytope by adding a new vertex @p newv to the polytope. The
 * new polytope is the convex hull of the new vertex together with the old
 * polytope. This new polytope includes new edges (by connecting the new vertex
 * with existing vertices) and new faces (by connecting the new vertex with
 * existing edges). We only keep the edges and faces that are on the boundary
 * of the new polytope. The edges/faces that are interior to the polytope are
 * discarded.
 * @param[in/out] pt The polytope.
 * @param[in] el The point on the boundary of the old polytope that is nearest
 * to the origin.
 * @param[in] newv The new vertex add to the polytope.
 * @retval status Returns 0 on success. Returns -2 otherwise. 
 */
static int expandPolytope(ccd_pt_t *pt, ccd_pt_el_t *el,
                          const ccd_support_t *newv)
{
  // The outline of the algorithm is as follows:
  // 1. Remove all the faces that can be seen from the new vertex, namely the
  //    new vertex is outside of that face. Algebraiclly, a face can be seen
  //    from the new vertex if
  //    face.normal.dot(new_vertex) > face.normal.dot(face.vertex[0])
  //    where face.normal points outward from the polytope.
  // 2. For each edge, if both neighbouring faces of the edge is removed, then
  //    remove that edge.
  // 3. For each vertex, if all edges connecting that vertex is removed, then
  //    remove that vertex.
  // 4. If an edge has one of its neighbouring face being removed, and one of
  //    its neighbouring face is preserved, we call this edge "silhouette edge".
  //    We connect the new vertex to each boundary edge, to form new faces and
  //    new edges.
  
  // To remove all faces that can be seen from the new vertex, we start with the
  // face on which the closest point lives, and then do a depth-first search on
  // its neighbouring triangles, until the triangle cannot be seen from the new
  // vertex.
  // TODO(hongkai.dai@tri.global): it is inefficient to store obsolete
  // faces/edges. A better implementation should remove obsolete faces/edges
  // inside silhouetteFloodFill function, when travering the faces on the
  // polytope. We focus on the correctness in the first place. Later
  // when we make sure that the whole EPA implementation is bug free, we will
  // improve the performance.
  //
  // Traverse the polytope faces to determine which face/edge shall be obsolete,
  // together with the silhouette edges.
  std::unordered_set<ccd_pt_face_t*> obsolete_faces;
  std::unordered_set<ccd_pt_edge_t*> obsolete_edges;
  std::unordered_set<ccd_pt_edge_t*> silhouette_edges;

  ccd_pt_face_t* start_face = NULL;
  // Start with the face on which the closest point lives
  if (el->type == CCD_PT_FACE) {
    start_face = (ccd_pt_face_t*)el;
  } else if (el->type == CCD_PT_EDGE) {
    // Check the two neighbouring faces of the edge.
    ccd_pt_face_t* f[2];
    ccdPtEdgeFaces((ccd_pt_edge_t*)el, &f[0], &f[1]);
    if (outsidePolytopeFace(pt, f[0], &newv->v)) {
      start_face = f[0];
    } else if (outsidePolytopeFace(pt, f[1], &newv->v)) {
      start_face = f[1];
    } else {
      throw std::logic_error(
          "This should not happen. Both the nearest point and the new vertex "
          "are on an edge, thus the nearest distance should be 0. This is "
          "touching contact, and we should not expand the polytope for "
          "touching contact.");
    }
  }
  obsolete_faces.insert(start_face);
  for (int i = 0; i < 3; ++i) {
    floodFillSilhouette(pt, start_face, i, &newv->v, &silhouette_edges,
                        &obsolete_faces, &obsolete_edges);
  }

  // Now remove all the obsolete faces.
  for (const auto& f : obsolete_faces) {
    ccdPtDelFace(pt, f);
  }

  // Now remove all the obsolete edges.
  for (const auto& e : obsolete_edges) {
    ccdPtDelEdge(pt, e);
  }
  // A vertex cannot be obsolete, since a vertex is always on the boundary of
  // the Minkowski difference A⊖B.
  
  // Now add the new vertex.
  ccd_pt_vertex_t* new_vertex = ccdPtAddVertex(pt, newv);

  // Now add the new edges and faces, by connecting the new vertex with vertices
  // on silhouette_edges.
  std::unordered_map<ccd_pt_vertex_t*, ccd_pt_edge_t*> new_edge_vertices;
  for (const auto& silhouette_edge : silhouette_edges) {
    ccd_pt_edge_t* e[2];  // The two new edges added by connecting new_vertex
                          // to the two vertices on silhouette_edge.
    for (int i = 0; i < 2; ++i) {
      auto it = new_edge_vertices.find(silhouette_edge->vertex[i]);
      if (it == new_edge_vertices.end()) {
        // This edge has not been added yet.
        e[i] = ccdPtAddEdge(pt, new_vertex, silhouette_edge->vertex[i]);
        new_edge_vertices.emplace_hint(it, silhouette_edge->vertex[i], e[i]);
      } else {
        e[i] = it->second;
      }
    }
    // Now add the face.
    ccdPtAddFace(pt, silhouette_edge, e[0], e[1]);
  }

  return 0;
}


/** In each iteration of EPA algorithm, given the nearest point on the polytope
 * boundary to the origin, a sampled direction will be computed, to find the
 * support of the Minkowski sum A⊖B along that direction, so as to expand the
 * polytope.
 * If we denote the nearest point as v, when the v is not the origin, then the
 * sampled direction is v. If v is the origin, then v should be an interior
 * point on a face, then the sampled direction is the normal of that face,
 * pointing outward from the polytope.
 * @param polytope The polytoped contained in A⊖B.
 * @param nearest_pt The nearest point on the boundary of the polytope to the
 * origin.
 * @retval dir The sampled direction along which to expand the polytope. Notice
 * that dir is a normalized vector.
 */
static ccd_vec3_t sampledEPADirection(const ccd_pt_t* polytope,
                                      const ccd_pt_el_t* nearest_pt) {
  ccd_vec3_t dir;
  if (ccdIsZero(nearest_pt->dist)) {
    // nearest_pt is the origin.
    switch (nearest_pt->type) {
      case CCD_PT_VERTEX: {
        throw std::runtime_error(
            "The nearest point to the origin is a vertex of the polytope. This "
            "should be identified as a touching contact, before calling this "
            "function.");
      }
      case CCD_PT_EDGE: {
        ccd_pt_edge_t* edge = (ccd_pt_edge_t*)nearest_pt;
        dir = faceNormalPointingOutward(polytope, edge->faces[0]);
        ccdVec3Normalize(&dir);
        break;
      }
      case CCD_PT_FACE: {
        // If origin is an interior point of a face, then choose the normal of
        // that face as the sample direction.
        ccd_pt_face_t* face = (ccd_pt_face_t*)nearest_pt;
        dir = faceNormalPointingOutward(polytope, face);
        ccdVec3Normalize(&dir);
        break;
      }
    }
  } else {
    ccdVec3Copy(&dir, &(nearest_pt->witness));
    ccdVec3Scale(&dir, ccd_real_t(1) / std::sqrt(nearest_pt->dist));
  }
  return dir;
}

/** Finds next support point (and stores it in out argument).
 * @param[in] polytope The current polytope contained inside the Minkowski sum
 * A⊖B.
 * @param[in] obj1 Geometric object A.
 * @param[in] obj2 Geometric object B.
 * @param[in] ccd The libccd solver.
 * @param[in] el The current nearest point on the boundary of the polytope to
 * the origin. 
 * @param[out] out The next support point.
 * @retval status If the program converges, then return -1. Otherwise return 0.
 * There are several cases that the program could converge.
 * 1. If the nearest point on the boundary of the polytope to the origin is a
 * vertex of the polytope. Then we know the two objects A and B are in touching
 * contact.
 * 2. If the difference between the upper bound and lower bound of the distance
 * is below sqrt(ccd->epa_tolerance), then we converge to a distance whose
 * difference from the real distance is less than sqrt(ccd->epa_tolerance).
 */
static int nextSupport(const ccd_pt_t* polytope, const void* obj1,
                       const void* obj2, const ccd_t* ccd,
                       const ccd_pt_el_t* el, ccd_support_t* out) {
  ccd_vec3_t *a, *b, *c;
  ccd_real_t dist;

  if (el->type == CCD_PT_VERTEX) return -1;

  const ccd_vec3_t dir = sampledEPADirection(polytope, el);

  __ccdSupport(obj1, obj2, &dir, ccd, out);

  // Compute dist of support point along element witness point direction
  // so we can determine whether we expanded a polytope surrounding the
  // origin a bit.
  dist = ccdVec3Dot(&out->v, &dir);

  if (dist - std::sqrt(el->dist) < ccd->epa_tolerance) return -1;

  if (el->type == CCD_PT_EDGE) {
    // fetch end points of edge
    ccdPtEdgeVec3((ccd_pt_edge_t*)el, &a, &b);

    // get distance from segment
    dist = ccdVec3PointSegmentDist2(&out->v, a, b, NULL);
  } else {  // el->type == CCD_PT_FACE
    // fetch vertices of triangle face
    ccdPtFaceVec3((ccd_pt_face_t*)el, &a, &b, &c);

    // check if new point can significantly expand polytope
    dist = ccdVec3PointTriDist2(&out->v, a, b, c, NULL);
  }

  if (std::sqrt(dist) < ccd->epa_tolerance) return -1;

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
    }else if (size == 3){
        ret = simplexToPolytope3(obj1, obj2, ccd, simplex, polytope, nearest);
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

    while (1){
        // get triangle nearest to origin
        *nearest = ccdPtNearest(polytope);

        // get next support point
        if (nextSupport(polytope, obj1, obj2, ccd, *nearest, &supp) != 0)
            break;

        // expand nearest triangle using new point - supp
        if (expandPolytope(polytope, *nearest, &supp) != 0)
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
  //   |pᵢ - qᵢ| <= ε·max(1, |pᵢ|, |pᵢ|)
  // And the points are coincident if the previous condition for all
  // `i ∈ {0, 1, 2}` (i.e. the x-, y-, *and* z-dimensions).
  using std::abs;
  using std::max;
  
  const ccd_real_t eps = constants<ccd_real_t>::eps();
  // NOTE: Wrapping "1.0" with ccd_real_t accounts for mac problems where ccd
  // is actually float based.
  for (int i = 0; i < 3; ++i) {
    const ccd_real_t scale =
        max({ccd_real_t{1}, abs(p.v[i]), abs(q.v[i])}) * eps;
    const ccd_real_t delta = abs(p.v[i] - q.v[i]);
    if (delta > scale) return false;
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
  const ccd_real_t eps = constants<ccd_real_t>::eps();
  // Second co-linearity condition.
  if (ccdVec3Len2(&n) < eps * eps) return true;
  return false;
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
    else if(ccdSimplexSize(simplex) == 3)
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
 * Given the nearest point on the polytope inside the Minkowski sum A⊖B, returns
 * the point p1 on geometric object A and p2 on geometric object B, such that
 * p1 is the deepest penetration point on A, and p2 is the deepest penetration
 * point on B.
 * @param[in] pt The polytope inside Minkowski sum A⊖B. Unused in this function.
 * @param[in] nearest The point that is nearest to the origin on the boundary of
 * the polytope.
 * @param[out] p1 the deepest penetration point on A.
 * @param[out] p2 the deepest penetration point on B.
 * @retval status Return 0 on success, and -1 on failure.
 */
static int penEPAPosClosest(const ccd_pt_t *pt, const ccd_pt_el_t *nearest,
                            ccd_vec3_t *p1, ccd_vec3_t* p2)
{
  // We reconstruct the simplex on which the nearest point lives, and then
  // compute the deepest penetration point on each geometric objects.
  if (nearest->type == CCD_PT_VERTEX) {
    ccd_pt_vertex_t* v = (ccd_pt_vertex_t*)nearest;
    if(v == NULL) {
      return -1;
    }
    ccdVec3Copy(p1, &v->v.v1);
    ccdVec3Copy(p2, &v->v.v2);
    return 0;
  } else {
    // (hongkai.dai@tri.global): I highly suspect this case should only
    // happen very rarely. Theoretically if the origin is strictly within the
    // interior of the polytope, then the nearest point should be an interior
    // point of a face. The only case that the nearest point is on an edge, is
    // when the two objects are in touching contact (namely the distance is 0),
    // and the origin is exactly on an edge of the polytope. 
    ccd_simplex_t s;
    ccdSimplexInit(&s);
    if (nearest->type == CCD_PT_EDGE) {
      ccd_pt_edge_t* e = (ccd_pt_edge_t*)nearest;
      if (e == NULL) {
        return -1;
      }
      ccdSimplexAdd(&s, &(e->vertex[0]->v));
      ccdSimplexAdd(&s, &(e->vertex[1]->v));
    } else if (nearest->type == CCD_PT_FACE) {
      ccd_pt_face_t* f = (ccd_pt_face_t*)nearest;
      if (f == NULL) {
        return -1;
      }
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
          //break;
        }
      }
    } else {
      throw std::logic_error(
          "Unsupported point type. The closest point should be either a "
          "vertex, on an edge, or an a face.\n");
    }
    // Now compute the closest point in the simplex.
    // TODO(hongkai.dai@tri.global): we do not need to compute the closest point
    // on the simplex, as that is already given in @p nearest. We only need to
    // extract the deepest penetration points on each geometric object.
    // Sean.Curtis@tri.global and I will refactor this code in the future, to
    // avoid calling extractClosestPoints.
    ccd_vec3_t p;
    ccdVec3Copy(&p, &(nearest->witness));
    extractClosestPoints(&s, p1, p2, &p);
    return 0;
  }
  FCL_UNUSED(pt);
}

static inline ccd_real_t ccdGJKSignedDist(const void* obj1, const void* obj2, const ccd_t* ccd, ccd_vec3_t* p1, ccd_vec3_t* p2)
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
      penEPAPosClosest(&polytope, nearest, &pos1, &pos2);

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
static inline ccd_real_t ccdGJKDist2(const void *obj1, const void *obj2, const ccd_t *ccd, ccd_vec3_t* p1, ccd_vec3_t* p2)
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
static void shapeToGJK(const ShapeBase<S>& s, const Transform3<S>& tf, ccd_obj_t* o)
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
static void capToGJK(const Capsule<S>& s, const Transform3<S>& tf, ccd_cap_t* cap)
{
  shapeToGJK(s, tf, cap);
  cap->radius = s.radius;
  cap->height = s.lz / 2;
}

template <typename S>
static void cylToGJK(const Cylinder<S>& s, const Transform3<S>& tf, ccd_cyl_t* cyl)
{
  shapeToGJK(s, tf, cyl);
  cyl->radius = s.radius;
  cyl->height = s.lz / 2;
}

template <typename S>
static void coneToGJK(const Cone<S>& s, const Transform3<S>& tf, ccd_cone_t* cone)
{
  shapeToGJK(s, tf, cone);
  cone->radius = s.radius;
  cone->height = s.lz / 2;
}

template <typename S>
static void sphereToGJK(const Sphere<S>& s, const Transform3<S>& tf, ccd_sphere_t* sph)
{
  shapeToGJK(s, tf, sph);
  sph->radius = s.radius;
}

template <typename S>
static void ellipsoidToGJK(const Ellipsoid<S>& s, const Transform3<S>& tf, ccd_ellipsoid_t* ellipsoid)
{
  shapeToGJK(s, tf, ellipsoid);
  ellipsoid->radii[0] = s.radii[0];
  ellipsoid->radii[1] = s.radii[1];
  ellipsoid->radii[2] = s.radii[2];
}

template <typename S>
static void convexToGJK(const Convex<S>& s, const Transform3<S>& tf, ccd_convex_t<S>* conv)
{
  shapeToGJK(s, tf, conv);
  conv->convex = &s;
}

/** Support functions */
static inline void supportBox(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
{
  const ccd_box_t* o = static_cast<const ccd_box_t*>(obj);
  ccd_vec3_t dir;
  ccdVec3Copy(&dir, dir_);
  ccdQuatRotVec(&dir, &o->rot_inv);
  ccdVec3Set(v, ccdSign(ccdVec3X(&dir)) * o->dim[0],
             ccdSign(ccdVec3Y(&dir)) * o->dim[1],
             ccdSign(ccdVec3Z(&dir)) * o->dim[2]);
  ccdQuatRotVec(v, &o->rot);
  ccdVec3Add(v, &o->pos);
}

static inline void supportCap(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
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

  if(ccdVec3Z (&dir) > 0)
    ccdVec3Copy(v, &pos1);
  else
    ccdVec3Copy(v, &pos2);

  // transform support vertex
  ccdQuatRotVec(v, &o->rot);
  ccdVec3Add(v, &o->pos);
}

static inline void supportCyl(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
{
  const ccd_cyl_t* cyl = static_cast<const ccd_cyl_t*>(obj);
  ccd_vec3_t dir;
  double zdist, rad;

  ccdVec3Copy(&dir, dir_);
  ccdQuatRotVec(&dir, &cyl->rot_inv);

  zdist = dir.v[0] * dir.v[0] + dir.v[1] * dir.v[1];
  zdist = sqrt(zdist);
  if(ccdIsZero(zdist))
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

static inline void supportCone(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
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

  if(dir.v[2] > len * sin_a)
    ccdVec3Set(v, 0., 0., cone->height);
  else if(zdist > 0)
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

static inline void supportSphere(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
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

static inline void supportEllipsoid(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
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
static void supportConvex(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
{
  const auto* c = (const ccd_convex_t<S>*)obj;
  ccd_vec3_t dir, p;
  ccd_real_t maxdot, dot;
  int i;
  Vector3<S>* curp;
  const auto& center = c->convex->center;

  ccdVec3Copy(&dir, dir_);
  ccdQuatRotVec(&dir, &c->rot_inv);

  maxdot = -CCD_REAL_MAX;
  curp = c->convex->points;

  for(i = 0; i < c->convex->num_points; ++i, curp += 1)
  {
    ccdVec3Set(&p, (*curp)[0] - center[0], (*curp)[1] - center[1], (*curp)[2] - center[2]);
    dot = ccdVec3Dot(&dir, &p);
    if(dot > maxdot)
    {
      ccdVec3Set(v, (*curp)[0], (*curp)[1], (*curp)[2]);
      maxdot = dot;
    }
  }

  // transform support vertex
  ccdQuatRotVec(v, &c->rot);
  ccdVec3Add(v, &c->pos);
}

static void supportTriangle(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
{
  const ccd_triangle_t* tri = static_cast<const ccd_triangle_t*>(obj);
  ccd_vec3_t dir, p;
  ccd_real_t maxdot, dot;
  int i;

  ccdVec3Copy(&dir, dir_);
  ccdQuatRotVec(&dir, &tri->rot_inv);

  maxdot = -CCD_REAL_MAX;

  for(i = 0; i < 3; ++i)
  {
    ccdVec3Set(&p, tri->p[i].v[0] - tri->c.v[0], tri->p[i].v[1] - tri->c.v[1], tri->p[i].v[2] - tri->c.v[2]);
    dot = ccdVec3Dot(&dir, &p);
    if(dot > maxdot)
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
  ccdVec3Set(c, o->convex->center[0], o->convex->center[1], o->convex->center[2]);
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
                Vector3<S>* contact_points, S* penetration_depth, Vector3<S>* normal)
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

  if(!contact_points)
  {
    return ccdMPRIntersect(obj1, obj2, &ccd);
  }


  /// libccd returns dir and pos in world space and dir is pointing from object 1 to object 2
  res = ccdMPRPenetration(obj1, obj2, &ccd, &depth, &dir, &pos);
  if(res == 0)
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
void* GJKInitializer<S, Cylinder<S>>::createGJKObject(const Cylinder<S>& s, const Transform3<S>& tf)
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
void* GJKInitializer<S, Sphere<S>>::createGJKObject(const Sphere<S>& s, const Transform3<S>& tf)
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
void* GJKInitializer<S, Ellipsoid<S>>::createGJKObject(const Ellipsoid<S>& s, const Transform3<S>& tf)
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
void* GJKInitializer<S, Box<S>>::createGJKObject(const Box<S>& s, const Transform3<S>& tf)
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
void* GJKInitializer<S, Capsule<S>>::createGJKObject(const Capsule<S>& s, const Transform3<S>& tf)
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
void* GJKInitializer<S, Cone<S>>::createGJKObject(const Cone<S>& s, const Transform3<S>& tf)
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
void* GJKInitializer<S, Convex<S>>::createGJKObject(const Convex<S>& s, const Transform3<S>& tf)
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
void* triCreateGJKObject(const Vector3<S>& P1, const Vector3<S>& P2, const Vector3<S>& P3)
{
  ccd_triangle_t* o = new ccd_triangle_t;
  Vector3<S> center((P1[0] + P2[0] + P3[0]) / 3, (P1[1] + P2[1] + P3[1]) / 3, (P1[2] + P2[2] + P3[2]) / 3);

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
void* triCreateGJKObject(const Vector3<S>& P1, const Vector3<S>& P2, const Vector3<S>& P3, const Transform3<S>& tf)
{
  ccd_triangle_t* o = new ccd_triangle_t;
  Vector3<S> center((P1[0] + P2[0] + P3[0]) / 3, (P1[1] + P2[1] + P3[1]) / 3, (P1[2] + P2[2] + P3[2]) / 3);

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
