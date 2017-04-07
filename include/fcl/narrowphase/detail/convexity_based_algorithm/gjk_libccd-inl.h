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

#include "fcl/common/unused.h"
#include "fcl/common/warning.h"

namespace fcl
{

namespace detail
{

//==============================================================================
extern template
class GJKInitializer<double, Cylinder<double>>;

//==============================================================================
extern template
class GJKInitializer<double, Sphere<double>>;

//==============================================================================
extern template
class GJKInitializer<double, Ellipsoid<double>>;

//==============================================================================
extern template
class GJKInitializer<double, Box<double>>;

//==============================================================================
extern template
class GJKInitializer<double, Capsule<double>>;

//==============================================================================
extern template
class GJKInitializer<double, Cone<double>>;

//==============================================================================
extern template
class GJKInitializer<double, Convex<double>>;

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




/** Expands polytope's tri by new vertex v. Triangle tri is replaced by
 *  three triangles each with one vertex in v. */
static int expandPolytope(ccd_pt_t *pt, ccd_pt_el_t *el,
                          const ccd_support_t *newv)
{
    ccd_pt_vertex_t *v[5];
    ccd_pt_edge_t *e[8];
    ccd_pt_face_t *f[2];


    // element can be either segment or triangle
    if (el->type == CCD_PT_EDGE){
        // In this case, segment should be replaced by new point.
        // Simpliest case is when segment stands alone and in this case
        // this segment is replaced by two other segments both connected to
        // newv.
        // Segment can be also connected to max two faces and in that case
        // each face must be replaced by two other faces. To do this
        // correctly it is necessary to have correctly ordered edges and
        // vertices which is exactly what is done in following code.
        //

        ccdPtEdgeVertices((const ccd_pt_edge_t *)el, &v[0], &v[2]);

        ccdPtEdgeFaces((ccd_pt_edge_t *)el, &f[0], &f[1]);

        if (f[0]){
            ccdPtFaceEdges(f[0], &e[0], &e[1], &e[2]);
            if (e[0] == (ccd_pt_edge_t *)el){
                e[0] = e[2];
            }else if (e[1] == (ccd_pt_edge_t *)el){
                e[1] = e[2];
            }
            ccdPtEdgeVertices(e[0], &v[1], &v[3]);
            if (v[1] != v[0] && v[3] != v[0]){
                e[2] = e[0];
                e[0] = e[1];
                e[1] = e[2];
                if (v[1] == v[2])
                    v[1] = v[3];
            }else{
                if (v[1] == v[0])
                    v[1] = v[3];
            }

            if (f[1]){
                ccdPtFaceEdges(f[1], &e[2], &e[3], &e[4]);
                if (e[2] == (ccd_pt_edge_t *)el){
                    e[2] = e[4];
                }else if (e[3] == (ccd_pt_edge_t *)el){
                    e[3] = e[4];
                }
                ccdPtEdgeVertices(e[2], &v[3], &v[4]);
                if (v[3] != v[2] && v[4] != v[2]){
                    e[4] = e[2];
                    e[2] = e[3];
                    e[3] = e[4];
                    if (v[3] == v[0])
                        v[3] = v[4];
                }else{
                    if (v[3] == v[2])
                        v[3] = v[4];
                }
            }


            v[4] = ccdPtAddVertex(pt, newv);

            ccdPtDelFace(pt, f[0]);
            if (f[1]){
                ccdPtDelFace(pt, f[1]);
                ccdPtDelEdge(pt, (ccd_pt_edge_t *)el);
            }

            e[4] = ccdPtAddEdge(pt, v[4], v[2]);
            e[5] = ccdPtAddEdge(pt, v[4], v[0]);
            e[6] = ccdPtAddEdge(pt, v[4], v[1]);
            if (f[1])
                e[7] = ccdPtAddEdge(pt, v[4], v[3]);


            if (ccdPtAddFace(pt, e[1], e[4], e[6]) == NULL
                    || ccdPtAddFace(pt, e[0], e[6], e[5]) == NULL){
                return -2;
            }

            if (f[1]){
                FCL_SUPPRESS_MAYBE_UNINITIALIZED_BEGIN
                if (ccdPtAddFace(pt, e[3], e[5], e[7]) == NULL
                        || ccdPtAddFace(pt, e[4], e[7], e[2]) == NULL){
                    return -2;
                }
                FCL_SUPPRESS_MAYBE_UNINITIALIZED_END
            }else{
                if (ccdPtAddFace(pt, e[4], e[5], (ccd_pt_edge_t *)el) == NULL)
                    return -2;
            }
        }
    }else{ // el->type == CCD_PT_FACE
        // replace triangle by tetrahedron without base (base would be the
        // triangle that will be removed)

        // get triplet of surrounding edges and vertices of triangle face
        ccdPtFaceEdges((const ccd_pt_face_t *)el, &e[0], &e[1], &e[2]);
        ccdPtEdgeVertices(e[0], &v[0], &v[1]);
        ccdPtEdgeVertices(e[1], &v[2], &v[3]);

        // following code sorts edges to have e[0] between vertices 0-1,
        // e[1] between 1-2 and e[2] between 2-0
        if (v[2] != v[1] && v[3] != v[1]){
            // swap e[1] and e[2]
            e[3] = e[1];
            e[1] = e[2];
            e[2] = e[3];
        }
        if (v[3] != v[0] && v[3] != v[1])
            v[2] = v[3];

        // remove triangle face
        ccdPtDelFace(pt, (ccd_pt_face_t *)el);

        // expand triangle to tetrahedron
        v[3] = ccdPtAddVertex(pt, newv);
        e[3] = ccdPtAddEdge(pt, v[3], v[0]);
        e[4] = ccdPtAddEdge(pt, v[3], v[1]);
        e[5] = ccdPtAddEdge(pt, v[3], v[2]);

        if (ccdPtAddFace(pt, e[3], e[4], e[0]) == NULL
                || ccdPtAddFace(pt, e[4], e[5], e[1]) == NULL
                || ccdPtAddFace(pt, e[5], e[3], e[2]) == NULL){
            return -2;
        }
    }

    return 0;
}

/** Finds next support point (and stores it in out argument).
 *  Returns 0 on success, -1 otherwise */
static int nextSupport(const void *obj1, const void *obj2, const ccd_t *ccd,
                       const ccd_pt_el_t *el,
                       ccd_support_t *out)
{
    ccd_vec3_t *a, *b, *c;
    ccd_real_t dist;

    if (el->type == CCD_PT_VERTEX)
        return -1;

    // touch contact
    if (ccdIsZero(el->dist))
        return -1;

    __ccdSupport(obj1, obj2, &el->witness, ccd, out);

    // Compute dist of support point along element witness point direction
    // so we can determine whether we expanded a polytope surrounding the
    // origin a bit.
    dist = ccdVec3Dot(&out->v, &el->witness);

    if (dist - el->dist < ccd->epa_tolerance)
        return -1;

    if (el->type == CCD_PT_EDGE){
        // fetch end points of edge
        ccdPtEdgeVec3((ccd_pt_edge_t *)el, &a, &b);

        // get distance from segment
        dist = ccdVec3PointSegmentDist2(&out->v, a, b, NULL);
    }else{ // el->type == CCD_PT_FACE
        // fetch vertices of triangle face
        ccdPtFaceVec3((ccd_pt_face_t *)el, &a, &b, &c);

        // check if new point can significantly expand polytope
        dist = ccdVec3PointTriDist2(&out->v, a, b, c, NULL);
    }

    if (dist < ccd->epa_tolerance)
        return -1;

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
        if (nextSupport(obj1, obj2, ccd, *nearest, &supp) != 0)
            break;

        // expand nearest triangle using new point - supp
        if (expandPolytope(polytope, *nearest, &supp) != 0)
            return -2;
    }

    return 0;
}


static inline ccd_real_t _ccdDist(const void *obj1, const void *obj2,
                                  const ccd_t *ccd,
                                  ccd_simplex_t* simplex,
                                  ccd_vec3_t* p1, ccd_vec3_t* p2)
{
  unsigned long iterations;
  ccd_support_t last; // last support point
  ccd_vec3_t dir; // direction vector
  ccd_real_t dist, last_dist = CCD_REAL_MAX;

  for (iterations = 0UL; iterations < ccd->max_iterations; ++iterations)
  {
    // get a next direction vector
    // we are trying to find out a point on the minkowski difference
    // that is nearest to the origin, so we obtain a point on the
    // simplex that is nearest and try to exapand the simplex towards
    // the origin
    if (ccdSimplexSize(simplex) == 1)
    {
      ccdVec3Copy(&dir, &ccdSimplexPoint(simplex, 0)->v);
      dist = ccdVec3Len2(&ccdSimplexPoint(simplex, 0)->v);
      dist = CCD_SQRT(dist);
    }
    else if (ccdSimplexSize(simplex) == 2)
    {
      dist = ccdVec3PointSegmentDist2(ccd_vec3_origin,
                                      &ccdSimplexPoint(simplex, 0)->v,
                                      &ccdSimplexPoint(simplex, 1)->v,
                                      &dir);
      dist = CCD_SQRT(dist);
    }
    else if(ccdSimplexSize(simplex) == 3)
    {
      dist = ccdVec3PointTriDist2(ccd_vec3_origin,
                                  &ccdSimplexPoint(simplex, 0)->v,
                                  &ccdSimplexPoint(simplex, 1)->v,
                                  &ccdSimplexPoint(simplex, 2)->v,
                                  &dir);
      dist = CCD_SQRT(dist);
    }
    else
    { // ccdSimplexSize(&simplex) == 4
      dist = simplexReduceToTriangle(simplex, last_dist, &dir);
    }

    // check whether we improved for at least a minimum tolerance
    if ((last_dist - dist) < ccd->dist_tolerance)
    {
      if(p1) *p1 = last.v1;
      if(p2) *p2 = last.v2;
      return dist;
    }

    // point direction towards the origin
    ccdVec3Scale(&dir, -CCD_ONE);
    ccdVec3Normalize(&dir);

    // find out support point
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
      if(p1) *p1 = last.v1;
      if(p2) *p2 = last.v2;
      return last_dist;
    }

    // add a point to simplex
    ccdSimplexAdd(simplex, &last);
  }

  return -CCD_REAL(1.);
}

static int penEPAPosCmp(const void *a, const void *b)
{
    ccd_pt_vertex_t *v1, *v2;
    v1 = *(ccd_pt_vertex_t **)a;
    v2 = *(ccd_pt_vertex_t **)b;

    if (ccdEq(v1->dist, v2->dist)){
        return 0;
    }else if (v1->dist < v2->dist){
        return -1;
    }else{
        return 1;
    }
}

static int penEPAPosClosest(const ccd_pt_t *pt, const ccd_pt_el_t *nearest,
                            ccd_vec3_t *p1, ccd_vec3_t* p2)
{
    FCL_UNUSED(nearest);

    ccd_pt_vertex_t *v;
    ccd_pt_vertex_t **vs;
    size_t i, len;
    // compute median
    len = 0;
    ccdListForEachEntry(&pt->vertices, v, ccd_pt_vertex_t, list){
        len++;
    }

    vs = CCD_ALLOC_ARR(ccd_pt_vertex_t*, len);
    if (vs == NULL)
        return -1;

    i = 0;
    ccdListForEachEntry(&pt->vertices, v, ccd_pt_vertex_t, list){
        vs[i++] = v;
    }

    qsort(vs, len, sizeof(ccd_pt_vertex_t*), penEPAPosCmp);

    ccdVec3Copy(p1, &vs[0]->v.v1);
    ccdVec3Copy(p2, &vs[0]->v.v2);

    free(vs);

    return 0;
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


/// change the libccd distance to add two closest points
static inline ccd_real_t ccdGJKDist2(const void *obj1, const void *obj2, const ccd_t *ccd, ccd_vec3_t* p1, ccd_vec3_t* p2)
{
  unsigned long iterations;
  ccd_simplex_t simplex;
  ccd_support_t last; // last support point
  ccd_vec3_t dir; // direction vector
  ccd_real_t dist, last_dist;

  // first find an intersection
  if (__ccdGJK(obj1, obj2, ccd, &simplex) == 0)
    return -CCD_ONE;

  last_dist = CCD_REAL_MAX;

  for (iterations = 0UL; iterations < ccd->max_iterations; ++iterations) {
    // get a next direction vector
    // we are trying to find out a point on the minkowski difference
    // that is nearest to the origin, so we obtain a point on the
    // simplex that is nearest and try to exapand the simplex towards
    // the origin
    if (ccdSimplexSize(&simplex) == 1){
      ccdVec3Copy(&dir, &ccdSimplexPoint(&simplex, 0)->v);
      dist = ccdVec3Len2(&ccdSimplexPoint(&simplex, 0)->v);
      dist = CCD_SQRT(dist);
    }else if (ccdSimplexSize(&simplex) == 2){
      dist = ccdVec3PointSegmentDist2(ccd_vec3_origin,
                                      &ccdSimplexPoint(&simplex, 0)->v,
                                      &ccdSimplexPoint(&simplex, 1)->v,
                                      &dir);
      dist = CCD_SQRT(dist);
    }else if(ccdSimplexSize(&simplex) == 3){
      dist = ccdVec3PointTriDist2(ccd_vec3_origin,
                                  &ccdSimplexPoint(&simplex, 0)->v,
                                  &ccdSimplexPoint(&simplex, 1)->v,
                                  &ccdSimplexPoint(&simplex, 2)->v,
                                  &dir);
      dist = CCD_SQRT(dist);
    }else{ // ccdSimplexSize(&simplex) == 4
      dist = simplexReduceToTriangle(&simplex, last_dist, &dir);
    }

    // check whether we improved for at least a minimum tolerance
    if ((last_dist - dist) < ccd->dist_tolerance)
    {
      if(p1) *p1 = last.v1;
      if(p2) *p2 = last.v2;
      return dist;
    }

    // point direction towards the origin
    ccdVec3Scale(&dir, -CCD_ONE);
    ccdVec3Normalize(&dir);

    // find out support point
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
      if(p1) *p1 = last.v1;
      if(p2) *p2 = last.v2;
      return last_dist;
    }

    // add a point to simplex
    ccdSimplexAdd(&simplex, &last);
  }

  return -CCD_REAL(1.);
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


/// p1 and p2 are in global coordinate, so needs transform in the narrowphase.h functions
template <typename S>
bool GJKDistance(void* obj1, ccd_support_fn supp1,
                 void* obj2, ccd_support_fn supp2,
                 unsigned int max_iterations, S tolerance,
                 S* res, Vector3<S>* p1, Vector3<S>* p2)
{
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
  dist = libccd_extension::ccdGJKDist2(obj1, obj2, &ccd, &p1_, &p2_);
  if(p1) *p1 << ccdVec3X(&p1_), ccdVec3Y(&p1_), ccdVec3Z(&p1_);
  if(p2) *p2 << ccdVec3X(&p2_), ccdVec3Y(&p2_), ccdVec3Z(&p2_);
  if(res) *res = dist;
  if(dist < 0) return false;
  else return true;
}


/// p1 and p2 are in global coordinate, so needs transform in the narrowphase.h functions
template <typename S>
bool GJKSignedDistance(void* obj1, ccd_support_fn supp1,
                       void* obj2, ccd_support_fn supp2,
                       unsigned int max_iterations, S tolerance,
                       S* res, Vector3<S>* p1, Vector3<S>* p2)
{
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
  dist = libccd_extension::ccdGJKSignedDist(obj1, obj2, &ccd, &p1_, &p2_);
  if(p1) *p1 << ccdVec3X(&p1_), ccdVec3Y(&p1_), ccdVec3Z(&p1_);
  if(p2) *p2 << ccdVec3X(&p2_), ccdVec3Y(&p2_), ccdVec3Z(&p2_);
  if(res) *res = dist;
  if(dist < 0) return false;
  else return true;
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
