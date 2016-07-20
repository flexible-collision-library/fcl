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


#include "fcl/narrowphase/gjk_libccd.h"
#include "fcl/ccd/simplex.h"
#include <ccd/vec3.h>
#include <ccd/ccd.h>

namespace fcl
{

namespace details
{

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

struct ccd_convex_t : public ccd_obj_t
{
  const Convex* convex;
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
  dist = ccdVec3PointTriDist2(ccd_vec3_origin, &A->v, &B->v, &C->v, NULL);
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
  dist = ccdVec3PointTriDist2(&A->v, &B->v, &C->v, &D->v, NULL);
  if (ccdIsZero(dist)){
    return -1;
  }

  // check if origin lies on some of tetrahedron's face - if so objects
  // intersect
  dist = ccdVec3PointTriDist2(ccd_vec3_origin, &A->v, &B->v, &C->v, NULL);
  if (ccdIsZero(dist))
    return 1;
  dist = ccdVec3PointTriDist2(ccd_vec3_origin, &A->v, &C->v, &D->v, NULL);
  if (ccdIsZero(dist))
    return 1;
  dist = ccdVec3PointTriDist2(ccd_vec3_origin, &A->v, &B->v, &D->v, NULL);
  if (ccdIsZero(dist))
    return 1;
  dist = ccdVec3PointTriDist2(ccd_vec3_origin, &B->v, &C->v, &D->v, NULL);
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

/// change the libccd distance to add two closest points
ccd_real_t ccdGJKDist2(const void *obj1, const void *obj2, const ccd_t *ccd, ccd_vec3_t* p1, ccd_vec3_t* p2)
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

    // touching contact -- do we really need this?
    // maybe __ccdGJK() solve this alredy.
    if (ccdIsZero(dist))
      return -CCD_ONE;

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

}


/** Basic shape to ccd shape */
static void shapeToGJK(const ShapeBase& s, const Transform3f& tf, ccd_obj_t* o)
{
  const Quaternion3f& q = tf.getQuatRotation();
  const Vec3f& T = tf.getTranslation();
  ccdVec3Set(&o->pos, T[0], T[1], T[2]);
  ccdQuatSet(&o->rot, q.getX(), q.getY(), q.getZ(), q.getW());
  ccdQuatInvert2(&o->rot_inv, &o->rot);
}

static void boxToGJK(const Box& s, const Transform3f& tf, ccd_box_t* box)
{
  shapeToGJK(s, tf, box);
  box->dim[0] = s.side[0] / 2.0;
  box->dim[1] = s.side[1] / 2.0;
  box->dim[2] = s.side[2] / 2.0;
}

static void capToGJK(const Capsule& s, const Transform3f& tf, ccd_cap_t* cap)
{
  shapeToGJK(s, tf, cap);
  cap->radius = s.radius;
  cap->height = s.lz / 2;
}

static void cylToGJK(const Cylinder& s, const Transform3f& tf, ccd_cyl_t* cyl)
{
  shapeToGJK(s, tf, cyl);
  cyl->radius = s.radius;
  cyl->height = s.lz / 2;
}

static void coneToGJK(const Cone& s, const Transform3f& tf, ccd_cone_t* cone)
{
  shapeToGJK(s, tf, cone);
  cone->radius = s.radius;
  cone->height = s.lz / 2;
}

static void sphereToGJK(const Sphere& s, const Transform3f& tf, ccd_sphere_t* sph)
{
  shapeToGJK(s, tf, sph);
  sph->radius = s.radius;
}

static void ellipsoidToGJK(const Ellipsoid& s, const Transform3f& tf, ccd_ellipsoid_t* ellipsoid)
{
  shapeToGJK(s, tf, ellipsoid);
  ellipsoid->radii[0] = s.radii[0];
  ellipsoid->radii[1] = s.radii[1];
  ellipsoid->radii[2] = s.radii[2];
}

static void convexToGJK(const Convex& s, const Transform3f& tf, ccd_convex_t* conv)
{
  shapeToGJK(s, tf, conv);
  conv->convex = &s;
}

/** Support functions */
static void supportBox(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
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

static void supportCap(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
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

static void supportCyl(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
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

static void supportCone(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
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

static void supportSphere(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
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

static void supportEllipsoid(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
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

static void supportConvex(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
{
  const ccd_convex_t* c = (const ccd_convex_t*)obj;
  ccd_vec3_t dir, p;
  ccd_real_t maxdot, dot;
  int i;
  Vec3f* curp;
  const Vec3f& center = c->convex->center;

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

static void centerShape(const void* obj, ccd_vec3_t* c)
{
  const ccd_obj_t *o = static_cast<const ccd_obj_t*>(obj);
  ccdVec3Copy(c, &o->pos);
}

static void centerConvex(const void* obj, ccd_vec3_t* c)
{
  const ccd_convex_t *o = static_cast<const ccd_convex_t*>(obj);
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

bool GJKCollide(void* obj1, ccd_support_fn supp1, ccd_center_fn cen1,
                void* obj2, ccd_support_fn supp2, ccd_center_fn cen2,
                unsigned int max_iterations, FCL_REAL tolerance,
                Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal)
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
    contact_points->setValue(ccdVec3X(&pos), ccdVec3Y(&pos), ccdVec3Z(&pos));
    *penetration_depth = depth;
    normal->setValue(ccdVec3X(&dir), ccdVec3Y(&dir), ccdVec3Z(&dir));

    return true;
  }

  return false;
}


/// p1 and p2 are in global coordinate, so needs transform in the narrowphase.h functions
bool GJKDistance(void* obj1, ccd_support_fn supp1,
                 void* obj2, ccd_support_fn supp2,
                 unsigned int max_iterations, FCL_REAL tolerance,
                 FCL_REAL* res, Vec3f* p1, Vec3f* p2)
{
  ccd_t ccd;
  ccd_real_t dist;
  CCD_INIT(&ccd);
  ccd.support1 = supp1;
  ccd.support2 = supp2;
  
  ccd.max_iterations = max_iterations;
  ccd.dist_tolerance = tolerance;

  ccd_vec3_t p1_, p2_;
  dist = libccd_extension::ccdGJKDist2(obj1, obj2, &ccd, &p1_, &p2_);
  if(p1) p1->setValue(ccdVec3X(&p1_), ccdVec3Y(&p1_), ccdVec3Z(&p1_));
  if(p2) p2->setValue(ccdVec3X(&p2_), ccdVec3Y(&p2_), ccdVec3Z(&p2_));
  if(res) *res = dist;
  if(dist < 0) return false;
  else return true;
}


GJKSupportFunction GJKInitializer<Cylinder>::getSupportFunction()
{
  return &supportCyl;
}


GJKCenterFunction GJKInitializer<Cylinder>::getCenterFunction()
{
  return &centerShape;
}


void* GJKInitializer<Cylinder>::createGJKObject(const Cylinder& s, const Transform3f& tf)
{
  ccd_cyl_t* o = new ccd_cyl_t;
  cylToGJK(s, tf, o);
  return o;
}


void GJKInitializer<Cylinder>::deleteGJKObject(void* o_)
{
  ccd_cyl_t* o = static_cast<ccd_cyl_t*>(o_);
  delete o;
}


GJKSupportFunction GJKInitializer<Sphere>::getSupportFunction()
{
  return &supportSphere;
}


GJKCenterFunction GJKInitializer<Sphere>::getCenterFunction()
{
  return &centerShape;
}


void* GJKInitializer<Sphere>::createGJKObject(const Sphere& s, const Transform3f& tf)
{
  ccd_sphere_t* o = new ccd_sphere_t;
  sphereToGJK(s, tf, o);
  return o;
}

void GJKInitializer<Sphere>::deleteGJKObject(void* o_)
{
  ccd_sphere_t* o = static_cast<ccd_sphere_t*>(o_);
  delete o;
}

GJKSupportFunction GJKInitializer<Ellipsoid>::getSupportFunction()
{
  return &supportEllipsoid;
}

GJKCenterFunction GJKInitializer<Ellipsoid>::getCenterFunction()
{
  return &centerShape;
}

void* GJKInitializer<Ellipsoid>::createGJKObject(const Ellipsoid& s, const Transform3f& tf)
{
  ccd_ellipsoid_t* o = new ccd_ellipsoid_t;
  ellipsoidToGJK(s, tf, o);
  return o;
}

void GJKInitializer<Ellipsoid>::deleteGJKObject(void* o_)
{
  ccd_ellipsoid_t* o = static_cast<ccd_ellipsoid_t*>(o_);
  delete o;
}

GJKSupportFunction GJKInitializer<Box>::getSupportFunction()
{
  return &supportBox;
}


GJKCenterFunction GJKInitializer<Box>::getCenterFunction()
{
  return &centerShape;
}


void* GJKInitializer<Box>::createGJKObject(const Box& s, const Transform3f& tf)
{
  ccd_box_t* o = new ccd_box_t;
  boxToGJK(s, tf, o);
  return o;
}


void GJKInitializer<Box>::deleteGJKObject(void* o_)
{
  ccd_box_t* o = static_cast<ccd_box_t*>(o_);
  delete o;
}


GJKSupportFunction GJKInitializer<Capsule>::getSupportFunction()
{
  return &supportCap;
}


GJKCenterFunction GJKInitializer<Capsule>::getCenterFunction()
{
  return &centerShape;
}


void* GJKInitializer<Capsule>::createGJKObject(const Capsule& s, const Transform3f& tf)
{
  ccd_cap_t* o = new ccd_cap_t;
  capToGJK(s, tf, o);
  return o;
}


void GJKInitializer<Capsule>::deleteGJKObject(void* o_)
{
  ccd_cap_t* o = static_cast<ccd_cap_t*>(o_);
  delete o;
}


GJKSupportFunction GJKInitializer<Cone>::getSupportFunction()
{
  return &supportCone;
}


GJKCenterFunction GJKInitializer<Cone>::getCenterFunction()
{
  return &centerShape;
}


void* GJKInitializer<Cone>::createGJKObject(const Cone& s, const Transform3f& tf)
{
  ccd_cone_t* o = new ccd_cone_t;
  coneToGJK(s, tf, o);
  return o;
}


void GJKInitializer<Cone>::deleteGJKObject(void* o_)
{
  ccd_cone_t* o = static_cast<ccd_cone_t*>(o_);
  delete o;
}


GJKSupportFunction GJKInitializer<Convex>::getSupportFunction()
{
  return &supportConvex;
}


GJKCenterFunction GJKInitializer<Convex>::getCenterFunction()
{
  return &centerConvex;
}


void* GJKInitializer<Convex>::createGJKObject(const Convex& s, const Transform3f& tf)
{
  ccd_convex_t* o = new ccd_convex_t;
  convexToGJK(s, tf, o);
  return o;
}


void GJKInitializer<Convex>::deleteGJKObject(void* o_)
{
  ccd_convex_t* o = static_cast<ccd_convex_t*>(o_);
  delete o;
}


GJKSupportFunction triGetSupportFunction()
{
  return &supportTriangle;
}


GJKCenterFunction triGetCenterFunction()
{
  return &centerTriangle;
}


void* triCreateGJKObject(const Vec3f& P1, const Vec3f& P2, const Vec3f& P3)
{
  ccd_triangle_t* o = new ccd_triangle_t;
  Vec3f center((P1[0] + P2[0] + P3[0]) / 3, (P1[1] + P2[1] + P3[1]) / 3, (P1[2] + P2[2] + P3[2]) / 3);

  ccdVec3Set(&o->p[0], P1[0], P1[1], P1[2]);
  ccdVec3Set(&o->p[1], P2[0], P2[1], P2[2]);
  ccdVec3Set(&o->p[2], P3[0], P3[1], P3[2]);
  ccdVec3Set(&o->c, center[0], center[1], center[2]);
  ccdVec3Set(&o->pos, 0., 0., 0.);
  ccdQuatSet(&o->rot, 0., 0., 0., 1.);
  ccdQuatInvert2(&o->rot_inv, &o->rot);

  return o;
}

void* triCreateGJKObject(const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Transform3f& tf)
{
  ccd_triangle_t* o = new ccd_triangle_t;
  Vec3f center((P1[0] + P2[0] + P3[0]) / 3, (P1[1] + P2[1] + P3[1]) / 3, (P1[2] + P2[2] + P3[2]) / 3);

  ccdVec3Set(&o->p[0], P1[0], P1[1], P1[2]);
  ccdVec3Set(&o->p[1], P2[0], P2[1], P2[2]);
  ccdVec3Set(&o->p[2], P3[0], P3[1], P3[2]);
  ccdVec3Set(&o->c, center[0], center[1], center[2]);
  const Quaternion3f& q = tf.getQuatRotation();
  const Vec3f& T = tf.getTranslation();
  ccdVec3Set(&o->pos, T[0], T[1], T[2]);
  ccdQuatSet(&o->rot, q.getX(), q.getY(), q.getZ(), q.getW());
  ccdQuatInvert2(&o->rot_inv, &o->rot);

  return o;
}

void triDeleteGJKObject(void* o_)
{
  ccd_triangle_t* o = static_cast<ccd_triangle_t*>(o_);
  delete o;
}

} // details

} // fcl
