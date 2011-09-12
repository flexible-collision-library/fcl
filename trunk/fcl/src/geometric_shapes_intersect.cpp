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


#include "fcl/geometric_shapes_intersect.h"

namespace fcl
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

struct ccd_convex_t : public ccd_obj_t
{
  const Convex* convex;
};

struct ccd_triangle_t : public ccd_obj_t
{
  ccd_vec3_t p[3];
  ccd_vec3_t c;
};

void transformGJKObject(void* obj, const Vec3f R[3], const Vec3f& T)
{
  ccd_obj_t* o = (ccd_obj_t*)obj;
  SimpleQuaternion q_;
  q_.fromRotation(R);

  ccd_vec3_t t;
  ccd_quat_t q;
  ccdVec3Set(&t, T[0], T[1], T[2]);
  ccdQuatSet(&q, q_.getX(), q_.getY(), q_.getZ(), q_.getW());

  ccd_quat_t tmp;
  ccdQuatMul2(&tmp, &q, &o->rot); // make sure it is correct here!!
  ccdQuatCopy(&o->rot, &tmp);
  ccdQuatRotVec(&o->pos, &q);
  ccdVec3Add(&o->pos, &t);
  ccdQuatInvert2(&o->rot_inv, &o->rot);
}

/** Basic shape to ccd shape */
static void shapeToGJK(const ShapeBase& s, ccd_obj_t* o)
{

  SimpleQuaternion q;
  Vec3f R[3];
  matMulMat(s.getRotation(), s.getLocalRotation(), R);
  q.fromRotation(R);
  Vec3f T = matMulVec(s.getRotation(), s.getLocalTranslation()) + s.getTranslation();
  ccdVec3Set(&o->pos, T[0], T[1], T[2]);
  ccdQuatSet(&o->rot, q.getX(), q.getY(), q.getZ(), q.getW());
  ccdQuatInvert2(&o->rot_inv, &o->rot);
/*
  SimpleQuaternion q;
  q.fromRotation(s.getLocalRotation());
  Vec3f T = s.getLocalTranslation();
  ccdVec3Set(&o->pos, T[0], T[1], T[2]);
  ccdQuatSet(&o->rot, q.getX(), q.getY(), q.getZ(), q.getW());
  ccdQuatInvert2(&o->rot_inv, &o->rot);

  transformGJKObject(o, s.getRotation(), s.getTranslation());
*/
}

static void boxToGJK(const Box& s, ccd_box_t* box)
{
  shapeToGJK(s, box);
  box->dim[0] = s.side[0] / 2.0;
  box->dim[1] = s.side[1] / 2.0;
  box->dim[2] = s.side[2] / 2.0;
}

static void capToGJK(const Capsule& s, ccd_cap_t* cap)
{
  shapeToGJK(s, cap);
  cap->radius = s.radius;
  cap->height = s.lz / 2;
}

static void cylToGJK(const Cylinder& s, ccd_cyl_t* cyl)
{
  shapeToGJK(s, cyl);
  cyl->radius = s.radius;
  cyl->height = s.lz / 2;
}

static void coneToGJK(const Cone& s, ccd_cone_t* cone)
{
  shapeToGJK(s, cone);
  cone->radius = s.radius;
  cone->height = s.lz / 2;
}

static void sphereToGJK(const Sphere& s, ccd_sphere_t* sph)
{
  shapeToGJK(s, sph);
  sph->radius = s.radius;
}

static void convexToGJK(const Convex& s, ccd_convex_t* conv)
{
  shapeToGJK(s, conv);
  conv->convex = &s;
}

/** Support functions */
static void supportBox(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
{
  const ccd_box_t* o = (const ccd_box_t*)obj;
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
  const ccd_cap_t* o = (const ccd_cap_t*)obj;
  ccd_vec3_t dir, pos1, pos2;

  ccdVec3Copy(&dir, dir_);
  ccdQuatRotVec(&dir, &o->rot_inv);

  ccdVec3Set(&pos1, CCD_ZERO, CCD_ZERO, o->height);
  ccdVec3Set(&pos2, CCD_ZERO, CCD_ZERO, -o->height);

  ccdVec3Copy(v, &dir);
  ccdVec3Scale(v, o->radius);
  ccdVec3Add(&pos1, v);
  ccdVec3Add(&pos2, v);

  if(ccdVec3Dot(&dir, &pos1) > ccdVec3Dot(&dir, &pos2))
    ccdVec3Copy(v, &pos1);
  else
    ccdVec3Copy(v, &pos2);

  // transform support vertex
  ccdQuatRotVec(v, &o->rot);
  ccdVec3Add(v, &o->pos);
}

static void supportCyl(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
{
  const ccd_cyl_t* cyl = (const ccd_cyl_t*)obj;
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
  const ccd_cone_t* cone = (const ccd_cone_t*)obj;
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
  const ccd_sphere_t* s = (const ccd_sphere_t*)obj;
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
  const ccd_triangle_t* tri = (const ccd_triangle_t*)obj;
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
    const ccd_obj_t *o = (const ccd_obj_t*)obj;
    ccdVec3Copy(c, &o->pos);
}

static void centerConvex(const void* obj, ccd_vec3_t* c)
{
  const ccd_convex_t *o = (const ccd_convex_t*)obj;
  ccdVec3Set(c, o->convex->center[0], o->convex->center[1], o->convex->center[2]);
  ccdQuatRotVec(c, &o->rot);
  ccdVec3Add(c, &o->pos);
}

static void centerTriangle(const void* obj, ccd_vec3_t* c)
{
    const ccd_triangle_t *o = (const ccd_triangle_t*)obj;
    ccdVec3Copy(c, &o->c);
    ccdQuatRotVec(c, &o->rot);
    ccdVec3Add(c, &o->pos);
}

bool GJKCollide(void* obj1, ccd_support_fn supp1, ccd_center_fn cen1,
               void* obj2, ccd_support_fn supp2, ccd_center_fn cen2,
               Vec3f* contact_points, BVH_REAL* penetration_depth, Vec3f* normal)
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
  ccd.max_iterations = 500;
  ccd.mpr_tolerance = 1e-6;

  if(!contact_points)
  {
    return ccdMPRIntersect(obj1, obj2, &ccd);
  }


  res = ccdMPRPenetration(obj1, obj2, &ccd, &depth, &dir, &pos);
  if(res == 0)
  {
    *contact_points = Vec3f(ccdVec3X(&pos), ccdVec3Y(&pos), ccdVec3Z(&pos));
    *penetration_depth = depth;
    *normal = Vec3f(-ccdVec3X(&dir), -ccdVec3Y(&dir), -ccdVec3Z(&dir));

    return true;
  }

  return false;
}


GJKSupportFunction GJKInitializer<Cylinder>::getSupportFunction()
{
  return &supportCyl;
}


GJKCenterFunction GJKInitializer<Cylinder>::getCenterFunction()
{
  return &centerShape;
}


void* GJKInitializer<Cylinder>::createGJKObject(const Cylinder& s)
{
  ccd_cyl_t* o = new ccd_cyl_t;
  cylToGJK(s, o);
  return o;
}


void GJKInitializer<Cylinder>::deleteGJKObject(void* o_)
{
  ccd_cyl_t* o = (ccd_cyl_t*)o_;
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


void* GJKInitializer<Sphere>::createGJKObject(const Sphere& s)
{
  ccd_sphere_t* o = new ccd_sphere_t;
  sphereToGJK(s, o);
  return o;
}

void GJKInitializer<Sphere>::deleteGJKObject(void* o_)
{
  ccd_sphere_t* o = (ccd_sphere_t*)o_;
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


void* GJKInitializer<Box>::createGJKObject(const Box& s)
{
  ccd_box_t* o = new ccd_box_t;
  boxToGJK(s, o);
  return o;
}


void GJKInitializer<Box>::deleteGJKObject(void* o_)
{
  ccd_box_t* o = (ccd_box_t*)o_;
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


void* GJKInitializer<Capsule>::createGJKObject(const Capsule& s)
{
  ccd_cap_t* o = new ccd_cap_t;
  capToGJK(s, o);
  return o;
}


void GJKInitializer<Capsule>::deleteGJKObject(void* o_)
{
  ccd_cap_t* o = (ccd_cap_t*)o_;
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


void* GJKInitializer<Cone>::createGJKObject(const Cone& s)
{
  ccd_cone_t* o = new ccd_cone_t;
  coneToGJK(s, o);
  return o;
}


void GJKInitializer<Cone>::deleteGJKObject(void* o_)
{
  ccd_cone_t* o = (ccd_cone_t*)o_;
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


void* GJKInitializer<Convex>::createGJKObject(const Convex& s)
{
  ccd_convex_t* o = new ccd_convex_t;
  convexToGJK(s, o);
  return o;
}


void GJKInitializer<Convex>::deleteGJKObject(void* o_)
{
  ccd_convex_t* o = (ccd_convex_t*)o_;
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

void* triCreateGJKObject(const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Vec3f R[3], Vec3f const& T)
{
  ccd_triangle_t* o = new ccd_triangle_t;
  Vec3f center((P1[0] + P2[0] + P3[0]) / 3, (P1[1] + P2[1] + P3[1]) / 3, (P1[2] + P2[2] + P3[2]) / 3);

  ccdVec3Set(&o->p[0], P1[0], P1[1], P1[2]);
  ccdVec3Set(&o->p[1], P2[0], P2[1], P2[2]);
  ccdVec3Set(&o->p[2], P3[0], P3[1], P3[2]);
  ccdVec3Set(&o->c, center[0], center[1], center[2]);
  SimpleQuaternion q;
  q.fromRotation(R);
  ccdVec3Set(&o->pos, T[0], T[1], T[2]);
  ccdQuatSet(&o->rot, q.getX(), q.getY(), q.getZ(), q.getW());
  ccdQuatInvert2(&o->rot_inv, &o->rot);

  return o;
}

void triDeleteGJKObject(void* o_)
{
  ccd_triangle_t* o = (ccd_triangle_t*)o_;
  delete o;
}

}
