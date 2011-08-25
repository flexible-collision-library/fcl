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


#ifndef FCL_GEOMETRIC_SHAPES_H
#define FCL_GEOMETRIC_SHAPES_H

#include "fcl/collision_object.h"
#include "fcl/vec_3f.h"
#include <string.h>

/** \brief Main namespace */
namespace fcl
{

/** \brief Base class for all basic geometric shapes */
class ShapeBase : public CollisionObject
{
public:
  ShapeBase()
  {
    Rloc[0][0] = 1;
    Rloc[1][1] = 1;
    Rloc[2][2] = 1;
  }

  void setLocalTransform(const Vec3f R_[3], const Vec3f& T_)
  {
    Rloc[0] = R_[0];
    Rloc[1] = R_[1];
    Rloc[2] = R_[2];
    Tloc = T_;
  }

  void setLocalRotation(const Vec3f R[3])
  {
    Rloc[0] = R[0];
    Rloc[1] = R[1];
    Rloc[2] = R[2];
  }

  void setLocalTranslation(const Vec3f& T)
  {
    Tloc = T;
  }

  void advanceLocalTransform(const Vec3f R[3], const Vec3f& T)
  {
    Vec3f R0[3];
    for(int i = 0; i < 3; ++i)
      R0[i] = Rloc[i];
    MxM(R, R0, Rloc);
    Tloc = MxV(R, Tloc) + T;
  }

  void getLocalTransfrom(Vec3f R[3], Vec3f& T) const
  {
    T = Tloc;
    R[0] = Rloc[0];
    R[1] = Rloc[1];
    R[2] = Rloc[2];
  }

  inline const Vec3f& getLocalPosition() const
  {
    return Tloc;
  }

  inline const Vec3f* getLocalRotation() const
  {
    return Rloc;
  }

  OBJECT_TYPE getObjectType() const { return OT_GEOM; }

protected:

  Vec3f Rloc[3];
  Vec3f Tloc;
};


/** Center at zero point, axis aligned box */
class Box : public ShapeBase
{
public:
  Box(BVH_REAL x, BVH_REAL y, BVH_REAL z) : ShapeBase(), side(x, y, z) {}

  /** box side length */
  Vec3f side;

  void computeAABB();

  NODE_TYPE getNodeType() const { return GEOM_BOX; }
};

/** Center at zero point sphere */
class Sphere : public ShapeBase
{
public:
  Sphere(BVH_REAL radius_) : ShapeBase(), radius(radius_) {}
  BVH_REAL radius;

  void computeAABB();

  NODE_TYPE getNodeType() const { return GEOM_SPHERE; }
};

/** Center at zero point capsule */
class Capsule : public ShapeBase
{
public:
  Capsule(BVH_REAL radius_, BVH_REAL lz_) : ShapeBase(), radius(radius_), lz(lz_) {}
  BVH_REAL radius;
  BVH_REAL lz; // length along z axis

  void computeAABB();

  NODE_TYPE getNodeType() const { return GEOM_CAPSULE; }
};

/** Center at zero cone */
class Cone : public ShapeBase
{
public:
  Cone(BVH_REAL radius_, BVH_REAL lz_) : ShapeBase(), radius(radius_), lz(lz_) {}
  BVH_REAL radius;
  BVH_REAL lz; // length along z axis

  void computeAABB();

  NODE_TYPE getNodeType() const { return GEOM_CONE; }
};

/** Center at zero cylinder */
class Cylinder : public ShapeBase
{
public:
  Cylinder(BVH_REAL radius_, BVH_REAL lz_) : ShapeBase(), radius(radius_), lz(lz_) {}
  BVH_REAL radius;
  BVH_REAL lz; // length along z axis

  void computeAABB();

  NODE_TYPE getNodeType() const { return GEOM_CYLINDER; }
};

/** Convex polytope */
class Convex : public ShapeBase
{
public:
  Convex(Vec3f* plane_normals_,
         BVH_REAL* plane_dis_,
         int num_planes_,
         Vec3f* points_,
         int num_points_,
         int* polygons_) : ShapeBase()
  {
    plane_normals = plane_normals_;
    plane_dis = plane_dis_;
    num_planes = num_planes_;
    points = points_;
    polygons = polygons_;
    edges = NULL;

    Vec3f sum;
    for(int i = 0; i < num_points; ++i)
    {
      sum += points[i];
    }

    center = sum * (BVH_REAL)(1.0 / num_points);

    fillEdges();
  }

  Convex(const Convex& other) : ShapeBase(other)
  {
    plane_normals = other.plane_normals;
    plane_dis = other.plane_dis;
    num_planes = other.num_planes;
    points = other.points;
    polygons = other.polygons;
    edges = new Edge[other.num_edges];
    memcpy(edges, other.edges, sizeof(Edge) * num_edges);
  }

  ~Convex()
  {
    delete [] edges;
  }

  void computeAABB();

  NODE_TYPE getNodeType() const { return GEOM_CONVEX; }

  Vec3f* plane_normals;
  BVH_REAL* plane_dis;

  /** An array of indices to the points of each polygon, it should be the number of vertices
   * followed by that amount of indices to "points" in counter clockwise order
   */
  int* polygons;

  Vec3f* points;
  int num_points;
  int num_edges;
  int num_planes;

  struct Edge
  {
    int first, second;
  };

  Edge* edges;

  Vec3f center;

protected:
  void fillEdges();
};

/** Infinite plane */
class Plane : public ShapeBase
{
public:
  Plane(const Vec3f& n_, BVH_REAL d_) : ShapeBase(), n(n_), d(d_) { unitNormalTest(); }
  Plane(BVH_REAL a, BVH_REAL b, BVH_REAL c, BVH_REAL d_)
  {
    n = Vec3f(a, b, c);
    d = d_;
    unitNormalTest();
  }

  void computeAABB();

  NODE_TYPE getNodeType() const { return GEOM_PLANE; }

  Vec3f n;
  BVH_REAL d;

protected:

  void unitNormalTest();
};


}

#endif
