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
  /** \brief Default Constructor */
  ShapeBase()
  {
    Rloc[0][0] = 1;
    Rloc[1][1] = 1;
    Rloc[2][2] = 1;
  }

  /** \brief Set the local frame of the shape */
  void setLocalTransform(const Vec3f R_[3], const Vec3f& T_)
  {
    Rloc[0] = R_[0];
    Rloc[1] = R_[1];
    Rloc[2] = R_[2];
    Tloc = T_;
  }

  /** \brief Set the local orientation of the shape */
  void setLocalRotation(const Vec3f R[3])
  {
    Rloc[0] = R[0];
    Rloc[1] = R[1];
    Rloc[2] = R[2];
  }

  /** \brief Set the local position of the shape */
  void setLocalTranslation(const Vec3f& T)
  {
    Tloc = T;
  }

  /** \brief Append additional transform to the local transform */
  void appendLocalTransform(const Vec3f R[3], const Vec3f& T)
  {
    Vec3f R0[3];
    for(int i = 0; i < 3; ++i)
      R0[i] = Rloc[i];
    matMulMat(R, R0, Rloc);
    Tloc = matMulVec(R, Tloc) + T;
  }

  /** \brief Get local transform */
  void getLocalTransform(Vec3f R[3], Vec3f& T) const
  {
    T = Tloc;
    R[0] = Rloc[0];
    R[1] = Rloc[1];
    R[2] = Rloc[2];
  }

  /** \brief Get local position */
  inline const Vec3f& getLocalTranslation() const
  {
    return Tloc;
  }

  /** \brief Get local orientation */
  inline const Vec3f* getLocalRotation() const
  {
    return Rloc;
  }

  /** \brief Get object type: a geometric shape */
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

  /** \brief Compute AABB */
  void computeLocalAABB();

  /** \brief Get node type: a box */
  NODE_TYPE getNodeType() const { return GEOM_BOX; }
};

/** Center at zero point sphere */
class Sphere : public ShapeBase
{
public:
  Sphere(BVH_REAL radius_) : ShapeBase(), radius(radius_) {}
  
  /** \brief Radius of the sphere */
  BVH_REAL radius;

  /** \brief Compute AABB */
  void computeLocalAABB();

  /** \brief Get node type: a sphere */
  NODE_TYPE getNodeType() const { return GEOM_SPHERE; }
};

/** Center at zero point capsule */
class Capsule : public ShapeBase
{
public:
  Capsule(BVH_REAL radius_, BVH_REAL lz_) : ShapeBase(), radius(radius_), lz(lz_) {}

  /** \brief Radius of capsule */
  BVH_REAL radius;

  /** \brief Length along z axis */
  BVH_REAL lz;

  /** \brief Compute AABB */
  void computeLocalAABB();

  /** \brief Get node type: a capsule */
  NODE_TYPE getNodeType() const { return GEOM_CAPSULE; }
};

/** Center at zero cone */
class Cone : public ShapeBase
{
public:
  Cone(BVH_REAL radius_, BVH_REAL lz_) : ShapeBase(), radius(radius_), lz(lz_) {}
  
  /** \brief Radius of the cone */
  BVH_REAL radius;

  /** \brief Length along z axis */
  BVH_REAL lz;

  /** \brief Compute AABB */
  void computeLocalAABB();

  /** \brief Get node type: a cone */
  NODE_TYPE getNodeType() const { return GEOM_CONE; }
};

/** Center at zero cylinder */
class Cylinder : public ShapeBase
{
public:
  Cylinder(BVH_REAL radius_, BVH_REAL lz_) : ShapeBase(), radius(radius_), lz(lz_) {}
  
  /** \brief Radius of the cylinder */
  BVH_REAL radius;

  /** \brief Length along z axis */
  BVH_REAL lz;

  /** \brief Compute AABB */
  void computeLocalAABB();

  /** \brief Get node type: a cylinder */
  NODE_TYPE getNodeType() const { return GEOM_CYLINDER; }
};

/** Convex polytope */
class Convex : public ShapeBase
{
public:
  /** Constructing a convex, providing normal and offset of each polytype surface, and the points and shape topology information */
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

  /** Copy constructor */
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

  /** Compute AABB */
  void computeLocalAABB();

  /** Get node type: a conex polytope */
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

  /** \brief center of the convex polytope, this is used for collision: center is guaranteed in the internal of the polytope (as it is convex) */
  Vec3f center;

protected:
  /** \brief Get edge information */
  void fillEdges();
};

/** Infinite plane */
class Plane : public ShapeBase
{
public:
  /** \brief Construct a plane with normal direction and offset */
  Plane(const Vec3f& n_, BVH_REAL d_) : ShapeBase(), n(n_), d(d_) { unitNormalTest(); }
  
  /** \brief Construct a plane with normal direction and offset */
  Plane(BVH_REAL a, BVH_REAL b, BVH_REAL c, BVH_REAL d_)
  {
    n = Vec3f(a, b, c);
    d = d_;
    unitNormalTest();
  }

  /** \brief Compute AABB */
  void computeLocalAABB();

  /** \brief Get node type: a plane */
  NODE_TYPE getNodeType() const { return GEOM_PLANE; }

  /** \brief Plane normal */
  Vec3f n;

  /** \brief Plane offset */
  BVH_REAL d;

protected:
  
  /** \brief Turn non-unit normal into unit */
  void unitNormalTest();
};


}

#endif
