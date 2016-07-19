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


#ifndef FCL_GEOMETRIC_SHAPES_H
#define FCL_GEOMETRIC_SHAPES_H

#include "fcl/collision_object.h"
#include "fcl/math/vec_3f.h"
#include <string.h>

namespace fcl
{

/// @brief Base class for all basic geometric shapes
class ShapeBase : public CollisionGeometry
{
public:
  ShapeBase() {}

  /// @brief Get object type: a geometric shape
  OBJECT_TYPE getObjectType() const { return OT_GEOM; }
};

/// @brief Triangle stores the points instead of only indices of points
class TriangleP : public ShapeBase
{
public:
  TriangleP(const Vec3f& a_, const Vec3f& b_, const Vec3f& c_) : ShapeBase(), a(a_), b(b_), c(c_)
  {
  }

  /// @brief virtual function of compute AABB in local coordinate
  void computeLocalAABB();
  
  NODE_TYPE getNodeType() const { return GEOM_TRIANGLE; }

  Vec3f a, b, c;
};

/// @brief Center at zero point, axis aligned box
class Box : public ShapeBase
{
public:
  Box(FCL_REAL x, FCL_REAL y, FCL_REAL z) : ShapeBase(), side(x, y, z)
  {
  }

  Box(const Vec3f& side_) : ShapeBase(), side(side_) 
  {
  }

  Box() {}

  /// @brief box side length
  Vec3f side;

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a box
  NODE_TYPE getNodeType() const { return GEOM_BOX; }

  FCL_REAL computeVolume() const
  {
    return side[0] * side[1] * side[2];
  }

  Matrix3f computeMomentofInertia() const
  {
    FCL_REAL V = computeVolume();
    FCL_REAL a2 = side[0] * side[0] * V;
    FCL_REAL b2 = side[1] * side[1] * V;
    FCL_REAL c2 = side[2] * side[2] * V;
    return Matrix3f((b2 + c2) / 12, 0, 0,
                    0, (a2 + c2) / 12, 0,
                    0, 0, (a2 + b2) / 12);
  }
};

/// @brief Center at zero point sphere
class Sphere : public ShapeBase
{
public:
  Sphere(FCL_REAL radius_) : ShapeBase(), radius(radius_)
  {
  }

  /// @brief Radius of the sphere
  FCL_REAL radius;

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a sphere
  NODE_TYPE getNodeType() const { return GEOM_SPHERE; }

  Matrix3f computeMomentofInertia() const
  {
    FCL_REAL I = 0.4 * radius * radius * computeVolume();
    return Matrix3f(I, 0, 0,
                    0, I, 0,
                    0, 0, I);
  }

  FCL_REAL computeVolume() const
  {
    return 4.0 * constants::pi * radius * radius * radius / 3.0;
  }
};

/// @brief Center at zero point ellipsoid
class Ellipsoid : public ShapeBase
{
public:
  Ellipsoid(FCL_REAL a, FCL_REAL b, FCL_REAL c) : ShapeBase(), radii(a, b, c)
  {
  }

  Ellipsoid(const Vec3f& radii_) : ShapeBase(), radii(radii_)
  {
  }

  /// @brief Radii of the ellipsoid
  Vec3f radii;

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a sphere
  NODE_TYPE getNodeType() const { return GEOM_ELLIPSOID; }

  Matrix3f computeMomentofInertia() const
  {
    const FCL_REAL V = computeVolume();
    const FCL_REAL a2 = radii[0] * radii[0] * V;
    const FCL_REAL b2 = radii[1] * radii[1] * V;
    const FCL_REAL c2 = radii[2] * radii[2] * V;
    return Matrix3f(0.2 * (b2 + c2), 0, 0,
                    0, 0.2 * (a2 + c2), 0,
                    0, 0, 0.2 * (a2 + b2));
  }

  FCL_REAL computeVolume() const
  {
    const FCL_REAL pi = constants::pi;
    return 4.0 * pi * radii[0] * radii[1] * radii[2] / 3.0;
  }
};

/// @brief Center at zero point capsule 
class Capsule : public ShapeBase
{
public:
  Capsule(FCL_REAL radius_, FCL_REAL lz_) : ShapeBase(), radius(radius_), lz(lz_)
  {
  }

  /// @brief Radius of capsule 
  FCL_REAL radius;

  /// @brief Length along z axis 
  FCL_REAL lz;

  /// @brief Compute AABB 
  void computeLocalAABB();

  /// @brief Get node type: a capsule 
  NODE_TYPE getNodeType() const { return GEOM_CAPSULE; }

  FCL_REAL computeVolume() const
  {
    return constants::pi * radius * radius *(lz + radius * 4/3.0);
  }

  Matrix3f computeMomentofInertia() const
  {
    FCL_REAL v_cyl = radius * radius * lz * constants::pi;
    FCL_REAL v_sph = radius * radius * radius * constants::pi * 4 / 3.0;
    
    FCL_REAL ix = v_cyl * lz * lz / 12.0 + 0.25 * v_cyl * radius + 0.4 * v_sph * radius * radius + 0.25 * v_sph * lz * lz;
    FCL_REAL iz = (0.5 * v_cyl + 0.4 * v_sph) * radius * radius;

    return Matrix3f(ix, 0, 0,
                    0, ix, 0,
                    0, 0, iz);
  }
  
};

/// @brief Center at zero cone 
class Cone : public ShapeBase
{
public:
  Cone(FCL_REAL radius_, FCL_REAL lz_) : ShapeBase(), radius(radius_), lz(lz_)
  {
  }

  /// @brief Radius of the cone 
  FCL_REAL radius;

  /// @brief Length along z axis 
  FCL_REAL lz;

  /// @brief Compute AABB 
  void computeLocalAABB();

  /// @brief Get node type: a cone 
  NODE_TYPE getNodeType() const { return GEOM_CONE; }

  FCL_REAL computeVolume() const
  {
    return constants::pi * radius * radius * lz / 3;
  }

  Matrix3f computeMomentofInertia() const
  {
    FCL_REAL V = computeVolume();
    FCL_REAL ix = V * (0.1 * lz * lz + 3 * radius * radius / 20);
    FCL_REAL iz = 0.3 * V * radius * radius;

    return Matrix3f(ix, 0, 0,
                    0, ix, 0,
                    0, 0, iz);
  }

  Vec3f computeCOM() const
  {
    return Vec3f(0, 0, -0.25 * lz);
  }
};

/// @brief Center at zero cylinder 
class Cylinder : public ShapeBase
{
public:
  Cylinder(FCL_REAL radius_, FCL_REAL lz_) : ShapeBase(), radius(radius_), lz(lz_)
  {
  }

  
  /// @brief Radius of the cylinder 
  FCL_REAL radius;

  /// @brief Length along z axis 
  FCL_REAL lz;

  /// @brief Compute AABB 
  void computeLocalAABB();

  /// @brief Get node type: a cylinder 
  NODE_TYPE getNodeType() const { return GEOM_CYLINDER; }

  FCL_REAL computeVolume() const
  {
    return constants::pi * radius * radius * lz;
  }

  Matrix3f computeMomentofInertia() const
  {
    FCL_REAL V = computeVolume();
    FCL_REAL ix = V * (3 * radius * radius + lz * lz) / 12;
    FCL_REAL iz = V * radius * radius / 2;
    return Matrix3f(ix, 0, 0,
                    0, ix, 0,
                    0, 0, iz);
  }
};

/// @brief Convex polytope 
class Convex : public ShapeBase
{
public:
  /// @brief Constructing a convex, providing normal and offset of each polytype surface, and the points and shape topology information 
  Convex(Vec3f* plane_normals_,
         FCL_REAL* plane_dis_,
         int num_planes_,
         Vec3f* points_,
         int num_points_,
         int* polygons_) : ShapeBase()
  {
    plane_normals = plane_normals_;
    plane_dis = plane_dis_;
    num_planes = num_planes_;
    points = points_;
    num_points = num_points_;
    polygons = polygons_;
    edges = NULL;

    Vec3f sum;
    for(int i = 0; i < num_points; ++i)
    {
      sum += points[i];
    }

    center = sum * (FCL_REAL)(1.0 / num_points);

    fillEdges();
  }

  /// @brief Copy constructor 
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

  /// @brief Compute AABB 
  void computeLocalAABB();

  /// @brief Get node type: a conex polytope 
  NODE_TYPE getNodeType() const { return GEOM_CONVEX; }

  
  Vec3f* plane_normals;
  FCL_REAL* plane_dis;

  /// @brief An array of indices to the points of each polygon, it should be the number of vertices
  /// followed by that amount of indices to "points" in counter clockwise order
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

  /// @brief center of the convex polytope, this is used for collision: center is guaranteed in the internal of the polytope (as it is convex) 
  Vec3f center;

  /// based on http://number-none.com/blow/inertia/bb_inertia.doc
  Matrix3f computeMomentofInertia() const
  {
    
    Matrix3f C(0, 0, 0,
               0, 0, 0,
               0, 0, 0);

    Matrix3f C_canonical(1/60.0, 1/120.0, 1/120.0,
                         1/120.0, 1/60.0, 1/120.0,
                         1/120.0, 1/120.0, 1/60.0);

    int* points_in_poly = polygons;
    int* index = polygons + 1;
    for(int i = 0; i < num_planes; ++i)
    {
      Vec3f plane_center;

      // compute the center of the polygon
      for(int j = 0; j < *points_in_poly; ++j)
        plane_center += points[index[j]];
      plane_center = plane_center * (1.0 / *points_in_poly);

      // compute the volume of tetrahedron making by neighboring two points, the plane center and the reference point (zero) of the convex shape
      const Vec3f& v3 = plane_center;
      for(int j = 0; j < *points_in_poly; ++j)
      {
        int e_first = index[j];
        int e_second = index[(j+1)%*points_in_poly];
        const Vec3f& v1 = points[e_first];
        const Vec3f& v2 = points[e_second];
        FCL_REAL d_six_vol = (v1.cross(v2)).dot(v3);
        Matrix3f A(v1, v2, v3); // this is A' in the original document
        C += transpose(A) * C_canonical * A * d_six_vol; // change accordingly
      }
      
      points_in_poly += (*points_in_poly + 1);
      index = points_in_poly + 1;
    }

    FCL_REAL trace_C = C(0, 0) + C(1, 1) + C(2, 2);

    return Matrix3f(trace_C - C(0, 0), -C(0, 1), -C(0, 2),
                    -C(1, 0), trace_C - C(1, 1), -C(1, 2),
                    -C(2, 0), -C(2, 1), trace_C - C(2, 2));
    
  }

  Vec3f computeCOM() const
  {
    Vec3f com;
    FCL_REAL vol = 0;
    int* points_in_poly = polygons;
    int* index = polygons + 1;
    for(int i = 0; i < num_planes; ++i)
    {
      Vec3f plane_center;

      // compute the center of the polygon
      for(int j = 0; j < *points_in_poly; ++j)
        plane_center += points[index[j]];
      plane_center = plane_center * (1.0 / *points_in_poly);

      // compute the volume of tetrahedron making by neighboring two points, the plane center and the reference point (zero) of the convex shape
      const Vec3f& v3 = plane_center;
      for(int j = 0; j < *points_in_poly; ++j)
      {
        int e_first = index[j];
        int e_second = index[(j+1)%*points_in_poly];
        const Vec3f& v1 = points[e_first];
        const Vec3f& v2 = points[e_second];
        FCL_REAL d_six_vol = (v1.cross(v2)).dot(v3);
        vol += d_six_vol;
        com += (points[e_first] + points[e_second] + plane_center) * d_six_vol;
      }
      
      points_in_poly += (*points_in_poly + 1);
      index = points_in_poly + 1;
    }

    return com / (vol * 4); // here we choose zero as the reference
  }

  FCL_REAL computeVolume() const
  {
    FCL_REAL vol = 0;
    int* points_in_poly = polygons;
    int* index = polygons + 1;
    for(int i = 0; i < num_planes; ++i)
    {
      Vec3f plane_center;

      // compute the center of the polygon
      for(int j = 0; j < *points_in_poly; ++j)
        plane_center += points[index[j]];
      plane_center = plane_center * (1.0 / *points_in_poly);

      // compute the volume of tetrahedron making by neighboring two points, the plane center and the reference point (zero point) of the convex shape
      const Vec3f& v3 = plane_center;
      for(int j = 0; j < *points_in_poly; ++j)
      {
        int e_first = index[j];
        int e_second = index[(j+1)%*points_in_poly];
        const Vec3f& v1 = points[e_first];
        const Vec3f& v2 = points[e_second];
        FCL_REAL d_six_vol = (v1.cross(v2)).dot(v3);
        vol += d_six_vol;
      }
      
      points_in_poly += (*points_in_poly + 1);
      index = points_in_poly + 1;
    }

    return vol / 6;
  }

  

protected:
  /// @brief Get edge information 
  void fillEdges();
};


/// @brief Half Space: this is equivalent to the Plane in ODE. The separation plane is defined as n * x = d;
/// Points in the negative side of the separation plane (i.e. {x | n * x < d}) are inside the half space and points
/// in the positive side of the separation plane (i.e. {x | n * x > d}) are outside the half space
class Halfspace : public ShapeBase
{
public:
  /// @brief Construct a half space with normal direction and offset
  Halfspace(const Vec3f& n_, FCL_REAL d_) : ShapeBase(), n(n_), d(d_)
  {
    unitNormalTest();
  }

  /// @brief Construct a plane with normal direction and offset
  Halfspace(FCL_REAL a, FCL_REAL b, FCL_REAL c, FCL_REAL d_) : ShapeBase(), n(a, b, c), d(d_)
  {
    unitNormalTest();
  }

  Halfspace() : ShapeBase(), n(1, 0, 0), d(0)
  {
  }

  FCL_REAL signedDistance(const Vec3f& p) const
  {
    return n.dot(p) - d;
  }

  FCL_REAL distance(const Vec3f& p) const
  {
    return std::abs(n.dot(p) - d);
  }

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a half space
  NODE_TYPE getNodeType() const { return GEOM_HALFSPACE; }
  
  /// @brief Plane normal
  Vec3f n;
  
  /// @brief Plane offset
  FCL_REAL d;

protected:

  /// @brief Turn non-unit normal into unit
  void unitNormalTest();
};

/// @brief Infinite plane 
class Plane : public ShapeBase
{
public:
  /// @brief Construct a plane with normal direction and offset 
  Plane(const Vec3f& n_, FCL_REAL d_) : ShapeBase(), n(n_), d(d_) 
  { 
    unitNormalTest(); 
  }
  
  /// @brief Construct a plane with normal direction and offset 
  Plane(FCL_REAL a, FCL_REAL b, FCL_REAL c, FCL_REAL d_) : ShapeBase(), n(a, b, c), d(d_)
  {
    unitNormalTest();
  }

  Plane() : ShapeBase(), n(1, 0, 0), d(0)
  {}

  FCL_REAL signedDistance(const Vec3f& p) const
  {
    return n.dot(p) - d;
  }

  FCL_REAL distance(const Vec3f& p) const
  {
    return std::abs(n.dot(p) - d);
  }

  /// @brief Compute AABB 
  void computeLocalAABB();

  /// @brief Get node type: a plane 
  NODE_TYPE getNodeType() const { return GEOM_PLANE; }

  /// @brief Plane normal 
  Vec3f n;

  /// @brief Plane offset 
  FCL_REAL d;

protected:
  
  /// @brief Turn non-unit normal into unit 
  void unitNormalTest();
};


}

#endif
