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


#ifndef GEOMETRIC_SHAPE_TO_BVH_MODEL_H
#define GEOMETRIC_SHAPE_TO_BVH_MODEL_H

#include "fcl/shape/geometric_shapes.h"
#include "fcl/BVH/BVH_model.h"

namespace fcl
{

/// @brief Generate BVH model from box
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Box& shape, const Transform3f& pose)
{
  double a = shape.side[0];
  double b = shape.side[1];
  double c = shape.side[2];
  std::vector<Vec3f> points(8);
  std::vector<Triangle> tri_indices(12);
  points[0].setValue(0.5 * a, -0.5 * b, 0.5 * c);
  points[1].setValue(0.5 * a, 0.5 * b, 0.5 * c);
  points[2].setValue(-0.5 * a, 0.5 * b, 0.5 * c);
  points[3].setValue(-0.5 * a, -0.5 * b, 0.5 * c);
  points[4].setValue(0.5 * a, -0.5 * b, -0.5 * c);
  points[5].setValue(0.5 * a, 0.5 * b, -0.5 * c);
  points[6].setValue(-0.5 * a, 0.5 * b, -0.5 * c);
  points[7].setValue(-0.5 * a, -0.5 * b, -0.5 * c);

  tri_indices[0].set(0, 4, 1);
  tri_indices[1].set(1, 4, 5);
  tri_indices[2].set(2, 6, 3);
  tri_indices[3].set(3, 6, 7);
  tri_indices[4].set(3, 0, 2);
  tri_indices[5].set(2, 0, 1);
  tri_indices[6].set(6, 5, 7);
  tri_indices[7].set(7, 5, 4);
  tri_indices[8].set(1, 5, 2);
  tri_indices[9].set(2, 5, 6);
  tri_indices[10].set(3, 7, 0);
  tri_indices[11].set(0, 7, 4);

  for(unsigned int i = 0; i < points.size(); ++i)
  {
    points[i] = pose.transform(points[i]);
  }

  model.beginModel();
  model.addSubModel(points, tri_indices);
  model.endModel();
  model.computeLocalAABB();
}

/// @brief Generate BVH model from sphere, given the number of segments along longitude and number of rings along latitude.
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Sphere& shape, const Transform3f& pose, unsigned int seg, unsigned int ring)
{
  std::vector<Vec3f> points;
  std::vector<Triangle> tri_indices;

  double r = shape.radius;
  double phi, phid;
  const double pi = constants::pi;
  phid = pi * 2 / seg;
  phi = 0;

  double theta, thetad;
  thetad = pi / (ring + 1);
  theta = 0;

  for(unsigned int i = 0; i < ring; ++i)
  {
    double theta_ = theta + thetad * (i + 1);
    for(unsigned int j = 0; j < seg; ++j)
    {
      points.push_back(Vec3f(r * sin(theta_) * cos(phi + j * phid), r * sin(theta_) * sin(phi + j * phid), r * cos(theta_)));
    }
  }
  points.push_back(Vec3f(0, 0, r));
  points.push_back(Vec3f(0, 0, -r));

  for(unsigned int i = 0; i < ring - 1; ++i)
  {
    for(unsigned int j = 0; j < seg; ++j)
    {
       unsigned int a, b, c, d;
       a = i * seg + j;
       b = (j == seg - 1) ? (i * seg) : (i * seg + j + 1);
       c = (i + 1) * seg + j;
       d = (j == seg - 1) ? ((i + 1) * seg) : ((i + 1) * seg + j + 1);
       tri_indices.push_back(Triangle(a, c, b));
       tri_indices.push_back(Triangle(b, c, d));
    }
  }

  for(unsigned int j = 0; j < seg; ++j)
  {
    unsigned int a, b;
    a = j;
    b = (j == seg - 1) ? 0 : (j + 1);
    tri_indices.push_back(Triangle(ring * seg, a, b));

    a = (ring - 1) * seg + j;
    b = (j == seg - 1) ? (ring - 1) * seg : ((ring - 1) * seg + j + 1);
    tri_indices.push_back(Triangle(a, ring * seg + 1, b));
  }

  for(unsigned int i = 0; i < points.size(); ++i)
  {
    points[i] = pose.transform(points[i]);
  }

  model.beginModel();
  model.addSubModel(points, tri_indices);
  model.endModel();
  model.computeLocalAABB();
}

/// @brief Generate BVH model from sphere
/// The difference between generateBVHModel is that it gives the number of triangles faces N for a sphere with unit radius. For sphere of radius r,
/// then the number of triangles is r * r * N so that the area represented by a single triangle is approximately the same.s
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Sphere& shape, const Transform3f& pose, unsigned int n_faces_for_unit_sphere)
{
  double r = shape.radius;
  double n_low_bound = sqrtf(n_faces_for_unit_sphere / 2.0) * r * r;
  unsigned int ring = ceil(n_low_bound);
  unsigned int seg = ceil(n_low_bound);

  generateBVHModel(model, shape, pose, seg, ring);  
}

/// @brief Generate BVH model from ellipsoid, given the number of segments along longitude and number of rings along latitude.
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Ellipsoid& shape, const Transform3f& pose, unsigned int seg, unsigned int ring)
{
  std::vector<Vec3f> points;
  std::vector<Triangle> tri_indices;

  const FCL_REAL& a = shape.radii[0];
  const FCL_REAL& b = shape.radii[1];
  const FCL_REAL& c = shape.radii[2];

  FCL_REAL phi, phid;
  const FCL_REAL pi = constants::pi;
  phid = pi * 2 / seg;
  phi = 0;

  FCL_REAL theta, thetad;
  thetad = pi / (ring + 1);
  theta = 0;

  for(unsigned int i = 0; i < ring; ++i)
  {
    double theta_ = theta + thetad * (i + 1);
    for(unsigned int j = 0; j < seg; ++j)
    {
      points.push_back(Vec3f(a * sin(theta_) * cos(phi + j * phid), b * sin(theta_) * sin(phi + j * phid), c * cos(theta_)));
    }
  }
  points.push_back(Vec3f(0, 0, c));
  points.push_back(Vec3f(0, 0, -c));

  for(unsigned int i = 0; i < ring - 1; ++i)
  {
    for(unsigned int j = 0; j < seg; ++j)
    {
       unsigned int a, b, c, d;
       a = i * seg + j;
       b = (j == seg - 1) ? (i * seg) : (i * seg + j + 1);
       c = (i + 1) * seg + j;
       d = (j == seg - 1) ? ((i + 1) * seg) : ((i + 1) * seg + j + 1);
       tri_indices.push_back(Triangle(a, c, b));
       tri_indices.push_back(Triangle(b, c, d));
    }
  }

  for(unsigned int j = 0; j < seg; ++j)
  {
    unsigned int a, b;
    a = j;
    b = (j == seg - 1) ? 0 : (j + 1);
    tri_indices.push_back(Triangle(ring * seg, a, b));

    a = (ring - 1) * seg + j;
    b = (j == seg - 1) ? (ring - 1) * seg : ((ring - 1) * seg + j + 1);
    tri_indices.push_back(Triangle(a, ring * seg + 1, b));
  }

  for(unsigned int i = 0; i < points.size(); ++i)
  {
    points[i] = pose.transform(points[i]);
  }

  model.beginModel();
  model.addSubModel(points, tri_indices);
  model.endModel();
  model.computeLocalAABB();
}

/// @brief Generate BVH model from ellipsoid
/// The difference between generateBVHModel is that it gives the number of triangles faces N for an ellipsoid with unit radii (1, 1, 1). For ellipsoid of radii (a, b, c),
/// then the number of triangles is ((a^p * b^p + b^p * c^p + c^p * a^p)/3)^(1/p) * N, where p is 1.6075, so that the area represented by a single triangle is approximately the same.
/// Reference: https://en.wikipedia.org/wiki/Ellipsoid#Approximate_formula
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Ellipsoid& shape, const Transform3f& pose, unsigned int n_faces_for_unit_ellipsoid)
{
  const FCL_REAL p = 1.6075;

  const FCL_REAL& ap = std::pow(shape.radii[0], p);
  const FCL_REAL& bp = std::pow(shape.radii[1], p);
  const FCL_REAL& cp = std::pow(shape.radii[2], p);

  const FCL_REAL ratio = std::pow((ap * bp + bp * cp + cp * ap) / 3.0, 1.0 / p);
  const FCL_REAL n_low_bound = std::sqrt(n_faces_for_unit_ellipsoid / 2.0) * ratio;

  const unsigned int ring = std::ceil(n_low_bound);
  const unsigned int seg = std::ceil(n_low_bound);

  generateBVHModel(model, shape, pose, seg, ring);
}

/// @brief Generate BVH model from cylinder, given the number of segments along circle and the number of segments along axis.
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Cylinder& shape, const Transform3f& pose, unsigned int tot, unsigned int h_num)
{
  std::vector<Vec3f> points;
  std::vector<Triangle> tri_indices;

  double r = shape.radius;
  double h = shape.lz;
  double phi, phid;
  const double pi = constants::pi;
  phid = pi * 2 / tot;
  phi = 0;

  double hd = h / h_num;

  for(unsigned int i = 0; i < tot; ++i)
    points.push_back(Vec3f(r * cos(phi + phid * i), r * sin(phi + phid * i), h / 2));

  for(unsigned int i = 0; i < h_num - 1; ++i)
  {
    for(unsigned int j = 0; j < tot; ++j)
    {
      points.push_back(Vec3f(r * cos(phi + phid * j), r * sin(phi + phid * j), h / 2 - (i + 1) * hd));
    }
  }

  for(unsigned int i = 0; i < tot; ++i)
    points.push_back(Vec3f(r * cos(phi + phid * i), r * sin(phi + phid * i), - h / 2));

  points.push_back(Vec3f(0, 0, h / 2));
  points.push_back(Vec3f(0, 0, -h / 2));

  for(unsigned int i = 0; i < tot; ++i)
  {
    Triangle tmp((h_num + 1) * tot, i, ((i == tot - 1) ? 0 : (i + 1)));
    tri_indices.push_back(tmp);
  }

  for(unsigned int i = 0; i < tot; ++i)
  {
    Triangle tmp((h_num + 1) * tot + 1, h_num * tot + ((i == tot - 1) ? 0 : (i + 1)), h_num * tot + i);
    tri_indices.push_back(tmp);
  }

  for(unsigned int i = 0; i < h_num; ++i)
  {
    for(unsigned int j = 0; j < tot; ++j)
    {
      int a, b, c, d;
      a = j;
      b = (j == tot - 1) ? 0 : (j + 1);
      c = j + tot;
      d = (j == tot - 1) ? tot : (j + 1 + tot);

      int start = i * tot;
      tri_indices.push_back(Triangle(start + b, start + a, start + c));
      tri_indices.push_back(Triangle(start + b, start + c, start + d));
    }
  }

  for(unsigned int i = 0; i < points.size(); ++i)
  {
    points[i] = pose.transform(points[i]);
  }

  model.beginModel();
  model.addSubModel(points, tri_indices);
  model.endModel();
  model.computeLocalAABB();
}

/// @brief Generate BVH model from cylinder
/// Difference from generateBVHModel: is that it gives the circle split number tot for a cylinder with unit radius. For cylinder with
/// larger radius, the number of circle split number is r * tot.
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Cylinder& shape, const Transform3f& pose, unsigned int tot_for_unit_cylinder)
{
  double r = shape.radius;
  double h = shape.lz;

  const double pi = constants::pi;
  unsigned int tot = tot_for_unit_cylinder * r;
  double phid = pi * 2 / tot;

  double circle_edge = phid * r;
  unsigned int h_num = ceil(h / circle_edge);
  
  generateBVHModel(model, shape, pose, tot, h_num);
}


/// @brief Generate BVH model from cone, given the number of segments along circle and the number of segments along axis.
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Cone& shape, const Transform3f& pose, unsigned int tot, unsigned int h_num)
{
  std::vector<Vec3f> points;
  std::vector<Triangle> tri_indices;

  double r = shape.radius;
  double h = shape.lz;

  double phi, phid;
  const double pi = constants::pi;
  phid = pi * 2 / tot;
  phi = 0;

  double hd = h / h_num;

  for(unsigned int i = 0; i < h_num - 1; ++i)
  {
    double h_i = h / 2 - (i + 1) * hd;
    double rh = r * (0.5 - h_i / h);
    for(unsigned int j = 0; j < tot; ++j)
    {
      points.push_back(Vec3f(rh * cos(phi + phid * j), rh * sin(phi + phid * j), h_i));
    }
  }

  for(unsigned int i = 0; i < tot; ++i)
    points.push_back(Vec3f(r * cos(phi + phid * i), r * sin(phi + phid * i), - h / 2));

  points.push_back(Vec3f(0, 0, h / 2));
  points.push_back(Vec3f(0, 0, -h / 2));

  for(unsigned int i = 0; i < tot; ++i)
  {
    Triangle tmp(h_num * tot, i, (i == tot - 1) ? 0 : (i + 1));
    tri_indices.push_back(tmp);
  }

  for(unsigned int i = 0; i < tot; ++i)
  {
    Triangle tmp(h_num * tot + 1, (h_num - 1) * tot + ((i == tot - 1) ? 0 : (i + 1)), (h_num - 1) * tot + i);
    tri_indices.push_back(tmp);
  }

  for(unsigned int i = 0; i < h_num - 1; ++i)
  {
    for(unsigned int j = 0; j < tot; ++j)
    {
      int a, b, c, d;
      a = j;
      b = (j == tot - 1) ? 0 : (j + 1);
      c = j + tot;
      d = (j == tot - 1) ? tot : (j + 1 + tot);

      int start = i * tot;
      tri_indices.push_back(Triangle(start + b, start + a, start + c));
      tri_indices.push_back(Triangle(start + b, start + c, start + d));
    }
  }

  for(unsigned int i = 0; i < points.size(); ++i)
  {
    points[i] = pose.transform(points[i]);
  }

  model.beginModel();
  model.addSubModel(points, tri_indices);
  model.endModel();
  model.computeLocalAABB();
}

/// @brief Generate BVH model from cone
/// Difference from generateBVHModel: is that it gives the circle split number tot for a cylinder with unit radius. For cone with
/// larger radius, the number of circle split number is r * tot.
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Cone& shape, const Transform3f& pose, unsigned int tot_for_unit_cone)
{
  double r = shape.radius;
  double h = shape.lz;

  const double pi = constants::pi;
  unsigned int tot = tot_for_unit_cone * r;
  double phid = pi * 2 / tot;

  double circle_edge = phid * r;
  unsigned int h_num = ceil(h / circle_edge);

  generateBVHModel(model, shape, pose, tot, h_num);
}

}

#endif
