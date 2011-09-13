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


#ifndef GEOMETRIC_SHAPE_TO_BVH_MODEL_H
#define GEOMETRIC_SHAPE_TO_BVH_MODEL_H

#include "fcl/geometric_shapes.h"
#include "fcl/BVH_model.h"
#include <boost/math/constants/constants.hpp>

/** \brief Main namespace */
namespace fcl
{

/** \brief Generate BVH model from box */
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Box& shape)
{
  double a = shape.side[0];
  double b = shape.side[1];
  double c = shape.side[2];
  std::vector<Vec3f> points(8);
  std::vector<Triangle> tri_indices(12);
  points[0] = Vec3f(0.5 * a, -0.5 * b, 0.5 * c);
  points[1] = Vec3f(0.5 * a, 0.5 * b, 0.5 * c);
  points[2] = Vec3f(-0.5 * a, 0.5 * b, 0.5 * c);
  points[3] = Vec3f(-0.5 * a, -0.5 * b, 0.5 * c);
  points[4] = Vec3f(0.5 * a, -0.5 * b, -0.5 * c);
  points[5] = Vec3f(0.5 * a, 0.5 * b, -0.5 * c);
  points[6] = Vec3f(-0.5 * a, 0.5 * b, -0.5 * c);
  points[7] = Vec3f(-0.5 * a, -0.5 * b, -0.5 * c);

  tri_indices[0] = Triangle(0, 4, 1);
  tri_indices[1] = Triangle(1, 4, 5);
  tri_indices[2] = Triangle(2, 6, 3);
  tri_indices[3] = Triangle(3, 6, 7);
  tri_indices[4] = Triangle(3, 0, 2);
  tri_indices[5] = Triangle(2, 0, 1);
  tri_indices[6] = Triangle(6, 5, 7);
  tri_indices[7] = Triangle(7, 5, 4);
  tri_indices[8] = Triangle(1, 5, 2);
  tri_indices[9] = Triangle(2, 5, 6);
  tri_indices[10] = Triangle(3, 7, 0);
  tri_indices[11] = Triangle(0, 7, 4);

  for(unsigned int i = 0; i < points.size(); ++i)
  {
    Vec3f v = matMulVec(shape.getLocalRotation(), points[i]) + shape.getLocalTranslation();
    v = matMulVec(shape.getRotation(), v) + shape.getTranslation();
    points[i] = v;
  }

  model.beginModel();
  model.addSubModel(points, tri_indices);
  model.endModel();
  model.computeLocalAABB();
}

/** Generate BVH model from sphere */
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Sphere& shape, unsigned int seg = 16, unsigned int ring = 16)
{
  std::vector<Vec3f> points;
  std::vector<Triangle> tri_indices;

  double r = shape.radius;
  double phi, phid;
  const double pi = boost::math::constants::pi<double>();
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
    Vec3f v = matMulVec(shape.getLocalRotation(), points[i]) + shape.getLocalTranslation();
    v = matMulVec(shape.getRotation(), v) + shape.getTranslation();
    points[i] = v;
  }

  model.beginModel();
  model.addSubModel(points, tri_indices);
  model.endModel();
  model.computeLocalAABB();
}


/** \brief Generate BVH model from cylinder */
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Cylinder& shape, unsigned int tot = 16)
{
  std::vector<Vec3f> points;
  std::vector<Triangle> tri_indices;

  double r = shape.radius;
  double h = shape.lz;
  double phi, phid;
  const double pi = boost::math::constants::pi<double>();
  phid = pi * 2 / tot;
  phi = 0;

  double circle_edge = phid * r;
  unsigned int h_num = ceil(h / circle_edge);
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
    Vec3f v = matMulVec(shape.getLocalRotation(), points[i]) + shape.getLocalTranslation();
    v = matMulVec(shape.getRotation(), v) + shape.getTranslation();
    points[i] = v;
  }

  model.beginModel();
  model.addSubModel(points, tri_indices);
  model.endModel();
  model.computeLocalAABB();
}


/** \brief Generate BVH model from cone */
template<typename BV>
void generateBVHModel(BVHModel<BV>& model, const Cone& shape, unsigned int tot = 16)
{
  std::vector<Vec3f> points;
  std::vector<Triangle> tri_indices;

  double r = shape.radius;
  double h = shape.lz;

  double phi, phid;
  const double pi = boost::math::constants::pi<double>();
  phid = pi * 2 / tot;
  phi = 0;

  double circle_edge = phid * r;
  unsigned int h_num = ceil(h / circle_edge);
  double hd = h / h_num;

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
    Triangle tmp(h_num * tot, i, (i == tot - 1) ? 0 : (i + 1));
    tri_indices.push_back(tmp);
  }

  for(unsigned int i = 0; i < tot; ++i)
  {
    Triangle tmp(h_num * tot + 1, (h_num - 1) * tot + (i == tot - 1) ? 0 : (i + 1), (h_num - 1) * tot + i);
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
    Vec3f v = matMulVec(shape.getLocalRotation(), points[i]) + shape.getLocalTranslation();
    v = matMulVec(shape.getRotation(), v) + shape.getTranslation();
    points[i] = v;
  }

  model.beginModel();
  model.addSubModel(points, tri_indices);
  model.endModel();
  model.computeLocalAABB();
}

}

#endif
