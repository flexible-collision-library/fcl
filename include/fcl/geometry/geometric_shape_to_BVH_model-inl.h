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

#ifndef FCL_SHAPE_GEOMETRICSHAPETOBVHMODEL_INL_H
#define FCL_SHAPE_GEOMETRICSHAPETOBVHMODEL_INL_H

#include "fcl/geometry/geometric_shape_to_BVH_model.h"

namespace fcl
{

//==============================================================================
// Local helper function to ease conditional adding of triangles to a BVHModel
template<typename BV>
int addTriangles(BVHModel<BV>& model, const std::vector<Vector3<typename BV::S>>& points, const std::vector<Triangle>& tri_indices, FinalizeModel finalize_model)
{
  int retval = BVH_OK;
  if(model.build_state == BVH_BUILD_STATE_EMPTY){
    retval = model.beginModel();
  }

  if(retval == BVH_OK){
    retval = model.addSubModel(points, tri_indices);
  }

  if(retval == BVH_OK && finalize_model == FinalizeModel::DO){
    retval = model.endModel();
    model.computeLocalAABB();
  }
  return retval;
}

//==============================================================================
template<typename BV>
int generateBVHModel(BVHModel<BV>& model, const Box<typename BV::S>& shape, const Transform3<typename BV::S>& pose, FinalizeModel finalize_model)
{
  using S = typename BV::S;

  S a = shape.side[0];
  S b = shape.side[1];
  S c = shape.side[2];
  std::vector<Vector3<S>> points(8);
  std::vector<Triangle> tri_indices(12);
  points[0] << 0.5 * a, -0.5 * b, 0.5 * c;
  points[1] << 0.5 * a, 0.5 * b, 0.5 * c;
  points[2] << -0.5 * a, 0.5 * b, 0.5 * c;
  points[3] << -0.5 * a, -0.5 * b, 0.5 * c;
  points[4] << 0.5 * a, -0.5 * b, -0.5 * c;
  points[5] << 0.5 * a, 0.5 * b, -0.5 * c;
  points[6] << -0.5 * a, 0.5 * b, -0.5 * c;
  points[7] << -0.5 * a, -0.5 * b, -0.5 * c;

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
    points[i] = pose * points[i];
  }

  return addTriangles(model, points, tri_indices, finalize_model);
}


//==============================================================================
template<typename BV>
int generateBVHModel(BVHModel<BV>& model, const Sphere<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int seg, unsigned int ring, FinalizeModel finalize_model)
{
  using S = typename BV::S;

  std::vector<Vector3<S>> points;
  std::vector<Triangle> tri_indices;

  S r = shape.radius;
  S phi, phid;
  const S pi = constants<S>::pi();
  phid = pi * 2 / seg;
  phi = 0;

  S theta, thetad;
  thetad = pi / (ring + 1);
  theta = 0;

  for(unsigned int i = 0; i < ring; ++i)
  {
    S theta_ = theta + thetad * (i + 1);
    for(unsigned int j = 0; j < seg; ++j)
    {
      points.emplace_back(r * sin(theta_) * cos(phi + j * phid), r * sin(theta_) * sin(phi + j * phid), r * cos(theta_));
    }
  }
  points.emplace_back(0, 0, r);
  points.emplace_back(0, 0, -r);

  for(unsigned int i = 0; i < ring - 1; ++i)
  {
    for(unsigned int j = 0; j < seg; ++j)
    {
       unsigned int a, b, c, d;
       a = i * seg + j;
       b = (j == seg - 1) ? (i * seg) : (i * seg + j + 1);
       c = (i + 1) * seg + j;
       d = (j == seg - 1) ? ((i + 1) * seg) : ((i + 1) * seg + j + 1);
       tri_indices.emplace_back(a, c, b);
       tri_indices.emplace_back(b, c, d);
    }
  }

  for(unsigned int j = 0; j < seg; ++j)
  {
    unsigned int a, b;
    a = j;
    b = (j == seg - 1) ? 0 : (j + 1);
    tri_indices.emplace_back(ring * seg, a, b);

    a = (ring - 1) * seg + j;
    b = (j == seg - 1) ? (ring - 1) * seg : ((ring - 1) * seg + j + 1);
    tri_indices.emplace_back(a, ring * seg + 1, b);
  }

  for(unsigned int i = 0; i < points.size(); ++i)
  {
    points[i] = pose * points[i];
  }

  return addTriangles(model, points, tri_indices, finalize_model);
}

//==============================================================================
template<typename BV>
int generateBVHModel(BVHModel<BV>& model, const Sphere<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int n_faces_for_unit_sphere, FinalizeModel finalize_model)
{
  using S = typename BV::S;

  S r = shape.radius;
  S n_low_bound = sqrtf(n_faces_for_unit_sphere / 2.0) * r * r;
  unsigned int ring = ceil(n_low_bound);
  unsigned int seg = ceil(n_low_bound);

  return generateBVHModel(model, shape, pose, seg, ring, finalize_model);
}

//==============================================================================
template<typename BV>
int generateBVHModel(BVHModel<BV>& model, const Ellipsoid<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int seg, unsigned int ring, FinalizeModel finalize_model)
{
  using S = typename BV::S;

  std::vector<Vector3<S>> points;
  std::vector<Triangle> tri_indices;

  const S& a = shape.radii[0];
  const S& b = shape.radii[1];
  const S& c = shape.radii[2];

  S phi, phid;
  const S pi = constants<S>::pi();
  phid = pi * 2 / seg;
  phi = 0;

  S theta, thetad;
  thetad = pi / (ring + 1);
  theta = 0;

  for(unsigned int i = 0; i < ring; ++i)
  {
    S theta_ = theta + thetad * (i + 1);
    for(unsigned int j = 0; j < seg; ++j)
    {
      points.emplace_back(a * sin(theta_) * cos(phi + j * phid), b * sin(theta_) * sin(phi + j * phid), c * cos(theta_));
    }
  }
  points.emplace_back(0, 0, c);
  points.emplace_back(0, 0, -c);

  for(unsigned int i = 0; i < ring - 1; ++i)
  {
    for(unsigned int j = 0; j < seg; ++j)
    {
       unsigned int a, b, c, d;
       a = i * seg + j;
       b = (j == seg - 1) ? (i * seg) : (i * seg + j + 1);
       c = (i + 1) * seg + j;
       d = (j == seg - 1) ? ((i + 1) * seg) : ((i + 1) * seg + j + 1);
       tri_indices.emplace_back(a, c, b);
       tri_indices.emplace_back(b, c, d);
    }
  }

  for(unsigned int j = 0; j < seg; ++j)
  {
    unsigned int a, b;
    a = j;
    b = (j == seg - 1) ? 0 : (j + 1);
    tri_indices.emplace_back(ring * seg, a, b);

    a = (ring - 1) * seg + j;
    b = (j == seg - 1) ? (ring - 1) * seg : ((ring - 1) * seg + j + 1);
    tri_indices.emplace_back(a, ring * seg + 1, b);
  }

  for(unsigned int i = 0; i < points.size(); ++i)
  {
    points[i] = pose * points[i];
  }

  return addTriangles(model, points, tri_indices, finalize_model);
}

//==============================================================================
template<typename BV>
int generateBVHModel(BVHModel<BV>& model, const Ellipsoid<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int n_faces_for_unit_ellipsoid, FinalizeModel finalize_model)
{
  using S = typename BV::S;

  const S p = 1.6075;

  const S& ap = std::pow(shape.radii[0], p);
  const S& bp = std::pow(shape.radii[1], p);
  const S& cp = std::pow(shape.radii[2], p);

  const S ratio = std::pow((ap * bp + bp * cp + cp * ap) / 3.0, 1.0 / p);
  const S n_low_bound = std::sqrt(n_faces_for_unit_ellipsoid / 2.0) * ratio;

  const unsigned int ring = std::ceil(n_low_bound);
  const unsigned int seg = std::ceil(n_low_bound);

  return generateBVHModel(model, shape, pose, seg, ring, finalize_model);
}

//==============================================================================
template<typename BV>
int generateBVHModel(BVHModel<BV>& model, const Cylinder<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int circle_split_tot, unsigned int h_num, FinalizeModel finalize_model)
{
  using S = typename BV::S;

  std::vector<Vector3<S>> points;
  std::vector<Triangle> tri_indices;

  S r = shape.radius;
  S h = shape.lz;
  S phi, phid;
  const S pi = constants<S>::pi();
  phid = pi * 2 / circle_split_tot;
  phi = 0;

  S hd = h / h_num;

  for(unsigned int i = 0; i < circle_split_tot; ++i)
    points.emplace_back(r * cos(phi + phid * i), r * sin(phi + phid * i), h / 2);

  for(unsigned int i = 0; i < h_num - 1; ++i)
  {
    for(unsigned int j = 0; j < circle_split_tot; ++j)
    {
      points.emplace_back(r * cos(phi + phid * j), r * sin(phi + phid * j), h / 2 - (i + 1) * hd);
    }
  }

  for(unsigned int i = 0; i < circle_split_tot; ++i)
    points.emplace_back(r * cos(phi + phid * i), r * sin(phi + phid * i), - h / 2);

  points.emplace_back(0, 0, h / 2);
  points.emplace_back(0, 0, -h / 2);

  for(unsigned int i = 0; i < circle_split_tot; ++i)
    tri_indices.emplace_back((h_num + 1) * circle_split_tot, i, ((i == circle_split_tot - 1) ? 0 : (i + 1)));

  for(unsigned int i = 0; i < circle_split_tot; ++i)
    tri_indices.emplace_back((h_num + 1) * circle_split_tot + 1, h_num * circle_split_tot + ((i == circle_split_tot - 1) ? 0 : (i + 1)), h_num * circle_split_tot + i);

  for(unsigned int i = 0; i < h_num; ++i)
  {
    for(unsigned int j = 0; j < circle_split_tot; ++j)
    {
      int a, b, c, d;
      a = j;
      b = (j == circle_split_tot - 1) ? 0 : (j + 1);
      c = j + circle_split_tot;
      d = (j == circle_split_tot - 1) ? circle_split_tot : (j + 1 + circle_split_tot);

      int start = i * circle_split_tot;
      tri_indices.emplace_back(start + b, start + a, start + c);
      tri_indices.emplace_back(start + b, start + c, start + d);
    }
  }

  for(unsigned int i = 0; i < points.size(); ++i)
  {
    points[i] = pose * points[i];
  }

  return addTriangles(model, points, tri_indices, finalize_model);
}

//==============================================================================
template<typename BV>
int generateBVHModel(BVHModel<BV>& model, const Cylinder<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int circle_split_tot_for_unit_cylinder, FinalizeModel finalize_model)
{
  using S = typename BV::S;

  S r = shape.radius;
  S h = shape.lz;

  const S pi = constants<S>::pi();
  unsigned int circle_split_tot = circle_split_tot_for_unit_cylinder * r;
  S phid = pi * 2 / circle_split_tot;

  S circle_edge = phid * r;
  unsigned int h_num = ceil(h / circle_edge);

  return generateBVHModel(model, shape, pose, circle_split_tot, h_num, finalize_model);
}

//==============================================================================
template<typename BV>
int generateBVHModel(BVHModel<BV>& model, const Cone<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int circle_split_tot, unsigned int h_num, FinalizeModel finalize_model)
{
  using S = typename BV::S;

  std::vector<Vector3<S>> points;
  std::vector<Triangle> tri_indices;

  S r = shape.radius;
  S h = shape.lz;

  S phi, phid;
  const S pi = constants<S>::pi();
  phid = pi * 2 / circle_split_tot;
  phi = 0;

  S hd = h / h_num;

  for(unsigned int i = 0; i < h_num - 1; ++i)
  {
    S h_i = h / 2 - (i + 1) * hd;
    S rh = r * (0.5 - h_i / h);
    for(unsigned int j = 0; j < circle_split_tot; ++j)
    {
      points.emplace_back(rh * cos(phi + phid * j), rh * sin(phi + phid * j), h_i);
    }
  }

  for(unsigned int i = 0; i < circle_split_tot; ++i)
    points.emplace_back(r * cos(phi + phid * i), r * sin(phi + phid * i), - h / 2);

  points.emplace_back(0, 0, h / 2);
  points.emplace_back(0, 0, -h / 2);

  for(unsigned int i = 0; i < circle_split_tot; ++i)
    tri_indices.emplace_back(h_num * circle_split_tot, i, (i == circle_split_tot - 1) ? 0 : (i + 1));

  for(unsigned int i = 0; i < circle_split_tot; ++i)
    tri_indices.emplace_back(h_num * circle_split_tot + 1, (h_num - 1) * circle_split_tot + ((i == circle_split_tot - 1) ? 0 : (i + 1)), (h_num - 1) * circle_split_tot + i);

  for(unsigned int i = 0; i < h_num - 1; ++i)
  {
    for(unsigned int j = 0; j < circle_split_tot; ++j)
    {
      int a, b, c, d;
      a = j;
      b = (j == circle_split_tot - 1) ? 0 : (j + 1);
      c = j + circle_split_tot;
      d = (j == circle_split_tot - 1) ? circle_split_tot : (j + 1 + circle_split_tot);

      int start = i * circle_split_tot;
      tri_indices.emplace_back(start + b, start + a, start + c);
      tri_indices.emplace_back(start + b, start + c, start + d);
    }
  }

  for(unsigned int i = 0; i < points.size(); ++i)
  {
    points[i] = pose * points[i];
  }

  return addTriangles(model, points, tri_indices, finalize_model);
  
}

//==============================================================================
template<typename BV>
int generateBVHModel(BVHModel<BV>& model, const Cone<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int circle_split_tot_for_unit_cone, FinalizeModel finalize_model)
{
  using S = typename BV::S;


  S r = shape.radius;
  S h = shape.lz;

  const S pi = constants<S>::pi();
  unsigned int circle_split_tot = circle_split_tot_for_unit_cone * r;
  S phid = pi * 2 / circle_split_tot;

  S circle_edge = phid * r;
  unsigned int h_num = ceil(h / circle_edge);

  return generateBVHModel(model, shape, pose, circle_split_tot, h_num, finalize_model);
}

} // namespace fcl

#endif