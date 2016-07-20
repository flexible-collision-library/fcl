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


#include "fcl/shape/geometric_shapes.h"
#include "fcl/shape/geometric_shapes_utility.h"

namespace fcl
{

void Convex::fillEdges()
{
  int* points_in_poly = polygons;
  if(edges) delete [] edges;

  int num_edges_alloc = 0;
  for(int i = 0; i < num_planes; ++i)
  {
    num_edges_alloc += *points_in_poly;
    points_in_poly += (*points_in_poly + 1);
  }

  edges = new Edge[num_edges_alloc];

  points_in_poly = polygons;
  int* index = polygons + 1;
  num_edges = 0;
  Edge e;
  bool isinset;
  for(int i = 0; i < num_planes; ++i)
  {
    for(int j = 0; j < *points_in_poly; ++j)
    {
      e.first = std::min(index[j], index[(j+1)%*points_in_poly]);
      e.second = std::max(index[j], index[(j+1)%*points_in_poly]);
      isinset = false;
      for(int k = 0; k < num_edges; ++k)
      {
        if((edges[k].first == e.first) && (edges[k].second == e.second))
        {
          isinset = true;
          break;
        }
      }

      if(!isinset)
      {
        edges[num_edges].first = e.first;
        edges[num_edges].second = e.second;
        ++num_edges;
      }
    }

    points_in_poly += (*points_in_poly + 1);
    index = points_in_poly + 1;
  }

  if(num_edges < num_edges_alloc)
  {
    Edge* tmp = new Edge[num_edges];
    memcpy(tmp, edges, num_edges * sizeof(Edge));
    delete [] edges;
    edges = tmp;
  }
}

void Halfspace::unitNormalTest()
{
  FCL_REAL l = n.length();
  if(l > 0)
  {
    FCL_REAL inv_l = 1.0 / l;
    n *= inv_l;
    d *= inv_l;
  }
  else
  {
    n.setValue(1, 0, 0);
    d = 0;
  }  
}

void Plane::unitNormalTest()
{
  FCL_REAL l = n.length();
  if(l > 0)
  {
    FCL_REAL inv_l = 1.0 / l;
    n *= inv_l;
    d *= inv_l;
  }
  else
  {
    n.setValue(1, 0, 0);
    d = 0;
  }
}


void Box::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).length();
}

void Sphere::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = radius;
}

void Ellipsoid::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).length();
}

void Capsule::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).length();
}

void Cone::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).length();
}

void Cylinder::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).length();
}

void Convex::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).length();
}

void Halfspace::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).length();
}

void Plane::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).length();
}

void TriangleP::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).length();
}


}
