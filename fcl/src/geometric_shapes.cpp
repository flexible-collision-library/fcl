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


#include "fcl/geometric_shapes.h"
#include "fcl/geometric_shapes_utility.h"

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

void Plane::unitNormalTest()
{
  BVH_REAL l = n.length();
  if(l > 0)
  {
    BVH_REAL inv_l = 1.0 / l;
    n[0] *= inv_l;
    n[1] *= inv_l;
    n[2] *= inv_l;
    d *= inv_l;
  }
  else
  {
    n = Vec3f(1, 0, 0);
    d = 0;
  }
}

}
