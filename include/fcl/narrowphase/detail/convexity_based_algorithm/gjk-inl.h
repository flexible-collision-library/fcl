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

#ifndef FCL_NARROWPHASE_DETAIL_GJK_INL_H
#define FCL_NARROWPHASE_DETAIL_GJK_INL_H

#include "fcl/narrowphase/detail/convexity_based_algorithm/gjk.h"

namespace fcl
{

namespace detail
{

//==============================================================================
extern template
struct GJK<double>;

//==============================================================================
template <typename S>
GJK<S>::GJK(unsigned int max_iterations_, S tolerance_)
  : max_iterations(max_iterations_), tolerance(tolerance_)
{
  initialize();
}

//==============================================================================
template <typename S>
void GJK<S>::initialize()
{
  ray.setZero();
  nfree = 0;
  status = Failed;
  current = 0;
  distance = 0.0;
  simplex = nullptr;
}

//==============================================================================
template <typename S>
Vector3<S> GJK<S>::getGuessFromSimplex() const
{
  return ray;
}

//==============================================================================
template <typename S>
typename GJK<S>::Status GJK<S>::evaluate(const MinkowskiDiff<S>& shape_, const Vector3<S>& guess)
{
  size_t iterations = 0;
  S alpha = 0;
  Vector3<S> lastw[4];
  size_t clastw = 0;

  free_v[0] = &store_v[0];
  free_v[1] = &store_v[1];
  free_v[2] = &store_v[2];
  free_v[3] = &store_v[3];

  nfree = 4;
  current = 0;
  status = Valid;
  shape = shape_;
  distance = 0.0;
  simplices[0].rank = 0;
  ray = guess;

  appendVertex(simplices[0], (ray.squaredNorm() > 0) ? (-ray).eval() : Vector3<S>::UnitX());
  simplices[0].p[0] = 1;
  ray = simplices[0].c[0]->w;
  lastw[0] = lastw[1] = lastw[2] = lastw[3] = ray; // cache previous support points, the new support point will compare with it to avoid too close support points

  do
  {
    size_t next = 1 - current;
    Simplex& curr_simplex = simplices[current];
    Simplex& next_simplex = simplices[next];

    // check A: when origin is near the existing simplex, stop
    S rl = ray.norm();
    if(rl < tolerance) // mean origin is near the face of original simplex, return touch
    {
      status = Inside;
      break;
    }

    appendVertex(curr_simplex, -ray); // see below, ray points away from origin

    // check B: when the new support point is close to previous support points, stop (as the new simplex is degenerated)
    Vector3<S>& w = curr_simplex.c[curr_simplex.rank - 1]->w;
    bool found = false;
    for(size_t i = 0; i < 4; ++i)
    {
      if((w - lastw[i]).squaredNorm() < tolerance)
      {
        found = true; break;
      }
    }

    if(found)
    {
      removeVertex(simplices[current]);
      break;
    }
    else
    {
      lastw[clastw = (clastw+1)&3] = w;
    }

    // check C: when the new support point is close to the sub-simplex where the ray point lies, stop (as the new simplex again is degenerated)
    S omega = ray.dot(w) / rl;
    alpha = std::max(alpha, omega);
    if((rl - alpha) - tolerance * rl <= 0)
    {
      removeVertex(simplices[current]);
      break;
    }

    typename Project<S>::ProjectResult project_res;
    switch(curr_simplex.rank)
    {
    case 2:
      project_res = Project<S>::projectLineOrigin(curr_simplex.c[0]->w, curr_simplex.c[1]->w); break;
    case 3:
      project_res = Project<S>::projectTriangleOrigin(curr_simplex.c[0]->w, curr_simplex.c[1]->w, curr_simplex.c[2]->w); break;
    case 4:
      project_res = Project<S>::projectTetrahedraOrigin(curr_simplex.c[0]->w, curr_simplex.c[1]->w, curr_simplex.c[2]->w, curr_simplex.c[3]->w); break;
    }

    if(project_res.sqr_distance >= 0)
    {
      next_simplex.rank = 0;
      ray.setZero();
      current = next;
      for(size_t i = 0; i < curr_simplex.rank; ++i)
      {
        if(project_res.encode & (1 << i))
        {
          next_simplex.c[next_simplex.rank] = curr_simplex.c[i];
          next_simplex.p[next_simplex.rank++] = project_res.parameterization[i]; // weights[i];
          ray += curr_simplex.c[i]->w * project_res.parameterization[i]; // weights[i];
        }
        else
          free_v[nfree++] = curr_simplex.c[i];
      }
      if(project_res.encode == 15) status = Inside; // the origin is within the 4-simplex, collision
    }
    else
    {
      removeVertex(simplices[current]);
      break;
    }

    status = ((++iterations) < max_iterations) ? status : Failed;

  } while(status == Valid);

  simplex = &simplices[current];
  switch(status)
  {
  case Valid: distance = ray.norm(); break;
  case Inside: distance = 0; break;
  default: break;
  }
  return status;
}

//==============================================================================
template <typename S>
void GJK<S>::getSupport(const Vector3<S>& d, SimplexV& sv) const
{
  sv.d = d.normalized();
  sv.w = shape.support(sv.d);
}

//==============================================================================
template <typename S>
void GJK<S>::getSupport(const Vector3<S>& d, const Vector3<S>& v, SimplexV& sv) const
{
  sv.d = d.normalized();
  sv.w = shape.support(sv.d, v);
}

//==============================================================================
template <typename S>
void GJK<S>::removeVertex(Simplex& simplex)
{
  free_v[nfree++] = simplex.c[--simplex.rank];
}

//==============================================================================
template <typename S>
void GJK<S>::appendVertex(Simplex& simplex, const Vector3<S>& v)
{
  simplex.p[simplex.rank] = 0; // initial weight 0
  simplex.c[simplex.rank] = free_v[--nfree]; // set the memory
  getSupport(v, *simplex.c[simplex.rank++]);
}

//==============================================================================
template <typename S>
bool GJK<S>::encloseOrigin()
{
  switch(simplex->rank)
  {
  case 1:
    {
      for(size_t i = 0; i < 3; ++i)
      {
        Vector3<S> axis = Vector3<S>::Zero();
        axis[i] = 1;
        appendVertex(*simplex, axis);
        if(encloseOrigin()) return true;
        removeVertex(*simplex);
        appendVertex(*simplex, -axis);
        if(encloseOrigin()) return true;
        removeVertex(*simplex);
      }
    }
    break;
  case 2:
    {
      Vector3<S> d = simplex->c[1]->w - simplex->c[0]->w;
      for(size_t i = 0; i < 3; ++i)
      {
        Vector3<S> axis = Vector3<S>::Zero();
        axis[i] = 1;
        Vector3<S> p = d.cross(axis);
        if(p.squaredNorm() > 0)
        {
          appendVertex(*simplex, p);
          if(encloseOrigin()) return true;
          removeVertex(*simplex);
          appendVertex(*simplex, -p);
          if(encloseOrigin()) return true;
          removeVertex(*simplex);
        }
      }
    }
    break;
  case 3:
    {
      Vector3<S> n = (simplex->c[1]->w - simplex->c[0]->w).cross(simplex->c[2]->w - simplex->c[0]->w);
      if(n.squaredNorm() > 0)
      {
        appendVertex(*simplex, n);
        if(encloseOrigin()) return true;
        removeVertex(*simplex);
        appendVertex(*simplex, -n);
        if(encloseOrigin()) return true;
        removeVertex(*simplex);
      }
    }
    break;
  case 4:
    {
      if(std::abs(triple(simplex->c[0]->w - simplex->c[3]->w, simplex->c[1]->w - simplex->c[3]->w, simplex->c[2]->w - simplex->c[3]->w)) > 0)
        return true;
    }
    break;
  }

  return false;
}

//==============================================================================
template <typename S>
typename GJK<S>::Simplex* GJK<S>::getSimplex() const
{
  return simplex;
}

//==============================================================================
template <typename S>
GJK<S>::Simplex::Simplex()
  : rank(0)
{
  // Do nothing
}

} // namespace detail
} // namespace fcl

#endif
