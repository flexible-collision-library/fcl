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

#ifndef FCL_NARROWPHASE_DETAIL_EPA_INL_H
#define FCL_NARROWPHASE_DETAIL_EPA_INL_H

#include "fcl/narrowphase/detail/convexity_based_algorithm/epa.h"

namespace fcl
{

namespace detail
{

//==============================================================================
extern template
struct EPA<double>;

//==============================================================================
template <typename S>
EPA<S>::SimplexList::SimplexList()
  : root(nullptr), count(0)
{
  // Do nothing
}

//==============================================================================
template <typename S>
void EPA<S>::SimplexList::append(typename EPA<S>::SimplexF* face)
{
  face->l[0] = nullptr;
  face->l[1] = root;
  if(root) root->l[0] = face;
  root = face;
  ++count;
}

//==============================================================================
template <typename S>
void EPA<S>::SimplexList::remove(typename EPA<S>::SimplexF* face)
{
  if(face->l[1]) face->l[1]->l[0] = face->l[0];
  if(face->l[0]) face->l[0]->l[1] = face->l[1];
  if(face == root) root = face->l[1];
  --count;
}

//==============================================================================
template <typename S>
void EPA<S>::bind(SimplexF* fa, size_t ea, SimplexF* fb, size_t eb)
{
  fa->e[ea] = eb; fa->f[ea] = fb;
  fb->e[eb] = ea; fb->f[eb] = fa;
}

//==============================================================================
template <typename S>
EPA<S>::EPA(
    unsigned int max_face_num_,
    unsigned int max_vertex_num_,
    unsigned int max_iterations_, S tolerance_)
  : max_face_num(max_face_num_),
    max_vertex_num(max_vertex_num_),
    max_iterations(max_iterations_),
    tolerance(tolerance_)
{
  initialize();
}

//==============================================================================
template <typename S>
EPA<S>::~EPA()
{
  delete [] sv_store;
  delete [] fc_store;
}

//==============================================================================
template <typename S>
void EPA<S>::initialize()
{
  sv_store = new SimplexV[max_vertex_num];
  fc_store = new SimplexF[max_face_num];
  status = Failed;
  normal = Vector3<S>(0, 0, 0);
  depth = 0;
  nextsv = 0;
  for(size_t i = 0; i < max_face_num; ++i)
    stock.append(&fc_store[max_face_num-i-1]);
}

//==============================================================================
template <typename S>
bool EPA<S>::getEdgeDist(SimplexF* face, SimplexV* a, SimplexV* b, S& dist)
{
  Vector3<S> ba = b->w - a->w;
  Vector3<S> n_ab = ba.cross(face->n);
  S a_dot_nab = a->w.dot(n_ab);

  if(a_dot_nab < 0) // the origin is on the outside part of ab
  {
    // following is similar to projectOrigin for two points
    // however, as we dont need to compute the parameterization, dont need to compute 0 or 1
    S a_dot_ba = a->w.dot(ba);
    S b_dot_ba = b->w.dot(ba);

    if(a_dot_ba > 0)
      dist = a->w.norm();
    else if(b_dot_ba < 0)
      dist = b->w.norm();
    else
    {
      S a_dot_b = a->w.dot(b->w);
      dist = std::sqrt(std::max(a->w.squaredNorm() * b->w.squaredNorm() - a_dot_b * a_dot_b, (S)0));
    }

    return true;
  }

  return false;
}

//==============================================================================
template <typename S>
typename EPA<S>::SimplexF* EPA<S>::newFace(
      typename GJK<S>::SimplexV* a,
      typename GJK<S>::SimplexV* b,
      typename GJK<S>::SimplexV* c,
      bool forced)
{
  if(stock.root)
  {
    SimplexF* face = stock.root;
    stock.remove(face);
    hull.append(face);
    face->pass = 0;
    face->c[0] = a;
    face->c[1] = b;
    face->c[2] = c;
    face->n.noalias() = (b->w - a->w).cross(c->w - a->w);
    S l = face->n.norm();

    if(l > tolerance)
    {
      if(!(getEdgeDist(face, a, b, face->d) ||
           getEdgeDist(face, b, c, face->d) ||
           getEdgeDist(face, c, a, face->d)))
      {
        face->d = a->w.dot(face->n) / l;
      }

      face->n /= l;
      if(forced || face->d >= -tolerance)
        return face;
      else
        status = NonConvex;
    }
    else
      status = Degenerated;

    hull.remove(face);
    stock.append(face);
    return nullptr;
  }

  status = stock.root ? OutOfVertices : OutOfFaces;
  return nullptr;
}

//==============================================================================
/** @brief Find the best polytope face to split */
template <typename S>
typename EPA<S>::SimplexF* EPA<S>::findBest()
{
  SimplexF* minf = hull.root;
  S mind = minf->d * minf->d;
  for(SimplexF* f = minf->l[1]; f; f = f->l[1])
  {
    S sqd = f->d * f->d;
    if(sqd < mind)
    {
      minf = f;
      mind = sqd;
    }
  }
  return minf;
}

//==============================================================================
template <typename S>
typename EPA<S>::Status EPA<S>::evaluate(GJK<S>& gjk, const Vector3<S>& guess)
{
  typename GJK<S>::Simplex& simplex = *gjk.getSimplex();
  if((simplex.rank > 1) && gjk.encloseOrigin())
  {
    while(hull.root)
    {
      SimplexF* f = hull.root;
      hull.remove(f);
      stock.append(f);
    }

    status = Valid;
    nextsv = 0;

    if((simplex.c[0]->w - simplex.c[3]->w).dot((simplex.c[1]->w - simplex.c[3]->w).cross(simplex.c[2]->w - simplex.c[3]->w)) < 0)
    {
      SimplexV* tmp = simplex.c[0];
      simplex.c[0] = simplex.c[1];
      simplex.c[1] = tmp;

      S tmpv = simplex.p[0];
      simplex.p[0] = simplex.p[1];
      simplex.p[1] = tmpv;
    }

    SimplexF* tetrahedron[] = {newFace(simplex.c[0], simplex.c[1], simplex.c[2], true),
                               newFace(simplex.c[1], simplex.c[0], simplex.c[3], true),
                               newFace(simplex.c[2], simplex.c[1], simplex.c[3], true),
                               newFace(simplex.c[0], simplex.c[2], simplex.c[3], true) };

    if(hull.count == 4)
    {
      SimplexF* best = findBest(); // find the best face (the face with the minimum distance to origin) to split
      SimplexF outer = *best;
      size_t pass = 0;
      size_t iterations = 0;

      // set the face connectivity
      bind(tetrahedron[0], 0, tetrahedron[1], 0);
      bind(tetrahedron[0], 1, tetrahedron[2], 0);
      bind(tetrahedron[0], 2, tetrahedron[3], 0);
      bind(tetrahedron[1], 1, tetrahedron[3], 2);
      bind(tetrahedron[1], 2, tetrahedron[2], 1);
      bind(tetrahedron[2], 2, tetrahedron[3], 1);

      status = Valid;
      for(; iterations < max_iterations; ++iterations)
      {
        if(nextsv < max_vertex_num)
        {
          SimplexHorizon horizon;
          SimplexV* w = &sv_store[nextsv++];
          bool valid = true;
          best->pass = ++pass;
          gjk.getSupport(best->n, *w);
          S wdist = best->n.dot(w->w) - best->d;
          if(wdist > tolerance)
          {
            for(size_t j = 0; (j < 3) && valid; ++j)
            {
              valid &= expand(pass, w, best->f[j], best->e[j], horizon);
            }


            if(valid && horizon.nf >= 3)
            {
              // need to add the edge connectivity between first and last faces
              bind(horizon.ff, 2, horizon.cf, 1);
              hull.remove(best);
              stock.append(best);
              best = findBest();
              outer = *best;
            }
            else
            {
              status = InvalidHull; break;
            }
          }
          else
          {
            status = AccuracyReached; break;
          }
        }
        else
        {
          status = OutOfVertices; break;
        }
      }

      Vector3<S> projection = outer.n * outer.d;
      normal = outer.n;
      depth = outer.d;
      result.rank = 3;
      result.c[0] = outer.c[0];
      result.c[1] = outer.c[1];
      result.c[2] = outer.c[2];
      result.p[0] = ((outer.c[1]->w - projection).cross(outer.c[2]->w - projection)).norm();
      result.p[1] = ((outer.c[2]->w - projection).cross(outer.c[0]->w - projection)).norm();
      result.p[2] = ((outer.c[0]->w - projection).cross(outer.c[1]->w - projection)).norm();

      S sum = result.p[0] + result.p[1] + result.p[2];
      result.p[0] /= sum;
      result.p[1] /= sum;
      result.p[2] /= sum;
      return status;
    }
  }

  status = FallBack;
  normal = -guess;
  S nl = normal.norm();
  if(nl > 0) normal /= nl;
  else normal = Vector3<S>(1, 0, 0);
  depth = 0;
  result.rank = 1;
  result.c[0] = simplex.c[0];
  result.p[0] = 1;
  return status;
}

//==============================================================================
/** @brief the goal is to add a face connecting vertex w and face edge f[e] */
template <typename S>
bool EPA<S>::expand(size_t pass, SimplexV* w, SimplexF* f, size_t e, SimplexHorizon& horizon)
{
  static const size_t nexti[] = {1, 2, 0};
  static const size_t previ[] = {2, 0, 1};

  if(f->pass != pass)
  {
    const size_t e1 = nexti[e];

    // case 1: the new face is not degenerated, i.e., the new face is not coplanar with the old face f.
    if(f->n.dot(w->w) - f->d < -tolerance)
    {
      SimplexF* nf = newFace(f->c[e1], f->c[e], w, false);
      if(nf)
      {
        // add face-face connectivity
        bind(nf, 0, f, e);

        // if there is last face in the horizon, then need to add another connectivity, i.e. the edge connecting the current new add edge and the last new add edge.
        // This does not finish all the connectivities because the final face need to connect with the first face, this will be handled in the evaluate function.
        // Notice the face is anti-clockwise, so the edges are 0 (bottom), 1 (right), 2 (left)
        if(horizon.cf)
          bind(nf, 2, horizon.cf, 1);
        else
          horizon.ff = nf;

        horizon.cf = nf;
        ++horizon.nf;
        return true;
      }
    }
    else // case 2: the new face is coplanar with the old face f. We need to add two faces and delete the old face
    {
      const size_t e2 = previ[e];
      f->pass = pass;
      if(expand(pass, w, f->f[e1], f->e[e1], horizon) && expand(pass, w, f->f[e2], f->e[e2], horizon))
      {
        hull.remove(f);
        stock.append(f);
        return true;
      }
    }
  }

  return false;
}

} // namespace detail
} // namespace fcl

#endif
