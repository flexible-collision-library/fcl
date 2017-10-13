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

#ifndef FCL_NARROWPHASE_DETAIL_EPA_H
#define FCL_NARROWPHASE_DETAIL_EPA_H

#include "fcl/narrowphase/detail/convexity_based_algorithm/gjk.h"

namespace fcl
{

namespace detail
{

//static const size_t EPA_MAX_FACES = 128;
//static const size_t EPA_MAX_VERTICES = 64;
//static const S EPA_EPS = 0.000001;
//static const size_t EPA_MAX_ITERATIONS = 255;
// TODO(JS): remove?

/// @brief class for EPA algorithm
template <typename S>
struct FCL_EXPORT EPA
{
private:
  using SimplexV = typename GJK<S>::SimplexV;

  struct SimplexF
  {
    Vector3<S> n;
    S d;
    SimplexV* c[3]; // a face has three vertices
    SimplexF* f[3]; // a face has three adjacent faces
    SimplexF* l[2]; // the pre and post faces in the list
    size_t e[3];
    size_t pass;
  };

  struct SimplexList
  {
    SimplexF* root;
    size_t count;

    SimplexList();

    void append(SimplexF* face);

    void remove(SimplexF* face);
  };

  static void bind(SimplexF* fa, size_t ea, SimplexF* fb, size_t eb);

  struct SimplexHorizon
  {
    SimplexF* cf; // current face in the horizon
    SimplexF* ff; // first face in the horizon
    size_t nf; // number of faces in the horizon
    SimplexHorizon() : cf(nullptr), ff(nullptr), nf(0) {}
  };

private:
  unsigned int max_face_num;
  unsigned int max_vertex_num;
  unsigned int max_iterations;
  S tolerance;

public:

  enum Status {Valid, Touching, Degenerated, NonConvex, InvalidHull, OutOfFaces, OutOfVertices, AccuracyReached, FallBack, Failed};
  
  Status status;
  typename GJK<S>::Simplex result;
  Vector3<S> normal;
  S depth;
  SimplexV* sv_store;
  SimplexF* fc_store;
  size_t nextsv;
  SimplexList hull, stock;

  EPA(
      unsigned int max_face_num_,
      unsigned int max_vertex_num_,
      unsigned int max_iterations_,
      S tolerance_);

  ~EPA();

  void initialize();

  bool getEdgeDist(SimplexF* face, SimplexV* a, SimplexV* b, S& dist);

  SimplexF* newFace(SimplexV* a, SimplexV* b, SimplexV* c, bool forced);

  /// @brief Find the best polytope face to split
  SimplexF* findBest();

  Status evaluate(GJK<S>& gjk, const Vector3<S>& guess);

  /// @brief the goal is to add a face connecting vertex w and face edge f[e] 
  bool expand(size_t pass, SimplexV* w, SimplexF* f, size_t e, SimplexHorizon& horizon);  
};

using EPAf = EPA<float>;
using EPAd = EPA<double>;

} // namespace detail
} // namespace fcl

#include "fcl/narrowphase/detail/convexity_based_algorithm/epa-inl.h"

#endif
