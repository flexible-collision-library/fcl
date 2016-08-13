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

#ifndef FCL_NARROWPHASE_DETAIL_GJK_H
#define FCL_NARROWPHASE_DETAIL_GJK_H

#include "fcl/shape/geometric_shapes.h"
#include "fcl/intersect.h"

namespace fcl
{

namespace details
{

/// @brief the support function for shape
template <typename S, typename Derived>
Vector3<S> getSupport(
    const ShapeBase<S>* shape,
    const Eigen::MatrixBase<Derived>& dir);

/// @brief Minkowski difference class of two shapes
template <typename S>
struct MinkowskiDiff
{
  /// @brief points to two shapes
  const ShapeBase<S>* shapes[2];

  /// @brief rotation from shape0 to shape1
  Matrix3<S> toshape1;

  /// @brief transform from shape1 to shape0 
  Transform3<S> toshape0;

  MinkowskiDiff() { }

  /// @brief support function for shape0
  inline Vector3<S> support0(const Vector3<S>& d) const
  {
    return getSupport(shapes[0], d);
  }
  
  /// @brief support function for shape1
  inline Vector3<S> support1(const Vector3<S>& d) const
  {
    return toshape0 * getSupport(shapes[1], toshape1 * d);
  }

  /// @brief support function for the pair of shapes
  inline Vector3<S> support(const Vector3<S>& d) const
  {
    return support0(d) - support1(-d);
  }

  /// @brief support function for the d-th shape (d = 0 or 1)
  inline Vector3<S> support(const Vector3<S>& d, size_t index) const
  {
    if(index)
      return support1(d);
    else
      return support0(d);
  }

  /// @brief support function for translating shape0, which is translating at velocity v
  inline Vector3<S> support0(const Vector3<S>& d, const Vector3<S>& v) const
  {
    if(d.dot(v) <= 0)
      return getSupport(shapes[0], d);
    else
      return getSupport(shapes[0], d) + v;
  }

  /// @brief support function for the pair of shapes, where shape0 is translating at velocity v
  inline Vector3<S> support(const Vector3<S>& d, const Vector3<S>& v) const
  {
    return support0(d, v) - support1(-d);
  }

  /// @brief support function for the d-th shape (d = 0 or 1), where shape0 is translating at velocity v
  inline Vector3<S> support(const Vector3<S>& d, const Vector3<S>& v, size_t index) const
  {
    if(index)
      return support1(d);
    else
      return support0(d, v);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using MinkowskiDifff = MinkowskiDiff<float>;
using MinkowskiDiffd = MinkowskiDiff<double>;

/// @brief class for GJK algorithm
template <typename S>
struct GJK
{
  struct SimplexV
  {
    /// @brief support direction
    Vector3<S> d;
    /// @brieg support vector (i.e., the furthest point on the shape along the support direction)
    Vector3<S> w;
  };

  struct Simplex
  {
    /// @brief simplex vertex
    SimplexV* c[4];
    /// @brief weight 
    S p[4];
    /// @brief size of simplex (number of vertices)
    size_t rank;

    Simplex() : rank(0) {}
  };

  enum Status {Valid, Inside, Failed};

  MinkowskiDiff<S> shape;
  Vector3<S> ray;
  S distance;
  Simplex simplices[2];


  GJK(unsigned int max_iterations_, S tolerance_)  : max_iterations(max_iterations_),
                                                            tolerance(tolerance_)
  {
    initialize(); 
  }
  
  void initialize();

  /// @brief GJK algorithm, given the initial value guess
  Status evaluate(const MinkowskiDiff<S>& shape_, const Vector3<S>& guess);

  /// @brief apply the support function along a direction, the result is return in sv
  void getSupport(const Vector3<S>& d, SimplexV& sv) const;

  /// @brief apply the support function along a direction, the result is return is sv, here shape0 is translating at velocity v
  void getSupport(const Vector3<S>& d, const Vector3<S>& v, SimplexV& sv) const;

  /// @brief discard one vertex from the simplex
  void removeVertex(Simplex& simplex);

  /// @brief append one vertex to the simplex
  void appendVertex(Simplex& simplex, const Vector3<S>& v);

  /// @brief whether the simplex enclose the origin
  bool encloseOrigin();

  /// @brief get the underlying simplex using in GJK, can be used for cache in next iteration
  inline Simplex* getSimplex() const
  {
    return simplex;
  }

  /// @brief get the guess from current simplex
  Vector3<S> getGuessFromSimplex() const;

private:
  SimplexV store_v[4];
  SimplexV* free_v[4];
  size_t nfree;
  size_t current;
  Simplex* simplex;
  Status status;

  unsigned int max_iterations;
  S tolerance;

};

using GJKf = GJK<float>;
using GJKd = GJK<double>;

//static const size_t EPA_MAX_FACES = 128;
//static const size_t EPA_MAX_VERTICES = 64;
//static const S EPA_EPS = 0.000001;
//static const size_t EPA_MAX_ITERATIONS = 255;
// TODO(JS): remove?

/// @brief class for EPA algorithm
template <typename S>
struct EPA
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
    SimplexList() : root(nullptr), count(0) {}
    void append(SimplexF* face)
    {
      face->l[0] = nullptr;
      face->l[1] = root;
      if(root) root->l[0] = face;
      root = face;
      ++count;
    }

    void remove(SimplexF* face)
    {
      if(face->l[1]) face->l[1]->l[0] = face->l[0];
      if(face->l[0]) face->l[0]->l[1] = face->l[1];
      if(face == root) root = face->l[1];
      --count;
    }
  };

  static inline void bind(SimplexF* fa, size_t ea, SimplexF* fb, size_t eb)
  {
    fa->e[ea] = eb; fa->f[ea] = fb;
    fb->e[eb] = ea; fb->f[eb] = fa;
  }

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

  EPA(unsigned int max_face_num_, unsigned int max_vertex_num_, unsigned int max_iterations_, S tolerance_) : max_face_num(max_face_num_),
                                                                                                                     max_vertex_num(max_vertex_num_),
                                                                                                                     max_iterations(max_iterations_),
                                                                                                                     tolerance(tolerance_)
  {
    initialize();
  }

  ~EPA()
  {
    delete [] sv_store;
    delete [] fc_store;
  }

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

} // namespace details

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
namespace details
{

template <typename S, typename Derived>
Vector3<S> getSupport(
    const ShapeBase<S>* shape,
    const Eigen::MatrixBase<Derived>& dir)
{
  // Check the number of rows is 6 at compile time
  EIGEN_STATIC_ASSERT(
        Derived::RowsAtCompileTime == 3
        && Derived::ColsAtCompileTime == 1,
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  switch(shape->getNodeType())
  {
  case GEOM_TRIANGLE:
    {
      const auto* triangle = static_cast<const TriangleP<S>*>(shape);
      S dota = dir.dot(triangle->a);
      S dotb = dir.dot(triangle->b);
      S dotc = dir.dot(triangle->c);
      if(dota > dotb)
      {
        if(dotc > dota)
          return triangle->c;
        else
          return triangle->a;
      }
      else
      {
        if(dotc > dotb)
          return triangle->c;
        else
          return triangle->b;
      }
    }
    break;
  case GEOM_BOX:
    {
      const Box<S>* box = static_cast<const Box<S>*>(shape);
      return Vector3<S>((dir[0]>0)?(box->side[0]/2):(-box->side[0]/2),
                   (dir[1]>0)?(box->side[1]/2):(-box->side[1]/2),
                   (dir[2]>0)?(box->side[2]/2):(-box->side[2]/2));
    }
    break;
  case GEOM_SPHERE:
    {
      const Sphere<S>* sphere = static_cast<const Sphere<S>*>(shape);
      return dir * sphere->radius;
    }
    break;
  case GEOM_ELLIPSOID:
    {
      const Ellipsoid<S>* ellipsoid = static_cast<const Ellipsoid<S>*>(shape);

      const S a2 = ellipsoid->radii[0] * ellipsoid->radii[0];
      const S b2 = ellipsoid->radii[1] * ellipsoid->radii[1];
      const S c2 = ellipsoid->radii[2] * ellipsoid->radii[2];

      const Vector3<S> v(a2 * dir[0], b2 * dir[1], c2 * dir[2]);
      const S d = std::sqrt(v.dot(dir));

      return v / d;
    }
    break;
  case GEOM_CAPSULE:
    {
      const Capsule<S>* capsule = static_cast<const Capsule<S>*>(shape);
      S half_h = capsule->lz * 0.5;
      Vector3<S> pos1(0, 0, half_h);
      Vector3<S> pos2(0, 0, -half_h);
      Vector3<S> v = dir * capsule->radius;
      pos1 += v;
      pos2 += v;
      if(dir.dot(pos1) > dir.dot(pos2))
        return pos1;
      else return pos2;
    }
    break;
  case GEOM_CONE:
    {
      const Cone<S>* cone = static_cast<const Cone<S>*>(shape);
      S zdist = dir[0] * dir[0] + dir[1] * dir[1];
      S len = zdist + dir[2] * dir[2];
      zdist = std::sqrt(zdist);
      len = std::sqrt(len);
      S half_h = cone->lz * 0.5;
      S radius = cone->radius;

      S sin_a = radius / std::sqrt(radius * radius + 4 * half_h * half_h);

      if(dir[2] > len * sin_a)
        return Vector3<S>(0, 0, half_h);
      else if(zdist > 0)
      {
        S rad = radius / zdist;
        return Vector3<S>(rad * dir[0], rad * dir[1], -half_h);
      }
      else
        return Vector3<S>(0, 0, -half_h);
    }
    break;
  case GEOM_CYLINDER:
    {
      const Cylinder<S>* cylinder = static_cast<const Cylinder<S>*>(shape);
      S zdist = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1]);
      S half_h = cylinder->lz * 0.5;
      if(zdist == 0.0)
      {
        return Vector3<S>(0, 0, (dir[2]>0)? half_h:-half_h);
      }
      else
      {
        S d = cylinder->radius / zdist;
        return Vector3<S>(d * dir[0], d * dir[1], (dir[2]>0)?half_h:-half_h);
      }
    }
    break;
  case GEOM_CONVEX:
    {
      const Convex<S>* convex = static_cast<const Convex<S>*>(shape);
      S maxdot = - std::numeric_limits<S>::max();
      Vector3<S>* curp = convex->points;
      Vector3<S> bestv = Vector3<S>::Zero();
      for(int i = 0; i < convex->num_points; ++i, curp+=1)
      {
        S dot = dir.dot(*curp);
        if(dot > maxdot)
        {
          bestv = *curp;
          maxdot = dot;
        }
      }
      return bestv;
    }
    break;
  case GEOM_PLANE:
  break;
  default:
    ; // nothing
  }

  return Vector3<S>::Zero();
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
/** \brief Find the best polytope face to split */
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
/** \brief the goal is to add a face connecting vertex w and face edge f[e] */
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

} // details

} // namespace fcl

#endif
