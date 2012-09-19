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

#include "fcl/narrowphase/gjk.h"

namespace fcl
{

namespace details
{

Vec3f getSupport(const ShapeBase* shape, const Vec3f& dir)
{
  switch(shape->getNodeType())
  {
  case GEOM_TRIANGLE:
    {
      const Triangle2* triangle = static_cast<const Triangle2*>(shape);
      FCL_REAL dota = dir.dot(triangle->a);
      FCL_REAL dotb = dir.dot(triangle->b);
      FCL_REAL dotc = dir.dot(triangle->c);
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
      const Box* box = static_cast<const Box*>(shape);
      return Vec3f((dir[0]>0)?(box->side[0]/2):(-box->side[0]/2),
                   (dir[1]>0)?(box->side[1]/2):(-box->side[1]/2),
                   (dir[2]>0)?(box->side[2]/2):(-box->side[2]/2));
    }
    break;
  case GEOM_SPHERE:
    {
      const Sphere* sphere = static_cast<const Sphere*>(shape);
      return dir * sphere->radius;
    }
    break;
  case GEOM_CAPSULE:
    {
      const Capsule* capsule = static_cast<const Capsule*>(shape);
      FCL_REAL half_h = capsule->lz * 0.5;
      Vec3f pos1(0, 0, half_h);
      Vec3f pos2(0, 0, -half_h);
      Vec3f v = dir * capsule->radius;
      pos1 += v;
      pos2 += v;
      if(dir.dot(pos1) > dir.dot(pos2))
        return pos1;
      else return pos2;
    }
    break;
  case GEOM_CONE:
    {
      const Cone* cone = static_cast<const Cone*>(shape);
      FCL_REAL zdist = dir[0] * dir[0] + dir[1] * dir[1];
      FCL_REAL len = zdist + dir[2] * dir[2];
      zdist = std::sqrt(zdist);
      len = std::sqrt(len);
      FCL_REAL half_h = cone->lz * 0.5;
      FCL_REAL radius = cone->radius;

      FCL_REAL sin_a = radius / std::sqrt(radius * radius + 4 * half_h * half_h);

      if(dir[2] > len * sin_a)
        return Vec3f(0, 0, half_h);
      else if(zdist > 0)
      {
        FCL_REAL rad = radius / zdist;
        return Vec3f(rad * dir[0], rad * dir[1], -half_h);
      }
      else
        return Vec3f(0, 0, -half_h);
    }
    break;
  case GEOM_CYLINDER:
    {
      const Cylinder* cylinder = static_cast<const Cylinder*>(shape);
      FCL_REAL zdist = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1]);
      FCL_REAL half_h = cylinder->lz * 0.5;
      if(zdist == 0.0)
      {
        return Vec3f(0, 0, (dir[2]>0)? half_h:-half_h);
      }
      else
      {
        FCL_REAL d = cylinder->radius / zdist;
        return Vec3f(d * dir[0], d * dir[1], (dir[2]>0)?half_h:-half_h);
      }
    }
    break;
  case GEOM_CONVEX:
    {
      const Convex* convex = static_cast<const Convex*>(shape);
      FCL_REAL maxdot = - std::numeric_limits<FCL_REAL>::max();
      Vec3f* curp = convex->points;
      Vec3f bestv;
      for(size_t i = 0; i < convex->num_points; ++i, curp+=1)
      {
        FCL_REAL dot = dir.dot(*curp);
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
    {
      return Vec3f(0, 0, 0);
    }
    break;
  default:
    ; // nothing
  }

  return Vec3f(0, 0, 0);
}


FCL_REAL projectOrigin(const Vec3f& a, const Vec3f& b, FCL_REAL* w, size_t& m)
{
  const Vec3f d = b - a;
  const FCL_REAL l = d.sqrLength();
  if(l > 0)
  {
    const FCL_REAL t(l > 0 ? - a.dot(d) / l : 0);
    if(t >= 1) { w[0] = 0; w[1] = 1; m = 2; return b.sqrLength(); } // m = 0x10 
    else if(t <= 0) { w[0] = 1; w[1] = 0; m = 1; return a.sqrLength(); } // m = 0x01
    else { w[0] = 1 - (w[1] = t); m = 3; return (a + d * t).sqrLength(); } // m = 0x11
  }

  return -1;
}

FCL_REAL projectOrigin(const Vec3f& a, const Vec3f& b, const Vec3f& c, FCL_REAL* w, size_t& m)
{
  static const size_t nexti[3] = {1, 2, 0};
  const Vec3f* vt[] = {&a, &b, &c};
  const Vec3f dl[] = {a - b, b - c, c - a};
  const Vec3f& n = dl[0].cross(dl[1]);
  const FCL_REAL l = n.sqrLength();

  if(l > 0)
  {
    FCL_REAL mindist = -1;
    FCL_REAL subw[2] = {0, 0};
    size_t subm = 0;
    for(size_t i = 0; i < 3; ++i)
    {
      if(vt[i]->dot(dl[i].cross(n)) > 0) // origin is to the outside part of the triangle edge, then the optimal can only be on the edge
      {
        size_t j = nexti[i];
        FCL_REAL subd = projectOrigin(*vt[i], *vt[j], subw, subm);
        if(mindist < 0 || subd < mindist)
        {
          mindist = subd;
          m = static_cast<size_t>(((subm&1)?1<<i:0) + ((subm&2)?1<<j:0));
          w[i] = subw[0];
          w[j] = subw[1];
          w[nexti[j]] = 0;
        }
      }
    }
    
    if(mindist < 0) // the origin project is within the triangle
    {
      FCL_REAL d = a.dot(n);
      FCL_REAL s = sqrt(l);
      Vec3f p = n * (d / l);
      mindist = p.sqrLength();
      m = 7; // m = 0x111
      w[0] = dl[1].cross(b-p).length() / s;
      w[1] = dl[2].cross(c-p).length() / s;
      w[2] = 1 - (w[0] + w[1]);
    }

    return mindist;
  }
  return -1;
}

FCL_REAL projectOrigin(const Vec3f& a, const Vec3f& b, const Vec3f& c, const Vec3f& d, FCL_REAL* w, size_t& m)
{
  static const size_t nexti[] = {1, 2, 0};
  const Vec3f* vt[] = {&a, &b, &c, &d};
  const Vec3f dl[3] = {a-d, b-d, c-d};
  FCL_REAL vl = triple(dl[0], dl[1], dl[2]); 
  bool ng = (vl * a.dot((b-c).cross(a-b))) <= 0;
  if(ng && std::abs(vl) > 0) // abs(vl) == 0, the tetrahedron is degenerated; if ng is false, then the last vertex in the tetrahedron does not grow toward the origin (in fact origin is on the other side of the abc face)
  {
    FCL_REAL mindist = -1;
    FCL_REAL subw[3] = {0, 0, 0};
    size_t subm = 0;
    for(size_t i = 0; i < 3; ++i)
    {
      size_t j = nexti[i];
      FCL_REAL s = vl * d.dot(dl[i].cross(dl[j]));
      if(s > 0) // the origin is to the outside part of a triangle face, then the optimal can only be on the triangle face
      {
        FCL_REAL subd = projectOrigin(*vt[i], *vt[j], d, subw, subm);
        if(mindist < 0 || subd < mindist)
        {
          mindist = subd;
          m = static_cast<size_t>( (subm&1?1<<i:0) + (subm&2?1<<j:0) + (subm&4?8:0) );
          w[i] = subw[0];
          w[j] = subw[1];
          w[nexti[j]] = 0;
          w[3] = subw[2];
        }
      }
    }

    if(mindist < 0)
    {
      mindist = 0;
      m = 15;
      w[0] = triple(c, b, d) / vl;
      w[1] = triple(a, c, d) / vl;
      w[2] = triple(b, a, d) / vl;
      w[3] = 1 - (w[0] + w[1] + w[2]);
    }
    
    return mindist;
  }
  return -1;
}


void GJK::initialize()
{
  ray = Vec3f();
  nfree = 0;
  status = Failed;
  current = 0;
  distance = 0.0;
}

GJK::Status GJK::evaluate(const MinkowskiDiff& shape_, const Vec3f& guess)
{
  size_t iterations = 0;
  FCL_REAL sqdist = 0;
  FCL_REAL alpha = 0;
  Vec3f lastw[4];
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
    
  FCL_REAL sqrl = ray.sqrLength();
  appendVertex(simplices[0], (sqrl>0) ? -ray : Vec3f(1, 0, 0));
  simplices[0].p[0] = 1;
  ray = simplices[0].c[0]->w;
  sqdist = sqrl;
  lastw[0] = lastw[1] = lastw[2] = lastw[3] = ray;

  do
  {
    size_t next = 1 - current;
    Simplex& curr_simplex = simplices[current];
    Simplex& next_simplex = simplices[next];
      
    FCL_REAL rl = ray.sqrLength();
    if(rl < tolerance) // means origin is near the face of original simplex, return touch
    {
      status = Inside;
      break;
    }

    appendVertex(curr_simplex, -ray); // see below, ray points away from origin
      
    Vec3f& w = curr_simplex.c[curr_simplex.rank - 1]->w;
    bool found = false;
    for(size_t i = 0; i < 4; ++i)
    {
      if((w - lastw[i]).sqrLength() < tolerance)
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

    // check for termination, from bullet
    FCL_REAL omega = ray.dot(w) / rl;
    alpha = std::max(alpha, omega);
    if((rl - alpha) - tolerance * rl <= 0)
    {
      removeVertex(simplices[current]);
      break;
    }

    // reduce simplex and decide the extend direction
    FCL_REAL weights[4];
    size_t mask = 0; // decide the simplex vertices that compose the minimal distance
    switch(curr_simplex.rank)
    {
    case 2:
      sqdist = projectOrigin(curr_simplex.c[0]->w, curr_simplex.c[1]->w, weights, mask); break;
    case 3:
      sqdist = projectOrigin(curr_simplex.c[0]->w, curr_simplex.c[1]->w, curr_simplex.c[2]->w, weights, mask); break;
    case 4:
      sqdist = projectOrigin(curr_simplex.c[0]->w, curr_simplex.c[1]->w, curr_simplex.c[2]->w, curr_simplex.c[3]->w, weights, mask); break;
    }
      
    if(sqdist >= 0)
    {
      next_simplex.rank = 0;
      ray = Vec3f();
      current = next;
      for(size_t i = 0; i < curr_simplex.rank; ++i)
      {
        if(mask & (1 << i))
        {
          next_simplex.c[next_simplex.rank] = curr_simplex.c[i];
          next_simplex.p[next_simplex.rank++] = weights[i];
          ray += curr_simplex.c[i]->w * weights[i];
        }
        else
          free_v[nfree++] = curr_simplex.c[i];
      }
      if(mask == 15) status = Inside; // the origin is within the 4-simplex, collision
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
  case Valid: distance = ray.length(); break;
  case Inside: distance = 0; break;
  }
  return status;
}

void GJK::getSupport(const Vec3f& d, SimplexV& sv) const
{
  sv.d = normalize(d);
  sv.w = shape.support(sv.d);
}

void GJK::removeVertex(Simplex& simplex)
{
  free_v[nfree++] = simplex.c[--simplex.rank];
}

void GJK::appendVertex(Simplex& simplex, const Vec3f& v)
{
  simplex.p[simplex.rank] = 0; // initial weight 0
  simplex.c[simplex.rank] = free_v[--nfree]; // set the memory
  getSupport(v, *simplex.c[simplex.rank++]);
}

bool GJK::encloseOrigin()
{
  switch(simplex->rank)
  {
  case 1:
    {
      for(size_t i = 0; i < 3; ++i)
      {
        Vec3f axis;
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
      Vec3f d = simplex->c[1]->w - simplex->c[0]->w;
      for(size_t i = 0; i < 3; ++i)
      {
        Vec3f axis;
        axis[i] = 1;
        Vec3f p = d.cross(axis);
        if(p.sqrLength() > 0)
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
      Vec3f n = (simplex->c[1]->w - simplex->c[0]->w).cross(simplex->c[2]->w - simplex->c[0]->w);
      if(n.sqrLength() > 0)
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


void EPA::initialize()
{
  sv_store = new SimplexV[max_vertex_num];
  fc_store = new SimplexF[max_face_num];
  status = Failed;
  normal = Vec3f(0, 0, 0);
  depth = 0;
  nextsv = 0;
  for(size_t i = 0; i < max_face_num; ++i)
    stock.append(&fc_store[max_face_num-i-1]);
}

bool EPA::getEdgeDist(SimplexF* face, SimplexV* a, SimplexV* b, FCL_REAL& dist)
{
  Vec3f ba = b->w - a->w;
  Vec3f n_ab = ba.cross(face->n);
  FCL_REAL a_dot_nab = a->w.dot(n_ab);

  if(a_dot_nab < 0) // the origin is on the outside part of ab
  {
    FCL_REAL ba_l2 = ba.sqrLength();

    // following is similar to projectOrigin for two points
    // however, as we dont need to compute the parameterization, dont need to compute 0 or 1
    FCL_REAL a_dot_ba = a->w.dot(ba); 
    FCL_REAL b_dot_ba = b->w.dot(ba);

    if(a_dot_ba > 0) 
      dist = a->w.length();
    else if(b_dot_ba < 0)
      dist = b->w.length();
    else
    {
      FCL_REAL a_dot_b = a->w.dot(b->w);
      dist = std::sqrt(std::max(a->w.sqrLength() * b->w.sqrLength() - a_dot_b * a_dot_b, (FCL_REAL)0));
    }

    return true;
  }

  return false;
}

EPA::SimplexF* EPA::newFace(SimplexV* a, SimplexV* b, SimplexV* c, bool forced)
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
    face->n = (b->w - a->w).cross(c->w - a->w);
    FCL_REAL l = face->n.length();
      
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
    return NULL;
  }
    
  status = stock.root ? OutOfVertices : OutOfFaces;
  return NULL;
}

/** \brief Find the best polytope face to split */
EPA::SimplexF* EPA::findBest()
{
  SimplexF* minf = hull.root;
  FCL_REAL mind = minf->d * minf->d;
  for(SimplexF* f = minf->l[1]; f; f = f->l[1])
  {
    FCL_REAL sqd = f->d * f->d;
    if(sqd < mind)
    {
      minf = f;
      mind = sqd;
    }
  }
  return minf;
}

EPA::Status EPA::evaluate(GJK& gjk, const Vec3f& guess)
{
  GJK::Simplex& simplex = *gjk.getSimplex();
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

      FCL_REAL tmpv = simplex.p[0];
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
          FCL_REAL wdist = best->n.dot(w->w) - best->d;
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

      Vec3f projection = outer.n * outer.d;
      normal = outer.n;
      depth = outer.d;
      result.rank = 3;
      result.c[0] = outer.c[0];
      result.c[1] = outer.c[1];
      result.c[2] = outer.c[2];
      result.p[0] = ((outer.c[1]->w - projection).cross(outer.c[2]->w - projection)).length();
      result.p[1] = ((outer.c[2]->w - projection).cross(outer.c[0]->w - projection)).length();
      result.p[2] = ((outer.c[0]->w - projection).cross(outer.c[1]->w - projection)).length();

      FCL_REAL sum = result.p[0] + result.p[1] + result.p[2];
      result.p[0] /= sum;
      result.p[1] /= sum;
      result.p[2] /= sum;
      return status;
    }
  }

  status = FallBack;
  normal = -guess;
  FCL_REAL nl = normal.length();
  if(nl > 0) normal /= nl;
  else normal = Vec3f(1, 0, 0);
  depth = 0;
  result.rank = 1;
  result.c[0] = simplex.c[0];
  result.p[0] = 1;
  return status;
}


/** \brief the goal is to add a face connecting vertex w and face edge f[e] */
bool EPA::expand(size_t pass, SimplexV* w, SimplexF* f, size_t e, SimplexHorizon& horizon)
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

} // fcl
