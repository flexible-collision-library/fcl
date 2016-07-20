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

#include "fcl/narrowphase/gjk.h"
#include "fcl/intersect.h"

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
      const TriangleP* triangle = static_cast<const TriangleP*>(shape);
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
  case GEOM_ELLIPSOID:
    {
      const Ellipsoid* ellipsoid = static_cast<const Ellipsoid*>(shape);

      const FCL_REAL a2 = ellipsoid->radii[0] * ellipsoid->radii[0];
      const FCL_REAL b2 = ellipsoid->radii[1] * ellipsoid->radii[1];
      const FCL_REAL c2 = ellipsoid->radii[2] * ellipsoid->radii[2];

      const Vec3f v(a2 * dir[0], b2 * dir[1], c2 * dir[2]);
      const FCL_REAL d = std::sqrt(v.dot(dir));

      return v / d;
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
      for(int i = 0; i < convex->num_points; ++i, curp+=1)
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
	break;
  default:
    ; // nothing
  }

  return Vec3f(0, 0, 0);
}

void GJK::initialize()
{
  ray = Vec3f();
  nfree = 0;
  status = Failed;
  current = 0;
  distance = 0.0;
  simplex = NULL;
}


Vec3f GJK::getGuessFromSimplex() const
{
  return ray;
}


GJK::Status GJK::evaluate(const MinkowskiDiff& shape_, const Vec3f& guess)
{
  size_t iterations = 0;
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

  appendVertex(simplices[0], (ray.sqrLength() > 0) ? -ray : Vec3f(1, 0, 0));
  simplices[0].p[0] = 1;
  ray = simplices[0].c[0]->w;
  lastw[0] = lastw[1] = lastw[2] = lastw[3] = ray; // cache previous support points, the new support point will compare with it to avoid too close support points

  do
  {
    size_t next = 1 - current;
    Simplex& curr_simplex = simplices[current];
    Simplex& next_simplex = simplices[next];

    // check A: when origin is near the existing simplex, stop
    FCL_REAL rl = ray.length();
    if(rl < tolerance) // mean origin is near the face of original simplex, return touch
    {
      status = Inside;
      break;
    }

    appendVertex(curr_simplex, -ray); // see below, ray points away from origin

    // check B: when the new support point is close to previous support points, stop (as the new simplex is degenerated)
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

    // check C: when the new support point is close to the sub-simplex where the ray point lies, stop (as the new simplex again is degenerated)
    FCL_REAL omega = ray.dot(w) / rl;
    alpha = std::max(alpha, omega);
    if((rl - alpha) - tolerance * rl <= 0)
    {
      removeVertex(simplices[current]);
      break;
    }

    Project::ProjectResult project_res;
    switch(curr_simplex.rank)
    {
    case 2:
      project_res = Project::projectLineOrigin(curr_simplex.c[0]->w, curr_simplex.c[1]->w); break;
    case 3:
      project_res = Project::projectTriangleOrigin(curr_simplex.c[0]->w, curr_simplex.c[1]->w, curr_simplex.c[2]->w); break;
    case 4:
      project_res = Project::projectTetrahedraOrigin(curr_simplex.c[0]->w, curr_simplex.c[1]->w, curr_simplex.c[2]->w, curr_simplex.c[3]->w); break;
    }
      
    if(project_res.sqr_distance >= 0)
    {
      next_simplex.rank = 0;
      ray = Vec3f();
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
  case Valid: distance = ray.length(); break;
  case Inside: distance = 0; break;
  default: break;
  }
  return status;
}

void GJK::getSupport(const Vec3f& d, SimplexV& sv) const
{
  sv.d = normalize(d);
  sv.w = shape.support(sv.d);
}

void GJK::getSupport(const Vec3f& d, const Vec3f& v, SimplexV& sv) const
{
  sv.d = normalize(d);
  sv.w = shape.support(sv.d, v);
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
