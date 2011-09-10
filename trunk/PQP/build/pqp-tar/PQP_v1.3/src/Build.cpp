/*************************************************************************\

  Copyright 1999 The University of North Carolina at Chapel Hill.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software and its
  documentation for educational, research and non-profit purposes, without
  fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL BE
  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
  OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGES.

  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF
  NORTH CAROLINA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

  The authors may be contacted via:

  US Mail:             S. Gottschalk, E. Larsen
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:               geom@cs.unc.edu


\**************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "PQP.h"
#include "MatVec.h"

// If this is set, build routines will use covariance matrix 
// and mean finding code from RAPID 2.

#define RAPID2_FIT 0

#if RAPID2_FIT

struct moment
{
  PQP_REAL A;  
  PQP_REAL m[3];
  PQP_REAL s[3][3];
};

struct accum
{
  PQP_REAL A;
  PQP_REAL m[3];
  PQP_REAL s[3][3];
};

inline
void
clear_accum(accum &a)
{
  a.m[0] = a.m[1] = a.m[2] = 0.0;
  a.s[0][0] = a.s[0][1] = a.s[0][2] = 0.0;
  a.s[1][0] = a.s[1][1] = a.s[1][2] = 0.0;
  a.s[2][0] = a.s[2][1] = a.s[2][2] = 0.0;
  a.A = 0.0;
}

inline
void
accum_moment(accum &a, moment &b)
{
  a.m[0] += b.m[0] * b.A;
  a.m[1] += b.m[1] * b.A;
  a.m[2] += b.m[2] * b.A;
  
  a.s[0][0] += b.s[0][0];
  a.s[0][1] += b.s[0][1];
  a.s[0][2] += b.s[0][2];
  a.s[1][0] += b.s[1][0];
  a.s[1][1] += b.s[1][1];
  a.s[1][2] += b.s[1][2];
  a.s[2][0] += b.s[2][0];
  a.s[2][1] += b.s[2][1];
  a.s[2][2] += b.s[2][2];

  a.A += b.A;
}

inline
void
mean_from_moment(PQP_REAL M[3], moment &m)
{
  M[0] = m.m[0];
  M[1] = m.m[1];
  M[2] = m.m[2];
}

inline
void
mean_from_accum(PQP_REAL M[3], accum &a)
{
  M[0] = a.m[0] / a.A;
  M[1] = a.m[1] / a.A;
  M[2] = a.m[2] / a.A;
}

inline
void
covariance_from_accum(PQP_REAL C[3][3], accum &a)
{
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      C[i][j] = a.s[i][j] - a.m[i]*a.m[j]/a.A;
}

inline
void
compute_moment(moment &M, PQP_REAL p[3], PQP_REAL q[3], PQP_REAL r[3])
{
  PQP_REAL u[3], v[3], w[3];

  // compute the area of the triangle
  VmV(u, q, p);
  VmV(v, r, p);
  VcrossV(w, u, v);
  M.A = 0.5 * Vlength(w);

  if (M.A == 0.0)
    {
      // This triangle has zero area.  The second order components
      // would be eliminated with the usual formula, so, for the 
      // sake of robustness we use an alternative form.  These are the 
      // centroid and second-order components of the triangle's vertices.

      // centroid
      M.m[0] = (p[0] + q[0] + r[0]) /3;
      M.m[1] = (p[1] + q[1] + r[1]) /3;
      M.m[2] = (p[2] + q[2] + r[2]) /3;

      // second-order components
      M.s[0][0] = (p[0]*p[0] + q[0]*q[0] + r[0]*r[0]);
      M.s[0][1] = (p[0]*p[1] + q[0]*q[1] + r[0]*r[1]);
      M.s[0][2] = (p[0]*p[2] + q[0]*q[2] + r[0]*r[2]);
      M.s[1][1] = (p[1]*p[1] + q[1]*q[1] + r[1]*r[1]);
      M.s[1][2] = (p[1]*p[2] + q[1]*q[2] + r[1]*r[2]);
      M.s[2][2] = (p[2]*p[2] + q[2]*q[2] + r[2]*r[2]);      
      M.s[2][1] = M.s[1][2];
      M.s[1][0] = M.s[0][1];
      M.s[2][0] = M.s[0][2];

      return;
    }

  // get the centroid
  M.m[0] = (p[0] + q[0] + r[0])/3;
  M.m[1] = (p[1] + q[1] + r[1])/3;
  M.m[2] = (p[2] + q[2] + r[2])/3;

  // get the second order components -- note the weighting by the area
  M.s[0][0] = M.A*(9*M.m[0]*M.m[0]+p[0]*p[0]+q[0]*q[0]+r[0]*r[0])/12;
  M.s[0][1] = M.A*(9*M.m[0]*M.m[1]+p[0]*p[1]+q[0]*q[1]+r[0]*r[1])/12;
  M.s[1][1] = M.A*(9*M.m[1]*M.m[1]+p[1]*p[1]+q[1]*q[1]+r[1]*r[1])/12;
  M.s[0][2] = M.A*(9*M.m[0]*M.m[2]+p[0]*p[2]+q[0]*q[2]+r[0]*r[2])/12;
  M.s[1][2] = M.A*(9*M.m[1]*M.m[2]+p[1]*p[2]+q[1]*q[2]+r[1]*r[2])/12;
  M.s[2][2] = M.A*(9*M.m[2]*M.m[2]+p[2]*p[2]+q[2]*q[2]+r[2]*r[2])/12;
  M.s[2][1] = M.s[1][2];
  M.s[1][0] = M.s[0][1];
  M.s[2][0] = M.s[0][2];
}

inline
void
compute_moments(moment *M, Tri *tris, int num_tris)
{
  int i;

  // first collect all the moments, and obtain the area of the 
  // smallest nonzero area triangle.

  PQP_REAL Amin = 0.0;
  int zero = 0;
  int nonzero = 0;
  for(i=0; i<num_tris; i++)
  {
    compute_moment(M[i], 
		   tris[i].p1,
		   tris[i].p2, 
		   tris[i].p3);  
    if (M[i].A == 0.0)
    {
	    zero = 1;
    }
    else
    {
	    nonzero = 1;
	    if (Amin == 0.0) Amin = M[i].A;
	    else if (M[i].A < Amin) Amin = M[i].A;
    }
  }

  if (zero)
  {
    fprintf(stderr, "----\n");
    fprintf(stderr, "Warning!  Some triangles have zero area!\n");
    fprintf(stderr, "----\n");

    // if there are any zero area triangles, go back and set their area
  
    // if ALL the triangles have zero area, then set the area thingy
    // to some arbitrary value.
    if (Amin == 0.0) Amin = 1.0;

    for(i=0; i<num_tris; i++)
    {
      if (M[i].A == 0.0) M[i].A = Amin;
    }    
  }
}

#else

PQP_REAL max(PQP_REAL a, PQP_REAL b, PQP_REAL c, PQP_REAL d)
{
  PQP_REAL t = a;
  if (b > t) t = b;
  if (c > t) t = c;
  if (d > t) t = d;
  return t;
}

PQP_REAL min(PQP_REAL a, PQP_REAL b, PQP_REAL c, PQP_REAL d)
{
  PQP_REAL t = a;
  if (b < t) t = b;
  if (c < t) t = c;
  if (d < t) t = d;
  return t;
}

void
get_centroid_triverts(PQP_REAL c[3], Tri *tris, int num_tris)
{
  int i;

  c[0] = c[1] = c[2] = 0.0;

  // get center of mass
  for(i=0; i<num_tris; i++)
  {
    PQP_REAL *p1 = tris[i].p1;
    PQP_REAL *p2 = tris[i].p2;
    PQP_REAL *p3 = tris[i].p3;

    c[0] += p1[0] + p2[0] + p3[0];
    c[1] += p1[1] + p2[1] + p3[1];
    c[2] += p1[2] + p2[2] + p3[2];      
  }

  PQP_REAL n = (PQP_REAL)(3 * num_tris);

  c[0] /= n;
  c[1] /= n;
  c[2] /= n;
}

void
get_covariance_triverts(PQP_REAL M[3][3], Tri *tris, int num_tris)
{
  int i;
  PQP_REAL S1[3];
  PQP_REAL S2[3][3];

  S1[0] = S1[1] = S1[2] = 0.0;
  S2[0][0] = S2[1][0] = S2[2][0] = 0.0;
  S2[0][1] = S2[1][1] = S2[2][1] = 0.0;
  S2[0][2] = S2[1][2] = S2[2][2] = 0.0;

  // get center of mass
  for(i=0; i<num_tris; i++)
  {
    PQP_REAL *p1 = tris[i].p1;
    PQP_REAL *p2 = tris[i].p2;
    PQP_REAL *p3 = tris[i].p3;

    S1[0] += p1[0] + p2[0] + p3[0];
    S1[1] += p1[1] + p2[1] + p3[1];
    S1[2] += p1[2] + p2[2] + p3[2];

    S2[0][0] += (p1[0] * p1[0] +  
                 p2[0] * p2[0] +  
                 p3[0] * p3[0]);
    S2[1][1] += (p1[1] * p1[1] +  
                 p2[1] * p2[1] +  
                 p3[1] * p3[1]);
    S2[2][2] += (p1[2] * p1[2] +  
                 p2[2] * p2[2] +  
                 p3[2] * p3[2]);
    S2[0][1] += (p1[0] * p1[1] +  
                 p2[0] * p2[1] +  
                 p3[0] * p3[1]);
    S2[0][2] += (p1[0] * p1[2] +  
                 p2[0] * p2[2] +  
                 p3[0] * p3[2]);
    S2[1][2] += (p1[1] * p1[2] +  
                 p2[1] * p2[2] +  
                 p3[1] * p3[2]);
  }

  PQP_REAL n = (PQP_REAL)(3 * num_tris);

  // now get covariances

  M[0][0] = S2[0][0] - S1[0]*S1[0] / n;
  M[1][1] = S2[1][1] - S1[1]*S1[1] / n;
  M[2][2] = S2[2][2] - S1[2]*S1[2] / n;
  M[0][1] = S2[0][1] - S1[0]*S1[1] / n;
  M[1][2] = S2[1][2] - S1[1]*S1[2] / n;
  M[0][2] = S2[0][2] - S1[0]*S1[2] / n;
  M[1][0] = M[0][1];
  M[2][0] = M[0][2];
  M[2][1] = M[1][2];
}

#endif

// given a list of triangles, a splitting axis, and a coordinate on
// that axis, partition the triangles into two groups according to
// where their centroids fall on the axis (under axial projection).
// Returns the number of tris in the first half

int 
split_tris(Tri *tris, int num_tris, PQP_REAL a[3], PQP_REAL c)
{
  int i;
  int c1 = 0;
  PQP_REAL p[3];
  PQP_REAL x;
  Tri temp;

  for(i = 0; i < num_tris; i++)
  {
    // loop invariant: up to (but not including) index c1 in group 1,
    // then up to (but not including) index i in group 2
    //
    //  [1] [1] [1] [1] [2] [2] [2] [x] [x] ... [x]
    //                   c1          i
    //
    VcV(p, tris[i].p1);
    VpV(p, p, tris[i].p2);
    VpV(p, p, tris[i].p3);      
    x = VdotV(p, a);
    x /= 3.0;
    if (x <= c)
    {
	    // group 1
	    temp = tris[i];
	    tris[i] = tris[c1];
	    tris[c1] = temp;
	    c1++;
    }
    else
    {
	    // group 2 -- do nothing
    }
  }

  // split arbitrarily if one group empty

  if ((c1 == 0) || (c1 == num_tris)) c1 = num_tris/2;

  return c1;
}

// Fits m->child(bn) to the num_tris triangles starting at first_tri
// Then, if num_tris is greater than one, partitions the tris into two
// sets, and recursively builds two children of m->child(bn)

int
build_recurse(PQP_Model *m, int bn, int first_tri, int num_tris)
{
  BV *b = m->child(bn);

  // compute a rotation matrix

  PQP_REAL C[3][3], E[3][3], R[3][3], s[3], axis[3], mean[3], coord;

#if RAPID2_FIT
  moment *tri_moment = new moment[num_tris];
  compute_moments(tri_moment, &(m->tris[first_tri]), num_tris);  
  accum acc;
  clear_accum(acc);
  for(int i = 0; i < num_tris; i++) accum_moment(acc, tri_moment[i]);
  delete [] tri_moment;
  covariance_from_accum(C,acc);
#else
  get_covariance_triverts(C,&m->tris[first_tri],num_tris);
#endif

  Meigen(E, s, C);

  // place axes of E in order of increasing s

  int min, mid, max;
  if (s[0] > s[1]) { max = 0; min = 1; }
  else { min = 0; max = 1; }
  if (s[2] < s[min]) { mid = min; min = 2; }
  else if (s[2] > s[max]) { mid = max; max = 2; }
  else { mid = 2; }
  McolcMcol(R,0,E,max);
  McolcMcol(R,1,E,mid);
  R[0][2] = E[1][max]*E[2][mid] - E[1][mid]*E[2][max];
  R[1][2] = E[0][mid]*E[2][max] - E[0][max]*E[2][mid];
  R[2][2] = E[0][max]*E[1][mid] - E[0][mid]*E[1][max];

  // fit the BV

  b->FitToTris(R, &m->tris[first_tri], num_tris);

  if (num_tris == 1)
  {
    // BV is a leaf BV - first_child will index a triangle

    b->first_child = -(first_tri + 1);
  }
  else if (num_tris > 1)
  {
    // BV not a leaf - first_child will index a BV

    b->first_child = m->num_bvs;
    m->num_bvs+=2;

    // choose splitting axis and splitting coord

    McolcV(axis,R,0);

#if RAPID2_FIT
    mean_from_accum(mean,acc);
#else
    get_centroid_triverts(mean,&m->tris[first_tri],num_tris);
#endif
    coord = VdotV(axis, mean);

    // now split

    int num_first_half = split_tris(&m->tris[first_tri], num_tris, 
                                    axis, coord);

    // recursively build the children

    build_recurse(m, m->child(bn)->first_child, first_tri, num_first_half); 
    build_recurse(m, m->child(bn)->first_child + 1,
                  first_tri + num_first_half, num_tris - num_first_half); 
  }
  return PQP_OK;
}

// this descends the hierarchy, converting world-relative 
// transforms to parent-relative transforms

void 
make_parent_relative(PQP_Model *m, int bn,
                     const PQP_REAL parentR[3][3]
#if PQP_BV_TYPE & RSS_TYPE
                     ,const PQP_REAL parentTr[3]
#endif
#if PQP_BV_TYPE & OBB_TYPE
                     ,const PQP_REAL parentTo[3]
#endif
                    )
{
  PQP_REAL Rpc[3][3], Tpc[3];

  if (!m->child(bn)->Leaf())
  {
    // make children parent-relative

    make_parent_relative(m,m->child(bn)->first_child, 
                         m->child(bn)->R
#if PQP_BV_TYPE & RSS_TYPE
                         ,m->child(bn)->Tr
#endif
#if PQP_BV_TYPE & OBB_TYPE
                         ,m->child(bn)->To
#endif
                         );
    make_parent_relative(m,m->child(bn)->first_child+1, 
                         m->child(bn)->R
#if PQP_BV_TYPE & RSS_TYPE
                         ,m->child(bn)->Tr
#endif
#if PQP_BV_TYPE & OBB_TYPE
                         ,m->child(bn)->To
#endif
                         );
  }

  // make self parent relative

  MTxM(Rpc,parentR,m->child(bn)->R);
  McM(m->child(bn)->R,Rpc);
#if PQP_BV_TYPE & RSS_TYPE
  VmV(Tpc,m->child(bn)->Tr,parentTr);
  MTxV(m->child(bn)->Tr,parentR,Tpc);
#endif
#if PQP_BV_TYPE & OBB_TYPE
  VmV(Tpc,m->child(bn)->To,parentTo);
  MTxV(m->child(bn)->To,parentR,Tpc);
#endif

}

int
build_model(PQP_Model *m)
{
  // set num_bvs to 1, the first index for a child bv

  m->num_bvs = 1;

  // build recursively

  build_recurse(m, 0, 0, m->num_tris);

  // change BV orientations from world-relative to parent-relative

  PQP_REAL R[3][3],T[3];
  Midentity(R);
  Videntity(T);

  make_parent_relative(m,0,R
#if PQP_BV_TYPE & RSS_TYPE
                      ,T
#endif
#if PQP_BV_TYPE & OBB_TYPE
                      ,T
#endif
                      );

  return PQP_OK;
}
