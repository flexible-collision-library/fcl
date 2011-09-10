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

  US Mail:             E. Larsen
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:               geom@cs.unc.edu


\**************************************************************************/

#ifndef PQP_BV_H
#define PQP_BV_H

#include <math.h>
#include "Tri.h"
#include "PQP_Compile.h"

struct BV
{
  PQP_REAL R[3][3];     // orientation of RSS & OBB

#if PQP_BV_TYPE & RSS_TYPE
  PQP_REAL Tr[3];       // position of rectangle
  PQP_REAL l[2];        // side lengths of rectangle
  PQP_REAL r;           // radius of sphere summed with rectangle to form RSS
#endif

#if PQP_BV_TYPE & OBB_TYPE
  PQP_REAL To[3];       // position of obb
  PQP_REAL d[3];        // (half) dimensions of obb
#endif

  int first_child;      // positive value is index of first_child bv
                        // negative value is -(index + 1) of triangle

  BV();
  ~BV();
  int      Leaf()    { return first_child < 0; }
  PQP_REAL GetSize(); 
  void     FitToTris(PQP_REAL O[3][3], Tri *tris, int num_tris);
};

inline
PQP_REAL 
BV::GetSize()
{
#if PQP_BV_TYPE & RSS_TYPE
  return (sqrt(l[0]*l[0] + l[1]*l[1]) + 2*r);
#else
  return (d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
#endif
}

int
BV_Overlap(PQP_REAL R[3][3], PQP_REAL T[3], BV *b1, BV *b2);

#if PQP_BV_TYPE & RSS_TYPE
PQP_REAL
BV_Distance(PQP_REAL R[3][3], PQP_REAL T[3], BV *b1, BV *b2);
#endif

#endif


