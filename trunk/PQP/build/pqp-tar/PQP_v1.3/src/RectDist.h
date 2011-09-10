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

#ifndef PQP_RECTDIST_H
#define PQP_RECTDIST_H

#include <math.h>
#include "MatVec.h" 
#include "PQP_Compile.h"
  
// ClipToRange
//
// clips val between a and b

inline 
void
ClipToRange(PQP_REAL &val, const PQP_REAL &a, const PQP_REAL &b)
{
  if (val < a) val = a;
  else if (val > b) val = b;
}

// SegCoords
//
// finds the parameters t & u corresponding to the two closest points 
// on a pair of line segments 
//
// The first segment is defined as 
//
// Pa + A*t, 0 <= t <= a, 
// 
// where "Pa" is one endpoint of the segment, "A" is a unit vector 
// pointing to the other endpoint, and t is a scalar that produces
// all the points between the two endpoints. Since "A" is a unit
// vector, "a" is the segment's length.
//
// The second segment is 
//
// Pb + B*u, 0 <= u <= b
//
// In my application, many of the terms needed by the algorithm
// are already computed for other purposes, so I pass these terms to 
// the function instead of complete specifications of each segment. 
// "T" in the dot products is the vector between Pa and Pb.
//
// The algorithm is from
//
// Vladimir J. Lumelsky,
// On fast computation of distance between line segments.
// In Information Processing Letters, no. 21, pages 55-61, 1985.   

inline
void 
SegCoords(PQP_REAL& t, PQP_REAL& u, 
          const PQP_REAL& a, const PQP_REAL& b, 
          const PQP_REAL& A_dot_B, 
          const PQP_REAL& A_dot_T, 
          const PQP_REAL& B_dot_T)
{  
  PQP_REAL denom = 1 - (A_dot_B)*(A_dot_B);

  if (denom == 0) t = 0;
  else
  {
    t = (A_dot_T - B_dot_T*A_dot_B)/denom;
    ClipToRange(t,0,a);
  }
  
  u = t*A_dot_B - B_dot_T;
  if (u < 0) 
  {
    u = 0;
    t = A_dot_T;
    ClipToRange(t,0,a);
  }
  else if (u > b) 
  {
    u = b;
    t = u*A_dot_B + A_dot_T;
    ClipToRange(t,0,a);
  }
}

// InVoronoi
//
// returns whether the nearest point on rectangle edge 
// Pb + B*u, 0 <= u <= b, to the rectangle edge,
// Pa + A*t, 0 <= t <= a, is within the half space 
// determined by the point Pa and the direction Anorm.
//
// A,B, and Anorm are unit vectors.
// T is the vector between Pa and Pb.

inline
int 
InVoronoi(const PQP_REAL &a, 
          const PQP_REAL &b,  
          const PQP_REAL &Anorm_dot_B, 
          const PQP_REAL &Anorm_dot_T,  
          const PQP_REAL &A_dot_B,
          const PQP_REAL &A_dot_T,
          const PQP_REAL &B_dot_T)
{ 
  if (myfabs(Anorm_dot_B) < 1e-7) return 0;

  PQP_REAL t, u, v;
 
  u = -Anorm_dot_T / Anorm_dot_B; 
  ClipToRange(u,0,b); 
  
  t = u*A_dot_B + A_dot_T; 
  ClipToRange(t,0,a); 
  
  v = t*A_dot_B - B_dot_T; 
  
  if (Anorm_dot_B > 0) 
  {
    if (v > (u + 1e-7)) return 1;
  }
  else 
  {
    if (v < (u - 1e-7)) return 1;
  }
  return 0; 
} 


// RectDist
//
// Finds the distance between two rectangles A and B.  A is assumed
// to have its corner on the origin, one side aligned with
// x, the other side aligned with y, and its normal aligned with z.
// 
// [Rab,Tab] gives the orientation and corner position of rectangle B
// 
// a[2] are the side lengths of A, b[2] are the side lengths of B

inline
PQP_REAL
RectDist(PQP_REAL Rab[3][3], PQP_REAL Tab[3], 
          PQP_REAL a[2], PQP_REAL b[2])
{
  PQP_REAL A0_dot_B0, A0_dot_B1, A1_dot_B0, A1_dot_B1;

  A0_dot_B0 = Rab[0][0];
  A0_dot_B1 = Rab[0][1];
  A1_dot_B0 = Rab[1][0];
  A1_dot_B1 = Rab[1][1];

  PQP_REAL aA0_dot_B0, aA0_dot_B1, aA1_dot_B0, aA1_dot_B1;
  PQP_REAL bA0_dot_B0, bA0_dot_B1, bA1_dot_B0, bA1_dot_B1; 
 
  aA0_dot_B0 = a[0]*A0_dot_B0;
  aA0_dot_B1 = a[0]*A0_dot_B1;
  aA1_dot_B0 = a[1]*A1_dot_B0;
  aA1_dot_B1 = a[1]*A1_dot_B1;
  bA0_dot_B0 = b[0]*A0_dot_B0;
  bA1_dot_B0 = b[0]*A1_dot_B0;
  bA0_dot_B1 = b[1]*A0_dot_B1;
  bA1_dot_B1 = b[1]*A1_dot_B1;

  PQP_REAL Tba[3];
  MTxV(Tba,Rab,Tab);

  PQP_REAL S[3], t, u;

  // determine if any edge pair contains the closest points

  PQP_REAL ALL_x, ALU_x, AUL_x, AUU_x;
  PQP_REAL BLL_x, BLU_x, BUL_x, BUU_x;
  PQP_REAL LA1_lx, LA1_ux, UA1_lx, UA1_ux, LB1_lx, LB1_ux, UB1_lx, UB1_ux;

  ALL_x = -Tba[0];
  ALU_x = ALL_x + aA1_dot_B0;
  AUL_x = ALL_x + aA0_dot_B0;
  AUU_x = ALU_x + aA0_dot_B0;

  if (ALL_x < ALU_x)
  { 
    LA1_lx = ALL_x;
    LA1_ux = ALU_x;
    UA1_lx = AUL_x;    
    UA1_ux = AUU_x;
  }
  else
  { 
    LA1_lx = ALU_x;
    LA1_ux = ALL_x;
    UA1_lx = AUU_x;    
    UA1_ux = AUL_x;
  }

  BLL_x = Tab[0];
  BLU_x = BLL_x + bA0_dot_B1;
  BUL_x = BLL_x + bA0_dot_B0;
  BUU_x = BLU_x + bA0_dot_B0;
  
  if (BLL_x < BLU_x)
  { 
    LB1_lx = BLL_x;
    LB1_ux = BLU_x;
    UB1_lx = BUL_x;    
    UB1_ux = BUU_x;
  }
  else
  { 
    LB1_lx = BLU_x;
    LB1_ux = BLL_x;
    UB1_lx = BUU_x;    
    UB1_ux = BUL_x;
  }

  // UA1, UB1
  
  if ((UA1_ux > b[0]) && (UB1_ux > a[0]))
  {
    if (((UA1_lx > b[0]) || 
          InVoronoi(b[1],a[1],A1_dot_B0,aA0_dot_B0 - b[0] - Tba[0],
		                A1_dot_B1, aA0_dot_B1 - Tba[1], 
                    -Tab[1] - bA1_dot_B0))
        &&
	
        ((UB1_lx > a[0]) || 
          InVoronoi(a[1],b[1],A0_dot_B1,Tab[0] + bA0_dot_B0 - a[0],
                    A1_dot_B1,Tab[1] + bA1_dot_B0,Tba[1] - aA0_dot_B1)))
    {            
      SegCoords(t,u,a[1],b[1],A1_dot_B1,Tab[1] + bA1_dot_B0,
                Tba[1] - aA0_dot_B1);
      
      S[0] = Tab[0] + Rab[0][0]*b[0] + Rab[0][1]*u - a[0] ;
      S[1] = Tab[1] + Rab[1][0]*b[0] + Rab[1][1]*u - t;
      S[2] = Tab[2] + Rab[2][0]*b[0] + Rab[2][1]*u;
      return sqrt(VdotV(S,S));
    }    
  }


  // UA1, LB1

  if ((UA1_lx < 0) && (LB1_ux > a[0]))
  {
    if (((UA1_ux < 0) ||
          InVoronoi(b[1],a[1],-A1_dot_B0,Tba[0] - aA0_dot_B0,
                    A1_dot_B1, aA0_dot_B1 - Tba[1], -Tab[1]))
        &&

        ((LB1_lx > a[0]) ||
          InVoronoi(a[1],b[1],A0_dot_B1,Tab[0] - a[0],
                    A1_dot_B1,Tab[1],Tba[1] - aA0_dot_B1)))
    {
      SegCoords(t,u,a[1],b[1],A1_dot_B1,Tab[1],Tba[1] - aA0_dot_B1);

      S[0] = Tab[0] + Rab[0][1]*u - a[0];
      S[1] = Tab[1] + Rab[1][1]*u - t;
      S[2] = Tab[2] + Rab[2][1]*u;
      return sqrt(VdotV(S,S));
    }
  }

  // LA1, UB1

  if ((LA1_ux > b[0]) && (UB1_lx < 0))
  {
    if (((LA1_lx > b[0]) || 
          InVoronoi(b[1],a[1],A1_dot_B0,-Tba[0] - b[0],
                    A1_dot_B1,-Tba[1], -Tab[1] - bA1_dot_B0))
          &&
	
        ((UB1_ux < 0) || 
          InVoronoi(a[1],b[1],-A0_dot_B1, -Tab[0] - bA0_dot_B0,
                    A1_dot_B1, Tab[1] + bA1_dot_B0,Tba[1])))
    {

      SegCoords(t,u,a[1],b[1],A1_dot_B1,Tab[1] + bA1_dot_B0,Tba[1]);

      S[0] = Tab[0] + Rab[0][0]*b[0] + Rab[0][1]*u;
      S[1] = Tab[1] + Rab[1][0]*b[0] + Rab[1][1]*u - t;
      S[2] = Tab[2] + Rab[2][0]*b[0] + Rab[2][1]*u;
      return sqrt(VdotV(S,S));
    }
  }

  // LA1, LB1

  if ((LA1_lx < 0) && (LB1_lx < 0))
  {   
    if (((LA1_ux < 0) || 
          InVoronoi(b[1],a[1],-A1_dot_B0,Tba[0],A1_dot_B1,
                    -Tba[1],-Tab[1]))
          &&

        ((LB1_ux < 0) || 
          InVoronoi(a[1],b[1],-A0_dot_B1,-Tab[0],A1_dot_B1,
                    Tab[1], Tba[1])))
    {
      SegCoords(t,u,a[1],b[1],A1_dot_B1,Tab[1],Tba[1]);    

      S[0] = Tab[0] + Rab[0][1]*u;
      S[1] = Tab[1] + Rab[1][1]*u - t;
      S[2] = Tab[2] + Rab[2][1]*u;
      return sqrt(VdotV(S,S));
    }
  }

  PQP_REAL ALL_y, ALU_y, AUL_y, AUU_y;

  ALL_y = -Tba[1];
  ALU_y = ALL_y + aA1_dot_B1;
  AUL_y = ALL_y + aA0_dot_B1;
  AUU_y = ALU_y + aA0_dot_B1;
  
  PQP_REAL LA1_ly, LA1_uy, UA1_ly, UA1_uy, LB0_lx, LB0_ux, UB0_lx, UB0_ux;

  if (ALL_y < ALU_y)
  { 
    LA1_ly = ALL_y;
    LA1_uy = ALU_y;
    UA1_ly = AUL_y;    
    UA1_uy = AUU_y;
  }
  else
  { 
    LA1_ly = ALU_y;
    LA1_uy = ALL_y;
    UA1_ly = AUU_y;    
    UA1_uy = AUL_y;
  }

  if (BLL_x < BUL_x)
  {
    LB0_lx = BLL_x;
    LB0_ux = BUL_x;
    UB0_lx = BLU_x;
    UB0_ux = BUU_x;
  }
  else
  {
    LB0_lx = BUL_x;
    LB0_ux = BLL_x;
    UB0_lx = BUU_x;
    UB0_ux = BLU_x;
  }

  // UA1, UB0

  if ((UA1_uy > b[1]) && (UB0_ux > a[0]))
  {   
    if (((UA1_ly > b[1]) || 
          InVoronoi(b[0],a[1],A1_dot_B1, aA0_dot_B1 - Tba[1] - b[1],
                    A1_dot_B0, aA0_dot_B0 - Tba[0], -Tab[1] - bA1_dot_B1))
          &&
	
        ((UB0_lx > a[0]) || 
          InVoronoi(a[1],b[0],A0_dot_B0, Tab[0] - a[0] + bA0_dot_B1,
                    A1_dot_B0, Tab[1] + bA1_dot_B1, Tba[0] - aA0_dot_B0)))
    {
      SegCoords(t,u,a[1],b[0],A1_dot_B0,Tab[1] + bA1_dot_B1,
                Tba[0] - aA0_dot_B0);

      S[0] = Tab[0] + Rab[0][1]*b[1] + Rab[0][0]*u - a[0] ;
      S[1] = Tab[1] + Rab[1][1]*b[1] + Rab[1][0]*u - t;
      S[2] = Tab[2] + Rab[2][1]*b[1] + Rab[2][0]*u;
      return sqrt(VdotV(S,S));
    }
  }

  // UA1, LB0

  if ((UA1_ly < 0) && (LB0_ux > a[0]))
  {
    if (((UA1_uy < 0) || 
          InVoronoi(b[0],a[1],-A1_dot_B1, Tba[1] - aA0_dot_B1,A1_dot_B0,
                    aA0_dot_B0 - Tba[0], -Tab[1]))
          &&

        ((LB0_lx > a[0]) || 
          InVoronoi(a[1],b[0],A0_dot_B0,Tab[0] - a[0],
                    A1_dot_B0,Tab[1],Tba[0] - aA0_dot_B0)))
    {
      SegCoords(t,u,a[1],b[0],A1_dot_B0,Tab[1],Tba[0] - aA0_dot_B0);

      S[0] = Tab[0] + Rab[0][0]*u - a[0];
      S[1] = Tab[1] + Rab[1][0]*u - t;
      S[2] = Tab[2] + Rab[2][0]*u;
      return sqrt(VdotV(S,S)); 
    }
  }

  // LA1, UB0

  if ((LA1_uy > b[1]) && (UB0_lx < 0))
  {
    if (((LA1_ly > b[1]) || 
        InVoronoi(b[0],a[1],A1_dot_B1,-Tba[1] - b[1],
                  A1_dot_B0, -Tba[0], -Tab[1] - bA1_dot_B1))     
        &&

        ((UB0_ux < 0) ||             
          InVoronoi(a[1],b[0],-A0_dot_B0, -Tab[0] - bA0_dot_B1,A1_dot_B0,
                    Tab[1] + bA1_dot_B1,Tba[0])))
    {
      SegCoords(t,u,a[1],b[0],A1_dot_B0,Tab[1] + bA1_dot_B1,Tba[0]);

      S[0] = Tab[0] + Rab[0][1]*b[1] + Rab[0][0]*u;
      S[1] = Tab[1] + Rab[1][1]*b[1] + Rab[1][0]*u - t;
      S[2] = Tab[2] + Rab[2][1]*b[1] + Rab[2][0]*u;
      return sqrt(VdotV(S,S));
    }
  }

  // LA1, LB0

  if ((LA1_ly < 0) && (LB0_lx < 0))
  {
    if (((LA1_uy < 0) || 
          InVoronoi(b[0],a[1],-A1_dot_B1,Tba[1],A1_dot_B0,
                    -Tba[0],-Tab[1]))
        && 

        ((LB0_ux < 0) || 
          InVoronoi(a[1],b[0],-A0_dot_B0,-Tab[0],A1_dot_B0,
                    Tab[1],Tba[0])))
    {
      SegCoords(t,u,a[1],b[0],A1_dot_B0,Tab[1],Tba[0]);
	
      S[0] = Tab[0] + Rab[0][0]*u;
      S[1] = Tab[1] + Rab[1][0]*u - t;
      S[2] = Tab[2] + Rab[2][0]*u;
      return sqrt(VdotV(S,S));
    }
  }

  PQP_REAL BLL_y, BLU_y, BUL_y, BUU_y;

  BLL_y = Tab[1];
  BLU_y = BLL_y + bA1_dot_B1;
  BUL_y = BLL_y + bA1_dot_B0;
  BUU_y = BLU_y + bA1_dot_B0;

  PQP_REAL LA0_lx, LA0_ux, UA0_lx, UA0_ux, LB1_ly, LB1_uy, UB1_ly, UB1_uy;

  if (ALL_x < AUL_x)
  {
    LA0_lx = ALL_x;
    LA0_ux = AUL_x;
    UA0_lx = ALU_x;
    UA0_ux = AUU_x;
  }
  else
  {
    LA0_lx = AUL_x;
    LA0_ux = ALL_x;
    UA0_lx = AUU_x;
    UA0_ux = ALU_x;
  }

  if (BLL_y < BLU_y)
  {
    LB1_ly = BLL_y;
    LB1_uy = BLU_y;
    UB1_ly = BUL_y;
    UB1_uy = BUU_y;
  }
  else
  {
    LB1_ly = BLU_y;
    LB1_uy = BLL_y;
    UB1_ly = BUU_y;
    UB1_uy = BUL_y;
  }
    
  // UA0, UB1
  
  if ((UA0_ux > b[0]) && (UB1_uy > a[1]))
  {
    if (((UA0_lx > b[0]) || 
          InVoronoi(b[1],a[0],A0_dot_B0, aA1_dot_B0 - Tba[0] - b[0],
                    A0_dot_B1,aA1_dot_B1 - Tba[1], -Tab[0] - bA0_dot_B0))
        &&
	
        ((UB1_ly > a[1]) || 
          InVoronoi(a[0],b[1],A1_dot_B1, Tab[1] - a[1] + bA1_dot_B0,
                    A0_dot_B1,Tab[0] + bA0_dot_B0, Tba[1] - aA1_dot_B1)))
    {
      SegCoords(t,u,a[0],b[1],A0_dot_B1,Tab[0] + bA0_dot_B0,
                Tba[1] - aA1_dot_B1);
    
      S[0] = Tab[0] + Rab[0][0]*b[0] + Rab[0][1]*u - t;
      S[1] = Tab[1] + Rab[1][0]*b[0] + Rab[1][1]*u - a[1];
      S[2] = Tab[2] + Rab[2][0]*b[0] + Rab[2][1]*u;
      return sqrt(VdotV(S,S));
    }
  }

  // UA0, LB1

  if ((UA0_lx < 0) && (LB1_uy > a[1]))
  {
    if (((UA0_ux < 0) || 
          InVoronoi(b[1],a[0],-A0_dot_B0, Tba[0] - aA1_dot_B0,A0_dot_B1,
                    aA1_dot_B1 - Tba[1],-Tab[0]))
        &&

        ((LB1_ly > a[1]) || 
          InVoronoi(a[0],b[1],A1_dot_B1,Tab[1] - a[1],A0_dot_B1,Tab[0],
                    Tba[1] - aA1_dot_B1)))
    {
      SegCoords(t,u,a[0],b[1],A0_dot_B1,Tab[0],Tba[1] - aA1_dot_B1);

      S[0] = Tab[0] + Rab[0][1]*u - t;
      S[1] = Tab[1] + Rab[1][1]*u - a[1];
      S[2] = Tab[2] + Rab[2][1]*u;
      return sqrt(VdotV(S,S));
    }
  }

  // LA0, UB1

  if ((LA0_ux > b[0]) && (UB1_ly < 0))
  {
    if (((LA0_lx > b[0]) || 
          InVoronoi(b[1],a[0],A0_dot_B0,-b[0] - Tba[0],A0_dot_B1,-Tba[1],
                    -bA0_dot_B0 - Tab[0]))
        &&

        ((UB1_uy < 0) || 
          InVoronoi(a[0],b[1],-A1_dot_B1, -Tab[1] - bA1_dot_B0,A0_dot_B1,
                    Tab[0] + bA0_dot_B0,Tba[1])))
    {
      SegCoords(t,u,a[0],b[1],A0_dot_B1,Tab[0] + bA0_dot_B0,Tba[1]);

      S[0] = Tab[0] + Rab[0][0]*b[0] + Rab[0][1]*u - t;
      S[1] = Tab[1] + Rab[1][0]*b[0] + Rab[1][1]*u;
      S[2] = Tab[2] + Rab[2][0]*b[0] + Rab[2][1]*u;
      return sqrt(VdotV(S,S));
    }
  }
  
  // LA0, LB1

  if ((LA0_lx < 0) && (LB1_ly < 0))
  {
    if (((LA0_ux < 0) || 
          InVoronoi(b[1],a[0],-A0_dot_B0,Tba[0],A0_dot_B1,-Tba[1],
                    -Tab[0]))
        &&
	
        ((LB1_uy < 0) || 
          InVoronoi(a[0],b[1],-A1_dot_B1,-Tab[1],A0_dot_B1,
                    Tab[0],Tba[1])))
    {
      SegCoords(t,u,a[0],b[1],A0_dot_B1,Tab[0],Tba[1]);

      S[0] = Tab[0] + Rab[0][1]*u - t;
      S[1] = Tab[1] + Rab[1][1]*u;
      S[2] = Tab[2] + Rab[2][1]*u;
      return sqrt(VdotV(S,S));
    }
  }

  PQP_REAL LA0_ly, LA0_uy, UA0_ly, UA0_uy, LB0_ly, LB0_uy, UB0_ly, UB0_uy;

  if (ALL_y < AUL_y)
  {
    LA0_ly = ALL_y;
    LA0_uy = AUL_y;
    UA0_ly = ALU_y;
    UA0_uy = AUU_y;
  }
  else
  {
    LA0_ly = AUL_y;
    LA0_uy = ALL_y;
    UA0_ly = AUU_y;
    UA0_uy = ALU_y;
  }

  if (BLL_y < BUL_y)
  {
    LB0_ly = BLL_y;
    LB0_uy = BUL_y;
    UB0_ly = BLU_y;
    UB0_uy = BUU_y;
  }
  else
  {
    LB0_ly = BUL_y;
    LB0_uy = BLL_y;
    UB0_ly = BUU_y;
    UB0_uy = BLU_y;
  }

  // UA0, UB0

  if ((UA0_uy > b[1]) && (UB0_uy > a[1]))
  {
    if (((UA0_ly > b[1]) || 
          InVoronoi(b[0],a[0],A0_dot_B1, aA1_dot_B1 - Tba[1] - b[1],
                    A0_dot_B0, aA1_dot_B0 - Tba[0], -Tab[0] - bA0_dot_B1))
        &&
	
        ((UB0_ly > a[1]) || 
          InVoronoi(a[0],b[0],A1_dot_B0,Tab[1] - a[1] + bA1_dot_B1,A0_dot_B0,
                    Tab[0] + bA0_dot_B1, Tba[0] - aA1_dot_B0)))
    {
      SegCoords(t,u,a[0],b[0],A0_dot_B0,Tab[0] + bA0_dot_B1,
                Tba[0] - aA1_dot_B0);
      
      S[0] = Tab[0] + Rab[0][1]*b[1] + Rab[0][0]*u - t;
      S[1] = Tab[1] + Rab[1][1]*b[1] + Rab[1][0]*u - a[1];
      S[2] = Tab[2] + Rab[2][1]*b[1] + Rab[2][0]*u;
      return sqrt(VdotV(S,S));
    }
  }

  // UA0, LB0

  if ((UA0_ly < 0) && (LB0_uy > a[1]))
  {
    if (((UA0_uy < 0) || 
          InVoronoi(b[0],a[0],-A0_dot_B1,Tba[1] - aA1_dot_B1,A0_dot_B0,
                    aA1_dot_B0 - Tba[0],-Tab[0]))
        &&      

        ((LB0_ly > a[1]) || 
          InVoronoi(a[0],b[0],A1_dot_B0,Tab[1] - a[1],
                    A0_dot_B0,Tab[0],Tba[0] - aA1_dot_B0)))
    {
      SegCoords(t,u,a[0],b[0],A0_dot_B0,Tab[0],Tba[0] - aA1_dot_B0);

      S[0] = Tab[0] + Rab[0][0]*u - t;
      S[1] = Tab[1] + Rab[1][0]*u - a[1];
      S[2] = Tab[2] + Rab[2][0]*u;
      return sqrt(VdotV(S,S));
    }
  }

  // LA0, UB0

  if ((LA0_uy > b[1]) && (UB0_ly < 0))
  {  
    if (((LA0_ly > b[1]) ||
          InVoronoi(b[0],a[0],A0_dot_B1,-Tba[1] - b[1], A0_dot_B0,-Tba[0],
                    -Tab[0] - bA0_dot_B1))
        &&
	
        ((UB0_uy < 0) ||
          InVoronoi(a[0],b[0],-A1_dot_B0, -Tab[1] - bA1_dot_B1, A0_dot_B0,
                    Tab[0] + bA0_dot_B1,Tba[0])))
    {
      SegCoords(t,u,a[0],b[0],A0_dot_B0,Tab[0] + bA0_dot_B1,Tba[0]);
      
      S[0] = Tab[0] + Rab[0][1]*b[1] + Rab[0][0]*u - t;
      S[1] = Tab[1] + Rab[1][1]*b[1] + Rab[1][0]*u;
      S[2] = Tab[2] + Rab[2][1]*b[1] + Rab[2][0]*u;
      return sqrt(VdotV(S,S));
    }
  }

  // LA0, LB0

  if ((LA0_ly < 0) && (LB0_ly < 0))
  {   
    if (((LA0_uy < 0) || 
          InVoronoi(b[0],a[0],-A0_dot_B1,Tba[1],A0_dot_B0,
                    -Tba[0],-Tab[0]))
        &&
	
        ((LB0_uy < 0) || 
          InVoronoi(a[0],b[0],-A1_dot_B0,-Tab[1],A0_dot_B0,
                    Tab[0],Tba[0])))
    {
      SegCoords(t,u,a[0],b[0],A0_dot_B0,Tab[0],Tba[0]);

      S[0] = Tab[0] + Rab[0][0]*u - t;
      S[1] = Tab[1] + Rab[1][0]*u;
      S[2] = Tab[2] + Rab[2][0]*u;
      return sqrt(VdotV(S,S));
    }
  }

  // no edges passed, take max separation along face normals

  PQP_REAL sep1, sep2;
 
  if (Tab[2] > 0.0)
  {
    sep1 = Tab[2];
    if (Rab[2][0] < 0.0) sep1 += b[0]*Rab[2][0];
    if (Rab[2][1] < 0.0) sep1 += b[1]*Rab[2][1];
  }
  else
  {
    sep1 = -Tab[2];
    if (Rab[2][0] > 0.0) sep1 -= b[0]*Rab[2][0];
    if (Rab[2][1] > 0.0) sep1 -= b[1]*Rab[2][1];
  }
  
  if (Tba[2] < 0)
  {
    sep2 = -Tba[2];
    if (Rab[0][2] < 0.0) sep2 += a[0]*Rab[0][2];
    if (Rab[1][2] < 0.0) sep2 += a[1]*Rab[1][2];
  }
  else
  {
    sep2 = Tba[2];
    if (Rab[0][2] > 0.0) sep2 -= a[0]*Rab[0][2];
    if (Rab[1][2] > 0.0) sep2 -= a[1]*Rab[1][2];
  }

  PQP_REAL sep = (sep1 > sep2? sep1 : sep2);
  return (sep > 0? sep : 0);
}

#endif
