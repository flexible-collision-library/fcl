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

#ifndef PQP_COMPILE_H
#define PQP_COMPILE_H

// prevents compiler warnings when PQP_REAL is float

#include <math.h>
inline float sqrt(float x) { return (float)sqrt((double)x); }
inline float cos(float x) { return (float)cos((double)x); }
inline float sin(float x) { return (float)sin((double)x); }
inline float fabs(float x) { return (float)fabs((double)x); }

//-------------------------------------------------------------------------
//
// PQP_REAL
//
// This is the floating point type used throughout PQP.  doubles are
// recommended, both for their precision and because the software has
// mainly been tested using them.  However, floats appear to be faster 
// (by 60% on some machines).
//
//-------------------------------------------------------------------------

typedef double PQP_REAL;

//-------------------------------------------------------------------------
//
// PQP_BV_TYPE
//
// PQP introduces a bounding volume (BV) type known as the "rectangle
// swept sphere" (RSS) - the volume created by sweeping a sphere so
// that its center visits every point on a rectangle; it looks
// something like a rounded box.
//
// In our experiments, the RSS type is comparable to the oriented 
// bounding box (OBB) in terms of the number of BV-pair and triangle-pair 
// tests incurred.  However, with our present implementations, overlap 
// tests are cheaper for OBBs, while distance tests are cheaper for the 
// RSS type (we used a public gjk implementation for the OBB distance test).
//
// Consequently, PQP is configured to use the RSS type in distance and 
// tolerance queries (which use BV distance tests) and to use OBBs for
// collision queries (which use BV overlap tests). Using both requires six
// more PQP_REALs per BV node than using just one type. 
//
// To save space, you can configure PQP to use only one type, however, 
// with RSS alone, collision queries will typically be slower.  With OBB's 
// alone, distance and tolerance queries are currently not supported, since 
// we have not developed our own OBB distance test.  The three options are:
//
// #define PQP_BV_TYPE  RSS_TYPE           
// #define PQP_BV_TYPE  OBB_TYPE           
// #define PQP_BV_TYPE  RSS_TYPE | OBB_TYPE
//
//-------------------------------------------------------------------------

#define RSS_TYPE     1
#define OBB_TYPE     2

#define PQP_BV_TYPE  RSS_TYPE | OBB_TYPE

#endif
