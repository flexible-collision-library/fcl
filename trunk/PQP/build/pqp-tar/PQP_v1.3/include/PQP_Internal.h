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

#include "Tri.h"
#include "BV.h"

class PQP_Model
{

public:

  int build_state;

  Tri *tris;  
  int num_tris;
  int num_tris_alloced;

  BV *b;
  int num_bvs;
  int num_bvs_alloced;

  Tri *last_tri;       // closest tri on this model in last distance test
  
  BV *child(int n) { return &b[n]; }

  PQP_Model();
  ~PQP_Model();

  int BeginModel(int num_tris = 8); // preallocate for num_tris triangles;
                                    // the parameter is optional, since
                                    // arrays are reallocated as needed
  int AddTri(const PQP_REAL *p1, const PQP_REAL *p2, const PQP_REAL *p3, 
             int id);
  int EndModel();
  int MemUsage(int msg);  // returns model mem usage.  
                          // prints message to stderr if msg == TRUE
};

struct CollisionPair
{
  int id1;
  int id2;
};

struct PQP_CollideResult  
{
  // stats

  int num_bv_tests;
  int num_tri_tests;
  double query_time_secs;

  // xform from model 1 to model 2

  PQP_REAL R[3][3];
  PQP_REAL T[3];

  int num_pairs_alloced;
  int num_pairs;
  CollisionPair *pairs;

  void SizeTo(int n);    
  void Add(int i1, int i2); 

  PQP_CollideResult();
  ~PQP_CollideResult();

  // statistics

  int NumBVTests() { return num_bv_tests; }
  int NumTriTests() { return num_tri_tests; }
  double QueryTimeSecs() { return query_time_secs; }

  // free the list of contact pairs; ordinarily this list is reused
  // for each query, and only deleted in the destructor.

  void FreePairsList(); 

  // query results

  int Colliding() { return (num_pairs > 0); }
  int NumPairs() { return num_pairs; }
  int Id1(int k) { return pairs[k].id1; }
  int Id2(int k) { return pairs[k].id2; }
};

#if PQP_BV_TYPE & RSS_TYPE // distance/tolerance are only available with RSS

struct PQP_DistanceResult 
{
  // stats

  int num_bv_tests;
  int num_tri_tests;
  double query_time_secs;

  // xform from model 1 to model 2

  PQP_REAL R[3][3];
  PQP_REAL T[3];

  PQP_REAL rel_err; 
  PQP_REAL abs_err; 

  PQP_REAL distance;
  PQP_REAL p1[3]; 
  PQP_REAL p2[3];
  int qsize;
  
  // statistics

  int NumBVTests() { return num_bv_tests; }
  int NumTriTests() { return num_tri_tests; }
  double QueryTimeSecs() { return query_time_secs; }

  // The following distance and points established the minimum distance
  // for the models, within the relative and absolute error bounds 
  // specified.
  // Points are defined: PQP_REAL p1[3], p2[3];

  PQP_REAL Distance() { return distance; }
  const PQP_REAL *P1() { return p1; }
  const PQP_REAL *P2() { return p2; }
};

struct PQP_ToleranceResult 
{
  // stats

  int num_bv_tests;
  int num_tri_tests;
  double query_time_secs;

  // xform from model 1 to model 2

  PQP_REAL R[3][3];
  PQP_REAL T[3];

  int    closer_than_tolerance;   
  PQP_REAL tolerance;      

  PQP_REAL distance;
  PQP_REAL p1[3]; 
  PQP_REAL p2[3]; 
  int qsize;

  // statistics

  int NumBVTests() { return num_bv_tests; }
  int NumTriTests() { return num_tri_tests; }
  double QueryTimeSecs() { return query_time_secs; }

  // If the models are closer than ( <= ) tolerance, these points 
  // and distance were what established this.  Otherwise, 
  // distance and point values are not meaningful.

  PQP_REAL Distance() { return distance; }
  const PQP_REAL *P1() { return p1; }
  const PQP_REAL *P2() { return p2; }

  // boolean says whether models are closer than tolerance distance

  int CloserThanTolerance() { return closer_than_tolerance; }
};

#endif
