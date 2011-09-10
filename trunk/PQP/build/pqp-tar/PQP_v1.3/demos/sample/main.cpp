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
#include <math.h>
#include "PQP.h"

#define PI 3.14159265359
#define LISTS 0

main()
{
  // initialize PQP model pointers

  PQP_Model *b1 = new PQP_Model;
  PQP_Model *b2 = new PQP_Model;
  
  // Add trianges to form tori

  fprintf(stderr, "loading tris into PQP_Model objects...");  fflush(stderr);
  
  PQP_REAL a = (PQP_REAL)1.0;  // major radius of the tori
  PQP_REAL b = (PQP_REAL)0.2;  // minor radius of the tori

  int n1 = 50;     // tori will have n1*n2*2 triangles each
  int n2 = 50;

  int uc, vc;
  int count = 0;
  
  b1->BeginModel();
  b2->BeginModel();
  for(uc=0; uc<n1; uc++)
  {
    for(vc=0; vc<n2; vc++)
    {
      PQP_REAL u1 = (PQP_REAL)(2.0*PI*uc) / n1; 
      PQP_REAL u2 = (PQP_REAL)(2.0*PI*(uc+1)) / n1; 
      PQP_REAL v1 = (PQP_REAL)(2.0*PI*vc) / n2; 
      PQP_REAL v2 = (PQP_REAL)(2.0*PI*(vc+1)) / n2; 

      PQP_REAL p1[3], p2[3], p3[3], p4[3];

      p1[0] = (a - b * cos(v1)) * cos(u1);
      p2[0] = (a - b * cos(v1)) * cos(u2);
      p3[0] = (a - b * cos(v2)) * cos(u1);
      p4[0] = (a - b * cos(v2)) * cos(u2);
      p1[1] = (a - b * cos(v1)) * sin(u1);
      p2[1] = (a - b * cos(v1)) * sin(u2);
      p3[1] = (a - b * cos(v2)) * sin(u1);
      p4[1] = (a - b * cos(v2)) * sin(u2);
      p1[2] = b * sin(v1);
      p2[2] = b * sin(v1);
      p3[2] = b * sin(v2);
      p4[2] = b * sin(v2);

      b1->AddTri(p1, p2, p3, count);
      b1->AddTri(p4, p2, p3, count+1);
      b2->AddTri(p1, p2, p3, count);
      b2->AddTri(p4, p2, p3, count+1);

      count += 2;
    }
  }

  fprintf(stderr, "done\n");  fflush(stderr);
  fprintf(stderr, "Tori have %d triangles each.\n", count);
  fprintf(stderr, "building hierarchies...");  fflush(stderr);
  b1->EndModel();
  b2->EndModel();
  fprintf(stderr, "done.\n"); 
  b1->MemUsage(1);
  b2->MemUsage(1);
  fflush(stderr); 
  
  // now we are free to call the proximity routines.
  // but first, construct the transformations that define the placement
  // of our two hierarchies in world space:

  // this placement causes them to overlap a large amount.

  PQP_REAL R1[3][3], R2[3][3], T1[3], T2[3];
  
  R1[0][0] = R1[1][1] = R1[2][2] = 1.0;
  R1[0][1] = R1[1][0] = R1[2][0] = 0.0;
  R1[0][2] = R1[1][2] = R1[2][1] = 0.0;

  R2[0][0] = R2[1][1] = R2[2][2] = 1.0;
  R2[0][1] = R2[1][0] = R2[2][0] = 0.0;
  R2[0][2] = R2[1][2] = R2[2][1] = 0.0;
  
  T1[0] = 1.0;  T1[1] = 0.0; T1[2] = 0.0;
  T2[0] = 0.0;  T2[1] = 0.0; T2[2] = 0.0;

  // perform a collision query

  PQP_CollideResult cres;
  PQP_Collide(&cres, R1, T1, b1, R2, T2, b2, PQP_ALL_CONTACTS);

  // looking at the report, we can see where all the contacts were, and
  // also how many tests were necessary:

  printf("\nAll contact collision query between overlapping tori:\n");
  printf("Num BV tests: %d\n", cres.NumBVTests());
  printf("Num Tri tests: %d\n", cres.NumTriTests());
  printf("Num contact pairs: %d\n", cres.NumPairs());
#if LISTS
  int i;
  for(i=0; i<cres.NumPairs(); i++)
  {
    printf("\t contact %4d: tri %4d and tri %4d\n",
           i,
           cres.Id1(i),
           cres.Id2(i));
  }
#endif

  // Notice the PQP_ALL_CONTACTS flag we used in the call to PQP_Collide.
  // The alternative is to use the PQP_FIRST_CONTACT flag, instead.
  // The result is that the collide routine searches for any contact,
  // but not all of them.  It can take many many fewer tests to locate a single
  // contact.

  PQP_Collide(&cres, R1, T1, b1, R2, T2, b2, PQP_FIRST_CONTACT);

  printf("\nFirst contact collision query between overlapping tori:\n");
  printf("Num BV tests: %d\n", cres.NumBVTests());
  printf("Num Tri tests: %d\n", cres.NumTriTests());
  printf("Num contact pairs: %d\n", cres.NumPairs());
#if LISTS
  for(i=0; i<cres.NumPairs(); i++)
  {
    printf("\t contact %4d: tri %4d and tri %4d\n", 
           i, 
           cres.Id1(i), 
           cres.Id2(i));
  }
#endif
  
  // Perform a distance query, which should return a distance of 0.0

  PQP_DistanceResult dres;
  PQP_Distance(&dres, R1, T1, b1, R2, T2, b2, 0.0, 0.0);

  printf("\nDistance query between overlapping tori\n");
  printf("Num BV tests: %d\n", dres.NumBVTests());
  printf("Num Tri tests: %d\n", dres.NumTriTests());
  printf("Distance: %lf\n", dres.Distance());

  // by rotating one of them around the x-axis 90 degrees, they 
  // are now interlocked, but not quite touching.

  R1[0][0] = 1.0;  R1[0][1] = 0.0;  R1[0][2] = 0.0;
  R1[1][0] = 0.0;  R1[1][1] = 0.0;  R1[1][2] =-1.0;
  R1[2][0] = 0.0;  R1[2][1] = 1.0;  R1[2][2] = 0.0;
  
  PQP_Collide(&cres, R1, T1, b1, R2, T2, b2, PQP_FIRST_CONTACT);

  printf("\nCollision query between interlocked but nontouching tori:\n");
  printf("Num BV tests: %d\n", cres.NumBVTests());
  printf("Num Tri tests: %d\n", cres.NumTriTests());
  printf("Num contact pairs: %d\n", cres.NumPairs());
#if LISTS
  for(i=0; i<cres.NumPairs(); i++)
  {
    printf("\t contact %4d: tri %4d and tri %4d\n", 
           i, 
           cres.Id1(i), 
           cres.Id2(i));
  }
#endif

  // Perform a distance query - the distance found should be greater than zero

  PQP_Distance(&dres, R1, T1, b1, R2, T2, b2, 0.0, 0.0);

  printf("\nDistance query between interlocked but nontouching tori\n");
  printf("Num BV tests: %d\n", dres.NumBVTests());
  printf("Num Tri tests: %d\n", dres.NumTriTests());
  printf("Distance: %lf\n", dres.Distance());

  // Perform two tolerance queries. One tolerance setting is greater than the 
  // distance between the models, and one tolerance is less than the distance.

  PQP_ToleranceResult tres;
  PQP_REAL tolerance = (PQP_REAL).60;
  PQP_Tolerance(&tres, R1, T1, b1, R2, T2, b2, tolerance);

  printf("\nTolerance query between interlocked but nontouching tori\n"
         "with tolerance %lf\n", tolerance);
  printf("Num BV tests: %d\n", tres.NumBVTests());
  printf("Num Tri tests: %d\n", tres.NumTriTests());
  printf("Closer than tolerance? ",tolerance);
  if (tres.CloserThanTolerance()) printf("yes.\n"); else printf("no.\n");

  tolerance = (PQP_REAL).40;
  PQP_Tolerance(&tres, R1, T1, b1, R2, T2, b2, tolerance);

  printf("\nTolerance query between interlocked but nontouching tori\n"
         "with tolerance %lf\n", tolerance);
  printf("Num BV tests: %d\n", tres.NumBVTests());
  printf("Num Tri tests: %d\n", tres.NumTriTests());
  printf("Closer than tolerance? ",tolerance);
  if (tres.CloserThanTolerance()) printf("yes.\n"); else printf("no.\n");

  // by moving one of the tori closer to the other, they
  // almost touch.  This is the case that requires a lot
  // of work wiht methods which use bounding boxes of limited
  // aspect ratio.  Oriented bounding boxes are more efficient
  // at determining noncontact than spheres, octree, or axis-aligned
  // bounding boxes for scenarios like this.  In this case, the interlocked
  // tori are separated by 0.0001 at their closest point.


  T1[0] = (PQP_REAL)1.5999;
  
  PQP_Collide(&cres, R1, T1, b1, R2, T2, b2, PQP_FIRST_CONTACT);

  printf("\nCollision query on interlocked and almost touching tori:\n");
  printf("Num BV tests: %d\n", cres.NumBVTests());
  printf("Num Tri tests: %d\n", cres.NumTriTests());
  printf("Num contact pairs: %d\n", cres.NumPairs());
#if LISTS
  for(i=0; i<cres.NumPairs(); i++)
  {
    printf("\t contact %4d: tri %4d and tri %4d\n", 
           i, 
           cres.Id1(i), 
           cres.Id2(i));
  }
#endif

  PQP_Distance(&dres, R1, T1, b1, R2, T2, b2, 0.0, 0.0);

  printf("\nDistance query between interlocked and almost touching tori\n");
  printf("Num BV tests: %d\n", dres.NumBVTests());
  printf("Num Tri tests: %d\n", dres.NumTriTests());
  printf("Distance: %lf\n", dres.Distance());

  tolerance = (PQP_REAL)0.00015;
  PQP_Tolerance(&tres, R1, T1, b1, R2, T2, b2, tolerance);

  printf("\nTolerance query between interlocked and almost touching tori\n"
         "with tolerance %lf\n", tolerance);
  printf("Num BV tests: %d\n", tres.NumBVTests());
  printf("Num Tri tests: %d\n", tres.NumTriTests());
  printf("Closer than tolerance? ",tolerance);
  if (tres.CloserThanTolerance()) printf("yes.\n"); else printf("no.\n");

  tolerance = (PQP_REAL)0.00005;
  PQP_Tolerance(&tres, R1, T1, b1, R2, T2, b2, tolerance);

  printf("\nTolerance query between interlocked and almost touching tori\n"
         "with tolerance %lf\n", tolerance);
  printf("Num BV tests: %d\n", tres.NumBVTests());
  printf("Num Tri tests: %d\n", tres.NumTriTests());
  printf("Closer than tolerance? ",tolerance);
  if (tres.CloserThanTolerance()) printf("yes.\n"); else printf("no.\n");

  delete b1;
  delete b2;

  return 0;  
}
