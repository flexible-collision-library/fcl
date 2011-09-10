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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "GL/glut.h"
#include "model.h"

inline
void
VmV(double Vr[3], const double V1[3], const double V2[3])
{
  Vr[0] = V1[0] - V2[0];
  Vr[1] = V1[1] - V2[1];
  Vr[2] = V1[2] - V2[2];
}

inline
void
VcrossV(double Vr[3], const double V1[3], const double V2[3])
{
  Vr[0] = V1[1]*V2[2] - V1[2]*V2[1];
  Vr[1] = V1[2]*V2[0] - V1[0]*V2[2];
  Vr[2] = V1[0]*V2[1] - V1[1]*V2[0];
}

inline
void
Vnormalize(double V[3])
{
  double d = 1.0 / sqrt(V[0]*V[0] + V[1]*V[1] + V[2]*V[2]);
  V[0] *= d;
  V[1] *= d;
  V[2] *= d;
}

Model::Model(char *tris_file)
{
  FILE *fp = fopen(tris_file,"r");
  if (fp == NULL)
  { 
    fprintf(stderr,"Model Constructor: Couldn't open %s\n",tris_file); 
    exit(-1); 
  }

  fscanf(fp,"%d",&ntris);
  tri = new ModelTri[ntris];

  int i;

  for (i = 0; i < ntris; i++)
  {
    // read the tri verts

    fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
           &tri[i].p0[0], &tri[i].p0[1], &tri[i].p0[2],
           &tri[i].p1[0], &tri[i].p1[1], &tri[i].p1[2],
           &tri[i].p2[0], &tri[i].p2[1], &tri[i].p2[2]);

    // set the normal

    double a[3],b[3];
    VmV(a,tri[i].p1,tri[i].p0);
    VmV(b,tri[i].p2,tri[i].p0);
    VcrossV(tri[i].n,a,b);
    Vnormalize(tri[i].n);
  }
  
  fclose(fp);

  // generate display list

  display_list = glGenLists(1);
  glNewList(display_list,GL_COMPILE);
  glBegin(GL_TRIANGLES);
  for (i = 0; i < ntris; i++)
  {
    glNormal3dv(tri[i].n);
    glVertex3dv(tri[i].p0);
    glVertex3dv(tri[i].p1);
    glVertex3dv(tri[i].p2);
  }
  glEnd();
  glEndList();  
}

Model::~Model()
{
  delete [] tri;
}

void
Model::Draw()
{
  glCallList(display_list);
}

void
Model::DrawTri(int index)
{
  glBegin(GL_TRIANGLES);
  glVertex3dv(tri[index].p0);
  glVertex3dv(tri[index].p1);
  glVertex3dv(tri[index].p2);
  glEnd();
}
