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
#include <GL/glut.h>
#include "PQP.h"
#include "model.h"
#include "MatVec.h"

PQP_Model bunny, torus;
Model *bunny_to_draw, *torus_to_draw;

int mode;
double beginx, beginy;
double dis = 10.0, azim = 0.0, elev = 0.0;
double ddis = 0.0, dazim = 0.0, delev = 0.0;
double rot1 = 0.0, rot2 = 0.0, rot3 = 0.0;
int animate = 0;

void
InitViewerWindow()
{
  GLfloat Ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };  
  GLfloat Diffuse[] = { 0.8f, 0.8f, 0.8f, 1.0f };  
  GLfloat Specular[] = { 0.2f, 0.2f, 0.2f, 1.0f };   
  GLfloat SpecularExp[] = { 50 };              
  GLfloat Emission[] = { 0.1f, 0.1f, 0.1f, 1.0f };

  glMaterialfv(GL_FRONT, GL_AMBIENT, Ambient);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, Diffuse);
  glMaterialfv(GL_FRONT, GL_SPECULAR, Specular);
  glMaterialfv(GL_FRONT, GL_SHININESS, SpecularExp);
  glMaterialfv(GL_FRONT, GL_EMISSION, Emission);

  glMaterialfv(GL_BACK, GL_AMBIENT, Ambient);
  glMaterialfv(GL_BACK, GL_DIFFUSE, Diffuse);
  glMaterialfv(GL_BACK, GL_SPECULAR, Specular);
  glMaterialfv(GL_BACK, GL_SHININESS, SpecularExp);
  glMaterialfv(GL_BACK, GL_EMISSION, Emission);

  glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
	
  glEnable(GL_COLOR_MATERIAL);

  GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHTING);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

  glDepthFunc(GL_LEQUAL);
  glEnable(GL_DEPTH_TEST);

  glShadeModel(GL_FLAT);
  glClearColor(0.0, 0.0, 0.0, 0.0);

  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glEnable(GL_NORMALIZE);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glFrustum(-0.004,0.004,-0.004,0.004,.01,100.0);

  glMatrixMode(GL_MODELVIEW);
}

void 
KeyboardCB(unsigned char key, int x, int y) 
{
  switch(key) 
  {
  case 'q': delete bunny_to_draw; delete torus_to_draw; exit(0); 
  default: animate = 1 - animate;
  }

  glutPostRedisplay();
}

void
MouseCB(int _b, int _s, int _x, int _y)
{
  if (_s == GLUT_UP)
  {
    dis += ddis;
    azim += dazim;
    elev += delev;
    ddis = 0.0;
    dazim = 0.0;
    delev = 0.0;
    return;
  }

  if (_b == GLUT_RIGHT_BUTTON)
  {
    mode = 0;
    beginy = _y;
    return;
  }
  else
  {
    mode = 1;
    beginx = _x;
    beginy = _y;
  }
}

void
MotionCB(int _x, int _y)
{
  if (mode == 0)
  {
    ddis = dis * (_y - beginy)/200.0;
  }
  else
  {
    dazim = (_x - beginx)/5.0;
    delev = (_y - beginy)/5.0;      
  }
  
  glutPostRedisplay();
}

inline void glVertex3v(float V[3]) { glVertex3fv(V); }
inline void glVertex3v(double V[3]) { glVertex3dv(V); }

void
BeginDraw()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glLoadIdentity();  
  glTranslatef(0.0, 0.0, -(dis+ddis));
  glRotated(elev+delev, 1.0, 0.0, 0.0);
  glRotated(azim+dazim, 0.0, 1.0, 0.0);
}

void
EndDraw()
{
  glFlush();
  glutSwapBuffers();
}

void
IdleCB() 
{
  glutPostRedisplay();
}

void
DisplayCB()
{
  BeginDraw();

  // set up model transformations

  if (animate) 
  {
    rot1 += .1;
    rot2 += .2;
    rot3 += .3;
  }

  PQP_REAL R1[3][3],R2[3][3],T1[3],T2[3];
  PQP_REAL M1[3][3],M2[3][3],M3[3][3];

  T1[0] = -1;
  T1[1] =  0.0;
  T1[2] =  0.0;

  T2[0] =  1;
  T2[1] =  0.0;
  T2[2] =  0.0;

  MRotX(M1,rot1);
  MRotY(M2,rot2);
  MxM(M3,M1,M2);
  MRotZ(M1,rot3);
  MxM(R1,M3,M1);

  MRotX(M1,rot3);
  MRotY(M2,rot1);
  MxM(M3,M1,M2);
  MRotZ(M1,rot2);
  MxM(R2,M3,M1);

  // perform distance query

  PQP_REAL rel_err = 0.0;
  PQP_REAL abs_err = 0.0;
  PQP_DistanceResult res;
  PQP_Distance(&res,R1,T1,&bunny,R2,T2,&torus,rel_err,abs_err);

  // draw the models

  glColor3d(0.0,0.0,1.0);
  double oglm[16];
  MVtoOGL(oglm,R1,T1);
  glPushMatrix();
  glMultMatrixd(oglm);
  bunny_to_draw->Draw();
  glPopMatrix();

  glColor3d(0.0,1.0,0.0);
  MVtoOGL(oglm,R2,T2);
  glPushMatrix();
  glMultMatrixd(oglm);
  torus_to_draw->Draw();
  glPopMatrix();

  // draw the closest points as small spheres

  glColor3d(1.0,0.0,0.0);

  PQP_REAL P1[3],P2[3],V1[3],V2[3];
  VcV(P1,res.P1());
  VcV(P2,res.P2());

  // each point is in the space of its model;
  // transform to world space

  MxVpV(V1,R1,P1,T1);

  glPushMatrix();
  glTranslated(V1[0],V1[1],V1[2]);
  glutSolidSphere(.05,15,15);
  glPopMatrix();

  MxVpV(V2,R2,P2,T2);

  glPushMatrix();
  glTranslated(V2[0],V2[1],V2[2]);
  glutSolidSphere(.05,15,15);
  glPopMatrix();

  // draw the line between the closest points

  glDisable(GL_LIGHTING);
  glBegin(GL_LINES);
  glVertex3v(V1);
  glVertex3v(V2);
  glEnd();
  glEnable(GL_LIGHTING);

  EndDraw();
}

void main(int argc, char **argv)
{
  glutInit(&argc, argv);
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_MULTISAMPLE);

  // create the window

  glutCreateWindow("PQP Demo - Spinning");

  // set OpenGL graphics state -- material props, perspective, etc.

  InitViewerWindow();

  // set the callbacks

  glutDisplayFunc(DisplayCB);
  glutIdleFunc(IdleCB);
  glutMouseFunc(MouseCB);
  glutMotionFunc(MotionCB);  
  glutKeyboardFunc(KeyboardCB);

  // initialize the bunny

  FILE *fp;
  int i, ntris;

  bunny_to_draw = new Model("bunny.tris");

  fp = fopen("bunny.tris","r");
  if (fp == NULL) { fprintf(stderr,"Couldn't open bunny.tris\n"); exit(-1); }
  fscanf(fp,"%d",&ntris);

  bunny.BeginModel();
  for (i = 0; i < ntris; i++)
  {
    double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
    fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf", 
           &p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
    PQP_REAL p1[3],p2[3],p3[3];
    p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
    p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
    p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
    bunny.AddTri(p1,p2,p3,i);
  }
  bunny.EndModel();
  fclose(fp);

  // initialize the torus

  torus_to_draw = new Model("torus.tris");

  fp = fopen("torus.tris","r");
  if (fp == NULL) { fprintf(stderr,"Couldn't open torus.tris\n"); exit(-1); }
  fscanf(fp,"%d",&ntris);

  torus.BeginModel();
  for (i = 0; i < ntris; i++)
  {
    double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
    fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf", 
           &p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
    PQP_REAL p1[3],p2[3],p3[3];
    p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
    p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
    p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
    torus.AddTri(p1,p2,p3,i);
  }
  torus.EndModel();
  fclose(fp);

  // print instructions

  printf("PQP Demo - Spinning:\n"
         "Press 'q' to quit.\n"
         "Press any other key to toggle animation.\n"
         "Left-drag left & right to change angle of view.\n"
         "Left-drag up & down to change elevation of view.\n"
         "Right-drag up & down to change distance of view.\n");

  // Enter the main loop.

  glutMainLoop();
}


