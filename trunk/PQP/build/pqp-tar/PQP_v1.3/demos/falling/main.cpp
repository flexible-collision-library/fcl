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
#include "model.h"
#include "PQP.h"
#include "MatVec.h"

PQP_Model *torus1_tested,*torus2_tested;
Model     *torus1_drawn, *torus2_drawn;

int mode;
double beginx, beginy;
double dis = 10.0, azim = 0.0, elev = 0.0;
double ddis = 0.0, dazim = 0.0, delev = 0.0;

int animating = 1;
int step = 0;
int number_of_steps;
int query_type = 0;
double tolerance = .05;

PQP_REAL (*R1)[3][3];
PQP_REAL (*T1)[3];
PQP_REAL (*R2)[3][3];
PQP_REAL (*T2)[3];

void
init_viewer_window()
{
  GLfloat Ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };  
  GLfloat Diffuse[] = { 0.8f, 0.8f, 0.8f, 1.0f };  
  GLfloat Specular[] = { 0.1f, 0.1f, 0.1f, 1.0f };   
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
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);

  glShadeModel(GL_FLAT);
  glClearColor(0.0, 0.0, 0.0, 0.0);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glFrustum(-0.004,0.004,-0.004,0.004,.01,100.0);

  glMatrixMode(GL_MODELVIEW);
}

void
cb_mouse(int _b, int _s, int _x, int _y)
{
  if (_s == GLUT_UP)
  {
    dis += ddis;
    if (dis < .1) dis = .1;
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
cb_motion(int _x, int _y)
{
  if (mode == 0)
  {
    ddis = dis * (double)(_y - beginy)/200.0;
  }
  else
  {
    dazim = (_x - beginx)/5;
    delev = (_y - beginy)/5;      
  }
  
  glutPostRedisplay();
}

void cb_keyboard(unsigned char key, int x, int y) 
{
  switch(key) 
  {
  case 'q': 
    delete torus1_drawn; 
    delete torus2_drawn; 
    delete torus1_tested;
    delete torus2_tested;
    delete [] R1;
    delete [] T1;
    delete [] R2;
    delete [] T2;
    exit(0);
  case '0': query_type = 0; break;
  case '1': query_type = 1; break;
  case '2': query_type = 2; break;
  case '3': query_type = 3; break;
  case '-': 
    tolerance -= .01; 
    if (tolerance < 0.0) tolerance = 0.0; 
    break;
  case '=': 
    tolerance += .01;
    break;
  default: animating = 1 - animating;
  }

  glutPostRedisplay();
}

void cb_idle()
{
  if (animating)
  {
    step = (step + 1) % number_of_steps;
    glutPostRedisplay();
  }
}

void
BeginDraw()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glLoadIdentity(); 
  glTranslatef(0.0, 0.0, -(dis+ddis));
  glRotated(elev+delev, 1.0, 0.0, 0.0);
  glRotated(azim+dazim, 0.0, 1.0, 0.0);
  glRotated(90.0,-1.0,0.0,0.0);
}

void
EndDraw()
{
  glFlush();
  glutSwapBuffers();
}

inline void glVertex3v(float V[3]) { glVertex3fv(V); }
inline void glVertex3v(double V[3]) { glVertex3dv(V); }

void
cb_display()
{
  BeginDraw();

  int i;
  PQP_CollideResult cres;
  PQP_DistanceResult dres;
  PQP_ToleranceResult tres;
  double oglm[16];

  switch(query_type)
  {
  case 0:

    // draw model 1

    glColor3f(1,1,1);                 // setup color and transform
    MVtoOGL(oglm,R1[step],T1[step]);
    glPushMatrix();
    glMultMatrixd(oglm);
    torus1_drawn->Draw();             // do gl rendering
    glPopMatrix();                    // restore transform

    // draw model 2

    MVtoOGL(oglm,R2[step],T2[step]);
    glPushMatrix();
    glMultMatrixd(oglm);
    torus2_drawn->Draw();
    glPopMatrix();

    break;

  case 1:

    // perform collision query
    
    PQP_Collide(&cres,R1[step],T1[step],torus1_tested,
                      R2[step],T2[step],torus2_tested,
		                  PQP_ALL_CONTACTS);

    // draw model 1 and its overlapping tris

    MVtoOGL(oglm,R1[step],T1[step]);
    glPushMatrix();
    glMultMatrixd(oglm);
    glColor3f(1,1,1);
    torus1_drawn->Draw();
    glColor3f(1,0,0);
    for(i = 0; i < cres.NumPairs(); i++)
    {
      torus1_drawn->DrawTri(cres.Id1(i));
    }
    glPopMatrix();

    // draw model 2 and its overlapping tris

    MVtoOGL(oglm,R2[step],T2[step]);
    glPushMatrix();
    glMultMatrixd(oglm);
    glColor3f(1,1,1);
    torus2_drawn->Draw();
    glColor3f(1,0,0);
    for(i = 0; i < cres.NumPairs(); i++)
    {
      torus2_drawn->DrawTri(cres.Id2(i));
    }
    glPopMatrix();
    
    break;

  case 2:

    // perform distance query
    
    PQP_Distance(&dres,R1[step],T1[step],torus1_tested,
                       R2[step],T2[step],torus2_tested,
                       0.0,0.0);

    // draw models

    glColor3f(1,1,1);

    MVtoOGL(oglm,R1[step],T1[step]);
    glPushMatrix();
    glMultMatrixd(oglm);
    torus1_drawn->Draw();
    glPopMatrix();

    MVtoOGL(oglm,R2[step],T2[step]);
    glPushMatrix();
    glMultMatrixd(oglm);
    torus2_drawn->Draw();
    glPopMatrix();

    // draw the closest points as small spheres
    
    glColor3f(0,1,0);
    
    PQP_REAL P1[3],P2[3],V1[3],V2[3];
    VcV(P1,dres.P1());
    VcV(P2,dres.P2());

    // each point is in the space of its model;
    // transform to world space
    
    MxVpV(V1,R1[step],P1,T1[step]);
    
    glPushMatrix();
    glTranslated(V1[0],V1[1],V1[2]);
    glutSolidSphere(.01,15,15);
    glPopMatrix();                                                               
    
    MxVpV(V2,R2[step],P2,T2[step]);
    
    glPushMatrix();
    glTranslated(V2[0],V2[1],V2[2]);
    glutSolidSphere(.01,15,15);
    glPopMatrix();
    
    // draw the line between the closest points
    
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glVertex3v(V1);
    glVertex3v(V2);
    glEnd();
    glEnable(GL_LIGHTING);
    break;

  case 3:

    // perform tolerance query
    
    PQP_Tolerance(&tres,R1[step],T1[step],torus1_tested,
                        R2[step],T2[step],torus2_tested,
                        tolerance);
    
    if (tres.CloserThanTolerance())
      glColor3f(0,0,1);
    else 
      glColor3f(1,1,1);

    // draw models

    MVtoOGL(oglm,R1[step],T1[step]);
    glPushMatrix();
    glMultMatrixd(oglm);
    torus1_drawn->Draw();
    glPopMatrix();

    MVtoOGL(oglm,R2[step],T2[step]);
    glPushMatrix();
    glMultMatrixd(oglm);
    torus2_drawn->Draw();
    glPopMatrix();

    break;

  }

  EndDraw();
}

void LoadPath(PQP_REAL (* &R)[3][3], PQP_REAL (* &T)[3], char *filename) 
{
  FILE *fp;
  if ( (fp = fopen(filename, "r")) == NULL ) 
  {
    fprintf(stderr, "Error opening file %s\n", filename);
    exit(1);
  }
  fscanf(fp, "%d", &number_of_steps);

  R = new PQP_REAL[number_of_steps][3][3];
  T = new PQP_REAL[number_of_steps][3];

  for (int i = 0; i < number_of_steps; i++) 
  {
    double a, b, c;
    fscanf(fp,"%lf %lf %lf",&a,&b,&c);
    R[i][0][0] = (PQP_REAL)a;
    R[i][0][1] = (PQP_REAL)b;
    R[i][0][2] = (PQP_REAL)c;
    fscanf(fp,"%lf %lf %lf",&a,&b,&c);
    R[i][1][0] = (PQP_REAL)a;
    R[i][1][1] = (PQP_REAL)b;
    R[i][1][2] = (PQP_REAL)c;
    fscanf(fp,"%lf %lf %lf",&a,&b,&c);
    R[i][2][0] = (PQP_REAL)a;
    R[i][2][1] = (PQP_REAL)b;
    R[i][2][2] = (PQP_REAL)c;
    fscanf(fp,"%lf %lf %lf",&a,&b,&c);
    T[i][0] = (PQP_REAL)a;
    T[i][1] = (PQP_REAL)b;
    T[i][2] = (PQP_REAL)c;
  }

  fclose(fp);
}

void main(int argc, char **argv)
{
  // init glut

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_MULTISAMPLE);

  // create the window

  glutCreateWindow("PQP Demo - Falling");

  // set OpenGL graphics state -- material props, perspective, etc.

  init_viewer_window();

  // set the callbacks

  glutDisplayFunc(cb_display);
  glutMouseFunc(cb_mouse);
  glutMotionFunc(cb_motion);  
  glutKeyboardFunc(cb_keyboard);
  glutIdleFunc(cb_idle);

  // create models

  FILE *fp;
  int ntris, i;
  double a,b,c;
  PQP_REAL p1[3],p2[3],p3[3];

  // model 1

  torus1_drawn = new Model("torus1.tris");

  torus1_tested = new PQP_Model();

  fp = fopen("torus1.tris","r");
  fscanf(fp,"%d",&ntris);

  torus1_tested->BeginModel();
  for (i = 0; i < ntris; i++)
  {
    fscanf(fp,"%lf %lf %lf",&a,&b,&c);
    p1[0] = (PQP_REAL)a;
    p1[1] = (PQP_REAL)b;
    p1[2] = (PQP_REAL)c;
    fscanf(fp,"%lf %lf %lf",&a,&b,&c);
    p2[0] = (PQP_REAL)a;
    p2[1] = (PQP_REAL)b;
    p2[2] = (PQP_REAL)c;
    fscanf(fp,"%lf %lf %lf",&a,&b,&c);
    p3[0] = (PQP_REAL)a;
    p3[1] = (PQP_REAL)b;
    p3[2] = (PQP_REAL)c;
    torus1_tested->AddTri(p1,p2,p3,i);
  }
  torus1_tested->EndModel();  

  fclose(fp);

  // model 2

  torus2_drawn = new Model("torus2.tris");

  torus2_tested = new PQP_Model();

  fp = fopen("torus2.tris","r");
  fscanf(fp,"%d",&ntris);

  torus2_tested->BeginModel();
  for (i = 0; i < ntris; i++)
  {
    fscanf(fp,"%lf %lf %lf",&a,&b,&c);
    p1[0] = (PQP_REAL)a;
    p1[1] = (PQP_REAL)b;
    p1[2] = (PQP_REAL)c;
    fscanf(fp,"%lf %lf %lf",&a,&b,&c);
    p2[0] = (PQP_REAL)a;
    p2[1] = (PQP_REAL)b;
    p2[2] = (PQP_REAL)c;
    fscanf(fp,"%lf %lf %lf",&a,&b,&c);
    p3[0] = (PQP_REAL)a;
    p3[1] = (PQP_REAL)b;
    p3[2] = (PQP_REAL)c;
    torus2_tested->AddTri(p1,p2,p3,i);
  }
  torus2_tested->EndModel();  

  fclose(fp);

  // load paths

  LoadPath(R1,T1,"torus1.path");
  LoadPath(R2,T2,"torus2.path");

  // print instructions

  printf("PQP Demo - Falling:\n"
         "Press:\n"
         "0 - no proximity query, just animation\n"
         "1 - collision query\n"
         "    overlapping triangles shown in red.\n"
         "2 - distance query\n"
         "    closest points connected by a line.\n"
         "3 - tolerance query\n"
         "    reduce/increase tolerance with -/= keys.\n"
         "    models turn blue when closer than the tolerance.\n"
         "any other key to toggle animation on/off\n");

  // Enter the main loop.

  glutMainLoop();
}


