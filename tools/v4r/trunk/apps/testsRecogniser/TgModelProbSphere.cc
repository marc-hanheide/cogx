/**
 * $Id$
 */


#include "TgModelProbSphere.hh"
#include <GL/gl.h>



namespace TomGine 
{



/************************************************************************************
 * Constructor/Destructor
 */

TgModelProbSphere::TgModelProbSphere()
{ 
  pthread_mutex_init(&mut,NULL); 
}

TgModelProbSphere::~TgModelProbSphere()
{
  pthread_mutex_destroy(&mut);
}


/***************************************************************************************/


/**
 * draw
 */
void TgModelProbSphere::Draw() 
{
  Lock();
  cv::Point3d a, b, c, n;
  double scale=0.15;
  cv::Point3d sh(0.,0.,0.);

  glDisable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_POINT_SMOOTH);
  glDisable(GL_TEXTURE_2D);

  for (unsigned i=0; i<subdivFaces.size(); i++)
  {
    float col = subdivFaces[i]->weight;
    glColor3f(col,col,col);

    n = subdivFaces[i]->n;
    a = vertices[subdivFaces[i]->vs[0]]*scale + sh;
    b = vertices[subdivFaces[i]->vs[1]]*scale + sh;
    c = vertices[subdivFaces[i]->vs[2]]*scale + sh;

    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(n.x, n.y, n.z);
    glVertex3f(a.x, a.y, a.z);
    glNormal3f(n.x, n.y, n.z);
    glVertex3f(b.x, b.y, b.z);
    glNormal3f(n.x, n.y, n.z);
    glVertex3f(c.x, c.y, c.z);
    glEnd();

    glColor3f(.5,.5,.5);
    glBegin(GL_LINES);
    glVertex3f(a.x, a.y, a.z);
    glVertex3f(b.x, b.y, b.z);
    glVertex3f(b.x, b.y, b.z);
    glVertex3f(c.x, c.y, c.z);
    glVertex3f(c.x, c.y, c.z);
    glVertex3f(a.x, a.y, a.z);
    glEnd( );
  }
  Unlock();
}


}





