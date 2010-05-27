/*
 * Author: Marko Mahnič
 * Created: 2010-05-10
 */

#include "QCastViewGL.hpp"
#include <QWheelEvent>
#include <QMouseEvent>
#include "../convenience.hpp"

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

QCastViewGL::QCastViewGL( QWidget* parent, Qt::WindowFlags flags )
{
   DTRACE("QCastViewGL::QCastViewGL");
   pView = NULL;
}

QCastViewGL::~QCastViewGL()
{
   DTRACE("QCastViewGL::~QCastViewGL");
   if (pView != NULL) {
      pView->viewObservers.removeObserver(this);
   }
   pView = NULL;
}

void QCastViewGL::setView(cogx::display::CDisplayView* pDisplayView)
{
   if (pView != NULL) {
      pView->viewObservers.removeObserver(this);
   }
   pView = pDisplayView;
   if (pView != NULL) {
      pView->viewObservers.addObserver(this);
   }
   update();
}

void QCastViewGL::onViewChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView)
{
   if (pView == this->pView) update();
}

static void qNormalizeAngle(int &angle)
{
   while (angle < 0) angle += 360 * 16;
   while (angle > 360 * 16) angle -= 360 * 16;
}

void QCastViewGL::setXRotation(int angle)
{
   qNormalizeAngle(angle);
   if (angle != xRot) {
      xRot = angle;
      emit xRotationChanged(angle);
      updateGL();
   }
}

void QCastViewGL::setYRotation(int angle)
{
   qNormalizeAngle(angle);
   if (angle != yRot) {
      yRot = angle;
      emit yRotationChanged(angle);
      updateGL();
   }
}

void QCastViewGL::setZRotation(int angle)
{
   qNormalizeAngle(angle);
   if (angle != zRot) {
      zRot = angle;
      emit zRotationChanged(angle);
      updateGL();
   }
}

void QCastViewGL::initializeGL()
{
   QColor qtPurple = QColor::fromCmykF(0.39, 0.39, 0.0, 0.0);
   qglClearColor(qtPurple.dark());

   //logo = new QtLogo(this, 64);
   //logo->setColor(qtGreen.dark());

   glEnable(GL_DEPTH_TEST);
   //glEnable(GL_CULL_FACE);
   glShadeModel(GL_SMOOTH);
   glEnable(GL_LIGHTING);
   glEnable(GL_LIGHT0);
   glEnable(GL_MULTISAMPLE);
   static GLfloat lightPosition[4] = { 0.5, 5.0, -7.0, 1.0 };
   glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
}

void QCastViewGL::paintGL()
{
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   // m_camera.Activate();
   if (pView) {
      glLoadIdentity();
      glTranslatef(0.0, 0.0, -10.0);
      glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
      glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
      glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);

      pView->drawGL();
   }
}

void QCastViewGL::resizeGL(int width, int height)
{
   int side = qMin(width, height);
   glViewport((width - side) / 2, (height - side) / 2, side, side);

   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
#ifdef QT_OPENGL_ES_1
   glOrthof(-0.5, +0.5, -0.5, +0.5, 4.0, 15.0);
#else
   glOrtho(-0.5, +0.5, -0.5, +0.5, 4.0, 15.0);
#endif
   //gluPerspective(15, 1.0*width/height, 4.0, 15.0);

   glMatrixMode(GL_MODELVIEW);

   //m_camera.SetViewport(width, height);
   //m_camera.SetPerspective();
}

void QCastViewGL::mousePressEvent(QMouseEvent *event)
{
   m_lastPos = event->pos();
}

void QCastViewGL::mouseMoveEvent(QMouseEvent *event)
{
   int dx = event->x() - m_lastPos.x();
   int dy = event->y() - m_lastPos.y();

   if (event->buttons() & Qt::LeftButton) {
      setXRotation(xRot + 8 * dy);
      setYRotation(yRot + 8 * dx);
      //m_camera.RotateF(0.1*dx);
      //m_camera.RotateY(0.1*dy);
   }
   else if (event->buttons() & Qt::RightButton) {
      setXRotation(xRot + 8 * dy);
      setZRotation(zRot + 8 * dx);
      //m_camera.RotateS(0.1*dx);
      //m_camera.RotateZ(0.1*dy);
   }

   m_lastPos = event->pos();
}
