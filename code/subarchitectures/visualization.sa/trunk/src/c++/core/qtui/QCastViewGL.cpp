/*
 * Author: Marko Mahnič
 * Created: 2010-05-10
 *
 * © Copyright 2010 Marko Mahnič. 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "QCastViewGL.hpp"
#include <QWheelEvent>
#include <QMouseEvent>
#include <cmath>
#include "../convenience.hpp"

QCastViewGL::QCastViewGL( QWidget* parent, Qt::WindowFlags flags )
{
   DTRACE("QCastViewGL::QCastViewGL");
   pView = NULL;
   xRot = 0;
   yRot = 0;
   zRot = 0;
   zoomLevel = 0;
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

static void qNormalizeAngle(float &angle)
{
   angle = fmod(angle, 360);
   if (angle < 0) angle += 360;
}

void QCastViewGL::setXRotation(float angle)
{
   qNormalizeAngle(angle);
   if (angle != xRot) {
      xRot = angle;
      emit xRotationChanged(angle);
      updateGL();
   }
}

void QCastViewGL::setYRotation(float angle)
{
   qNormalizeAngle(angle);
   if (angle != yRot) {
      yRot = angle;
      emit yRotationChanged(angle);
      updateGL();
   }
}

void QCastViewGL::setZRotation(float angle)
{
   qNormalizeAngle(angle);
   if (angle != zRot) {
      zRot = angle;
      emit zRotationChanged(angle);
      updateGL();
   }
}

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

void QCastViewGL::initializeGL()
{
   QColor qtPurple = QColor::fromCmykF(0.39, 0.39, 0.0, 0.0);
   qglClearColor(qtPurple.dark());

   //logo = new QtLogo(this, 64);
   //logo->setColor(qtGreen.dark());

   glEnable(GL_DEPTH_TEST);
   //glEnable(GL_CULL_FACE);
   //glShadeModel(GL_SMOOTH);
   glEnable(GL_LIGHTING);
   glEnable(GL_LIGHT0);
   //glEnable(GL_MULTISAMPLE);
   static GLfloat lightPosition[4] = { 3.0, 3.0, 3.0, 1.0 };
   glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
}

void QCastViewGL::resizeGL(int width, int height)
{
   float aspect = 1.0*width/height;

   glViewport(0, 0, width, height);
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   if (1) {
      float ang;
      if (zoomLevel >= 0) ang = 50/pow(2.0, zoomLevel);
      else ang = 50 - zoomLevel*25;
      if (ang < 1) ang = 1;
      if (ang > 170) ang = 170;
      gluPerspective(ang, aspect, 1.5, 20.0);
      // glFrustum (-1.0*zoomFactor, 1.0*zoomFactor, -1.0*zoomFactor, 1.0*zoomFactor, 1.5, 20.0);
   }
   //else {
   //   glOrtho(-0.5, +0.5, -0.5, +0.5, 4.0, 15.0);
   //}
   glMatrixMode(GL_MODELVIEW);
}

void QCastViewGL::paintGL()
{
   glEnable(GL_LIGHTING);
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   if (pView) {
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      gluLookAt(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
      //glTranslatef(0.0, 0.0, -10.0);
      glRotatef(xRot, 1.0, 0.0, 0.0);
      glRotatef(yRot, 0.0, 1.0, 0.0);
      glRotatef(zRot, 0.0, 0.0, 1.0);

      pView->drawGL();
   }
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
      setXRotation(xRot + dy);
      setYRotation(yRot + dx);
   }
   else if (event->buttons() & Qt::RightButton) {
      setXRotation(xRot + dy);
      setZRotation(zRot + dx);
   }

   m_lastPos = event->pos();
}

void QCastViewGL::wheelEvent(QWheelEvent *e)
{
   if (e->modifiers() & Qt::ControlModifier) {
      const double lvlmin = -5;
      const double lvlmax = 5;
      double lvl = zoomLevel;

      if (e->delta() > 0) zoomLevel += 1;
      else zoomLevel -= 1;
      if (zoomLevel < lvlmin) zoomLevel = lvlmin;
      if (zoomLevel > lvlmax) zoomLevel = lvlmax;

      if (lvl != zoomLevel) {
         resizeGL(this->width(), this->height());
         updateGL();
      }
   }
   else {
   }
}
