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
#include <QToolButton>
#include <QMenu>
#include <QAction>
#include "../convenience.hpp"

const int ZOOM_RANGE = 5; // -N .. N

void QCastViewGL::Camera::normalize()
{
   if (view.length() <= 0 || up.length() <= 0) return; // bad camera
   QCastViewGL::Vector3 n = normal();
   if (n.length() == 0) return; // bad camera
   view.normalize();

   QCastViewGL::Vector3 newup = view.cross(n);
   newup.normalize();
   up = newup;
}

QCastViewGL::QCastViewGL( QWidget* parent, Qt::WindowFlags flags )
{
   DTRACE("QCastViewGL::QCastViewGL");
   pView = 0;
   xRot = 0;
   yRot = 0;
   zRot = 0;
   zoomLevel = 0;
   m_pivot.set(0, 0, 0);
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

   // Look from the first camera
   CPtrVector<cogx::display::CDisplayCamera> cameras;
   pView->getCameras(cameras);
   if (cameras.size() > 0) selectCamera(cameras.front());
   update();
}

void QCastViewGL::selectCamera(cogx::display::CDisplayCamera* pCamera)
{
   if (! pCamera) return;
   m_camera.eye.set(pCamera->xEye, pCamera->yEye, pCamera->zEye);
   m_camera.view.set(pCamera->xView, pCamera->yView, pCamera->zView);
   m_camera.up.set(pCamera->xUp, pCamera->yUp, pCamera->zUp);
   m_camera.normalize();
   //std::cout << " *** Camera set to " << pCamera->name << std::endl;
}

void QCastViewGL::onViewChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView)
{
   if (pView == this->pView) update();
}

void QCastViewGL::onCameraItemChanged(int index)
{
   CPtrVector<cogx::display::CDisplayCamera> cameras;
   pView->getCameras(cameras);
   if (index >= cameras.size()) return;
   selectCamera(cameras[index]);
}

void QCastViewGL::onCameraChangeAction()
{
   QAction *pAct = qobject_cast< QAction* >(QObject::sender());
   if (!pAct) return;
   QVariant v = pAct->data();
   void* pc = v.value<void*>();
   cogx::display::CDisplayCamera* pCamera = static_cast<cogx::display::CDisplayCamera*>(pc);
   //std::cout << pAct << " " << pAct->text().toStdString() << " " << pc << " " << pCamera << std::endl;

   if (pCamera) {
      // TODO: should check if it is valid -- is it in getCameras()?
      selectCamera(pCamera);
   }

   // Replace the action for the button.
   // Parent hierarchy: action -> menu -> button
   QMenu *pMenu = qobject_cast< QMenu* >(pAct->parent());
   if (pMenu) {
      QToolButton* pBut = qobject_cast< QToolButton* >(pMenu->parent());
      if (pBut) {
         QAction *pOldAct = pBut->defaultAction();
         if (pOldAct && pOldAct->parent() == pAct->parent()) {
            //std::cout << "Changing default action" << std::endl;
            pBut->setDefaultAction(pAct);
         }
      }
   }
}

void QCastViewGL::onActConfigureCameras()
{
   // TODO: display dialog with one list for each camera
   // TODO: save camera selections to the registry; reload on (first) activation
}

void QCastViewGL::getToolbars(CPtrVector<QToolBar>& toolbars)
{
   if (! pView) return;
   CPtrVector<cogx::display::CDisplayCamera> cameras;
   pView->getCameras(cameras);
   if (cameras.size() < 1) return;
   QToolBar *pBar = new QToolBar(this); // parent will be reset in QViewContainer
   if (pBar) {
      int nc = cameras.size();
      if (nc > 3) nc = 3;
      cogx::display::CDisplayCamera* pCamera;
      for (int i= 0; i < nc; i++) {
         QToolButton *pBut = new QToolButton(pBar);
         pCamera = cameras[i];
         QString text = QString::fromStdString(pCamera->name);
         QAction* pAct = new QAction(QIcon(":/toolButton/camera-photo.png"), text, pBut);
         pAct->setData(qVariantFromValue((void*)pCamera));
         pBut->setDefaultAction(pAct);
         pBar->addWidget(pBut);
         pBar->connect(pAct, SIGNAL(triggered()), this, SLOT(onCameraChangeAction()));

         // With more than 3 cameras things become complicated...
         if (i == 2 && cameras.size() > 2) {
            QAction *pPopAct;
            QMenu *pMenu = new QMenu(pBut); // parent MUST be button, see onCameraChangeAction
            pBut->setMenu(pMenu);
            pBut->setPopupMode(QToolButton::MenuButtonPopup);

            for (int j = 0; j < cameras.size(); j++) {
               if (i == j) {
                  pMenu->addAction(pAct);
                  pAct->setParent(pMenu);   // parent MUST be menu, see onCameraChangeAction
               }
               else {
                  pCamera = cameras[j];
                  text = QString::fromStdString(pCamera->name);
                  pPopAct = pMenu->addAction(QIcon(":/toolButton/camera-photo.png"), text);
                  pPopAct->setData(qVariantFromValue((void*)pCamera));
                  pBar->connect(pPopAct, SIGNAL(triggered()), this, SLOT(onCameraChangeAction()));
               }
            }

            if (0) {
               text = "TODO: Configure camera buttons...";
               pPopAct = pMenu->addAction(QIcon(":/toolButton/camera-photo.png"), text);
               pBar->connect(pPopAct, SIGNAL(triggered()), this, SLOT(onActConfigureCameras()));
            }
         }
      }
      toolbars.push_back(pBar);
   }
}

static void qNormalizeAngle(float &angle)
{
   angle = fmod(angle, 360);
   if (angle < 0) angle += 360;
}

void QCastViewGL::setCameraEye(const Vector3 &e)
{
   m_camera.eye = e;
   emit cameraEyeChanged(e);
   updateGL();
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
   //glEnable(GL_MULTISAMPLE);
   glShadeModel(GL_SMOOTH);

   glEnable(GL_LIGHTING);
   //static GLfloat global_ambient[] = { 0.5f, 0.5f, 0.5f, 1.0f };
   //glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient);

   glEnable(GL_LIGHT0);
   static GLfloat specularL[] = {0.5f, 0.5f, 0.5f , 1.0f};
   static GLfloat ambientL[] = { 0.2f, 0.2f, 0.2f, 1.0f };
   static GLfloat diffuseL[] = { 0.8f, 0.8f, 0.8, 1.0f };
   static GLfloat lightPosition[4] = { -3.0, 3.0, 5.0, 1.0 };
   glLightfv(GL_LIGHT0, GL_AMBIENT, ambientL);
   glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseL);
   glLightfv(GL_LIGHT0, GL_SPECULAR, specularL);
   glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

   glEnable(GL_COLOR_MATERIAL);
   glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
}

void QCastViewGL::resizeGL(int width, int height)
{
   float aspect = 1.0*width/height;

   glViewport(0, 0, width, height);
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   if (1) {
      const float PI = 3.14159265;
      const float PI2 = PI / 2;
      const float x[3] = {0, 0.5, 1};
      const float zoomangles[3] = {170, 50, 5}; // at x[]
      const float A = 1.0 / ((x[0] - x[1]) * (x[0] - x[2]));
      const float B = 1.0 / ((x[1] - x[0]) * (x[1] - x[2]));
      const float C = 1.0 / ((x[2] - x[0]) * (x[2] - x[1]));
      float zl = (zoomLevel + ZOOM_RANGE) / (2 * ZOOM_RANGE); // 0 .. 1
      float ang =
         zoomangles[0] * (A * (zl-x[1]) * (zl-x[2])) +
         zoomangles[1] * (B * (zl-x[0]) * (zl-x[2])) +
         zoomangles[2] * (C * (zl-x[0]) * (zl-x[1]));

      if (ang < 5) ang = 5;
      if (ang > 170) ang = 170;
      gluPerspective(ang, aspect, 0.01, 20.0);

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

      Vector3 &e = m_camera.eye, &v = m_camera.view, &u = m_camera.up;
      gluLookAt(e.x, e.y, e.z, e.x + v.x, e.y + v.y, e.z + v.z, u.x, u.y, u.z);

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
   double trans_scale = 0.01, rot_scale = 1.0;
   double dx = (double)(event->x() - m_lastPos.x());
   double dy = (double)(event->y() - m_lastPos.y());

   if (event->buttons() & Qt::LeftButton) {
      setCameraEye(m_camera.eye + (m_camera.up*dy + m_camera.normal()*dx)*trans_scale);
   }
   else if(event->buttons() & Qt::MidButton) {
      setCameraEye(m_camera.eye - m_camera.view*dy*trans_scale);
   }
   else if(event->buttons() & Qt::RightButton) {
      // Rotate the scene (not the camera)
      // TODO: directions (signs) may change depending on m_camera position
      if (event->modifiers() & Qt::ControlModifier) {
         setXRotation(xRot - dy*rot_scale);
         setZRotation(zRot + dx*rot_scale);
      }
      else {
         // orbit around current pivot
         Vector3 dir = m_camera.normal()*dx + m_camera.up*dy;
         std::cout << dir.x << " " << dir.y << " " << dir.z << std::endl;

         // very simple approach: keep the radius
         double R = (m_camera.eye - m_pivot).length();
         Vector3 neweye = m_camera.eye - dir * (R/10);
         dir = neweye - m_pivot;
         dir.normalize();
         m_camera.view = dir * -1;
         m_camera.normalize();
         setCameraEye(dir * R);
      }
   }
   //printf("dx: %lf, dy: %lf, xRot: %f, yRot: %f, zRot: %f\n", dx, dy, xRot, yRot, zRot);
   m_lastPos = event->pos();
}

void QCastViewGL::wheelEvent(QWheelEvent *e)
{
   double trans_scale = 0.01, zoom_scale = 0.5;
   if (e->modifiers() & Qt::ControlModifier) {
      const double lvlmin = -ZOOM_RANGE;
      const double lvlmax = ZOOM_RANGE;
      double lvl = zoomLevel;

      if (e->delta() > 0) zoomLevel += zoom_scale;
      else zoomLevel -= zoom_scale;
      if (zoomLevel < lvlmin) zoomLevel = lvlmin;
      if (zoomLevel > lvlmax) zoomLevel = lvlmax;

      if (lvl != zoomLevel) {
         resizeGL(this->width(), this->height());
         updateGL();
      }
   }
   else {
      double dz = (double) e->delta();
      setCameraEye(m_camera.eye + m_camera.view * dz * trans_scale);
   }
}
