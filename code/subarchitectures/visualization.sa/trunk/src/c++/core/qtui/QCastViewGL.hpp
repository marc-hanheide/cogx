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
#ifndef QCASTVIEWGL_9ZC8AKWT
#define QCASTVIEWGL_9ZC8AKWT

#include "QCastViewBase.hpp"
#include <QGLWidget>

// class QCastViewGL: public QGLWidget, public cogx::display::CDisplayModelObserver
class QCastViewGL: public QGLWidget, public QCastViewBase
{
   Q_OBJECT
private:
   cogx::display::CDisplayView* pView;
   QPoint m_lastPos;
   float xRot;
   float yRot;
   float zRot;
   float zoomLevel;

private:
   struct Vector3
   {
      double x, y, z;
      Vector3(double _x=0, double _y=0, double _z=0) {
         x = _x;
         y = _y;
         z = _z;
      }

      void set(double _x, double _y, double _z) {
         x = _x;
         y = _y;
         z = _z;
      }

      Vector3 operator+ (const Vector3& v) {
         return Vector3( x+v.x, y+v.y, z+v.z );
      }

      Vector3 operator- (const Vector3& v) {
         return Vector3( x-v.x, y-v.y, z-v.z );
      }

      Vector3 operator* (double s) {
         return Vector3( s*x, s*y, s*z );
      }

      Vector3 dot(const Vector3& v) {
         return Vector3( x*v.x, y*v.y, z*v.z );
      }

      Vector3 cross(const Vector3& v) {
         return Vector3( y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x );
      }
   };

   struct Camera {
      Vector3 eye;  // eye point = origin of camera
      Vector3 view; // view direction of the camera (unit vector)
      Vector3 up;   // up vector of the camera (unit vector)
      Camera(): eye(0, 0, 0), up(0, 1, 0), view(0, 0, 1) {}
      Vector3 normal() {
         return up.cross(view);
      }
   };
   Camera m_camera;

public:
   QCastViewGL( QWidget* parent = 0, Qt::WindowFlags flags = 0 );
   ~QCastViewGL();

public:
   // QCastViewBase
   void setView(cogx::display::CDisplayView* pDisplayView); /*override*/
   cogx::display::CDisplayView* getView() { return pView; } /*override*/
   operator QWidget&() { return *this; } /*override*/
   // CDisplayModelObserver
   void onViewChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView); /*override*/
   void getToolbars(CPtrVector<QToolBar>& toolbars); /*override*/

public slots:
   void setCameraEye(const Vector3 &e);
   void setXRotation(float angle);
   void setYRotation(float angle);
   void setZRotation(float angle);

private slots:
   void onCameraItemChanged(int index);
   void onCameraChangeAction();
   void onActConfigureCameras();

signals:
   void cameraEyeChanged(const Vector3 &e);
   void xRotationChanged(float angle);
   void yRotationChanged(float angle);
   void zRotationChanged(float angle);

protected:
   void initializeGL();
   void paintGL();
   void resizeGL(int width, int height);
   void selectCamera(cogx::display::CDisplayCamera* pCamera);
   void mousePressEvent(QMouseEvent *event);
   void mouseMoveEvent(QMouseEvent *event);
   void wheelEvent(QWheelEvent *e);
};

#endif /* end of include guard: QCASTVIEWGL_9ZC8AKWT */
