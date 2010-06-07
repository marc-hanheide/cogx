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

public:
   QCastViewGL( QWidget* parent = 0, Qt::WindowFlags flags = 0 );
   ~QCastViewGL();

public:
   // QCastViewBase
   void setView(cogx::display::CDisplayView* pDisplayView); /*override*/
   operator QWidget&() { return *this; } /*override*/
   // CDisplayModelObserver
   void onViewChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView); /*override*/

public slots:
   void setXRotation(float angle);
   void setYRotation(float angle);
   void setZRotation(float angle);

signals:
   void xRotationChanged(float angle);
   void yRotationChanged(float angle);
   void zRotationChanged(float angle);

protected:
   void initializeGL();
   void paintGL();
   void resizeGL(int width, int height);
   void mousePressEvent(QMouseEvent *event);
   void mouseMoveEvent(QMouseEvent *event);
   void wheelEvent(QWheelEvent *e);
};

#endif /* end of include guard: QCASTVIEWGL_9ZC8AKWT */
