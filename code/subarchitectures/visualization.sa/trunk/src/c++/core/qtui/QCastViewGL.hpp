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
#include "tgCamera.h"

// class QCastViewGL: public QGLWidget, public cogx::display::CDisplayModelObserver
class QCastViewGL: public QGLWidget, public QCastViewBase
{
   Q_OBJECT
private:
   cogx::display::CDisplayView* pView;
   QPoint m_lastPos;
   int xRot;
   int yRot;
   int zRot;
   TomGine::tgCamera m_camera;

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
   void setXRotation(int angle);
   void setYRotation(int angle);
   void setZRotation(int angle);

signals:
   void xRotationChanged(int angle);
   void yRotationChanged(int angle);
   void zRotationChanged(int angle);

protected:
   void initializeGL();
   void paintGL();
   void resizeGL(int width, int height);
   void mousePressEvent(QMouseEvent *event);
   void mouseMoveEvent(QMouseEvent *event);
};

#endif /* end of include guard: QCASTVIEWGL_9ZC8AKWT */
