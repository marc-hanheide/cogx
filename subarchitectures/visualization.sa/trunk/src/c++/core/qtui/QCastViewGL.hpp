/*
 * Author: Marko Mahnič
 * Created: 2010-05-10
 */
#ifndef QCASTVIEWGL_9ZC8AKWT
#define QCASTVIEWGL_9ZC8AKWT

// #include <QWidget>
#include <QGLWidget>
#include "../Model.hpp"

class QCastViewGL: public QGLWidget, public cogx::display::CDisplayModelObserver
{
   Q_OBJECT
private:
   cogx::display::CDisplayView* pView;
   QPoint m_lastPos;
   int xRot;
   int yRot;
   int zRot;

public:
   QCastViewGL( QWidget* parent = 0, Qt::WindowFlags flags = 0 );
   ~QCastViewGL();
   void setView(cogx::display::CDisplayView* pDisplayView);

public:
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
