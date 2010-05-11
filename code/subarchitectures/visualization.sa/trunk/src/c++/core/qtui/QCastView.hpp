/*
 * Author: Marko Mahnič
 * Created: 2010-03-10
 */

#ifndef QCASTVIEW_NXKFPZKI
#define QCASTVIEW_NXKFPZKI

#include "QCastViewBase.hpp"
#include <QWidget>

class QCastView: public QWidget, public QCastViewBase
{
   Q_OBJECT
private:
   // cogx::display::CDisplayModel* pModel;
   cogx::display::CDisplayView* pView;

   // XXX: scale and offset should be stored in pView or in a map<pview,viewState>
   double m_scale;
   QPointF m_offset;

public:
   QCastView( QWidget* parent = 0, Qt::WindowFlags flags = 0 );
   ~QCastView();

public:
   // QCastViewBase
   void setView(cogx::display::CDisplayView* pDisplayView); /*override*/
   operator QWidget&() { return *this; } /*override*/
   // CDisplayModelObserver
   void onViewChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView); /*override*/

public slots:
   void resetOffset();
   void resetScale();

protected:
   // Qt events
   virtual void paintEvent(QPaintEvent * event); /*override*/
   virtual void wheelEvent(QWheelEvent *e); /*override*/
};

#endif /* end of include guard: QCASTVIEW_NXKFPZKI */
