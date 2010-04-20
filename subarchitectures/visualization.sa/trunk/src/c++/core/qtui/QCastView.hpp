/*
 * Author: Marko Mahnič
 * Created: 2010-03-10
 */


#ifndef QCASTVIEW_NXKFPZKI
#define QCASTVIEW_NXKFPZKI

#include <QWidget>
// #include <QAbstractScrollArea>
#include "../Model.hpp"

class QCastView: public QWidget, public cogx::display::CDisplayModelObserver
{
   Q_OBJECT
private:
   // cogx::display::CDisplayModel* pModel;
   cogx::display::CDisplayView* pView;
   double m_scale;
   QPointF m_offset;

public:
   QCastView( QWidget* parent = 0, Qt::WindowFlags flags = 0 );
   ~QCastView();
   void setView(cogx::display::CDisplayView* pDisplayView);
   void onViewChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView); /*override*/

protected:
   virtual void paintEvent ( QPaintEvent * event ); /*override*/
   virtual void wheelEvent(QWheelEvent *e); /*override*/

};

#endif /* end of include guard: QCASTVIEW_NXKFPZKI */
