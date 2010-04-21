/*
 * Author: Marko Mahnič
 * Created: 2010-03-10
 */

#include "QCastView.hpp"
#include <QPainter>
#include <QWheelEvent>

QCastView::QCastView( QWidget* parent, Qt::WindowFlags flags )
   : QWidget(parent, flags)
{
   pView = NULL;
   m_scale = 0.5;
   m_offset = QPointF(0, 0);
}

QCastView::~QCastView()
{
   pView = NULL;
}

void QCastView::setView(cogx::display::CDisplayView* pDisplayView)
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

void QCastView::paintEvent ( QPaintEvent * event )
{
   QPainter painter(this);

   if (pView) {
      painter.scale(m_scale, m_scale);
      painter.translate(m_offset);
      pView->draw2D(painter);
   }
   else {
      painter.drawText(1, 20, tr("QCastView: no view selected."));
   }
}

void QCastView::wheelEvent(QWheelEvent *e)
{
   const double sclmin = 1/32.0;
   const double sclmax = 32.0;

   // world-coord of pixel under cursor
   QPointF ppix = (e->pos() - m_offset * m_scale) / m_scale; 

   if (e->delta() > 0) m_scale *= 2;
   else m_scale /= 2;
   if (m_scale < sclmin) m_scale = sclmin;
   if (m_scale > sclmax) m_scale = sclmax;

   m_offset = (e->pos() - ppix * m_scale) / m_scale;
   update();
}

void QCastView::onViewChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView)
{
   if (pView == this->pView) update();
}

