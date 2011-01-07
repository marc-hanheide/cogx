/*
 * Author: Marko Mahnič
 * Created: 2011-01-07
 *
 * © Copyright 2011 Marko Mahnič. 
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

#include "QCastViewScene.hpp"
#include <QWheelEvent>

#ifdef DEBUG_TRACE
// #undef DEBUG_TRACE
#endif
#include "../convenience.hpp"

QCastViewScene::QCastViewScene( QWidget* parent, Qt::WindowFlags flags )
   : QGraphicsView(parent)
{
   pView = NULL;
   m_bNeedsRebuild = true;
   //m_scale = 1.0;
   //m_offset = QPointF(0, 0);
   m_pScene = new QGraphicsScene(this);
   setScene(m_pScene);

   // Although not necessary, we want call m_pScene->update() in the GUI thread.
   connect(this, 
      SIGNAL(signalViewChanged()), this,
      SLOT(requestFullRedraw()), Qt::QueuedConnection);
}

QCastViewScene::~QCastViewScene()
{
   DTRACE("QCastViewScene::~QCastViewScene");
   if (pView != NULL) {
      pView->viewObservers.removeObserver(this);
   }
   pView = NULL;
}

void QCastViewScene::setView(cogx::display::CDisplayView* pDisplayView)
{
   if (pView == pDisplayView) return;

   if (pView != NULL) {
      pView->viewObservers.removeObserver(this);
   }
   pView = pDisplayView;
   if (pView != NULL) {
      pView->viewObservers.addObserver(this);
   }
   emit signalViewChanged();
}

void QCastViewScene::onViewChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView)
{
   DTRACE("QCastViewScene::onViewChanged");
   if (pView == this->pView) {
      emit signalViewChanged();
   }
}

void QCastViewScene::requestFullRedraw()
{
   DTRACE("QCastViewScene::requestFullRedraw");
   m_bNeedsRebuild = true;
   // QGraphicsView::update() and similar have no effect. We have to call QGraphicsScene::update().
   if (m_pScene) m_pScene->update();
}

void QCastViewScene::rebuildScene(cogx::display::CDisplayView* pDisplayView)
{
   DTRACE("QCastViewScene::rebuildScene");
   m_bNeedsRebuild = false;

   if (m_pScene) {
      // XXX: ideally we would only replace the objects that have changed, not the whole scene.
      // Currently the model doesn't provide the required infromation.
      m_pScene->clear();
      if (pDisplayView) {
         pDisplayView->drawScene(*m_pScene);
         QList<QGraphicsItem*> items = m_pScene->items();
         DMESSAGE("Items in scene: " << items.size() << "  View: " << pDisplayView->m_id);
      }
   }
}


void QCastViewScene::paintEvent ( QPaintEvent * event )
{
   DTRACE("QCastViewScene::paintEvent");
   if (m_bNeedsRebuild) {
      // We are only allowed to rebuild the scene in the GUI thread,
      // so we do this in the paintEvent
      rebuildScene(pView);
   }
   QGraphicsView::paintEvent(event);
}

//void QCastViewScene::resetOffset()
//{
//   m_offset = QPointF(0, 0);
//}

//void QCastViewScene::resetScale()
//{
//   m_scale = 1.0;
//}


//void QCastViewScene::wheelEvent(QWheelEvent *e)
//{
//   if (e->modifiers() & Qt::ControlModifier) {
//      const double sclmin = 1/32.0;
//      const double sclmax = 32.0;

//      // world-coord of pixel under cursor
//      QPointF ppix = (e->pos() - m_offset * m_scale) / m_scale; 

//      if (e->delta() > 0) m_scale *= 2;
//      else m_scale /= 2;
//      if (m_scale < sclmin) m_scale = sclmin;
//      if (m_scale > sclmax) m_scale = sclmax;

//      m_offset = (e->pos() - ppix * m_scale) / m_scale;
//      update();
//   }
//   else {
//      QPointF yoffs(0, 32.0/m_scale);
//      if (e->delta() > 0) m_offset += yoffs;
//      else m_offset -= yoffs;
//      update();
//   }
//}

