/*
 * Author: Marko Mahnič
 * Created: 2010-06-10
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
#include "QCastViewHtml.hpp"
#include <QWebFrame>

#ifdef DEBUG_TRACE
// #undef DEBUG_TRACE
#endif
#include "convenience.hpp"

QCastViewHtml::QCastViewHtml(QWidget* parent, Qt::WindowFlags flags)
{
   pView = NULL;
   m_bModified = false;

   // The signal is queued an processed when Qt is idle.
   // We emit the signal in paintEvent so that the HTML content is recreated
   // only if it has to be shown.
   connect(this, SIGNAL(updateContent()),
         this, SLOT(doUpdateContent()),
         Qt::QueuedConnection);
}

QCastViewHtml::~QCastViewHtml()
{
   DTRACE("QCastViewHtml::~QCastViewHtml");
   if (pView != NULL) {
      pView->viewObservers.removeObserver(this);
   }
   pView = NULL;
}

void QCastViewHtml::setView(cogx::display::CDisplayView* pDisplayView)
{
   if (pView != NULL) {
      pView->viewObservers.removeObserver(this);
   }
   pView = pDisplayView;
   if (pView != NULL) {
      pView->viewObservers.addObserver(this);
   }
   m_bModified = true;
   update();
}

// TODO: reload the document! but not in paintEvent -- it causes a SIGBUS
// Queue an event an process when idle

void QCastViewHtml::paintEvent(QPaintEvent* event)
{
   if (m_bModified) {
      emit updateContent(); // Queued update
      m_bModified = false;
      return;
   }
   QWebView::paintEvent(event);
}

void QCastViewHtml::doUpdateContent()
{
   DTRACE("QCastViewHtml::doUpdateContent");

   if (pView) {
      QStringList list, head, body;
      QWebPage* pPage = page();
      QWebFrame* pFrame = NULL;
      if (pPage) pFrame = pPage->currentFrame();
      int sbv = 0;
      if (pFrame) sbv = pFrame->scrollBarValue(Qt::Vertical);
      pView->drawHtml(head, body);
      list.append("<html><head>");
      list << head;
      list.append("</head><body>");
      list << body;
      list.append("</body></html>");
      setHtml(list.join("\n"));
      if (pFrame) pFrame->setScrollBarValue(Qt::Vertical, sbv);
   }

   update();
}

void QCastViewHtml::onViewChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView)
{
   if (pView == this->pView) {
      m_bModified = true;
      update();
   }
}
