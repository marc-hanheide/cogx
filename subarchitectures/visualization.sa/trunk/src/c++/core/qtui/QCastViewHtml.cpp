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
#include "html/formcap.hpp"
#include <QWebFrame>
#include <QFile>

#ifdef DEBUG_TRACE
// #undef DEBUG_TRACE
#endif
#include "convenience.hpp"

namespace cxd = cogx::display;

// ("CastQFormProxy");
//static const QString jsObjectName = QString::fromStdString(cxd::CHtmlChunk::JavascriptObjectName);
QString QCastViewHtml::m_jQuery;

QCastViewHtml::QCastViewHtml(QWidget* parent, Qt::WindowFlags flags)
{
   pView = NULL;
   m_bModified = false;
   m_bHasForms = false;
   jsObjectName = QString::fromStdString(cxd::CHtmlChunk::JavascriptObjectName);

   // The signal is queued an processed when Qt is idle.
   // We emit the signal in paintEvent so that the HTML content is recreated
   // only if it has to be shown.
   connect(this, SIGNAL(updateContent()),
         this, SLOT(doUpdateContent()),
         Qt::QueuedConnection);

   if (m_jQuery.length() < 1) {
      DMESSAGE("Loading resource: jQuery");
      QFile file;
      file.setFileName(":/jquery.min.js");
      file.open(QIODevice::ReadOnly);
      m_jQuery = file.readAll();
      file.close();
   }

   // Capture GET and POST events from forms that support it
   m_jsFormCap = QCastFormProxy::getJavaScript(jsObjectName, true);
   QWebPage* pPage = page();
   QWebFrame* pFrame = NULL;
   if (pPage) pFrame = pPage->currentFrame();
   if (pFrame) {
      connect(pFrame, SIGNAL(javaScriptWindowObjectCleared()),
            this, SLOT(createJsObjects()));
      connect(this, SIGNAL(loadFinished(bool)),
            this, SLOT(finishLoading(bool)));
   }
}

QCastViewHtml::~QCastViewHtml()
{
   DTRACE("QCastViewHtml::~QCastViewHtml");
   if (pView != NULL) {
      pView->viewObservers.removeObserver(this);
   }
   pView = NULL;

   m_Chunks.clear(); // don't delete, just clear;
}

void QCastViewHtml::createJsObjects()
{
   DTRACE("QCastViewHtml::createJsObjects");
   QWebPage* pPage = page();
   if (! pPage) return;
   QWebFrame* pFrame = pPage->currentFrame();
   if (! pFrame) return;

   QCastFormProxy* pObj = new QCastFormProxy();

   if (m_bHasForms) {
      connect(pObj, SIGNAL(signalOwnerDataChanged(QString)),
           this, SLOT(doFillHtmlFrom(QString)),
           Qt::QueuedConnection);
   }

   cxd::CHtmlChunk* pChunk;
   FOR_EACH(pChunk, m_Chunks) {
      if (! pChunk ) continue;
      pObj->registerChunk(pChunk);
   }
   pFrame->addToJavaScriptWindowObject(jsObjectName, pObj);
}

void QCastViewHtml::finishLoading(bool)
{
   DTRACE("QCastViewHtml::finishLoading");

   if (m_bHasForms) {
      if (! pView) return;
      QWebPage* pPage = page();
      QWebFrame* pFrame = NULL;
      if (pPage) pFrame = pPage->currentFrame();
      if (!pFrame) return;

      cxd::CHtmlChunk* pForm;
      FOR_EACH(pForm, m_Chunks) {
         if (! pForm || pForm->type() != cxd::CHtmlChunk::form) continue;
         QString js = QString("CogxJsFillForm('#%1');").arg(QString::fromStdString(pForm->htmlid()));
         pFrame->evaluateJavaScript(js);
      }
   }
}

void QCastViewHtml::doFillHtmlFrom(const QString& formid)
{
   DTRACE("QCastViewHtml::doFillHtmlFrom");
   QWebPage* pPage = page();
   QWebFrame* pFrame = NULL;
   if (pPage) pFrame = pPage->currentFrame();
   if (pFrame) {
      QString js = QString("CogxJsFillForm('#%1');").arg(formid);
      pFrame->evaluateJavaScript(js);
   }
}

void QCastViewHtml::setModel(cogx::display::CDisplayModel* pDisplayModel)
{
   pModel = pDisplayModel;
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
      m_bModified = false;
      emit updateContent(); // Queued update
      return;
   }
   QWebView::paintEvent(event);
}

void QCastViewHtml::doUpdateContent()
{
   DTRACE("QCastViewHtml::doUpdateContent");
   m_Chunks.clear();
   pView->getHtmlChunks(m_Chunks, cxd::CHtmlChunk::form | cxd::CHtmlChunk::activehtml);
   m_bHasForms = false;
   cxd::CHtmlChunk* pChunk;
   FOR_EACH(pChunk, m_Chunks) {
      if (pChunk && pChunk->type() == cxd::CHtmlChunk::form) {
         m_bHasForms = true;
         break;
      }
   }

   // TODO: save current form values and restore after update
   //    - store variables in state[formid]
   //    - in finishLoading: if state[formid] exists use it, else use pChunk->m_formData
   //    - maybe: use timestamps; if m_formData is newer than state, use m_formData!
   if (pView) {
      QStringList list, head, body;
      QWebPage* pPage = page();
      QWebFrame* pFrame = NULL;
      if (pPage) pFrame = pPage->currentFrame();
      int sbv = 0;
      if (pFrame) sbv = pFrame->scrollBarValue(Qt::Vertical);
      pView->drawHtml(head, body);
      list.append("<html><head>");

      if (m_bHasForms) {
         // Render std Display Server elements
         list << "<style> .v11nformbar { background-color: #f0f0f8; } .v11nformtitle { width: 200px; } </style>\n";

         // Loading jQuery is _very_ time consuming
         list << "<script type=\"text/javascript\">\n";
         list << m_jQuery;
         list << "</script>\n";
         list << m_jsFormCap;
      }

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
