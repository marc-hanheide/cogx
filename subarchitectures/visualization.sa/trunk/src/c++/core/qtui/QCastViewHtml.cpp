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

static const QString jsObjectName("CastQFormProxy");
QString QCastViewHtml::m_jQuery;

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
}

void QCastViewHtml::createJsObjects()
{
   DTRACE("QCastViewHtml::createJsObjects");
   QWebPage* pPage = page();
   QWebFrame* pFrame = NULL;
   if (pPage) pFrame = pPage->currentFrame();
   if (pFrame) {
      QCastFormProxy* pObj = new QCastFormProxy();
      CPtrVector<cogx::display::CHtmlChunk> forms;
      if (pView) pView->getHtmlForms(forms);
      cogx::display::CHtmlChunk* pForm;
      FOR_EACH(pForm, forms) {
         pObj->registerForm(pForm);
      }
      pFrame->addToJavaScriptWindowObject(jsObjectName, pObj);
   }
}

void QCastViewHtml::finishLoading(bool)
{
   //progress = 100;
   //adjustTitle();
   // page()->mainFrame()->evaluateJavaScript(m_jQuery);
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

   // TODO: save current form values and restore after update
   if (pView) {
      QStringList list, head, body;
      QWebPage* pPage = page();
      QWebFrame* pFrame = NULL;
      if (pPage) pFrame = pPage->currentFrame();
      int sbv = 0;
      if (pFrame) sbv = pFrame->scrollBarValue(Qt::Vertical);
      pView->drawHtml(head, body);
      list.append("<html><head>");

      // TODO: if this->hasForms() {
      list << "<script type=\"text/javascript\">\n";
      list << m_jQuery;
      list << "</script>\n";
      list << m_jsFormCap;
      // }

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
