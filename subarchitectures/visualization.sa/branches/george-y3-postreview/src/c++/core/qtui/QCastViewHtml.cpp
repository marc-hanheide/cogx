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
#include "../ptrvector.hpp"
#include <QWebFrame>
#include <QFile>
#include <QAction>
#include <QInputDialog>
#include <QLineEdit>
#include <QMessageBox>
#include <QToolButton>
#include <QAction>

#ifdef DEBUG_TRACE
#undef DEBUG_TRACE
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
   m_bBlockUpdates = false;
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
      file.setFileName(":/javascript/jquery.min.js");
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

   QAction* actFindText = new QAction(this);
   actFindText->setShortcut(QKeySequence(tr("Ctrl+F", "Edit|Find on page...")));
   actFindText->setText(QKeySequence(tr("Edit|Find on page...")));
   connect(actFindText, SIGNAL(triggered()), this, SLOT(onFindOnPage()));
   addAction(actFindText);

   QAction* actFindAgain = new QAction(this);
   actFindAgain->setShortcut(QKeySequence(tr("Ctrl+G", "Edit|Find agian on page...")));
   actFindAgain->setText(QKeySequence(tr("Edit|Find again on page...")));
   connect(actFindAgain, SIGNAL(triggered()), this, SLOT(onFindAgainOnPage()));
   addAction(actFindAgain);
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

void QCastViewHtml::onFindOnPage()
{
   QWebPage* pPage = page();
   if (pPage) {
      bool ok;
      QString text = QInputDialog::getText(
            this, tr("Find - CAST Viewer"),
            tr("Find text on page:"), QLineEdit::Normal,
            m_TextToFind, &ok);
   
      if (ok) {
         m_TextToFind = text;
         onFindAgainOnPage();
      }
   }
}

void QCastViewHtml::onFindAgainOnPage()
{
   QWebPage* pPage = page();
   if (pPage) {
      if (m_TextToFind.length() < 1) {
         QMessageBox msgBox;
         msgBox.setText("The search string is empty. Use Ctrl-F to set it.");
         msgBox.exec();
         return;
      }
      bool found = pPage->findText(m_TextToFind, QWebPage::FindWrapsAroundDocument);
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
   if (m_bModified && !m_bBlockUpdates) {
      m_bModified = false;
      emit updateContent(); // Queued update
      return;
   }
   QWebView::paintEvent(event);
}

void QCastViewHtml::doUpdateContent()
{
   DTRACE("QCastViewHtml::doUpdateContent");
   if (m_bBlockUpdates) {
      DTRACE("blocked");
      return;
   }
   m_Chunks.clear();
   // TODO: should getHtmlChunks observe CViewedObjectState.m_bVisible?
   pView->getHtmlChunks(m_Chunks, cxd::CHtmlChunk::form | cxd::CHtmlChunk::activehtml);

   cxd::CHtmlChunk* pChunk;
   m_bHasForms = false;
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
      list.append(
            "<html><head>"
            "<style>"
            " .v11ninfo { font-size: 90%; color: #808080; }"
            "</style>\n"
            );

      if (m_bHasForms) {
         // Render std Display Server elements
         list <<
            "<style>"
            " .v11nformbar { background-color: #e0e0f0; }"
            " .v11nformtitle { width: 200px; }"
            " .v11nformbutton { background-color: white; color: blue; padding: 0 4 0 4px;}"
            "</style>\n";

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

void QCastViewHtml::onToggleBlockUpdates(bool checked)
{
   m_bBlockUpdates = checked;
   if (! m_bBlockUpdates) update();
}

void QCastViewHtml::getToolbars(CPtrVector<QToolBar>& toolbars)
{
   if (! pView) return;
   // pBar->parent will be reset in QViewContainer
   QToolBar *pBar = new QToolBar(QString::fromStdString(pView->m_id), this);
   if (pBar) {
      QToolButton *pBut = new QToolButton(pBar);
      QString text = "Block updates";
      QAction* pAct = new QAction(QIcon(":/toolButton/camera-photo.png"), text, pBut);
      pAct->setCheckable(true);
      pAct->setChecked(m_bBlockUpdates);
      pBut->setDefaultAction(pAct);
      pBar->addWidget(pBut);
      pBar->connect(pAct, SIGNAL(toggled(bool)), this, SLOT(onToggleBlockUpdates(bool)));

      toolbars.push_back(pBar);
   }
}
