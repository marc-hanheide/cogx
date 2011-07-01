/*
 * Author: Marko Mahnič
 * Created: 2010-04-21
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

#include "QCustomToolBar.hpp"

#include <QToolButton>
#include <QAction>
#include <QPixmap>
#include <QPainter>
#include "ChangeSlot.hpp"

#ifdef DEBUG_TRACE
#undef DEBUG_TRACE
#endif
#include "../convenience.hpp"

QCustomToolBar::QCustomToolBar(QWidget* parent)
   :QToolBar(parent)
{
   m_pView = NULL;
   // see: <url:QCastMainFrame.cpp#tn=QCastMainFrame::onViewAdded>
   connect(this, 
      SIGNAL(signalUiDataChanged(cogx::display::CDisplayModel*, cogx::display::CDisplayView*,
            cogx::display::CGuiElement*, QString)),
      this,
      SLOT(doUiDataChanged(cogx::display::CDisplayModel*, cogx::display::CDisplayView*,
            cogx::display::CGuiElement*, QString)),
      Qt::QueuedConnection);
   m_controlCount = 0;
}

QCustomToolBar::~QCustomToolBar()
{
   if (m_pView) m_pView->viewObservers -= this;
}

void QCustomToolBar::onModel_UiDataChanged(cogx::display::CDisplayModel *pModel,
      cogx::display::CDisplayView *pSourceView,
      cogx::display::CGuiElement *pElement, const std::string& newValue)
{
   if (pSourceView == m_pView) return;
   if (pElement == NULL) return;
   DTRACE("QCustomToolBar::onModel_UiDataChanged " << newValue);

   // If in the same thread, we can set the value for the control here
   // otherwise we emit a signal:
   // if (QThread::currentThread() != this->thread()) {
      QString val = QString::fromLatin1(newValue.c_str(), newValue.size());
      emit signalUiDataChanged(pModel, pSourceView, pElement, val);
      return;
   // }
   DVERIFYGUITHREAD("Custom Update", this);
}

void QCustomToolBar::doUiDataChanged(cogx::display::CDisplayModel *pModel,
      cogx::display::CDisplayView *pView,
      cogx::display::CGuiElement *pElement, QString newValue)
{
   DTRACE("slot QCustomToolBar::doUiDataChanged");
   DVERIFYGUITHREAD("Custom Update", this);

   // NOTE: Slots MUST be owned by the appropriate widgets
   QList<CChangeSlot*> cslots = findChildren<CChangeSlot*>();
   // DMESSAGE("Slots found: " << cslots.size());
   CChangeSlot *pslot;
   FOR_EACH(pslot, cslots) {
      if (!pslot) continue;
      if (pslot->guiElementPtr() != pElement) continue;
      //DMESSAGE("I'm going to set: " << pslot->m_sGuiObserverName << " to " << newValue.toStdString());
      pslot->onGuiElement_CtrlDataChanged(pElement, newValue.toStdString());
   }
}

void QCustomToolBar::removeUi()
{
   DTRACE("QCustomToolBar::removeUi");

   bool sigBlocked = blockSignals(true);
   clear();
   m_controlCount = 0;
   blockSignals(sigBlocked);
}

int QCustomToolBar::updateUi(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView)
{
   DTRACE("QCustomToolBar::updateUi");
   if (m_pView) m_pView->viewObservers -= this;

   removeUi();

   m_pView = pView;
   if (!pModel || !pView) return 0;

   CPtrVector<cogx::display::CGuiElement> elements;
   pModel->getGuiElements(pView->m_id, elements);
   if (elements.size() < 1) return 0;

   bool sigBlocked = blockSignals(true);
   m_controlCount = 0;

   // Create new widgets
   cogx::display::CGuiElement* pgel;
   CChangeSlot *pSlot;
   QToolBar *pBar = this;

   FOR_EACH(pgel, elements) {
      if (!pgel) continue;
      if (pgel->m_type != cogx::display::CGuiElement::wtAction)
         continue;
      QToolButton *pBtn;
      QIcon icon;
      pBtn = new QToolButton(pBar);
      QString text = QString::fromStdString(pgel->m_iconLabel);
      if (pgel->m_iconSvg.length() > 0) {
         if (strncmp(pgel->m_iconSvg.c_str(), "text:", 5) == 0) {
            QString sym = QString::fromStdString(pgel->m_iconSvg.substr(5));
            QPixmap pixmap(22, 22);
            pixmap.fill(QColor(0, 0, 0, 0));
            QPainter p(&pixmap);
            QRect r = p.fontMetrics().boundingRect(sym);
            int x = (22 - r.width()) / 2;
            if (x < 0) x = 0;
            int y = (22 - r.height()) / 2;
            if (y < 0) y = 0;
            y = 20 - y;
            p.drawText(QPointF(x, y), sym);
            icon = QIcon(pixmap);
         }
      }
      QAction* pAct = new QAction(icon, text, pBtn);
      pAct->setToolTip(QString::fromStdString(pgel->m_tooltip));
      pAct->setCheckable(pgel->m_bCheckable);
      pBtn->setDefaultAction(pAct);
      pBar->addWidget(pBtn);
      CChangeSlot* pSlot = new CChangeSlot(pgel, pView, pAct);
      connect(pAct, SIGNAL(triggered(bool)), pSlot, SLOT(onButtonClick(bool)));
      ++m_controlCount;
   }

   if (pView) pView->viewObservers += this;

   blockSignals(sigBlocked);
   return m_controlCount;
}
