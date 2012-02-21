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

#include "QCustomGuiPanel.hpp"

#include <QCheckBox>
#include <QPushButton>
#include <QVBoxLayout>
#include "ChangeSlot.hpp"

#ifdef DEBUG_TRACE
#undef DEBUG_TRACE
#endif
#include "../convenience.hpp"

QCustomGuiPanel::QCustomGuiPanel(QWidget* parent, Qt::WindowFlags flags)
   :QFrame(parent, flags)
{
   m_pView = nullptr;
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

QCustomGuiPanel::~QCustomGuiPanel()
{
   if (m_pView) m_pView->viewObservers -= this;
}

void QCustomGuiPanel::onModel_UiDataChanged(cogx::display::CDisplayModel *pModel,
      cogx::display::CDisplayView *pSourceView,
      cogx::display::CGuiElement *pElement, const std::string& newValue)
{
   if (pSourceView == m_pView) return;
   if (pElement == nullptr) return;
   DTRACE("QCustomGuiPanel::onModel_UiDataChanged " << newValue);

   // If in the same thread, we can set the value for the control here
   // otherwise we emit a signal:
   // if (QThread::currentThread() != this->thread()) {
      QString val = QString::fromLatin1(newValue.c_str(), newValue.size());
      emit signalUiDataChanged(pModel, pSourceView, pElement, val);
      return;
   // }
   DVERIFYGUITHREAD("Custom Update", this);
}

void QCustomGuiPanel::doUiDataChanged(cogx::display::CDisplayModel *pModel,
      cogx::display::CDisplayView *pView,
      cogx::display::CGuiElement *pElement, QString newValue)
{
   DTRACE("slot QCustomGuiPanel::doUiDataChanged");
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

void QCustomGuiPanel::removeUi()
{
   DTRACE("QCustomGuiPanel::removeUi");

   if (layout()) delete layout();

   QObject *pobj;
   // TODO: CHECK if delete(findChildren()) works with comboboxes and other controls
   //    delete(findChildren()) was causing problems with HTML comboboxes in QViewContainer::removeUi
   //    it was changed to delete(children()) and crashes disappeared
   QList<QObject*> wdgts = findChildren<QObject*>();
   FOR_EACH(pobj, wdgts) {
      if (pobj)
         pobj->deleteLater();
   }
   m_controlCount = 0;
}

int QCustomGuiPanel::updateUi(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView)
{
   DTRACE("QCustomGuiPanel::updateUi");
   if (m_pView) m_pView->viewObservers -= this;

   setVisible(false);
   removeUi();

   m_pView = pView;
   if (!pModel || !pView) return 0;

   CPtrVector<cogx::display::CGuiElement> elements;
   pModel->getGuiElements(pView->m_id, elements);
   if (elements.size() < 1) return 0;

   m_controlCount = 0;

   // Create new widgets
   cogx::display::CGuiElement* pgel;
   CChangeSlot *pSlot;
   QCheckBox *pBox;
   QPushButton *pButton;

   QLayout *pLayout = new QVBoxLayout();

   FOR_EACH(pgel, elements) {
      if (!pgel) continue;
      switch (pgel->m_type) {
         case cogx::display::CGuiElement::wtCheckBox:
            pBox = new QCheckBox(QString(pgel->m_label.c_str()), this);
            pLayout->addWidget(pBox);
            pSlot = new CChangeSlot(pgel, pView, pBox);
            connect(pBox, SIGNAL(stateChanged(int)), pSlot, SLOT(onCheckBoxChange(int)));
            m_controlCount++;
            break;
         case cogx::display::CGuiElement::wtButton:
            pButton = new QPushButton(QString(pgel->m_label.c_str()), this);
            pLayout->addWidget(pButton);
            pSlot = new CChangeSlot(pgel, pView, pButton);
            connect(pButton, SIGNAL(clicked(bool)), pSlot, SLOT(onButtonClick(bool)));
            m_controlCount++;
            break;
         default: break;
      }
   }

   setLayout(pLayout);
   if (pView) pView->viewObservers += this;
   setVisible(true);
   return m_controlCount;
}
