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
   m_pView = NULL;
   // see: <url:QCastMainFrame.cpp#tn=QCastMainFrame::onViewAdded>
   connect(this, 
      SIGNAL(signalUiDataChanged(cogx::display::CDisplayModel*, cogx::display::CDisplayView*,
            cogx::display::CGuiElement*, QString)),
      this,
      SLOT(doUiDataChanged(cogx::display::CDisplayModel*, cogx::display::CDisplayView*,
            cogx::display::CGuiElement*, QString)),
      Qt::QueuedConnection);
}

QCustomGuiPanel::~QCustomGuiPanel()
{
   if (m_pView) m_pView->viewObservers -= this;
}

void QCustomGuiPanel::onUiDataChanged(cogx::display::CDisplayModel *pModel,
      cogx::display::CDisplayView *pSourceView,
      cogx::display::CGuiElement *pElement, const std::string& newValue)
{
   if (pSourceView == m_pView) return;
   if (pElement == NULL) return;
   DTRACE("QCustomGuiPanel::onUiDataChanged " << newValue);

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
   QCheckBox *pBox;
   FOR_EACH(pslot, cslots) {
      if (!pslot) continue;
      if (pslot->m_pGuiElement != pElement) continue;
      // DMESSAGE("I'm going to set: " << pslot->m_pGuiElement->m_id << " to " << newValue.toStdString());
      switch (pElement->m_type) {
         case cogx::display::CGuiElement::wtCheckBox:
            pBox = dynamic_cast<QCheckBox*>(pslot->parent());
            if (pBox != NULL) {
               pBox->blockSignals(true);
               if (newValue == "0") pBox->setCheckState(Qt::Unchecked);
               else pBox->setCheckState(Qt::Checked);
               pBox->blockSignals(false);
            }
            break;
         default: break;
      };
   }
}

void QCustomGuiPanel::removeUi()
{
   DTRACE("QCustomGuiPanel::removeUi");

   if (layout()) delete layout();

   // Remove the current widgets; they should be deleted when todelete goes out of scope.
   QWidget todelete;
   QList<QObject*> wdgts = findChildren<QObject*>();
   QObject *pobj;
   FOR_EACH(pobj, wdgts) {
      if (pobj) pobj->setParent(&todelete);
   }
}

void QCustomGuiPanel::updateUi(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView)
{
   DTRACE("QCustomGuiPanel::updateUi");
   setVisible(false);
   removeUi();

   m_pView = pView;
   if (!pView) return;

   CPtrVector<cogx::display::CGuiElement> elements;
   elements = pModel->getGuiElements(pView->m_id);
   if (elements.size() < 1) {
      if (pView) pView->viewObservers -= this;
      return;
   }

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
            break;
         case cogx::display::CGuiElement::wtButton:
            pButton = new QPushButton(QString(pgel->m_label.c_str()), this);
            pLayout->addWidget(pButton);
            pSlot = new CChangeSlot(pgel, pView, pButton);
            connect(pButton, SIGNAL(clicked(bool)), pSlot, SLOT(onButtonClick(bool)));
            break;
         default: break;
      }
   }

   setLayout(pLayout);
   if (pView) pView->viewObservers += this;
   setVisible(true);
}
