/*
 * Author: Marko Mahnič
 * Created: 2010-03-10
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
#include "QCastMainFrame.hpp"
#include "../convenience.hpp"

QCastMainFrame::QCastMainFrame(QWidget * parent, Qt::WindowFlags flags)
   : QMainWindow(parent, flags)
{
   m_pModel = NULL;
   m_pControlDataProxy = NULL;
   ui.setupUi(this);
   m_winText = windowTitle();
   ui.listWidget->setSelectionMode(QAbstractItemView::SingleSelection);
   connect(ui.listWidget, SIGNAL(itemActivated(QListWidgetItem*)),
         this, SLOT(onViewActivated(QListWidgetItem*)));

   connect(ui.actShowViewList, SIGNAL(triggered()),
         this, SLOT(onShowViewListChanged()));
   ui.actShowViewList->setChecked(Qt::Checked);

   connect(ui.actRefreshViewList, SIGNAL(triggered()),
         this, SLOT(onRefreshViewList()));

   connect(ui.actNewWindow, SIGNAL(triggered()),
         this, SLOT(onNewWindow()));

   ui.wgCustomGui->setVisible(false);
   ui.dockWidget->setVisible(ui.actShowViewList->isChecked());

   // XXX: Passing pointers is unsafe
   qRegisterMetaType<cogx::display::CDisplayModel*>("cogx::display::CDisplayModel*");
   qRegisterMetaType<cogx::display::CDisplayView*>("cogx::display::CDisplayView*");
   qRegisterMetaType<cogx::display::CGuiElement*>("cogx::display::CGuiElement*"); // Custom UI
   connect(this, 
      SIGNAL(signalViewAdded(cogx::display::CDisplayModel*, cogx::display::CDisplayView*)),
      this,
      SLOT(doViewAdded(cogx::display::CDisplayModel*, cogx::display::CDisplayView*)),
      Qt::QueuedConnection);
}

QCastMainFrame::~QCastMainFrame()
{
   m_pModel = NULL; // don't delete
}

void QCastMainFrame::setModel(cogx::display::CDisplayModel* pDisplayModel)
{
   if (m_pModel) m_pModel->modelObservers -= this;
   m_pModel = pDisplayModel;
   if (m_pModel) m_pModel->modelObservers += this;

   updateViewList();
}

void QCastMainFrame::setControlDataProxy(CControlDataProxy *pProxy)
{
   m_pControlDataProxy = pProxy;
}

void QCastMainFrame::notifyObjectAdded(cogx::display::CDisplayObject *pObject)
{
   if (pObject != NULL) {
      DMESSAGE("TODO: check if view for " << pObject->m_id << " exists, diplay in view-list");
   }
   // TODO: if (m_pCurrentView->hasObject(pObject)) m_pCurrentView->update();
}

void QCastMainFrame::updateViewList()
{
   DTRACE("QCastMainFrame::updateViewList");
   // TODO: remember current view, make selected after the list is recreated
   ui.listWidget->clear();
   if (! m_pModel) return;

   DVERIFYGUITHREAD("QListWidgetItem-s", this);

   cogx::display::CDisplayView *pView;
   // TODO: RLock pModel->m_Views
   FOR_EACH_V(pView, m_pModel->m_Views) {
      QListWidgetItem* pItem = new QListWidgetItem(tr(pView->m_id.c_str()), ui.listWidget);
      // TODO: notify on click
   }
}

void QCastMainFrame::updateCustomUi(cogx::display::CDisplayView *pView)
{
   DTRACE("QCastMainFrame::updateCustomUi");
   DVERIFYGUITHREAD("Custom GUI", this);
   ui.wgCustomGui->updateUi(m_pModel, pView);
}

void QCastMainFrame::onShowViewListChanged()
{
   ui.dockWidget->setVisible(ui.actShowViewList->isChecked());
}

void QCastMainFrame::onRefreshViewList()
{
   updateViewList();
}

void QCastMainFrame::setChildMode()
{
   ui.actShowViewList->setChecked(Qt::Unchecked);
   ui.dockWidget->setVisible(ui.actShowViewList->isChecked());
}

void QCastMainFrame::onNewWindow()
{
   QCastMainFrame* pchild = new QCastMainFrame();
   pchild->setModel(m_pModel);
   pchild->setControlDataProxy(m_pControlDataProxy);
   pchild->setChildMode();
   pchild->show();
   pchild->setView(ui.drawingArea->getView());
}

void QCastMainFrame::setView(cogx::display::CDisplayView *pView)
{
   DTRACE("QCastMainFrame::setView " << pView);
   if (! m_pModel) return;
   if (pView && ! m_pModel->isValidView(pView)) {
      DMESSAGE("Invalid view, set to NULL");
      pView = NULL;
   }

   if (! pView) {
      setWindowTitle(m_winText);
      updateCustomUi(NULL);
      ui.drawingArea->setView(NULL);
   }
   else {
      setWindowTitle(QString::fromStdString(pView->m_id) + " - " + m_winText);
      if (! ui.wgCustomGui->hasView(pView)) {
         updateCustomUi(pView);
         // retrieve data for custom widgets from remote display clients
         if (m_pControlDataProxy) {
            cogx::display::CGuiElement* pgel;
            CPtrVector<cogx::display::CGuiElement> elements;
            elements = m_pModel->getGuiElements(pView->m_id);
            FOR_EACH(pgel, elements) {
               if (!pgel) continue;
               m_pControlDataProxy->getControlStateAsync(pgel);
            }
         }
      }
      ui.drawingArea->setView(pView);
   }
}

// A view was activated from the GUI
void QCastMainFrame::onViewActivated(QListWidgetItem *pSelected)
{
   DTRACE("QCastMainFrame::onViewActivated");
   if (! pSelected) return;
   if (! m_pModel) return;
   cogx::display::CDisplayView *pView;
   DMESSAGE(pSelected->text().toStdString());
   pView = m_pModel->getView(pSelected->text().toStdString());
   setView(pView);
}

// XXX: This function is called from another thread.
// According to Qt docs, all GUI elements should be created in the main (GUI) thread!
// Don't call updateViewList; instead add a request to some queue that will be processed by the
// main thread. This can be done with Qt 'queued' connections. onViewAdded() emits the signal
// signalViewAdded() which adds a request to the GUI thread's queue. When the thread wakes up
// it executes the (connected) slot doViewAdded() which can now safely update the GUI elements.
void QCastMainFrame::onViewAdded(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView)
{
   // TODO: only if the view is not in the list
   emit signalViewAdded(pModel, pView); // connection to doViewAdded has to be of type 'queued' or 'auto'
}

void QCastMainFrame::doViewAdded(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView)
{
   updateViewList();
   // ui.drawingArea->onViewChanged(pModel, pView);
}
