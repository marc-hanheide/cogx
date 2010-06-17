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
#include <QSettings>
#include <QCloseEvent>
#include <QMessageBox>

#include "../convenience.hpp"

QCastFrameManager::QCastFrameManager()
{
}

QList<QCastMainFrame*> QCastFrameManager::getCastFrames()
{
   QList<QCastMainFrame*> frames;
   foreach(QWidget* pWidget, QApplication::allWidgets()) {
      if (! pWidget->inherits("QCastMainFrame")) continue;
      QCastMainFrame *pFrame = dynamic_cast<QCastMainFrame*>(pWidget);
      if (! pFrame) continue;
      frames << pFrame;
   }
   return frames;
}

void QCastFrameManager::saveWindowList()
{
   QList<QCastMainFrame*> frames = getCastFrames();

   QSettings settings("CogX", "CastDisplayServer");
   settings.beginGroup("WindowLayout");
   settings.beginGroup("Default");
   settings.remove(""); // remove settings in current group
   int i = 0;
   foreach(QCastMainFrame* pFrame, frames) {
      i++;
      cogx::display::CDisplayView *pView = pFrame->ui.drawingArea->getView();
      settings.beginGroup(QString("frame%1").arg(i));
      settings.setValue("main", pFrame->m_isMainWindow);
      if (pView)
         settings.setValue("view", QString::fromStdString(pView->m_id));
      settings.setValue("size", pFrame->size());
      settings.setValue("pos", pFrame->pos());
      settings.endGroup(); // frame
   }
   settings.endGroup(); // Default
   settings.endGroup(); // WindowLayout
}

void QCastFrameManager::loadWindowList()
{
   DTRACE("QCastMainFrame::loadWindowList");

   m_frameList.clear();
   QSettings settings("CogX", "CastDisplayServer");
   settings.beginGroup("WindowLayout");
   settings.beginGroup("Default");
   QStringList frames = settings.childGroups();
   foreach(QString frameid, frames) {
      DMESSAGE(frameid.toStdString());
      FrameInfo info;
      settings.beginGroup(frameid);
      info.main = settings.value("main", false).toBool();
      info.viewid = settings.value("view", "").toString();
      info.size = settings.value("size", QSize(400, 400)).toSize();
      info.pos = settings.value("pos", QPoint(200, 200)).toPoint();
      settings.endGroup();
      m_frameList << info;
   }
   settings.endGroup(); // Default
   settings.endGroup(); // WindowLayout
}

// Make sure that exactly one of the windows is the main window (a m_isMainWindow)
QCastMainFrame* QCastFrameManager::checkMainWindow(QCastMainFrame* pSkip)
{
   DTRACE("QCastMainFrame::checkMainWindow");

   QList<QCastMainFrame*> frames = getCastFrames();
   if (frames.size() == 0) return NULL;
   QCastMainFrame* pMain = NULL;
   foreach(QCastMainFrame* pFrame, frames) {
      if (pFrame == pSkip) continue;
      if (pFrame->m_isMainWindow) {
         pMain = pFrame;
         break;
      }
   }
   if (pMain == NULL) {
      // try to find a window with a visible view list
      foreach(QCastMainFrame* pFrame, frames) {
         if (pFrame == pSkip) continue;
         if (! pFrame->ui.actShowViewList->isChecked()) {
            pMain = pFrame;
            break;
         }
      }
   }

   if (pMain == NULL) {
      typeof(frames.begin()) it = frames.begin();
      if (it != frames.end()) {
         pMain = *it;
         if (pMain == pSkip) {
            it++;
            if (it == frames.end()) pMain = NULL;
            else pMain = *it;
         }
      }
      DMESSAGE("MAIN WINDOW IS NULL");
   }

   foreach(QCastMainFrame* pFrame, frames) {
      pFrame->m_isMainWindow = (pFrame == pMain);
   }

   return pMain;
}

void QCastFrameManager::frameDestroyed(QObject *pObj)
{
   DTRACE("QCastFrameManager::frameDestroyed");
   QCastMainFrame *pFrame = dynamic_cast<QCastMainFrame*>(pObj);
   if (! pFrame) return;
   if (pFrame->m_isMainWindow) {
      checkMainWindow(pFrame);
   }
}

void QCastFrameManager::createMissingWindows(QCastMainFrame* pFrame, cogx::display::CDisplayModel *pModel)
{
   QMutexLocker lock(&m_creatorMutex);
   if (! pFrame || ! pModel) return;
   if (m_frameList.count() < 1) return;
   DTRACE("QCastFrameManager::createMissingWindows");

   QList<FrameInfo> newFrames = m_frameList;
   m_frameList.clear();

   // TODO: check if the view is in the list of saved views and create a frame for it if necessary
   // QList<QCastMainFrame*> frames = getCastFrames();
   for (int i = 0; i < newFrames.size(); ++i) {
      FrameInfo* pinfo = (FrameInfo*) &newFrames.at(i);
      if (pinfo->pFrame != NULL) continue;
      typeof(pModel->m_Views.begin()) it = pModel->m_Views.find(pinfo->viewid.toStdString());
      if (it == pModel->m_Views.end()) continue;

      cogx::display::CDisplayView *pView = it->second;
      QCastMainFrame* pchild = NULL;
      if (pinfo->main && pFrame->m_isMainWindow) {
         pchild = pFrame;
      }
      else {
         pchild = pFrame->createChildWindow();
      }
      if (pchild) {
         pinfo->pFrame = pchild;
         pchild->setView(pView);
         pchild->resize(pinfo->size);
         pchild->move(pinfo->pos);
         pchild->show();
         // usleep(200);
      }
   }

   foreach(FrameInfo info, newFrames) {
      if (info.pFrame == NULL)
         m_frameList << info;
   }
}

void QCastFrameManager::closeChildWindows()
{
   QMutexLocker lock(&m_closerMutex);
   DTRACE("QCastMainFrame::closeChildWindows");
   QCastMainFrame* pMain = checkMainWindow(NULL);
   if (! pMain) return;

   QList<QCastMainFrame*> frames = getCastFrames();
   if (frames.size() < 2) return;
   foreach(QCastMainFrame* pFrame, frames) {
      if (pFrame != pMain) pFrame->close();
   }
}

static QCastFrameManager FrameManager;

QCastMainFrame::QCastMainFrame(QWidget * parent, Qt::WindowFlags flags)
   : QMainWindow(parent, flags)
{
   m_pModel = NULL;
   m_pControlDataProxy = NULL;
   m_isMainWindow = true;

   ui.setupUi(this);
   m_winText = windowTitle();
   setAttribute(Qt::WA_DeleteOnClose);

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

   connect(ui.actSaveWindowLayout, SIGNAL(triggered()),
         this, SLOT(onSaveWindowList()));

   connect(ui.actCloseSomeWindows, SIGNAL(triggered()),
         this, SLOT(onCloseSomeWindows()));

   connect(ui.actRestoreWindowLayout, SIGNAL(triggered()),
         this, SLOT(onRestoreWindowLayout()));

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

   QStatusBar *pBar = statusBar();
   if (pBar) pBar->setVisible(false);
}

QCastMainFrame::~QCastMainFrame()
{
   if (m_pModel) m_pModel->modelObservers -= this;
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

   if (m_isMainWindow)
      FrameManager.createMissingWindows(this, m_pModel);
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
   m_isMainWindow = false;
   ui.actShowViewList->setChecked(Qt::Unchecked);
   ui.dockWidget->setVisible(ui.actShowViewList->isChecked());
}

QCastMainFrame* QCastMainFrame::createChildWindow()
{
   QCastMainFrame* pchild = new QCastMainFrame();
   pchild->setChildMode();
   pchild->setModel(m_pModel);
   pchild->setControlDataProxy(m_pControlDataProxy);
   return pchild;
}

void QCastMainFrame::onNewWindow()
{
   QCastMainFrame* pchild = createChildWindow();
   pchild->setView(ui.drawingArea->getView());
   pchild->show();
}

// Save window list to registry (conf)
void QCastMainFrame::onSaveWindowList()
{
   QMessageBox::StandardButton rv =
      QMessageBox::question(
            this, "Display Server", "Save window layout for later use.",
            QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
   if (rv != QMessageBox::Yes) return;
   FrameManager.saveWindowList();
}

// Close all windows except the main one
void QCastMainFrame::onCloseSomeWindows()
{
   FrameManager.closeChildWindows();
}

void QCastMainFrame::onRestoreWindowLayout()
{
   FrameManager.closeChildWindows();
   FrameManager.loadWindowList();
   FrameManager.createMissingWindows(this, m_pModel);
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

void QCastMainFrame::closeEvent(QCloseEvent *event)
{
   setView(NULL);
   FrameManager.frameDestroyed(this);
   event->accept();
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
