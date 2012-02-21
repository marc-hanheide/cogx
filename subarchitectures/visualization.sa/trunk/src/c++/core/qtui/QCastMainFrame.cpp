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
#include "QCastViewBase.hpp"
#include "../ptrvector.hpp"
#include "../HtmlElements.hpp"
#include "ChangeSlot.hpp"
#include <QSettings>
#include <QCloseEvent>
#include <QMessageBox>
#include <QInputDialog>
#include <QToolButton>
#include <QAction>
#ifdef V11N_OBJECT_HTML
#include <QWebSettings>
#endif
#include <QMdiArea>

#ifdef DEBUG_TRACE
// #undef DEBUG_TRACE
#endif
#include "../convenience.hpp"

#define EMPTY_LAYOUT  "<none>"

static void addToMru(QString mruName, QString value)
{
   QSettings settings("CogX", "CastDisplayServer");
   settings.beginGroup("MRU");
   QStringList list = settings.value(mruName, QStringList()).toStringList();
   list.removeOne(value);
   list.insert(0, value);
   settings.setValue(mruName, list);
   settings.endGroup();
}

static void applyMru(QString mruName, QStringList &list)
{
   QSettings settings("CogX", "CastDisplayServer");
   settings.beginGroup("MRU");
   QStringList mru = settings.value(mruName, QStringList()).toStringList();
   settings.endGroup();
   QStringList tmp(list);
   list.clear();
   for (int i = 0; i < mru.size(); i++) {
      if (mru[i].trimmed().size() < 1)
         continue;
      if (tmp.indexOf(mru[i]) >= 0) {
         list << mru[i].trimmed();
         tmp.removeOne(mru[i]);
      }
   }
   for (int i = 0; i < tmp.size(); i++) {
      if (tmp[i].trimmed().size() > 0)
         list << tmp[i].trimmed();
   }
}

QCastFrameManager FrameManager;

QCastFrameManager::QCastFrameManager()
{
   m_pDialogFrame = nullptr;
}

QCastFrameManager::~QCastFrameManager()
{
   if (m_pDialogFrame)
      m_pDialogFrame->close();
}

QCastDialogFrame* QCastFrameManager::getDialogManager()
{
   if (!m_pDialogFrame) {
      m_pDialogFrame = new QCastDialogFrame(0, 0);
      // ?????? connect(m_pDialogFrame, SIGNAL(itemClicked(QListWidgetItem*)),
      //      this, SLOT(dialogWindowHidden(QObject*)));
   }
   return m_pDialogFrame;
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

void QCastFrameManager::saveWindowList(QString listName)
{
   QList<QCastMainFrame*> frames = getCastFrames();

   listName = listName.trimmed();
   if (listName == "")
      listName = "Default";

   QSettings settings("CogX", "CastDisplayServer");
   settings.beginGroup("WindowLayout");
   settings.beginGroup(listName);
   settings.remove(""); // remove settings in current group
   int i = 0;
   foreach(QCastMainFrame* pFrame, frames) {
      i++;
      cogx::display::CDisplayView *pView = pFrame->ui.drawingArea->getActiveView();
      settings.beginGroup(QString("frame%1").arg(i));
      settings.setValue("main", pFrame->m_isMainWindow);
      if (pView)
         settings.setValue("view", QString::fromStdString(pView->m_id));
      settings.setValue("size", pFrame->size());
      settings.setValue("pos", pFrame->pos());
      settings.setValue("geometry", pFrame->saveGeometry());
      settings.setValue("state", pFrame->saveState());
      settings.endGroup(); // frame
   }
   {
      settings.beginGroup(QString("dialogFrame"));
      settings.setValue("visible", m_pDialogFrame && m_pDialogFrame->isVisible());
      if (m_pDialogFrame) {
         settings.setValue("geometry", m_pDialogFrame->saveGeometry());
         //settings.setValue("state", m_pDialogFrame->saveState());
      }
      settings.endGroup(); // dialogFrame
   }
   settings.endGroup(); // listName
   settings.endGroup(); // WindowLayout
   addToMru("WindowLayout", listName);
}

void QCastFrameManager::loadWindowList(QString listName)
{
   DTRACE("QCastMainFrame::loadWindowList");

   listName = listName.trimmed();
   if (listName == "")
      listName = "Default";

   m_frameList.clear();
   QSettings settings("CogX", "CastDisplayServer");
   settings.beginGroup("WindowLayout");
   settings.beginGroup(listName);
   QStringList frames = settings.childGroups();
   foreach(QString frameid, frames) {
      DMESSAGE(frameid.toStdString());
      FrameInfo info;
      settings.beginGroup(frameid);
      info.main = settings.value("main", false).toBool();
      info.viewid = settings.value("view", "").toString();
      info.geometry = settings.value("geometry", QByteArray()).toByteArray();
      info.state = settings.value("state", QByteArray()).toByteArray();
      settings.endGroup();
      m_frameList << info;
   }
   {
      settings.beginGroup(QString("dialogFrame"));
      bool bVisible = settings.value("visible", false).toBool();
      FrameInfo info;
      info.geometry = settings.value("geometry", QByteArray()).toByteArray();
      //info.state = settings.value("state", QByteArray()).toByteArray();

      getDialogManager(); // should create m_pDialogFrame
      if (m_pDialogFrame) {
         m_pDialogFrame->setVisible(bVisible);
         if (info.geometry.size() > 0)
            m_pDialogFrame->restoreGeometry(info.geometry);
         //if (info.state.size() > 0)
         //   m_pDialogFrame->restoreState(info.state);
      }

      settings.endGroup(); // dialogFrame
   }
   settings.endGroup(); // listName
   settings.endGroup(); // WindowLayout
   addToMru("WindowLayout", listName);
}

void QCastFrameManager::getWindowListNames(QStringList& names, bool bMru)
{
   QSettings settings("CogX", "CastDisplayServer");
   settings.beginGroup("WindowLayout");
   QStringList regnames = settings.childGroups();
   foreach(QString name, regnames) {
      names << name;
   }
   settings.endGroup(); // WindowLayout
   if (bMru)
      applyMru("WindowLayout", names);
}

QString QCastFrameManager::getStartupLayout()
{
   QSettings settings("CogX", "CastDisplayServer");
   settings.beginGroup("Startup");
   QString val = settings.value("WindowLayout", EMPTY_LAYOUT).toString();
   settings.endGroup(); // Startup
   return val;
}

void QCastFrameManager::setStartupLayout(QString name)
{
   QSettings settings("CogX", "CastDisplayServer");
   settings.beginGroup("Startup");
   settings.setValue("WindowLayout",name);
   settings.endGroup(); // Startup
}

// Make sure that exactly one of the windows is the main window (a m_isMainWindow)
QCastMainFrame* QCastFrameManager::checkMainWindow(QCastMainFrame* pSkip)
{
   DTRACE("QCastMainFrame::checkMainWindow");

   QList<QCastMainFrame*> frames = getCastFrames();
   if (frames.size() == 0) return nullptr;
   QCastMainFrame* pMain = nullptr;
   foreach(QCastMainFrame* pFrame, frames) {
      if (pFrame == pSkip) continue;
      if (pFrame->m_isMainWindow) {
         pMain = pFrame;
         break;
      }
   }
   if (pMain == nullptr) {
      // try to find a window with a visible view list
      foreach(QCastMainFrame* pFrame, frames) {
         if (pFrame == pSkip) continue;
         if (! pFrame->ui.actShowViewList->isChecked()) {
            pMain = pFrame;
            break;
         }
      }
   }

   if (pMain == nullptr) {
      auto it = frames.begin();
      if (it != frames.end()) {
         pMain = *it;
         if (pMain == pSkip) {
            it++;
            if (it == frames.end()) pMain = nullptr;
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

void QCastFrameManager::updateControlStateForAll()
{
   QList<QCastMainFrame*> frames = getCastFrames();
   foreach(QCastMainFrame* pFrame, frames) {
      pFrame->updateControlState();
   }
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

void QCastFrameManager::dialogWindowHidden(QObject *pObj)
{
   updateControlStateForAll();
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
      if (pinfo->pFrame != nullptr) continue;
      auto it = pModel->m_Views.find(pinfo->viewid.toStdString());
      if (it == pModel->m_Views.end()) continue;

      cogx::display::CDisplayView *pView = it->second;
      QCastMainFrame* pchild = nullptr;
      if (pinfo->main && pFrame->m_isMainWindow) {
         pchild = pFrame;
      }
      else {
         pchild = pFrame->createChildWindow();
      }
      if (pchild) {
         pinfo->pFrame = pchild;
         pchild->setView(pView);
         if (pinfo->geometry.size() > 0)
            pchild->restoreGeometry(pinfo->geometry);
         if (pinfo->state.size() > 0) {
            pchild->restoreState(pinfo->state);
            // TODO: update actions to reflect the visibility state of toolbars & contents
         }
         pchild->show();
      }
   }

   foreach(FrameInfo info, newFrames) {
      if (info.pFrame == nullptr)
         m_frameList << info;
   }
}

void QCastFrameManager::closeChildWindows()
{
   QMutexLocker lock(&m_closerMutex);
   DTRACE("QCastMainFrame::closeChildWindows");
   QCastMainFrame* pMain = checkMainWindow(nullptr);
   if (! pMain) return;

   QList<QCastMainFrame*> frames = getCastFrames();
   if (frames.size() < 2) return;
   foreach(QCastMainFrame* pFrame, frames) {
      if (pFrame != pMain) pFrame->close();
   }
}

QCastMainFrame::QCastMainFrame(QWidget * parent, Qt::WindowFlags flags)
   : QMainWindow(parent, flags)
{
   m_pModel = nullptr;
   m_pControlDataProxy = nullptr;
   m_isMainWindow = true;

   ui.setupUi(this);
   m_winText = windowTitle();
   setAttribute(Qt::WA_DeleteOnClose);

   ui.listWidget->setSelectionMode(QAbstractItemView::SingleSelection);

   // signal itemActivated requires double click on Gnome; use itemClicked instead. TODO: test
   connect(ui.listWidget, SIGNAL(itemClicked(QListWidgetItem*)),
         this, SLOT(onViewActivated(QListWidgetItem*)));

   connect(ui.actShowViewList, SIGNAL(triggered()),
         this, SLOT(onShowViewListChanged()));
   ui.actShowViewList->setChecked(Qt::Checked);

   connect(ui.actShowToolbars, SIGNAL(triggered()),
         this, SLOT(onShowToolbarsChanged()));
   ui.actShowToolbars->setChecked(Qt::Checked);

   connect(ui.actShowDialogWindow, SIGNAL(triggered()),
         this, SLOT(onShowDialogWindow()));

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

   connect(ui.actSetStartupLayout, SIGNAL(triggered()),
         this, SLOT(onSetStartupLayout()));

   connect(ui.actShowCustomControls, SIGNAL(triggered()),
         this, SLOT(onShowCustomControls()));

   connect(ui.treeObjects, SIGNAL(itemChanged(QTreeWidgetItem*, int)),
         this, SLOT(onTreeItemChanged(QTreeWidgetItem*, int)));

   ui.wgCustomGui->setVisible(false);
   ui.dockWidget->setVisible(ui.actShowViewList->isChecked());
   ui.ckCustomControls->setEnabled(false);
   ui.wgCustomGui->setVisible(false);

   // XXX: Passing pointers is unsafe
   qRegisterMetaType<cogx::display::CDisplayModel*>("cogx::display::CDisplayModel*");
   qRegisterMetaType<cogx::display::CDisplayView*>("cogx::display::CDisplayView*");
   qRegisterMetaType<cogx::display::CGuiElement*>("cogx::display::CGuiElement*"); // Custom UI
   qRegisterMetaType<cogx::display::CGuiDialog*>("cogx::display::CGuiDialog*"); // Custom UI
   connect(this, 
      SIGNAL(signalViewAdded(cogx::display::CDisplayModel*, cogx::display::CDisplayView*)),
      this,
      SLOT(doViewAdded(cogx::display::CDisplayModel*, cogx::display::CDisplayView*)),
      Qt::QueuedConnection);
   connect(this, 
      SIGNAL(signalViewChanged(cogx::display::CDisplayModel*, cogx::display::CDisplayView*)),
      this,
      SLOT(doViewChanged(cogx::display::CDisplayModel*, cogx::display::CDisplayView*)),
      Qt::QueuedConnection);
   connect(this, 
      SIGNAL(signalDialogAdded(cogx::display::CDisplayModel*, cogx::display::CGuiDialog*)),
      this,
      SLOT(doDialogAdded(cogx::display::CDisplayModel*, cogx::display::CGuiDialog*)),
      Qt::QueuedConnection);

   QStatusBar *pBar = statusBar();
   if (pBar) pBar->setVisible(false);

#ifdef V11N_OBJECT_HTML
#ifdef V11N_OBJECT_HTML_PLUGINS
   QWebSettings::globalSettings()->setAttribute(QWebSettings::PluginsEnabled, true);
#endif
   QWebSettings::globalSettings()->setAttribute(QWebSettings::JavascriptEnabled, true);
   QWebSettings::globalSettings()->setAttribute(QWebSettings::DeveloperExtrasEnabled, true);

   QWebSettings::globalSettings()->setFontSize(QWebSettings::DefaultFontSize, 12);
#endif

   setCentralWidget(ui.drawingArea);

#if 0
   QMdiArea* mdiArea = new QMdiArea();
   mdiArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
   mdiArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
   setCentralWidget(mdiArea);
   mdiArea->addSubWindow(ui.drawingArea);
   //ui.drawingArea->setAttribute(Qt::WA_DeleteOnClose);
   ui.drawingArea->showMaximized();
#endif
}

QCastMainFrame::~QCastMainFrame()
{
   if (m_pModel) m_pModel->modelObservers -= this;
   m_pModel = nullptr; // don't delete
}

void QCastMainFrame::setModel(cogx::display::CDisplayModel* pDisplayModel)
{
   if (m_pModel) m_pModel->modelObservers -= this;
   m_pModel = pDisplayModel;
   if (m_pModel) m_pModel->modelObservers += this;

   updateViewList();
   updateViewMenu();

   // XXX: Quick & Ugly hack. On model change, the old dialogs should be removed and
   // the new ones created. Here we assume that we only have one model.
   CPtrVector<cogx::display::CGuiDialog> dialogs;
   if (m_pModel) {
      m_pModel->getDialogs(dialogs);
      cogx::display::CGuiDialog *pdlg;
      FOR_EACH(pdlg, dialogs) {
         if (pdlg) {
            doDialogAdded(m_pModel, pdlg);
         }
      }
   }
}

void QCastMainFrame::setControlDataProxy(cogx::display::COwnerDataProxy *pProxy)
{
   m_pControlDataProxy = pProxy;
}

QSettings* QCastMainFrame::getPersistentStorage()
{
   std::string name = "DisplayServerData.ini";
   if (m_pControlDataProxy) {
      name = m_pControlDataProxy->getPersistentStorageName();
   }
   if (name.size() < 1) return nullptr;
   return new QSettings(name.c_str(), QSettings::IniFormat);
}

void QCastMainFrame::notifyObjectAdded(cogx::display::CDisplayObject *pObject)
{
   if (pObject != nullptr) {
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

void QCastMainFrame::syncViewListItem()
{
   //if (! ui.listWidget->isVisible()) return;
   cogx::display::CDisplayView *pView = ui.drawingArea->getActiveView();
   if (pView == nullptr) {
      ui.listWidget->setCurrentItem(nullptr);
      return;
   }

   QListWidgetItem* pCurrent = ui.listWidget->currentItem();
   if (pCurrent != nullptr && pCurrent->text().toStdString() == pView->m_id) {
      return;
   }

   for (int i = 0; i < ui.listWidget->count(); i++) {
      pCurrent = ui.listWidget->item(i);
      if (pCurrent != nullptr && pCurrent->text().toStdString() == pView->m_id) {
         ui.listWidget->setCurrentItem(pCurrent);
         break;
      }
   }
}

void QCastMainFrame::updateViewMenu()
{
   QMenu *pMenu = new QMenu(this);
   DVERIFYGUITHREAD("QViewSubmenu-s", this);
   //connect(pMenu, "triggered(QAction*)", this, "onViewActivated(QAction*)");

   cogx::display::CDisplayView *pView;
   // TODO: RLock pModel->m_Views
   FOR_EACH_V(pView, m_pModel->m_Views) {
      //QMenu* pSubmenu = pMenu->addMenu(tr(pView->m_id.c_str()));
      QAction* pAction = pMenu->addAction(tr(pView->m_id.c_str()));
      connect(pAction, SIGNAL(triggered()), this, SLOT(onViewActivatedAction()));
   }

   ui.actViewSelectSubmenu->setMenu(pMenu);
   ui.actViewSelectSubmenu->setEnabled(true);
}

// Brute-force update: remove everything and re-add
void QCastMainFrame::updateObjectList(cogx::display::CDisplayView *pView)
{
   QAbstractItemModel* pModel = ui.treeObjects->model();
   if (pModel) {
      int ni = pModel->rowCount();
      pModel->removeRows(0, ni);
   }
   if (!pView) return;

   // We block the singals so that itemChanged()..onTreeItemChanged() is not called.
   ui.treeObjects->blockSignals(true);
   CPtrVector<cogx::display::CDisplayObject> objects;
   pView->getObjects(objects);
   cogx::display::CDisplayObject *pObject;
   QStringList ss;
   FOR_EACH(pObject, objects) {
      ss.clear();
      ss << QString::fromStdString(pObject->m_id);
      QTreeWidgetItem* pItem = new QTreeWidgetItem(ui.treeObjects, ss);
      cogx::display::CViewedObjectState* pState = pView->getObjectState(pObject->m_id);
      pItem->setCheckState(0, pState->m_bVisible ? Qt::Checked : Qt::Unchecked);

      cogx::display::CDisplayObjectPart *pPart;
      CPtrVector<cogx::display::CDisplayObjectPart> parts;
      pObject->getParts(parts);
      FOR_EACH(pPart, parts) {
         ss.clear();
         ss << QString::fromStdString(pPart->m_id);
         QTreeWidgetItem *pChild = new QTreeWidgetItem(pItem, ss);
         
         pChild->setCheckState(0, pState->m_childState[pPart->m_id].m_bVisible ? Qt::Checked : Qt::Unchecked);
      }
   }
   ui.treeObjects->expandAll();
   ui.treeObjects->blockSignals(false);
}

void QCastMainFrame::updateControlState()
{
   bool hasDlgs = FrameManager.hasDialogs();
   ui.actShowDialogWindow->setEnabled(hasDlgs);
   ui.actShowDialogWindow->setChecked(hasDlgs && FrameManager.getDialogManager()->isVisible());
}

// This should only happen when the checkbox in column 0 changes.
// We update the object/part state in CDisplayView to reflect the value of the checkbox.
// TODO: The view should be redrawn!
void QCastMainFrame::onTreeItemChanged(QTreeWidgetItem* pItem, int column)
{
   DTRACE("onTreeItemChanged");
   if (column != 0) return;
   cogx::display::CDisplayView* pView = getView();
   if (!pView) return;

   bool bVisible = (pItem->checkState(column) == Qt::Checked);
   std::string objid;
   if (pItem->parent()) {
      objid = pItem->parent()->text(column).toStdString();
      QString partid = pItem->text(column);
      cogx::display::CViewedObjectState* pState = pView->getObjectState(objid);
      pState->m_childState[partid.toStdString()].m_bVisible = bVisible;
   }
   else {
      objid = pItem->text(column).toStdString();
      cogx::display::CViewedObjectState* pState = pView->getObjectState(objid);
      pState->m_bVisible = bVisible;
   }
   if (m_pModel) {
      m_pModel->refreshObject(objid, /*bNotifyChanged=*/ true);
   }

}

// Update the toolbars that depend on the current view type
void QCastMainFrame::updateToolBars()
{
   foreach(QToolBar* pbar, m_customToolbars) {
      pbar->deleteLater();
   }
   m_customToolbars.clear();

   ui.mainToolBar->setVisible(ui.actShowToolbars->isChecked());

   // Tools defined by the components for the current view
   ui.componentToolBar->setVisible(ui.actShowToolbars->isChecked() 
         && ui.componentToolBar->controlCount() > 0);

   QCastViewBase *pDisplay = ui.drawingArea->getDisplayWidget();
   if (!pDisplay)
      return;

   // Toolbars defined by the view type
   CPtrVector<QToolBar> bars;
   pDisplay->getToolbars(bars);
   QToolBar *pBar;
   FOR_EACH(pBar, bars) {
      if (pBar) {
         pBar->setVisible(ui.actShowToolbars->isChecked());
         addToolBar(pBar);
         m_customToolbars << pBar;
      }
   }
}

void QCastMainFrame::updateCustomUi(cogx::display::CDisplayView *pView)
{
   DTRACE("QCastMainFrame::updateCustomUi");
   DVERIFYGUITHREAD("Custom GUI", this);
   int nc = ui.wgCustomGui->updateUi(m_pModel, pView);
   ui.ckCustomControls->setEnabled(nc > 0);
   ui.wgCustomGui->setVisible(ui.ckCustomControls->isChecked() && nc > 0);

   nc = ui.componentToolBar->updateUi(m_pModel, pView);
}


void QCastMainFrame::onShowViewListChanged()
{
   ui.dockWidget->setVisible(ui.actShowViewList->isChecked());
}

void QCastMainFrame::onShowToolbarsChanged()
{
   foreach(QToolBar* pbar, m_customToolbars) {
      pbar->setVisible(ui.actShowToolbars->isChecked());
   }
   ui.mainToolBar->setVisible(ui.actShowToolbars->isChecked());
}

void QCastMainFrame::onShowCustomControls()
{
   ui.wgCustomGui->setVisible(ui.ckCustomControls->isChecked() && ui.wgCustomGui->controlCount() > 0);
}

void QCastMainFrame::onShowDialogWindow()
{
   if (! FrameManager.hasDialogs())
      return;

   QWidget* pw = FrameManager.getDialogManager();
   if (pw) {
      pw->setVisible(ui.actShowDialogWindow->isChecked());
      if (pw->isVisible())
         pw->setFocus();
      FrameManager.updateControlStateForAll();
   }
}

void QCastMainFrame::onRefreshViewList()
{
   updateViewList();
   updateViewMenu();
}

void QCastMainFrame::setChildMode(QCastMainFrame* pCreator)
{
   m_isMainWindow = false;
   ui.actShowViewList->setChecked(Qt::Unchecked);
   ui.actShowToolbars->setChecked(
         pCreator ? pCreator->ui.actShowToolbars->isChecked() : Qt::Checked);
   ui.dockWidget->setVisible(ui.actShowViewList->isChecked());
}

QCastMainFrame* QCastMainFrame::createChildWindow()
{
   QCastMainFrame* pchild = new QCastMainFrame();
   pchild->setChildMode(this);
   pchild->setModel(m_pModel);
   pchild->setControlDataProxy(m_pControlDataProxy);
   return pchild;
}

void QCastMainFrame::onNewWindow()
{
   QCastMainFrame* pchild = createChildWindow();
   ui.drawingArea->saveViewInfo();
   pchild->ui.drawingArea->initFrom(ui.drawingArea);
   pchild->setView(ui.drawingArea->getActiveView());
   pchild->show();
   FrameManager.updateControlStateForAll();
}

// Save window list to registry (conf)
void QCastMainFrame::onSaveWindowList()
{
   QInputDialog dlg(this);
   QStringList names;
   QString val;
   dlg.setWindowTitle("Display Server - Save Window Layout");
   dlg.setLabelText("Layout name");
   FrameManager.getWindowListNames(names, true);
   if (names.size() < 1)
      names << "Default";
   dlg.setComboBoxItems(names);
   dlg.setComboBoxEditable(true);
   dlg.setOkButtonText("Save");

   int rv = dlg.exec();
   if (rv != QDialog::Accepted)
      return;
   val = dlg.textValue().trimmed();
   if (val == "")
      val = "Default";

   FrameManager.saveWindowList(val);
}

// Close all windows except the main one
void QCastMainFrame::onCloseSomeWindows()
{
   FrameManager.closeChildWindows();
}

void QCastMainFrame::onRestoreWindowLayout()
{
   QInputDialog dlg(this);
   QStringList names;
   QString val;

   FrameManager.getWindowListNames(names, true);
   if (names.size() < 1)
      return;

   dlg.setWindowTitle("Display Server - Restore Window Layout");
   dlg.setLabelText("Layout name");
   dlg.setComboBoxItems(names);
   dlg.setComboBoxEditable(false);
   dlg.setOkButtonText("Restore");

   int rv = dlg.exec();
   if (rv != QDialog::Accepted)
      return;
   val = dlg.textValue().trimmed();
   if (val == "")
      val = names[0];

   FrameManager.closeChildWindows();
   FrameManager.loadWindowList(val);
   FrameManager.createMissingWindows(this, m_pModel);
   FrameManager.updateControlStateForAll();
}

void QCastMainFrame::onSetStartupLayout()
{
   QInputDialog dlg(this);
   QStringList names;
   QString val;

   FrameManager.getWindowListNames(names, true);
   val = FrameManager.getStartupLayout();
   names.removeOne(val);
   names.insert(0, val);
   if (names.indexOf(EMPTY_LAYOUT) < 0)
      names.insert(1, EMPTY_LAYOUT);

   dlg.setWindowTitle("Display Server - Startup Window Layout");
   dlg.setLabelText("Startup layout name");
   dlg.setComboBoxItems(names);
   dlg.setComboBoxEditable(false);

   int rv = dlg.exec();
   if (rv != QDialog::Accepted)
      return;
   val = dlg.textValue().trimmed();
   if (val == "")
      val = names[0];

   FrameManager.setStartupLayout(val);
}

void QCastMainFrame::loadStartupLayout()
{
   QString val = FrameManager.getStartupLayout().trimmed();
   if (val != "" && val != EMPTY_LAYOUT)
     FrameManager.loadWindowList(val);
}


void QCastMainFrame::setView(cogx::display::CDisplayView *pView)
{
   DTRACE("QCastMainFrame::setView " << pView);
   if (! m_pModel) return;
   if (pView && ! m_pModel->isValidView(pView)) {
      DMESSAGE("Invalid view, set to NULL");
      pView = nullptr;
   }

   if (! pView) {
      setWindowTitle(m_winText);
      updateCustomUi(nullptr);
      updateObjectList(nullptr);
      ui.drawingArea->setView(nullptr, nullptr);
   }
   else {
      if (ui.drawingArea->getActiveView() != pView) {
         setWindowTitle(QString::fromStdString(pView->m_id) + " - " + m_winText);
         ui.drawingArea->setView(m_pModel, pView);
         updateCustomUi(pView);
         updateObjectList(pView);
         retrieveControlData(pView);
      }
   }

   updateToolBars();
   syncViewListItem();
}

void QCastMainFrame::retrieveControlData(cogx::display::CDisplayView *pView)
{
   // retrieve data for custom widgets from remote display clients
   if (!m_pControlDataProxy || !pView) return;

   cogx::display::CGuiElement* pgel;
   CPtrVector<cogx::display::CGuiElement> elements;
   m_pModel->getGuiElements(pView->m_id, elements);
   FOR_EACH(pgel, elements) {
      if (!pgel) continue;
      m_pControlDataProxy->getControlStateAsync(pgel);
   }

   CPtrVector<cogx::display::CHtmlChunk> forms;
   // TODO: should getHtmlChunks observe CViewedObjectState.m_bVisible?
   pView->getHtmlChunks(forms, cogx::display::CHtmlChunk::form);
   cogx::display::CHtmlChunk* pForm;
   FOR_EACH(pForm, forms) {
      if (!pForm) continue;
      m_pControlDataProxy->getFormStateAsync(pForm);
   }
}

cogx::display::CDisplayView* QCastMainFrame::getView()
{
   if (ui.drawingArea)
      return ui.drawingArea->getActiveView();
   return nullptr;
}

void QCastMainFrame::closeEvent(QCloseEvent *event)
{
   setView(nullptr);
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

void QCastMainFrame::onViewActivatedAction()
{
   DTRACE("QCastMainFrame::onViewActivatedAction");
   if (! m_pModel) return;
   QAction* pSelected = dynamic_cast<QAction*>(sender());
   if (! pSelected) return;
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
   updateViewMenu();
   // ui.drawingArea->onViewChanged(pModel, pView);
}

void QCastMainFrame::onViewChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView)
{
   DTRACE("QCastMainFrame::onViewChanged " << (pView ? pView->m_id : "NULL"));
   if (pView == getView()) {
      DMESSAGE("emitting signalViewChanged");
      emit signalViewChanged(pModel, pView); // connection to doViewChanged has to be of type 'queued' or 'auto'
   }
}

void QCastMainFrame::doViewChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView)
{
   DTRACE("QCastMainFrame::doViewChanged");
   if (pView == getView())
      updateObjectList(pView);
}

void QCastMainFrame::onDialogAdded(cogx::display::CDisplayModel *pModel, cogx::display::CGuiDialog *pDialog)
{
   emit signalDialogAdded(pModel, pDialog); // connection to doDialogAdded has to be of type 'queued' or 'auto'
}

void QCastMainFrame::doDialogAdded(cogx::display::CDisplayModel *pModel, cogx::display::CGuiDialog *pDialog)
{
   DTRACE("QCastMainFrame::doDialogAdded");
   // let the FrameManager do the work (dialogs are all in one frame, tabbed)
   // GetDialogFrame()
   // FindDialog(pDialog)
   // if NULL: add tab, create ui, add to tab
   QCastDialogFrame* pdlgwin = FrameManager.getDialogManager();
   if (pdlgwin) {
      pdlgwin->addDialog(pDialog);
   }
   FrameManager.updateControlStateForAll();
}
