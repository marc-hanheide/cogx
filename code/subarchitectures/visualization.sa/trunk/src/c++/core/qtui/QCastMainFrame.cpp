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
#include <QSettings>
#include <QCloseEvent>
#include <QMessageBox>
#include <QInputDialog>
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
      settings.endGroup(); // frame
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
      info.size = settings.value("size", QSize(400, 400)).toSize();
      info.pos = settings.value("pos", QPoint(200, 200)).toPoint();
      settings.endGroup();
      m_frameList << info;
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

   // signal itemActivated requires double click on Gnome; use itemClicked instead. TODO: test
   connect(ui.listWidget, SIGNAL(itemClicked(QListWidgetItem*)),
         this, SLOT(onViewActivated(QListWidgetItem*)));

   connect(ui.actShowViewList, SIGNAL(triggered()),
         this, SLOT(onShowViewListChanged()));
   ui.actShowViewList->setChecked(Qt::Checked);

   connect(ui.actShowToolbars, SIGNAL(triggered()),
         this, SLOT(onShowToolbarsChanged()));
   ui.actShowToolbars->setChecked(Qt::Checked);

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
   m_pModel = NULL; // don't delete
}

void QCastMainFrame::setModel(cogx::display::CDisplayModel* pDisplayModel)
{
   if (m_pModel) m_pModel->modelObservers -= this;
   m_pModel = pDisplayModel;
   if (m_pModel) m_pModel->modelObservers += this;

   updateViewList();
   updateViewMenu();
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
   if (name.size() < 1) return NULL;
   return new QSettings(name.c_str(), QSettings::IniFormat);
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

void QCastMainFrame::syncViewListItem()
{
   //if (! ui.listWidget->isVisible()) return;
   cogx::display::CDisplayView *pView = ui.drawingArea->getActiveView();
   if (pView == NULL) {
      ui.listWidget->setCurrentItem(NULL);
      return;
   }

   QListWidgetItem* pCurrent = ui.listWidget->currentItem();
   if (pCurrent != NULL && pCurrent->text().toStdString() == pView->m_id) {
      return;
   }

   for (int i = 0; i < ui.listWidget->count(); i++) {
      pCurrent = ui.listWidget->item(i);
      if (pCurrent != NULL && pCurrent->text().toStdString() == pView->m_id) {
         ui.listWidget->setCurrentItem(pCurrent);
         break;
      }
   }
}

void QCastMainFrame::updateToolBars()
{
   QCastViewBase *pDisplay = ui.drawingArea->getDisplayWidget();

   foreach(QToolBar* pbar, m_customToolbars) {
      pbar->deleteLater();
   }
   m_customToolbars.clear();

   if (!pDisplay)
      return;

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

void QCastMainFrame::updateCustomUi(cogx::display::CDisplayView *pView)
{
   DTRACE("QCastMainFrame::updateCustomUi");
   DVERIFYGUITHREAD("Custom GUI", this);
   int nc = ui.wgCustomGui->updateUi(m_pModel, pView);
   ui.ckCustomControls->setEnabled(nc > 0);
   ui.wgCustomGui->setVisible(ui.ckCustomControls->isChecked() && nc > 0);
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
}

void QCastMainFrame::onShowCustomControls()
{
   ui.wgCustomGui->setVisible(ui.ckCustomControls->isChecked() && ui.wgCustomGui->controlCount() > 0);
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
   pchild->setView(ui.drawingArea->getActiveView());
   pchild->show();
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
      pView = NULL;
   }

   if (! pView) {
      setWindowTitle(m_winText);
      updateCustomUi(NULL);
      updateObjectList(NULL);
      ui.drawingArea->setView(NULL, NULL);
   }
   else {
      setWindowTitle(QString::fromStdString(pView->m_id) + " - " + m_winText);
      if (! ui.wgCustomGui->hasView(pView)) {
         updateCustomUi(pView);
         updateObjectList(pView);
         // retrieve data for custom widgets from remote display clients
         if (m_pControlDataProxy) {
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
      }
      ui.drawingArea->setView(m_pModel, pView);
   }

   syncViewListItem();
   updateToolBars();
}

cogx::display::CDisplayView* QCastMainFrame::getView()
{
   if (ui.drawingArea)
      return ui.drawingArea->getActiveView();
   return 0;
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
