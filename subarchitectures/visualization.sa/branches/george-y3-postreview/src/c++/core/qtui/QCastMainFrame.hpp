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
#ifndef QCASTMAINFRAME_DYSTP55V
#define QCASTMAINFRAME_DYSTP55V

#include "ui_castor.h"
#include <QMainWindow>
#include <QSettings>
#include <QByteArray>
#include <QMutex>
#include <QTreeWidgetItem>
#include <QToolBar>
#include "QCastDialogFrame.hpp"
#include "../Model.hpp"

class QCastMainFrame;
class QCastFrameManager: public QObject
{
   Q_OBJECT
private:
   struct FrameInfo
   {
      QCastMainFrame *pFrame;
      QString viewid;
      QByteArray geometry;
      QByteArray state;
      bool main;
      FrameInfo()
      {
         pFrame = NULL;
         main = false;
      }
   };
   QList<FrameInfo> m_frameList;
   QCastMainFrame* checkMainWindow(QCastMainFrame *pSkip);
   QList<QCastMainFrame*> getCastFrames();
   QMutex m_creatorMutex;
   QMutex m_closerMutex;

   QCastDialogFrame *m_pDialogFrame;

public:
   QCastFrameManager();
   ~QCastFrameManager();
   void saveWindowList(QString listName);
   void loadWindowList(QString listName);
   void getWindowListNames(QStringList& names, bool bMru=false);
   QString getStartupLayout();
   void setStartupLayout(QString name);
   void createMissingWindows(QCastMainFrame* pSomeFrame, cogx::display::CDisplayModel *pModel);
   void closeChildWindows();
   void updateControlStateForAll();
   QCastDialogFrame* getDialogManager();
   bool hasDialogs()
   {
      return m_pDialogFrame != NULL;
   }

public slots:
   void frameDestroyed(QObject *pobj);
   void dialogWindowHidden(QObject *pobj);
};

extern QCastFrameManager FrameManager;

class QCastMainFrame:
   public QMainWindow,
   public cogx::display::CDisplayModelObserver
{
   Q_OBJECT
private:
   friend class QCastFrameManager;
   Ui::MainWindow ui;
   QString m_winText;
   bool m_isMainWindow;
   QList<QToolBar*> m_customToolbars;

private:
   cogx::display::CDisplayModel* m_pModel;
   cogx::display::COwnerDataProxy* m_pControlDataProxy;

public:
   QCastMainFrame( QWidget * parent = 0, Qt::WindowFlags flags = 0 );
   ~QCastMainFrame();
   void setModel(cogx::display::CDisplayModel* pDisplayModel);
   void setControlDataProxy(cogx::display::COwnerDataProxy *pProxy);
   void loadStartupLayout();

   void notifyObjectAdded(cogx::display::CDisplayObject *pObject);

   QSettings* getPersistentStorage();

public slots:
   void onViewActivated(QListWidgetItem *pSelected);
   void onViewActivatedAction();

private slots:
   void onShowViewListChanged();
   void onShowToolbarsChanged();
   void onShowDialogWindow();
   void onShowCustomControls();
   void onRefreshViewList();
   void onNewWindow();
   void onSaveWindowList();
   void onCloseSomeWindows();
   void onRestoreWindowLayout();
   void onSetStartupLayout();

private:
   cogx::display::CDisplayView* getView();
   void updateCustomUi(cogx::display::CDisplayView *pView);
   void retrieveControlData(cogx::display::CDisplayView *pView);
   void updateViewList();
   void updateViewMenu();
   void updateToolBars();
   void updateControlState();
   void syncViewListItem();
   void updateObjectList(cogx::display::CDisplayView *pView);
   QWidgetList getCastFrames();

   // CDisplayModelObserver notifications
private:
   void onViewAdded(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView);
   void onViewChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView);
   void onDialogAdded(cogx::display::CDisplayModel *pModel, cogx::display::CGuiDialog *pDialog);
   void setView(cogx::display::CDisplayView *pView);
   void setChildMode(QCastMainFrame* pCreator = NULL);
   void closeEvent(QCloseEvent *event);
   QCastMainFrame* createChildWindow();

   // QTreeWidget events
private slots:
   void onTreeItemChanged(QTreeWidgetItem* pItem, int column);

// signals/slots for interthread communication
signals: 
   void signalViewAdded(cogx::display::CDisplayModel*, cogx::display::CDisplayView*);
   void signalViewChanged(cogx::display::CDisplayModel*, cogx::display::CDisplayView*);
   void signalDialogAdded(cogx::display::CDisplayModel *pModel, cogx::display::CGuiDialog *pDialog);
private slots:
   void doViewAdded(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView);
   void doViewChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView);
   void doDialogAdded(cogx::display::CDisplayModel *pModel, cogx::display::CGuiDialog *pDialog);
};


#endif /* QCASTMAINFRAME_DYSTP55V */
