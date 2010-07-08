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
#include <QMutex>
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
      QSize size;
      QPoint pos;
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

public:
   QCastFrameManager();
   void saveWindowList();
   void loadWindowList();
   void createMissingWindows(QCastMainFrame* pSomeFrame, cogx::display::CDisplayModel *pModel);
   void closeChildWindows();

public slots:
   void frameDestroyed(QObject *pobj);
};

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

private:
   cogx::display::CDisplayModel* m_pModel;
   cogx::display::COwnerDataProxy* m_pControlDataProxy;

public:
   QCastMainFrame( QWidget * parent = 0, Qt::WindowFlags flags = 0 );
   ~QCastMainFrame();
   void setModel(cogx::display::CDisplayModel* pDisplayModel);
   void setControlDataProxy(cogx::display::COwnerDataProxy *pProxy);

   void notifyObjectAdded(cogx::display::CDisplayObject *pObject);

   QSettings* getPersistentStorage();

public slots:
   void onViewActivated(QListWidgetItem *pSelected);

private slots:
   void onShowViewListChanged();
   void onRefreshViewList();
   void onNewWindow();
   void onSaveWindowList();
   void onCloseSomeWindows();
   void onRestoreWindowLayout();

private:
   void updateCustomUi(cogx::display::CDisplayView *pView);
   void updateViewList();
   QWidgetList getCastFrames();

   // CDisplayModelObserver notifications
private:
   void onViewAdded(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView);
   void setView(cogx::display::CDisplayView *pView);
   void setChildMode();
   void closeEvent(QCloseEvent *event);
   QCastMainFrame* createChildWindow();

// signals/slots for interthread communication
signals: 
   void signalViewAdded(cogx::display::CDisplayModel*, cogx::display::CDisplayView*);
private slots:
   void doViewAdded(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView);
};


#endif /* QCASTMAINFRAME_DYSTP55V */
