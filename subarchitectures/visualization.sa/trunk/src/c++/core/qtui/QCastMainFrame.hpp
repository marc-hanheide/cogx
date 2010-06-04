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
#include "../Model.hpp"

class QCastMainFrame:
   public QMainWindow,
   public cogx::display::CDisplayModelObserver
{
   Q_OBJECT
private:
   Ui::MainWindow ui;

private:
   cogx::display::CDisplayModel* m_pModel;

public:
   QCastMainFrame( QWidget * parent = 0, Qt::WindowFlags flags = 0 );
   ~QCastMainFrame();
   void setModel(cogx::display::CDisplayModel* pDisplayModel);

   void notifyObjectAdded(cogx::display::CDisplayObject *pObject);

public slots:
   void onViewActivated(QListWidgetItem *pSelected);

private:
   void updateCustomUi(cogx::display::CDisplayView *pView);
   void updateViewList();

   // CDisplayModelObserver notifications
private:
   void onViewAdded(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView);

// signals/slots for interthread communication
signals: 
   void signalViewAdded(cogx::display::CDisplayModel*, cogx::display::CDisplayView*);
private slots:
   void doViewAdded(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView);
};


#endif /* QCASTMAINFRAME_DYSTP55V */
