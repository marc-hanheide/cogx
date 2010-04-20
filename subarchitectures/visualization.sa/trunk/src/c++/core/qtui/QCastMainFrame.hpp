/*
 * Author: Marko Mahnič
 * Created: 2010-03-10
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

   // CDisplayModelObserver
private:
   void onViewAdded(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView);
};


#endif /* QCASTMAINFRAME_DYSTP55V */
