/*
 * Author: Marko Mahnič
 * Created: 2010-03-10
 */

#include "QCastMainFrame.hpp"
#include "../convenience.hpp"

#include <cstdio> // TODO: Temporary (printf); remove

QCastMainFrame::QCastMainFrame(QWidget * parent, Qt::WindowFlags flags)
   : QMainWindow(parent, flags)
{
   m_pModel = NULL;
   ui.setupUi(this);
   ui.listWidget->setSelectionMode(QAbstractItemView::SingleSelection);
   connect(ui.listWidget, SIGNAL(itemActivated(QListWidgetItem*)),
         this, SLOT(onViewActivated(QListWidgetItem*)));
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
   if (!m_pModel || !pView) {
      ui.wgCustomGui->setVisible(false);
      return;
   }
   DVERIFYGUITHREAD("Custom GUI", this);
   ui.wgCustomGui->updateUi(m_pModel, pView);
}

// A view was activated from the GUI
void QCastMainFrame::onViewActivated(QListWidgetItem *pSelected)
{
   DTRACE("QCastMainFrame::onViewActivated");
   if (! pSelected) return;
   cogx::display::CDisplayView *pView;
   DMESSAGE(pSelected->text().toStdString());
   pView = m_pModel->getView(pSelected->text().toStdString());
   if (pView) {
      if (! ui.wgCustomGui->hasView(pView)) {
         updateCustomUi(pView);
         // TODO: should retrieve data for custom widgets from appropriate remote display clients.
      }
      ui.drawingArea->setView(pView);
   }
}

// XXX: This function is called from another thread.
// According to Qt docs, all GUI elements should be created in the main (GUI) thread!
// TODO: Don't call updateViewList; instead add a request to some queue that will be
// processed by the main thread.
void QCastMainFrame::onViewAdded(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView)
{
   // TODO: only if the view is not in the list
   updateViewList();
   // ui.drawingArea->onViewChanged(pModel, pView);
}
