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
   ui.listWidget->clear();
   if (! m_pModel) return;

   cogx::display::CDisplayView *pView;
   FOR_EACH_V(pView, m_pModel->m_Views) {
      QListWidgetItem* pItem = new QListWidgetItem(tr(pView->m_id.c_str()), ui.listWidget);
      // TODO: notify on click
   }
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
         // TODO: should retrieve data for custom widgets from appropriate components.
      }
      ui.drawingArea->setView(pView);
   }
}

void QCastMainFrame::updateCustomUi(cogx::display::CDisplayView *pView)
{
   DTRACE("QCastMainFrame::updateCustomUi");
   if (!m_pModel || !pView) {
      ui.wgCustomGui->setVisible(false);
      return;
   }
   ui.wgCustomGui->updateUi(m_pModel, pView);
}

void QCastMainFrame::onViewAdded(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView)
{
   // TODO: only if the view is not in the list
   updateViewList();
   ui.drawingArea->onViewChanged(pModel, pView);
}
