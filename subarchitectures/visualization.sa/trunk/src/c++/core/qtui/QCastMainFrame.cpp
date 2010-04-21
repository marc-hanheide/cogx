/*
 * Author: Marko Mahnič
 * Created: 2010-03-10
 */

#include "QCastMainFrame.hpp"

#include <cstdio> // TODO: Temporary (printf); remove

QCastMainFrame::QCastMainFrame(QWidget * parent, Qt::WindowFlags flags)
   : QMainWindow(parent, flags)
{
   m_pModel = NULL;
   ui.setupUi(this);
   ui.listWidget->setSelectionMode(QAbstractItemView::SingleSelection);
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
   if (pObject != NULL)
      printf ("TODO: check if view for %s exists, diplay in view-list", pObject->m_id.c_str());
   // TODO: if (m_pCurrentView->hasObject(pObject)) m_pCurrentView->update();
}

void QCastMainFrame::updateViewList()
{
   ui.listWidget->clear();
   if (! m_pModel) return;

   cogx::display::CDisplayView *pView;
   FOR_EACH_V(pView, m_pModel->m_Views) {
      QListWidgetItem* pItem = new QListWidgetItem(tr(pView->m_id.c_str()), ui.listWidget);
      connect(ui.listWidget, SIGNAL(itemActivated(QListWidgetItem*)),
            this, SLOT(onViewActivated(QListWidgetItem*)));
      // TODO: notify on click
   }
}


// A view was activated from the GUI
void QCastMainFrame::onViewActivated(QListWidgetItem *pSelected)
{
   if (! pSelected) return;
   cogx::display::CDisplayView *pView;
   pView = m_pModel->getView(pSelected->text().toStdString());
   if (pView) {
      updateCustomUi(pView);
      ui.drawingArea->setView(pView);
      // TODO: should retrieve data for custom widgets from appropriate components.
   }
}

void QCastMainFrame::updateCustomUi(cogx::display::CDisplayView *pView)
{
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
