/*
 * Author: Marko Mahnič
 * Created: 2010-03-10
 */

#include "QCastMainFrame.hpp"
#include <QCheckBox>
#include <QPushButton>

#include <cstdio> // TODO: Temporary (printf); remove

#include "ChangeSlot.hpp"

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

   // XXX Testing
   if (m_pModel) {
      cogx::display::CDisplayView *pView;
      //pView = m_pModel->getView("video.viewer");
      //if (pView) ui.drawingArea->setView(pView);
      updateCustomUi(pView);
      updateViewList();
   }
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
   }
}

void QCastMainFrame::updateCustomUi(cogx::display::CDisplayView *pView)
{
   if (!m_pModel || !pView) {
      ui.wgCustomGui->setVisible(false);
      return;
   }
   CPtrVector<cogx::display::CGuiElement> elements;
   elements = m_pModel->getGuiElements(pView->m_id);
   if (elements.size() < 1) {
      ui.wgCustomGui->setVisible(false);
      return;
   }
   ui.wgCustomGui->setVisible(false);

   // Create new widgets
   cogx::display::CGuiElement* pgel;
   CChangeSlot *pSlot;
   QCheckBox *pBox;
   QPushButton *pButton;

   if (ui.wgCustomGui->layout()) delete ui.wgCustomGui->layout();
   QLayout *pLayout = new QVBoxLayout();

   // Remove the current widgets; they will be deleted when todelete goes out of scope.
   {
      // TODO This worked before layout was added: check if it still works
      QWidget todelete;
      QList<QObject*> wdgts = ui.wgCustomGui->findChildren<QObject*>();
      QObject *pobj;
      FOR_EACH(pobj, wdgts) {
         if (pobj) pobj->setParent(&todelete);
      }
   }


   FOR_EACH(pgel, elements) {
      if (!pgel) continue;
      switch (pgel->m_type) {
         case cogx::display::CGuiElement::wtCheckBox:
            pBox = new QCheckBox(QString(pgel->m_label.c_str()), ui.wgCustomGui);
            pLayout->addWidget(pBox);
            pSlot = new CChangeSlot(pgel, pBox);
            connect(pBox, SIGNAL(stateChanged(int)), pSlot, SLOT(onCheckBoxChange(int)));
            break;
         case cogx::display::CGuiElement::wtButton:
            pButton = new QPushButton(QString(pgel->m_label.c_str()), ui.wgCustomGui);
            pLayout->addWidget(pButton);
            pSlot = new CChangeSlot(pgel, pButton);
            connect(pButton, SIGNAL(clicked(bool)), pSlot, SLOT(onButtonClick(bool)));
            break;
      }
   }
   ui.wgCustomGui->setLayout(pLayout);
   ui.wgCustomGui->setVisible(true);
}

void QCastMainFrame::onViewAdded(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView)
{
   // TODO: only if the view is not in the list
   updateViewList();
   ui.drawingArea->onViewChanged(pModel, pView);
}
