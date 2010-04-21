/*
 * Author: Marko Mahnič
 * Created: 2010-04-21
 */

#include "QCustomGuiPanel.hpp"

#include <QCheckBox>
#include <QPushButton>
#include <QVBoxLayout>
#include "ChangeSlot.hpp"

#include "../convenience.hpp"

QCustomGuiPanel::QCustomGuiPanel(QWidget* parent, Qt::WindowFlags flags)
   :QFrame(parent, flags)
{
   m_pView = NULL;
}

QCustomGuiPanel::~QCustomGuiPanel()
{
   if (m_pView) m_pView->viewObservers -= this;
}

void QCustomGuiPanel::onUiDataChanged(cogx::display::CDisplayModel *pModel,
      cogx::display::CDisplayView *pSourceView,
      cogx::display::CGuiElement *pElement, const std::string& newValue)
{
   if (pSourceView == m_pView) return;

   // TODO: update the widget associated with pElement
   // pWidget = findWidget(pElement);
   // pWidget->blockSignals(true);
   // pWidget->setValue(newValue);
   // pWidget->blockSignals(false);
}

void QCustomGuiPanel::removeUi()
{
   DTRACE("QCustomGuiPanel::removeUi");

   if (layout()) delete layout();

   // Remove the current widgets; they should be deleted when todelete goes out of scope.
   QWidget todelete;
   QList<QObject*> wdgts = findChildren<QObject*>();
   QObject *pobj;
   FOR_EACH(pobj, wdgts) {
      if (pobj) pobj->setParent(&todelete);
   }
}

void QCustomGuiPanel::updateUi(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView)
{
   DTRACE("QCustomGuiPanel::updateUi");
   setVisible(false);
   removeUi();

   m_pView = pView;
   if (!pView) return;

   CPtrVector<cogx::display::CGuiElement> elements;
   elements = pModel->getGuiElements(pView->m_id);
   if (elements.size() < 1) {
      if (pView) pView->viewObservers -= this;
      return;
   }

   // Create new widgets
   cogx::display::CGuiElement* pgel;
   CChangeSlot *pSlot;
   QCheckBox *pBox;
   QPushButton *pButton;

   QLayout *pLayout = new QVBoxLayout();

   FOR_EACH(pgel, elements) {
      if (!pgel) continue;
      switch (pgel->m_type) {
         case cogx::display::CGuiElement::wtCheckBox:
            pBox = new QCheckBox(QString(pgel->m_label.c_str()), this);
            pLayout->addWidget(pBox);
            pSlot = new CChangeSlot(pgel, pView, pBox);
            connect(pBox, SIGNAL(stateChanged(int)), pSlot, SLOT(onCheckBoxChange(int)));
            break;
         case cogx::display::CGuiElement::wtButton:
            pButton = new QPushButton(QString(pgel->m_label.c_str()), this);
            pLayout->addWidget(pButton);
            pSlot = new CChangeSlot(pgel, pView, pButton);
            connect(pButton, SIGNAL(clicked(bool)), pSlot, SLOT(onButtonClick(bool)));
            break;
      }
   }

   setLayout(pLayout);
   if (pView) pView->viewObservers += this;
   setVisible(true);
}
