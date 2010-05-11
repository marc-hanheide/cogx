/*
 * @author:  Marko Mahnič
 * @created: 2010-05-11
 */

#include "QViewContainer.hpp"
#include "QCastView.hpp"
#include "QCastViewGL.hpp"
#include <QVBoxLayout>

#include "../convenience.hpp"

QViewContainer::QViewContainer( QWidget* parent, Qt::WindowFlags flags)
   : QFrame(parent, flags)
{
   m_pDisplay = NULL;
}

QViewContainer::~QViewContainer ()
{
   // The container is the owner of the display. It will be deleted 
   // with the Qt mechanisms.
   m_pDisplay = NULL;
}

void QViewContainer::removeUi()
{
   DTRACE("QViewContainer::removeUi");

   if (layout()) delete layout();

   // Remove the current widgets; they should be deleted when todelete goes out of scope.
   QWidget todelete;
   QList<QObject*> wdgts = findChildren<QObject*>();
   QObject *pobj;
   FOR_EACH(pobj, wdgts) {
      if (pobj) pobj->setParent(&todelete);
   }
}

void QViewContainer::setView(cogx::display::CDisplayView* pView)
{
   if (! pView) {
      m_pDisplay = NULL;
      removeUi();
      return;
   }

   // TODO: check if the current widget supports view's m_preferredContext
   // otherwise delete the view
   m_pDisplay = NULL;
   removeUi();
   QLayout *pLayout = new QVBoxLayout();
   setLayout(pLayout);

   // TODO: Also create toolbars for active views!
   if (! m_pDisplay) {
      if (pView->m_preferredContext == cogx::display::ContextGL) {
         m_pDisplay = new QCastViewGL(this);
      }
      else if (pView->m_preferredContext == cogx::display::Context2D) {
         m_pDisplay = new QCastView(this);
      }
      else {
         m_pDisplay = new QCastView(this);
      }

      if (m_pDisplay) {
         QWidget &wdg = *m_pDisplay;
         pLayout->addWidget(&wdg);
      }
   }

   if (m_pDisplay) m_pDisplay->setView(pView);
}

