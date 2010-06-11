/*
 * @author:  Marko Mahnič
 * @created: 2010-05-11
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

#include "QViewContainer.hpp"
#include "QCastView.hpp"
#include "QCastViewGL.hpp"
#ifdef V11N_OBJECT_HTML
#include "QCastViewHtml.hpp"
#endif
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
#ifdef V11N_OBJECT_HTML
      else if (pView->m_preferredContext == cogx::display::ContextHtml) {
         m_pDisplay = new QCastViewHtml(this);
      }
#endif
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

cogx::display::CDisplayView* QViewContainer::getView()
{
   if (! m_pDisplay) return NULL;
   return m_pDisplay->getView();
}

