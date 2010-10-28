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
#ifdef V11N_VIEW_GL
#include "QCastViewGL.hpp"
#endif
#ifdef V11N_OBJECT_HTML
#include "QCastViewHtml.hpp"
#endif
#ifdef V11N_OBJECT_HTML_PLUGINS
#include "QCastPlugins.hpp"
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
   DTRACE("QViewContainer::removeUi, mainthread:" << (QObject().thread() == qApp->thread()));

   if (layout()) delete layout();

   // Remove the current widgets; they should be deleted when todelete goes out of scope.
   QObject *pobj;
   QList<QObject*> wdgts = (QObjectList) children();
   DMESSAGE("Count: " << wdgts.size());
   FOR_EACH(pobj, wdgts) {
     if (!pobj) continue;
     QCastViewBase* pViewWin = dynamic_cast<QCastViewBase*>(pobj);
     if (pViewWin != NULL) {
        cogx::display::CDisplayView* pView = pViewWin->getView();
        if (pView) {
           pView->viewObservers.removeObserver(pViewWin);
        }
     }
     pobj->deleteLater();
   }

   m_pDisplay = NULL;
}

void QViewContainer::setView(cogx::display::CDisplayModel* pModel, cogx::display::CDisplayView* pView)
{
   DTRACE("QViewContainer::setView, mainthread:" << (QObject().thread() == qApp->thread()));
   if (! pView) {
      removeUi();
      return;
   }

   // TODO: check if the current widget supports view's m_preferredContext
   // otherwise delete the view
   removeUi();
   QLayout *pLayout = new QVBoxLayout();
   setLayout(pLayout);

   // TODO: Also create toolbars for active views!
   if (! m_pDisplay) {
      if (pView->m_preferredContext == cogx::display::Context2D) {
         m_pDisplay = new QCastView(this);
      }
#ifdef V11N_VIEW_GL
      else if (pView->m_preferredContext == cogx::display::ContextGL) {
         m_pDisplay = new QCastViewGL(this);
      }
#endif
#ifdef V11N_OBJECT_HTML
      else if (pView->m_preferredContext == cogx::display::ContextHtml) {
         m_pDisplay = new QCastViewHtml(this);
#ifdef V11N_OBJECT_HTML_PLUGINS
         QWebPage* pPage = dynamic_cast<QCastViewHtml*>(m_pDisplay)->page();
         if (pPage) {
           QCastPluginFactory* pFactory = new QCastPluginFactory();
           pFactory->setModel(pModel);
           pPage->setPluginFactory(pFactory);
         }
#endif
      }
#endif
      else {
         m_pDisplay = new QCastView(this);
      }

      if (m_pDisplay) {
         QWidget *pWdg = dynamic_cast<QWidget*>(m_pDisplay);
         if (pWdg) pLayout->addWidget(pWdg);
      }
   }

   if (m_pDisplay) {
      m_pDisplay->setModel(pModel);
      m_pDisplay->setView(pView);
   }
}

cogx::display::CDisplayView* QViewContainer::getView()
{
   if (! m_pDisplay) return NULL;
   return m_pDisplay->getView();
}

