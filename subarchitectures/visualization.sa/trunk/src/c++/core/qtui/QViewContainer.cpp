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
#include "QCastViewScene.hpp"
#ifdef V11N_VIEW_GL
#include "QCastViewGL.hpp"
// #include "QCastViewOgre.hpp"
#endif
#ifdef V11N_OBJECT_HTML
#include "QCastViewHtml.hpp"
#endif
#ifdef V11N_OBJECT_HTML_PLUGINS
#include "QCastPlugins.hpp"
#endif
#include <QVBoxLayout>
#include <QToolBar>
#include <QToolButton>

#include "../convenience.hpp"

QViewContainer::QViewContainer( QWidget* parent, Qt::WindowFlags flags)
   : QWidget(parent, flags)
{
   m_pDisplay = nullptr;
}

QViewContainer::~QViewContainer ()
{
   // The container is the owner of the display. It will be deleted 
   // with the Qt mechanisms.
   m_pDisplay = nullptr;
}

void QViewContainer::initFrom(QViewContainer* pContainer)
{
   if (!pContainer || pContainer == this) return;

   QHashIterator<QString, std::vector<double> > i(pContainer->m_viewPosMap);
   while (i.hasNext()) {
      i.next();
      m_viewPosMap[i.key()] = i.value();
   }
}

void QViewContainer::saveViewInfo()
{
   if (! m_pDisplay) return;
   cogx::display::CDisplayView* pView = m_pDisplay->getView();
   if (pView) {
      std::vector<double> data;
      m_pDisplay->getViewPosition(data);
      if (data.size() > 0)
         m_viewPosMap[QString::fromStdString(pView->m_id)] = data;
   }

}

void QViewContainer::removeUi()
{
   DTRACE("QViewContainer::removeUi, mainthread:" << (QObject().thread() == qApp->thread()));

   if (layout()) delete layout();

   // Remove the current widgets
   QObject *pobj;
   QList<QObject*> wdgts = (QObjectList) children();
   DMESSAGE("Count: " << wdgts.size());
   FOR_EACH(pobj, wdgts) {
     if (!pobj) continue;
     QCastViewBase* pViewWin = dynamic_cast<QCastViewBase*>(pobj);
     if (pViewWin != nullptr) {
        cogx::display::CDisplayView* pView = pViewWin->getView();
        if (pView) {
           pView->viewObservers.removeObserver(pViewWin);
           std::vector<double> data;
           pViewWin->getViewPosition(data);
           if (data.size() > 0)
              m_viewPosMap[QString::fromStdString(pView->m_id)] = data;
        }
     }
     pobj->deleteLater();
   }

   m_pDisplay = nullptr;
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
   QBoxLayout *pLayout = new QVBoxLayout();
   pLayout->setSpacing(2);
   pLayout->setContentsMargins(4, 0, 4, 4);
   setLayout(pLayout);

   if (! m_pDisplay) {
      if (pView->m_preferredContext == cogx::display::Context2D) {
         m_pDisplay = new QCastView(this);
      }
      else if (pView->m_preferredContext == cogx::display::ContextGraphics) {
         m_pDisplay = new QCastViewScene(this);
      }
#ifdef V11N_VIEW_GL
      else if (pView->m_preferredContext == cogx::display::ContextGL) {
         m_pDisplay = new QCastViewGL(this);
         //m_pDisplay = new QCastViewOgre(this);
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
      QHash<QString, std::vector<double> >::const_iterator ivp =
         m_viewPosMap.find(QString::fromStdString(pView->m_id));
      if (ivp != m_viewPosMap.end()) 
         m_pDisplay->setViewPosition(ivp.value());
   }
}

cogx::display::CDisplayView* QViewContainer::getActiveView()
{
   if (! m_pDisplay) return nullptr;
   return m_pDisplay->getView();
}

