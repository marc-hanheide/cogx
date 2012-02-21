/*
 * Author: Marko Mahnič
 * Created: 2010-06-17
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
#include "QCastPlugins.hpp"
#include "QViewContainer.hpp"
#include <QUrl>
#include <QFrame>

#include "../convenience.hpp"

QCastPluginFactory::QCastPluginFactory()
{
   m_pModel = NULL;
}

void QCastPluginFactory::setModel(cogx::display::CDisplayModel* pModel)
{
   m_pModel = pModel;
}

QObject* QCastPluginFactory::create (const QString &mimeType, const QUrl& url,
      const QStringList& argumentNames, const QStringList& argumentValues) const
{
   try {
      DTRACE("QCastPluginFactory::create " << mimeType.toStdString());
      if (mimeType == "application/cast-displayview") {
         QViewContainer* pPlug = new QViewContainer();
         DMESSAGE("QViewContainer created");
         if (m_pModel) {
            QString viewName = url.path().mid(1); // "cogxdisp://view/<path>"
            DMESSAGE("View to show " << viewName.toStdString());

            auto it = m_pModel->m_Views.find(viewName.toStdString());
            if (! (it == m_pModel->m_Views.end())) {
               pPlug->setView(m_pModel, it->second);
               DMESSAGE("setView applied: " << it->second->m_id);
            }
         }
         return pPlug;
      }
   }
   catch (...) {
      DMESSAGE("MISERABLY FAILED");
   }
   return NULL;
}

QList<QWebPluginFactory::Plugin> QCastPluginFactory::plugins () const
{
   QWebPluginFactory::MimeType mimeType;
   mimeType.name = "application/cast-displayview";
   mimeType.description = "A Display Server QCastView component";
   // mimeType.fileExtensions = QStringList() << "$nofileextensions$";

   QWebPluginFactory::Plugin plugin;
   plugin.name = "Display Server View viewer";
   plugin.description = "Display Server View viewer";
   plugin.mimeTypes = QList<MimeType>() << mimeType;

   return QList<QWebPluginFactory::Plugin>() << plugin;
}
