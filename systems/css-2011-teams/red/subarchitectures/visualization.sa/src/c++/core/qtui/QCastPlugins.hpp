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
#include "../Model.hpp"
#include <QWebPluginFactory>

class QCastPluginFactory: public QWebPluginFactory
{
   Q_OBJECT
private:
   cogx::display::CDisplayModel *m_pModel;
public:
   QCastPluginFactory();
   void setModel(cogx::display::CDisplayModel* pModel);
   QObject* create (const QString & mimeType, const QUrl& url,
         const QStringList& argumentNames, const QStringList& argumentValues) const;
   QList<Plugin> plugins () const;
};
