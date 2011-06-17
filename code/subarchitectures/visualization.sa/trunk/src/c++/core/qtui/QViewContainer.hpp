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

#ifndef QVIEWCONTAINER_NXZGNUIJ
#define QVIEWCONTAINER_NXZGNUIJ

#include "QCastViewBase.hpp"
#include <QWidget>
#include <QFrame>
#include <QHash>
#include <QString>
#include <vector>

class QViewContainer: public QWidget
{
   Q_OBJECT
private:
   // TODO: array of displays when grid of views is supported
   QCastViewBase *m_pDisplay; // Currently active QCastViewXX
   QHash<QString, std::vector<double> > m_viewPosMap;

public:
   QViewContainer( QWidget* parent = 0, Qt::WindowFlags flags = 0 );
   ~QViewContainer ();

   // copy information from pContainer (eg. last view positions)
   void initFrom(QViewContainer* pContainer);

   // save information about the current vie (eg. the view position)
   void saveViewInfo();

   // TODO: multiple views in a grid? -> setView(index, pView)
   void setView(cogx::display::CDisplayModel* pModel, cogx::display::CDisplayView* pView);
   cogx::display::CDisplayView* getActiveView();
   QCastViewBase* getDisplayWidget() { return m_pDisplay; }

private:
   void removeUi();
};

#endif /* end of include guard: QVIEWCONTAINER_NXZGNUIJ */
