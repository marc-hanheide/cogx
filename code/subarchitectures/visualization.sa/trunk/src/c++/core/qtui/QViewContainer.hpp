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

class QViewContainer: public QFrame
{
   Q_OBJECT
private:
   // TODO: array of displays when grid of views is supported
   QCastViewBase *m_pDisplay; // Currently active QCastViewXX

public:
   QViewContainer( QWidget* parent = 0, Qt::WindowFlags flags = 0 );
   ~QViewContainer ();

   // TODO: multiple views in a grid? -> setView(index, pView)
   void setView(cogx::display::CDisplayView* pView);
   cogx::display::CDisplayView* getView();

private:
   void removeUi();
};

#endif /* end of include guard: QVIEWCONTAINER_NXZGNUIJ */
