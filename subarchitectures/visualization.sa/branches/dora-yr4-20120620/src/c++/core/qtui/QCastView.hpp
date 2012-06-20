/*
 * Author: Marko Mahnič
 * Created: 2010-03-10
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

#ifndef QCASTVIEW_NXKFPZKI
#define QCASTVIEW_NXKFPZKI

#include "QCastViewBase.hpp"
#include <QWidget>

class QCastView: public QWidget, public QCastViewBase
{
   Q_OBJECT
private:
   // cogx::display::CDisplayModel* pModel;
   cogx::display::CDisplayView* pView;

   // XXX: scale and offset should be stored in pView or in a map<pview,viewState>
   double m_scale;
   QPointF m_offset;

public:
   QCastView( QWidget* parent = 0, Qt::WindowFlags flags = 0 );
   ~QCastView();

public:
   // QCastViewBase
   void setView(cogx::display::CDisplayView* pDisplayView); /*override*/
   cogx::display::CDisplayView* getView() { return pView; } /*override*/
   operator QWidget&() { return *this; } /*override*/
   // CDisplayModelObserver
   void onViewChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView); /*override*/

public slots:
   void resetOffset();
   void resetScale();

protected:
   // Qt events
   virtual void paintEvent(QPaintEvent * event); /*override*/
   virtual void wheelEvent(QWheelEvent *e); /*override*/
};

#endif /* end of include guard: QCASTVIEW_NXKFPZKI */
