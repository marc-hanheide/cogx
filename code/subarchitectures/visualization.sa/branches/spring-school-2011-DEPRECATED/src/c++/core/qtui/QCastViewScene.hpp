/*
 * Author: Marko Mahnič
 * Created: 2011-01-07
 *
 * © Copyright 2011 Marko Mahnič. 
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

#ifndef QCASTVIEWSCNE_NXKFPZKI
#define QCASTVIEWSCNE_NXKFPZKI

#include "QCastViewBase.hpp"
#include <QGraphicsScene>
#include <QGraphicsView>

class QCastViewScene: public QGraphicsView, public QCastViewBase
{
   Q_OBJECT
private:
   // cogx::display::CDisplayModel* pModel;
   cogx::display::CDisplayView* pView;

   // XXX: scale and offset should be stored in pView or in a map<pview,viewState>
   QGraphicsScene* m_pScene;
   bool m_bNeedsRebuild;
   double m_scale;

public:
   QCastViewScene( QWidget* parent = 0, Qt::WindowFlags flags = 0 );
   ~QCastViewScene();

public:
   // QCastViewBase
   void setView(cogx::display::CDisplayView* pDisplayView); /*override*/
   cogx::display::CDisplayView* getView() { return pView; } /*override*/
   operator QWidget&() { return *this; } /*override*/
   // CDisplayModelObserver
   void onViewChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView); /*override*/
   void getViewPosition(std::vector<double>& matrix); /*override*/
   void setViewPosition(const std::vector<double>& matrix); /*override*/

protected:
   void rebuildScene(cogx::display::CDisplayView* pDisplayView);

private slots:
   void requestFullRedraw();
//   void resetOffset();
//   void resetScale();

signals:
   void signalViewChanged();

protected:
   // Qt events
   virtual void paintEvent(QPaintEvent * event); /*override*/
   virtual void showEvent(QShowEvent * event); /*override*/
   virtual void wheelEvent(QWheelEvent *e); /*override*/

};

#endif /* end of include guard: QCASTVIEWSCNE_NXKFPZKI */
