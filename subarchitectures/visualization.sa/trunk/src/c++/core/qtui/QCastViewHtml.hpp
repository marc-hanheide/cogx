/*
 * Author: Marko Mahnič
 * Created: 2010-06-10
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
#ifndef QCASTVIEWHTML_X94VTZ5D
#define QCASTVIEWHTML_X94VTZ5D

#include "QCastViewBase.hpp"
#include <QWebView>

class QCastViewHtml: public QWebView, public QCastViewBase
{
   Q_OBJECT
private:
   cogx::display::CDisplayView* pView;
   bool m_bModified;
   QString m_jsFormCap;
   static QString m_jQuery;

public:
   QCastViewHtml( QWidget* parent = 0, Qt::WindowFlags flags = 0 );
   ~QCastViewHtml();

public:
   // QCastViewBase
   void setView(cogx::display::CDisplayView* pDisplayView); /*override*/
   cogx::display::CDisplayView* getView() { return pView; } /*override*/
   operator QWidget&() { return *this; } /*override*/
 
   // CDisplayModelObserver
   void onViewChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView); /*override*/
private:
   void paintEvent(QPaintEvent* event);

signals:
   void updateContent();

private slots:
   void doUpdateContent();
   void createJsObjects();
   void finishLoading(bool);
};

#endif /* end of include guard: QCASTVIEWHTML_X94VTZ5D */
