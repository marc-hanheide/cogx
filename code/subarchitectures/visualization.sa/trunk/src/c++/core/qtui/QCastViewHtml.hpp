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
#include "HtmlElements.hpp"
#include <QWebView>
#include <iostream>

class QCastViewHtml: public QWebView, public QCastViewBase
{
   Q_OBJECT
private:
   cogx::display::CDisplayModel* pModel;
   cogx::display::CDisplayView* pView;
   bool m_bModified;
   bool m_bHasForms;
   bool m_bBlockUpdates;
   QString m_jsFormCap;
   static QString m_jQuery;
   QString jsObjectName;
   CPtrVector<cogx::display::CHtmlChunk> m_Chunks; // activehtml and form chunks (js interaction)
   QString m_TextToFind;

public:
   QCastViewHtml( QWidget* parent = nullptr, Qt::WindowFlags flags = 0 );
   ~QCastViewHtml();

public:
   // QCastViewBase
   void setModel(cogx::display::CDisplayModel* pDisplayModel); /*override*/
   void setView(cogx::display::CDisplayView* pDisplayView); /*override*/
   cogx::display::CDisplayView* getView() { return pView; } /*override*/
   operator QWidget&() { return *this; } /*override*/
   void getToolbars(CPtrVector<QToolBar>& toolbars);
 
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
   void onFindOnPage();
   void onFindAgainOnPage();
   void onToggleBlockUpdates(bool);
private slots:
   // see: <url:html/formcap.hpp#tn=signalOwnerDataChanged>
   void doFillHtmlFrom(const QString& formid);
};

#endif /* end of include guard: QCASTVIEWHTML_X94VTZ5D */
// vim:sw=3:ts=8:et
