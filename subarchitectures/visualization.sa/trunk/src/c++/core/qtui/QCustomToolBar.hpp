/*
 * Author: Marko Mahnič
 * Created: 2010-04-21
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

#ifndef QCUSTOMTOOLBAR_VBN1BGY9
#define QCUSTOMTOOLBAR_VBN1BGY9

#include <QToolBar>
#include "../Model.hpp"

class QCustomToolBar: public QToolBar, public cogx::display::CDisplayModelObserver
{
   Q_OBJECT
private:
   cogx::display::CDisplayView* m_pView;

private:
   int m_controlCount;
   void removeUi();

public:
   QCustomToolBar( QWidget* parent = 0 );
   ~QCustomToolBar();

   // @returns the nubmer of created custom UI elements.
   int updateUi(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView);
   int controlCount() {
      return m_controlCount;
   }
   bool hasView(cogx::display::CDisplayView *pView) {
      return pView == m_pView;
   }

public:
   // CDisplayModelObserver
   void onModel_UiDataChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pSourceView,
         cogx::display::CGuiElement *pElement, const std::string& newValue); /*override*/

private:
// signals/slots for interthread communication
signals: 
   void signalUiDataChanged(cogx::display::CDisplayModel*, cogx::display::CDisplayView*,
         cogx::display::CGuiElement *pElement, QString newValue); /*override*/
private slots:
   void doUiDataChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView,
         cogx::display::CGuiElement *pElement, QString newValue); /*override*/
};

#endif /* end of include guard: QCUSTOMTOOLBAR_VBN1BGY9 */
