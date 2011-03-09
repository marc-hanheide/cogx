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
#ifndef QCASTVIEWBASE_MGC6RE0Z
#define QCASTVIEWBASE_MGC6RE0Z

#include "../Model.hpp"
#include <QWidget>
#include <QToolBar>
#include <vector>

/*abstract class (interface)*/
class QCastViewBase: public cogx::display::CDisplayModelObserver
{
public:
   virtual void setModel(cogx::display::CDisplayModel* pDisplayModel) {}
   virtual void setView(cogx::display::CDisplayView* pDisplayView) = 0;
   virtual cogx::display::CDisplayView* getView() = 0;
   virtual operator QWidget&() = 0;

   // Remember the current view position as an array of doubles (eg. transform).
   // The contents of the matrix depends on the DisplayContext.
   virtual void getViewPosition(std::vector<double>& data) {}
   virtual void setViewPosition(const std::vector<double>& data) {}
   virtual void getToolbars(CPtrVector<QToolBar>& toolbars) {}
};


#endif /* end of include guard: QCASTVIEWBASE_MGC6RE0Z */
