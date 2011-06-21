/*
 * Author: Marko Mahnič
 * Created: 2011-06-20
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
#ifndef QCASTDIALOGFRAME_DYSTP55V
#define QCASTDIALOGFRAME_DYSTP55V

#include "ui_dialogwin.h"
#include <QTabWidget>
#include "../Model.hpp"

#include <QBuffer>
#include <QByteArray>

class QCastDialogFrame:
   public QTabWidget,
   public cogx::display::CDisplayModelObserver
{
   Q_OBJECT
private:
   Ui::DialogWin ui;
public:
   QCastDialogFrame( QWidget * parent = 0, Qt::WindowFlags flags = 0 /* unused */ );
   void addDialog(cogx::display::CGuiDialog *pDialog);
};

#endif

