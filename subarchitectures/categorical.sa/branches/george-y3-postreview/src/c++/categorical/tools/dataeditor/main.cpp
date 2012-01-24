// ==================================================================
// DataEditor - Data Annotation Tool
// Copyright (C) 2008, 2009  Andrzej Pronobis
//
// This file is part of DataEditor.
//
// DataEditor is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License,
// or (at your option) any later version.
//
// DataEditor is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with DataEditor. If not, see <http://www.gnu.org/licenses/>.
// ==================================================================

/**
 * DataEditor
 * \file main.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-28
 */


#include <QApplication>
#include "MainWindow.h"

int main( int argc, char ** argv )
{
  // Create & configure application
  QApplication app( argc, argv );

  // Show main window
  MainWindow mw;
  mw.show();

  // Run application
  return app.exec();
}
