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
 * LabelDialog class.
 * \file LabelDialog.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-28
 */

// Place.SA
#include "LabelDialog.h"


// ------------------------------------------------------
LabelDialog::LabelDialog(QWidget *parent):
    QDialog(parent) //Qt::Tool|Qt::WindowStaysOnTopHint
{
  // Setup ui
  setupUi(this);


}

