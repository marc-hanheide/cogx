// ==================================================================
// Place.SA - Place Classification Subarchitecture
// Copyright (C) 2008, 2009  Andrzej Pronobis
//
// This file is part of Place.SA.
//
// Place.SA is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Place.SA is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Place.SA. If not, see <http://www.gnu.org/licenses/>.
// ==================================================================

/**
 * PullNodePlaceInfoDialog class.
 * \file PullNodePlaceInfoDialog.cpp
 * \author Andrzej Pronobis
 * \date 2008-09-04
 */

// Place.SA
#include "PullNodePlaceInfoDialog.h"
// QT
#include <QDesktopWidget>

using namespace place;


// ------------------------------------------------------
PullNodePlaceInfoDialog::PullNodePlaceInfoDialog(QWidget *parent):
    QDialog(parent, Qt::Tool) //|Qt::WindowStaysOnTopHint
{
  // Setup ui
  setupUi(this);

  // Center dialog
  move(QApplication::desktop()->availableGeometry(parent).center()-(frameGeometry().bottomRight()-frameGeometry().topLeft())/2);
}


// ------------------------------------------------------
void PullNodePlaceInfoDialog::showEvent(QShowEvent * event)
{
  QDialog::showEvent(event);
}


