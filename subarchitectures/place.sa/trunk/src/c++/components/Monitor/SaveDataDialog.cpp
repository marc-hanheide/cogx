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
 * SaveDataDialog class.
 * \file SaveDataDialog.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-12
 */

// Place.SA
#include "SaveDataDialog.h"
// Qt
#include <QStringList>
#include <QDesktopWidget>
// Std

using namespace place;


// ------------------------------------------------------
SaveDataDialog::SaveDataDialog(QWidget *parent):
    QDialog(parent, Qt::Tool|Qt::WindowTitleHint) // |Qt::WindowStaysOnTopHint
{
  // Setup ui
  setupUi(this);

  // Center dialog
  move(QApplication::desktop()->availableGeometry(parent).center()-(frameGeometry().bottomRight()-frameGeometry().topLeft())/2);

}


// ------------------------------------------------------
void SaveDataDialog::updateDataSavedInfo(bool vision, bool laser, bool odometry, long framesSaved, long frameNo, bool error)
{
  if (error)
    _error=true;

  if (frameNo>=0)
  {
    QStringList tmp;
    if (vision)
      tmp<<"Image";
    if (laser)
      tmp<<"Laser scan";
    if (odometry)
      tmp<<"Odometry";

    if (error)
      savedLabel->setText("ERROR OCCURED!");
    else
      savedLabel->setText(tmp.join(", "));
    lastFrameLabel->setText(QString::number(frameNo));
    totalFramesLabel->setText(QString::number(framesSaved));
  }
}


// ------------------------------------------------------
void SaveDataDialog::clearStats()
{
  _stopOnClose=true;
  _error=false;
  targetComboBox->setCurrentIndex(0);
  pauseButton->setChecked(true);
  savedLabel->setText("");
  lastFrameLabel->setText("");
  totalFramesLabel->setText("");
}


// ------------------------------------------------------
void SaveDataDialog::closeEvent ( QCloseEvent * event )
{
  if (_stopOnClose)
    emit closeEventHappened();
}





