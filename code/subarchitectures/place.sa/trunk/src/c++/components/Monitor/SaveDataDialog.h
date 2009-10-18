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
 * \file SaveDataDialog.h
 * \author Andrzej Pronobis
 * \date 2008-08-12
 */

#ifndef __PLACE_SAVE_DATA_DIALOG__
#define __PLACE_SAVE_DATA_DIALOG__

#include "ui_SaveDataDialog.h"
#include <QDialog>

namespace place
{


class SaveDataDialog : public QDialog, public Ui_SaveDataDialog
{
  Q_OBJECT

public:

  /** Constructor. */
  SaveDataDialog(QWidget *parent);

  void clearStats();

signals:

  void closeEventHappened();


public slots:

  void updateDataSavedInfo(bool vision, bool laser, bool odometry, long framesSaved, long frameNo, bool error);


public:

  bool _stopOnClose;


protected:

  void closeEvent ( QCloseEvent * event );


private:

  bool _error;
};


}

#endif

