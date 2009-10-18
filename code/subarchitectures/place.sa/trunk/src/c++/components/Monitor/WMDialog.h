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
 * WMDialog class.
 * \file WMDialog.h
 * \author Andrzej Pronobis
 * \date 2008-08-31
 */

#ifndef __PLACE_WM_DIALOG__
#define __PLACE_WM_DIALOG__

#include "ui_WMDialog.h"
#include <QDialog>

namespace place
{


class WMDialog : public QDialog, public Ui_WMDialog
{
  Q_OBJECT

public:

  /** Constructor. */
  WMDialog(QWidget *parent);


public slots:

  void updateWmInfo(int operation, QString id, QString src, QString type, double time);
  void updateCommandLog(QString cmd, QString params, QString src, double time);


private:
  void showEvent(QShowEvent * event);

  void clearAll();


private:

  QList<QIcon> _wmIcons;
  QMap<QString,QIcon> _typeIcons;
  QMap<QString,QIcon> _cmdIcons;

};


}

#endif

