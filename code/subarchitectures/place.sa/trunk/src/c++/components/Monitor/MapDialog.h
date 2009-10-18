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
 * MapDialog class.
 * \file MapDialog.h
 * \author Andrzej Pronobis
 * \date 2008-08-28
 */

#ifndef __PLACE_MAP_DIALOG__
#define __PLACE_MAP_DIALOG__

#include "ui_MapDialog.h"
#include <QDialog>

class QLabel;

namespace place
{
class OdomScanWidget;


class MapDialog : public QDialog, public Ui_MapDialog
{
  Q_OBJECT

public:

  /** Constructor. */
  MapDialog(QWidget *parent);

  void clearData();

public slots:

  void addLaserScan(double startAngle, double angleStep, double maxRange, QVector<double> range, long frameNo, double time);
  void addOdometry(double x, double y, double theta, long frameNo, double time);


private slots:

  void saveButtonClicked();
  void scaleChanged(int scale);


private:

  OdomScanWidget *_map;

  struct Scan
  {
    double startAngle;
    double angleStep;
    double maxRange;
    QVector<double> range;
    long frameNo;
    double time;
  };

  struct Odometry
  {
    double x;
    double y;
    double theta;
    long frameNo;
    double time;
  };

  Scan _lastScan;
  Odometry _lastOdom;
  double _lastTime;

};


}

#endif

