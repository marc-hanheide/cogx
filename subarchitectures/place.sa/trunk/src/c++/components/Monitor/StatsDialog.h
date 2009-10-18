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
 * StatsDialog class.
 * \file StatsDialog.h
 * \author Andrzej Pronobis
 * \date 2008-08-31
 */

#ifndef __PLACE_STATS_DIALOG__
#define __PLACE_STATS_DIALOG__

#include "ui_StatsDialog.h"
#include <QDialog>

namespace place
{


class StatsDialog : public QDialog, public Ui_StatsDialog
{
  Q_OBJECT

public:

  /** Constructor. */
  StatsDialog(QWidget *parent);


public slots:

  void updateOdometryStats(long frameNo, double time, double wmTime);
  void updateLaserScanStats(long frameNo, double time, double wmTime);
  void updateImageStats(long frameNo, double time, double wmTime);
  void updateVisualProcessorStats(double startTime, double extractionTime, double classificationTime);
  void updateLaserProcessorStats(double startTime, double extractionTime, double classificationTime);


private:

  void showEvent(QShowEvent * event);

  void clearStats();
  void updateDataStats();


private:

  struct DataInfo
  {
    long frameNo;
    double time;
    double wmTime;
  };

  QList<DataInfo> _imageInfoList;
  QList<DataInfo> _scanInfoList;
  QList<DataInfo> _odomInfoList;

  struct VisualLaserStatus
  {
    double startTime;
    double extractionTime;
    double classificationTime;
  };

  QList<VisualLaserStatus> _visualStatList;
  QList<VisualLaserStatus> _laserStatList;

  int _frameCount;

};


}

#endif

