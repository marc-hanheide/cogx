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
 * PreviewDialog class.
 * \file PreviewDialog.h
 * \author Andrzej Pronobis
 * \date 2008-08-12
 */

#ifndef __PLACE_PREVIEW_DIALOG__
#define __PLACE_PREVIEW_DIALOG__

#include "ui_PreviewDialog.h"
#include <QDialog>

namespace place
{


class PreviewDialog : public QDialog, public Ui_PreviewDialog
{
  Q_OBJECT

public:

  /** Constructor. */
  PreviewDialog(QWidget *parent);


public slots:

  void updateImage(QImage image, long frameNo, double time);
  void updateLaserScan(double startAngle, double angleStep, double maxRange, QVector<double> range, long frameNo, double time);
  void updateOdometry(double x, double y, double theta, long frameNo, double time);
  void updateTarget(long targetNo, QString targetName, long frameNo);

private:

  QGraphicsScene *_scanScene;
  QGraphicsScene *_odomScene;

  double _prevX, _prevY, _prevTheta, _prevTime;
  double _maxSpeed;
};


}

#endif

