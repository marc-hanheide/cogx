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
 * ResultsDialog class.
 * \file ResultsDialog.h
 * \author Andrzej Pronobis
 * \date 2008-08-31
 */

#ifndef __PLACE_RESULTS_DIALOG__
#define __PLACE_RESULTS_DIALOG__

#include "ui_ResultsDialog.h"
#include <PlaceData.hpp>
#include <QDialog>

namespace place
{


class ResultsDialog : public QDialog, public Ui_ResultsDialog
{
  Q_OBJECT

public:

  /** Constructor. */
  ResultsDialog(QWidget *parent);


public slots:

  void updateVisualResults(long frameNo, QVector<PlaceData::ClassifierResult> results);
  void updateLaserResults(long frameNo, QVector<PlaceData::ClassifierResult> results);
  void updateIntegratedResults(long frameNo, bool usedVision, bool usedLaser, QVector<PlaceData::ClassifierResult> results);
  void updateTarget(long targetNo, QString targetName, long frameNo);

private:

  void showEvent(QShowEvent * event);
  void clearResults();

private:

  int _visFramesNo;
  int _visPropClassd;
  int _visTargetFramesNo;

  int _lasFramesNo;
  int _lasPropClassd;
  int _lasTargetFramesNo;

  int _intgFramesNo;
  int _intgPropClassd;
  int _intgTargetFramesNo;


  struct Target
  {
    QString name;
    int no;
    int frameNo;
  };

  QList<Target> _targets;

};


}

#endif

