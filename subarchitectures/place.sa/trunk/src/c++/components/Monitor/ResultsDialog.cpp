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
 * \file ResultsDialog.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-31
 */

// Place.SA
#include "ResultsDialog.h"
// QT
#include <QDesktopWidget>

using namespace place;


// ------------------------------------------------------
ResultsDialog::ResultsDialog(QWidget *parent):
    QDialog(parent, Qt::Tool) //|Qt::WindowStaysOnTopHint
{
  // Setup ui
  setupUi(this);

  // Center dialog
  move(QApplication::desktop()->availableGeometry(parent).center()-(frameGeometry().bottomRight()-frameGeometry().topLeft())/2);

  clearResults();
}


// ------------------------------------------------------
void ResultsDialog::showEvent(QShowEvent * event)
{
  QDialog::showEvent(event);
  clearResults();
}


// ------------------------------------------------------
void ResultsDialog::clearResults()
{
  _visFramesNo=0;
  _visPropClassd=0;
  _visTargetFramesNo=0;

  _lasFramesNo=0;
  _lasPropClassd=0;
  _lasTargetFramesNo=0;

  _intgFramesNo=0;
  _intgPropClassd=0;
  _intgTargetFramesNo=0;

  _targets.clear();
}


// ------------------------------------------------------
void ResultsDialog::updateTarget(long targetNo, QString targetName, long frameNo)
{
  Target t;
  t.name = targetName;
  t.no = targetNo;
  t.frameNo = frameNo;
  _targets.append(t);
  if (_targets.count()>5)
    _targets.removeFirst();
}


// ------------------------------------------------------
void ResultsDialog::updateVisualResults(long frameNo, QVector<PlaceData::ClassifierResult> results)
{
  _visFramesNo++;

  // Last result
  visLastFrameLabel->setText(QString::number(frameNo));
  vis1stHypLabel->setText(
      QString::number(results[0].classNo) + " - " +
      QString::fromStdString(results[0].className) );
  visConfLabel->setText(QString::number(results[0].confidence, 'f', 4));
  QString hypStr="";
  for (int i=1; i<results.count(); ++i)
  {
    if (i>1)
      hypStr+="\n";
    hypStr+=QString::number(results[i].classNo)+" - "+
            QString::fromStdString(results[i].className)+" : " +
            QString::number(results[i].confidence, 'f', 4);
  }
  visOtherHypLabel->setText(hypStr);

  // Frames
  visFramesLabel->setText(QString::number(_visFramesNo));

  // Find target
  Target t;
  t.frameNo=-1;
  foreach(t, _targets)
  {
    if (t.frameNo == frameNo)
      break;
  }

  // Target
  if (t.frameNo == frameNo)
  {
    _visTargetFramesNo++;

    visTargetLabel->setText(QString::number(t.no)+" - "+t.name);

    // Calculate class rate
    if (t.no==results[0].classNo)
      _visPropClassd++;

    // Display
    visClassRateLabel->setText(QString("%1/%2 ").arg(_visPropClassd).arg(_visTargetFramesNo)+
        QString::number(100.0*(double(_visPropClassd)/double(_visTargetFramesNo)), 'f', 2)+"%");
  }
  else
  {
    visTargetLabel->setText("Missing!");
  }
}


// ------------------------------------------------------
void ResultsDialog::updateLaserResults(long frameNo, QVector<PlaceData::ClassifierResult> results)
{
  _lasFramesNo++;

  // Last result
  lasLastFrameLabel->setText(QString::number(frameNo));
  las1stHypLabel->setText(QString::number(results[0].classNo) + " - " +
      QString::fromStdString(results[0].className));
  lasConfLabel->setText(QString::number(results[0].confidence, 'f', 4));
  QString hypStr="";
  for (int i=1; i<results.count(); ++i)
  {
    if (i>1)
      hypStr+="\n";
    hypStr+=QString::number(results[i].classNo)+" - "+
        QString::fromStdString(results[i].className)+" : " +
        QString::number(results[i].confidence, 'f', 4);
  }
  lasOtherHypLabel->setText(hypStr);

  // Frames
  lasFramesLabel->setText(QString::number(_lasFramesNo));

  // Find target
  Target t;
  t.frameNo=-1;
  foreach(t, _targets)
  {
    if (t.frameNo == frameNo)
      break;
  }

  // Target
  if (t.frameNo == frameNo)
  {
    _lasTargetFramesNo++;

    lasTargetLabel->setText(QString::number(t.no)+" - "+t.name);

    // Calculate class rate
    if (t.no==results[0].classNo)
      _lasPropClassd++;

    // Display
    lasClassRateLabel->setText(QString("%1/%2 ").arg(_lasPropClassd).arg(_lasTargetFramesNo)+
        QString::number(100.0*(double(_lasPropClassd)/double(_lasTargetFramesNo)), 'f', 2)+"%");
  }
  else
  {
    lasTargetLabel->setText("Missing!");
  }
}


// ------------------------------------------------------
void ResultsDialog::updateIntegratedResults(long frameNo, bool usedVision, bool usedLaser, QVector<PlaceData::ClassifierResult> results)
{
  _intgFramesNo++;

  // Cues used
  QString cuesUsedStr;
  if (usedVision)
    cuesUsedStr+="vision";
  if (usedLaser)
  {
    if (!cuesUsedStr.isEmpty())
      cuesUsedStr+=", ";
    cuesUsedStr+="laser";
  }
  intgCuesUsed->setText(cuesUsedStr);

  // Last result
  intgLastFrameLabel->setText(QString::number(frameNo));
  intg1stHypLabel->setText(QString::number(results[0].classNo) + " - " +
      QString::fromStdString(results[0].className));
  intgConfLabel->setText(QString::number(results[0].confidence, 'f', 4));
  QString hypStr="";
  for (int i=1; i<results.count(); ++i)
  {
    if (i>1)
      hypStr+="\n";
    hypStr+=QString::number(results[i].classNo)+" - "+
        QString::fromStdString(results[i].className)+" : " +
        QString::number(results[i].confidence, 'f', 4);
  }
  intgOtherHypLabel->setText(hypStr);

  // Frames
  intgFramesLabel->setText(QString::number(_intgFramesNo));

  // Find target
  Target t;
  t.frameNo=-1;
  foreach(t, _targets)
  {
    if (t.frameNo == frameNo)
      break;
  }

  // Target
  if (t.frameNo == frameNo)
  {
    _intgTargetFramesNo++;

    intgTargetLabel->setText(QString::number(t.no)+" - "+t.name);

    // Calculate class rate
    if (t.no==results[0].classNo)
      _intgPropClassd++;

    // Display
    intgClassRateLabel->setText(QString("%1/%2 ").arg(_intgPropClassd).arg(_intgTargetFramesNo)+
        QString::number(100.0*(double(_intgPropClassd)/double(_intgTargetFramesNo)), 'f', 2)+"%");
  }
  else
  {
    intgTargetLabel->setText("Missing!");
  }
}


