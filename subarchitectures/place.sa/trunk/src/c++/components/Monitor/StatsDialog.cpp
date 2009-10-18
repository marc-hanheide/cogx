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
 * \file StatsDialog.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-31
 */

// Place.SA
#include "StatsDialog.h"
// Qt
#include <QDesktopWidget>
// STD
#include <math.h>
#include <iostream>

using namespace place;
using namespace std;

#define MAX_DATA_STATS_FRAMES 100



// ------------------------------------------------------
StatsDialog::StatsDialog(QWidget *parent):
    QDialog(parent, Qt::Tool) //|Qt::WindowStaysOnTopHint
{
  // Setup ui
  setupUi(this);

  // Center dialog
  move(QApplication::desktop()->availableGeometry(parent).center()-(frameGeometry().bottomRight()-frameGeometry().topLeft())/2);

  clearStats();
}


// ------------------------------------------------------
void StatsDialog::updateImageStats(long frameNo, double time, double wmTime)
{
  DataInfo di;
  di.frameNo = frameNo;
  di.time = time;
  di.wmTime = wmTime;
  _imageInfoList.append(di);
  if (_imageInfoList.count()>MAX_DATA_STATS_FRAMES)
    _imageInfoList.removeFirst();

  updateDataStats();
}


// ------------------------------------------------------
void StatsDialog::updateLaserScanStats(long frameNo, double time, double wmTime)
{
  DataInfo di;
  di.frameNo = frameNo;
  di.time = time;
  di.wmTime = wmTime;
  _scanInfoList.append(di);
  if (_scanInfoList.count()>MAX_DATA_STATS_FRAMES)
    _scanInfoList.removeFirst();

  updateDataStats();
}


// ------------------------------------------------------
void StatsDialog::updateOdometryStats(long frameNo, double time, double wmTime)
{
  DataInfo di;
  di.frameNo = frameNo;
  di.time = time;
  di.wmTime = wmTime;
  _odomInfoList.append(di);
  if (_odomInfoList.count()>MAX_DATA_STATS_FRAMES)
    _odomInfoList.removeFirst();

  updateDataStats();
}


// ------------------------------------------------------
void StatsDialog::showEvent(QShowEvent * event)
{
  QDialog::showEvent(event);
  clearStats();
}


// ------------------------------------------------------
void StatsDialog::clearStats()
{
  _imageInfoList.clear();
  _scanInfoList.clear();
  _odomInfoList.clear();
  _visualStatList.clear();
  _laserStatList.clear();
  _frameCount=0;

  framesLabel->setText("");
  lastFrameLabel->setText("");
  imOdTDLabel->setText("");
  scOdTDLabel->setText("");
  imScTDLabel->setText("");
  realFramerateLabel->setText("");
  wmFramerateLabel->setText("");
}


// ------------------------------------------------------
void StatsDialog::updateDataStats()
{
  if ( (_imageInfoList.empty()) || (_scanInfoList.empty()) || (_odomInfoList.empty()) )
    return;

  int ic = _imageInfoList.count();
  int sc = _scanInfoList.count();
  int oc = _odomInfoList.count();

  int ifr = _imageInfoList.back().frameNo;
  int sfr = _scanInfoList.back().frameNo;
  int ofr = _odomInfoList.back().frameNo;

  // Update only if all data acquired
  if ((ifr!=sfr) || (ifr!=ofr) || (ic!=sc) || (ic!=oc))
    return;

  _frameCount++;

  framesLabel->setText(QString::number(_frameCount));
  lastFrameLabel->setText(QString::number(ifr));

  // Get the stats
  double minIOTD = 10000000.0;
  double maxIOTD = 0.0;
  double minISTD = 10000000.0;
  double maxISTD = 0.0;
  double minSOTD = 10000000.0;
  double maxSOTD = 0.0;

  double avgIOTD = 0.0;
  double avgSOTD = 0.0;
  double avgISTD = 0.0;
  double avgImageFrameRate = 0;
  double avgImageWmFrameRate = 0;

  for (int i=0; i<ic; ++i)
  {
    double io = fabs(_imageInfoList[i].time - _odomInfoList[i].time);
    double is = fabs(_imageInfoList[i].time - _scanInfoList[i].time);
    double so = fabs(_scanInfoList[i].time - _odomInfoList[i].time);


    if (i>0)
    {
      double td = _imageInfoList[i].time - _imageInfoList[i-1].time;
      double tdWm = _imageInfoList[i].wmTime - _imageInfoList[i-1].wmTime;

      avgImageFrameRate +=td;
      avgImageWmFrameRate +=tdWm;
    }

    if (io<minIOTD)
      minIOTD=io;
    if (io>maxIOTD)
      maxIOTD=io;
    if (so<minSOTD)
      minSOTD=so;
    if (so>maxSOTD)
      maxSOTD=so;
    if (is<minISTD)
      minISTD=is;
    if (is>maxISTD)
      maxISTD=is;
    avgIOTD+=io;
    avgSOTD+=so;
    avgISTD+=is;
  }
  avgIOTD/=(double)ic;
  avgSOTD/=(double)ic;
  avgISTD/=(double)ic;
  avgImageFrameRate/=(double)ic;
  avgImageWmFrameRate/=(double)ic;

  imOdTDLabel->setText(QString("avg:%1, min:%2, max:%3")
      .arg(avgIOTD, 0, 'f', 3).arg(minIOTD, 0, 'f', 3).arg(maxIOTD, 0, 'f', 3));
  scOdTDLabel->setText(QString("avg:%1, min:%2, max:%3")
      .arg(avgSOTD, 0, 'f', 3).arg(minSOTD, 0, 'f', 3).arg(maxSOTD, 0, 'f', 3));
  imScTDLabel->setText(QString("avg:%1, min:%2, max:%3")
      .arg(avgISTD, 0, 'f', 3).arg(minISTD, 0, 'f', 3).arg(maxISTD, 0,  'f', 3));

  if (ic>1)
  {
    double td = _imageInfoList[_imageInfoList.count()-1].time - _imageInfoList[_imageInfoList.count()-2].time;
    double tdWm = _imageInfoList[_imageInfoList.count()-1].wmTime - _imageInfoList[_imageInfoList.count()-2].wmTime;

    td*=1000.0;
    tdWm*=1000.0;
    avgImageFrameRate*=1000.0;
    avgImageWmFrameRate*=1000.0;

    realFramerateLabel->setText(QString("last:%1ms, avg:%2ms").arg(td, 0, 'f', 1).arg(avgImageFrameRate, 0, 'f', 1));
    wmFramerateLabel->setText(QString("last:%1ms, avg:%2ms").arg(tdWm, 0, 'f', 1).arg(avgImageWmFrameRate, 0, 'f', 1));
  }
}


// ------------------------------------------------------
void StatsDialog::updateVisualProcessorStats(double startTime, double extractionTime, double classificationTime)
{
  // Append to list
  VisualLaserStatus vs;
  vs.startTime=startTime;
  vs.extractionTime=extractionTime;
  vs.classificationTime=classificationTime;
  _visualStatList.append(vs);
  if (_visualStatList.count()>MAX_DATA_STATS_FRAMES)
    _visualStatList.removeFirst();

  // Process the stats
  double minET=1000000.0;
  double minCT=1000000.0;
  double maxET=0.0;
  double maxCT=0.0;
  double avgET=0.0;
  double avgCT=0.0;
  for (int i=0; i<_visualStatList.count(); ++i)
  {
    double et=_visualStatList[i].extractionTime-_visualStatList[i].startTime;
    double ct=_visualStatList[i].classificationTime-_visualStatList[i].extractionTime;
    if (et<minET)
      minET=et;
    if (et>maxET)
      maxET=et;
    if (ct<minCT)
      minCT=ct;
    if (ct>maxCT)
      maxCT=ct;
    avgET+=et;
    avgCT+=ct;
  }
  avgET/=(double)_visualStatList.count();
  avgCT/=(double)_visualStatList.count();

  visExtrTimeLabel->setText(QString("avg:%1ms, min:%2ms, max:%3ms")
      .arg(avgET*1000.0, 0, 'f', 1).arg(minET*1000.0, 0, 'f', 1).arg(maxET*1000.0, 0, 'f', 1));
  visClassTimeLabel->setText(QString("avg:%1ms, min:%2ms, max:%3ms")
      .arg(avgCT*1000.0, 0, 'f', 1).arg(minCT*1000.0, 0, 'f', 1).arg(maxCT*1000.0, 0, 'f', 1));
}


// ------------------------------------------------------
void StatsDialog::updateLaserProcessorStats(double startTime, double extractionTime, double classificationTime)
{
  // Append to list
  VisualLaserStatus ls;
  ls.startTime=startTime;
  ls.extractionTime=extractionTime;
  ls.classificationTime=classificationTime;
  _laserStatList.append(ls);
  if (_laserStatList.count()>MAX_DATA_STATS_FRAMES)
    _laserStatList.removeFirst();

  // Process the stats
  double minET=1000000.0;
  double minCT=1000000.0;
  double maxET=0.0;
  double maxCT=0.0;
  double avgET=0.0;
  double avgCT=0.0;
  for (int i=0; i<_laserStatList.count(); ++i)
  {
    double et=_laserStatList[i].extractionTime-_laserStatList[i].startTime;
    double ct=_laserStatList[i].classificationTime-_laserStatList[i].extractionTime;
    if (et<minET)
      minET=et;
    if (et>maxET)
      maxET=et;
    if (ct<minCT)
      minCT=ct;
    if (ct>maxCT)
      maxCT=ct;
    avgET+=et;
    avgCT+=ct;
  }
  avgET/=(double)_laserStatList.count();
  avgCT/=(double)_laserStatList.count();

  lasExtrTimeLabel->setText(QString("avg:%1ms, min:%2ms, max:%3ms")
      .arg(avgET*1000.0, 0, 'f', 1).arg(minET*1000.0, 0, 'f', 1).arg(maxET*1000.0, 0, 'f', 1));
  lasClassTimeLabel->setText(QString("avg:%1ms, min:%2ms, max:%3ms")
      .arg(avgCT*1000.0, 0, 'f', 1).arg(minCT*1000.0, 0, 'f', 1).arg(maxCT*1000.0, 0, 'f', 1));
}

