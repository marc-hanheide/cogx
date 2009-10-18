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
 * \file MapDialog.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-28
 */

// Place.SA
#include "MapDialog.h"
#include "place/shared/OdomScanWidget.h"
// Qt
#include <QGraphicsScene>
#include <QPainter>
#include <QScrollArea>
#include <QHBoxLayout>
#include <QLabel>
#include <QFileDialog>
#include <QDesktopWidget>
// Std
#include <iostream>
#include <math.h>

using namespace place;
using namespace std;


// ------------------------------------------------------
MapDialog::MapDialog(QWidget *parent):
    QDialog(parent, Qt::Tool) //|Qt::WindowStaysOnTopHint
{
  // Setup ui
  setupUi(this);
  connect(saveButton, SIGNAL(clicked()), this, SLOT(saveButtonClicked()));
  connect(scaleSpinBox, SIGNAL(valueChanged(int)), this, SLOT(scaleChanged(int)));

  // Center dialog
  move(QApplication::desktop()->availableGeometry(parent).center()-(frameGeometry().bottomRight()-frameGeometry().topLeft())/2);

  // Map
  QHBoxLayout *hl = new QHBoxLayout(mapFrame);
  _map = new OdomScanWidget(mapFrame, 35.0, QSize(1500, 1500));
  hl->addWidget(_map);
  hl->setContentsMargins(2,2,2,2);
  _map->setFrameShape(QFrame::StyledPanel);
  _map->setFrameShadow(QFrame::Plain);

  // Scale spin box
  scaleSpinBox->setValue(_map->getScale());

  clearData();
}


// ------------------------------------------------------
void MapDialog::addLaserScan(double startAngle, double angleStep, double maxRange, QVector<double> range, long frameNo, double time)
{
  if (!scansCheckBox->isChecked())
    return;

  _lastScan.startAngle = startAngle;
  _lastScan.angleStep = angleStep;
  _lastScan.maxRange = maxRange;
  _lastScan.range = range;
  _lastScan.frameNo = frameNo;
  _lastScan.time = time;

  if (_lastOdom.frameNo==frameNo)
  {
    _map->plotScan(_lastOdom.x, _lastOdom.y, _lastOdom.theta+startAngle-3.1415926/2,
                   range, angleStep, maxRange);
    if (currentCheckBox->isChecked())
    {
      _map->clearOverlay();
      _map->plotPointOverlay(_lastOdom.x,_lastOdom.y);
      _map->plotScanOverlay(_lastOdom.x, _lastOdom.y, _lastOdom.theta+startAngle-3.1415926/2,
                            range, angleStep, maxRange);
    }
    _map->updateView();
  }
}


// ------------------------------------------------------
void MapDialog::addOdometry(double x, double y, double theta, long frameNo, double time)
{
  _lastOdom.x=x;
  _lastOdom.y=y;
  _lastOdom.theta=theta;
  _lastOdom.frameNo=frameNo;
  _lastOdom.time=time;
  _map->plotPathNode(x, y);
  _map->ensureVisiblePoint(x,y);

  if (_lastTime>time)
  {
    _map->clearCanvas();
    _map->clearOverlay();
  }
  _lastTime=time;

  if ((scansCheckBox->isChecked())&&(_lastScan.frameNo==frameNo))
  {
    _map->plotScan(x, y, theta+_lastScan.startAngle-3.1415926/2,
                   _lastScan.range, _lastScan.angleStep, _lastScan.maxRange);
    if (currentCheckBox->isChecked())
    {
      _map->clearOverlay();
      _map->plotPointOverlay(x,y);
      _map->plotScanOverlay(x, y, theta+_lastScan.startAngle-3.1415926/2,
                            _lastScan.range, _lastScan.angleStep, _lastScan.maxRange);
    }
  }


  _map->updateView();
}


// ------------------------------------------------------
void MapDialog::clearData()
{
  _lastScan.frameNo=-1;
  _lastOdom.frameNo=-1;
  _lastTime=-1;
  _map->clearCanvas();
  _map->clearOverlay();
  _map->updateView();
}


// ------------------------------------------------------
void MapDialog::scaleChanged(int scale)
{
  _map->setScale(scale);
  _map->updateView();
}


// ------------------------------------------------------
void MapDialog::saveButtonClicked()
{
  QString filePath = QFileDialog::getSaveFileName(this, "Select file",  "",
      "PNG Files (*.png)");
  if (!filePath.isEmpty())
    _map->saveCanvas(filePath, "PNG");
}
