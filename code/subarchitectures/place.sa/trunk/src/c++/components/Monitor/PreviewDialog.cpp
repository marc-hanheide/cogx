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
 * \file PreviewDialog.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-12
 */

// Place.SA
#include "PreviewDialog.h"
// Qt
#include <QGraphicsScene>
#include <QDesktopWidget>
// Std
#include <iostream>
#include <math.h>

using namespace place;
using namespace std;


// ------------------------------------------------------
PreviewDialog::PreviewDialog(QWidget *parent):
    QDialog(parent, Qt::Tool) //|Qt::WindowStaysOnTopHint
{
  _scanScene=0;
  _odomScene=0;
  _prevX=-10000000;
  _prevY=0;
  _prevTheta=0;
  _prevTime=0;
  _maxSpeed=0;

  // Setup ui
  setupUi(this);

  // Center dialog
  move(QApplication::desktop()->availableGeometry(parent).center()-(frameGeometry().bottomRight()-frameGeometry().topLeft())/2);

}


// ------------------------------------------------------
void PreviewDialog::updateImage(QImage image, long frameNo, double time)
{
  imageFrameLabel->setText(QString::number(frameNo));
  imageTimeLabel->setText(QString::number(time, 'f', 6));
  imageLabel->setPixmap(QPixmap::fromImage(image));
}


// ------------------------------------------------------
void PreviewDialog::updateLaserScan(double startAngle, double angleStep, double maxRange, QVector<double> range, long frameNo, double time)
{
  scanFrameLabel->setText(QString::number(frameNo));
  scanTimeLabel->setText(QString::number(time, 'f', 6));

  QGraphicsScene *old=_scanScene;
  _scanScene = new QGraphicsScene(this);

  double angle=startAngle;
  int prev=0;
  double prevAngle=startAngle;

  for(int i=0; i<range.size(); ++i)
  {
    if (range[i] < maxRange)
    {
      prev=i;
      prevAngle=angle;
      break;
    }
    angle+=angleStep;
  }


  for(int i=prev+1; i<range.size(); ++i)
  {
    angle+=angleStep;
    if (range[i] < maxRange)
    {
      double r1=range[prev];
      double r2=range[i];
      double a1=prevAngle;
      double a2=angle;
      double x1=r1*cos(a1);
      double y1=r1*sin(a1);
      double x2=r2*cos(a2);
      double y2=r2*sin(a2);

      prevAngle=angle;
      prev=i;

      _scanScene->addLine(x1, -y1, x2, -y2);
    }
  }

  scanGraphicsView->setScene(_scanScene);
  scanGraphicsView->fitInView(scanGraphicsView->sceneRect(), Qt::KeepAspectRatio);
  if (old)
    delete old;
}


// ------------------------------------------------------
void PreviewDialog::updateOdometry(double x, double y, double theta, long frameNo, double time)
{
  odometryFrameLabel->setText(QString::number(frameNo));
  odometryTimeLabel->setText(QString::number(time, 'f', 6));
  xLabel->setText(QString::number(x));
  yLabel->setText(QString::number(y));
  thetaLabel->setText(QString::number(theta));

  if (_prevX<-1000000)
  {
    _prevX=x;
    _prevY=y;
    _prevTheta=theta;
    _prevTime=time;
  }
  else
  {
    QGraphicsScene *old=_odomScene;
    _odomScene = new QGraphicsScene(this);

    double dt=time-_prevTime;
    double dx=x-_prevX;
    double dy=y-_prevY;
    double dr=sqrt(dx*dx+dy*dy);
    double dtheta=theta-_prevTheta;
    double speed=dr/dt;
    double rotspeed=dtheta/dt;
    _prevX=x;
    _prevY=y;
    _prevTheta=theta;
    _prevTime=time;

    speedLabel->setText(QString::number(speed));
    rotSpeedLabel->setText(QString::number(rotspeed));


    if (speed>_maxSpeed)
      _maxSpeed=speed;

    double tmp=_maxSpeed/30;
    speed+=4*tmp;

    _odomScene->addLine(0, 0, speed*cos(3.1415926/2+rotspeed), -speed*sin(3.1415926/2+rotspeed));
    _odomScene->addEllipse(-tmp, -tmp, 2*tmp, 2*tmp);

    odometryGraphicsView->setScene(_odomScene);
    odometryGraphicsView->fitInView(-_maxSpeed, -_maxSpeed, 2*_maxSpeed, 2*_maxSpeed);
    if (old)
      delete old;
  }
}


// ------------------------------------------------------
void PreviewDialog::updateTarget(long targetNo, QString targetName, long frameNo)
{
  targetFrameLabel->setText(QString::number(frameNo));
  targetLabel->setText(QString("%1 - %2").arg(targetNo).arg(targetName));
}

