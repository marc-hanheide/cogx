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
 * OdomScanWidget class.
 * \file OdomScanWidget.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-28
 */

#include "OdomScanWidget.h"
#include <QPainter>
#include <math.h>
#include <iostream>

using namespace categorical;
using namespace std;

const QPoint invalidPoint(-100000,-100000);


// ------------------------------------------------------
OdomScanWidget::OdomScanWidget(QWidget *parent, double scale, QSize canvasSize) :
    QScrollArea(parent), _scale(scale)
{
  // Create canvas
  _canvasSize=canvasSize;
  _center=QPoint(canvasSize.width()/2, canvasSize.height()/2);
  _canvas = new QPixmap(_canvasSize);
  clearCanvas();

  // Create overlay
  _overlay = new QPixmap(_canvasSize);
  clearOverlay();

  // Create label displaying the canvas
  _dispLabel=new QLabel();
  updateView();
  setWidget(_dispLabel);

  // Set some defaults
  _scanOverlayStyle=SOS_LINES;
}


// ------------------------------------------------------
OdomScanWidget::~OdomScanWidget()
{
  delete _canvas;
}


// ------------------------------------------------------
void OdomScanWidget::clearOverlay()
{
  _overlay->fill(QColor(0,0,0,0));
  _emptyOverlay=true;
}


// ------------------------------------------------------
void OdomScanWidget::clearCanvas()
{
  // Set some variables
  _lastPathNode=invalidPoint;

  // Clear
  _canvas->fill();
  QPainter painter;
  painter.begin(_canvas);
  // Create grid
  double gridStep=2.0;
  painter.setPen(QPen(QColor(200,200,200), 1, Qt::DotLine));
  double x=0;
  for(int i=0; x<_canvasSize.width(); ++i)
  {
    x=_center.x() + ((double)(i))*gridStep*_scale;
    painter.drawLine(QPoint(x, 0), QPoint(x, _canvasSize.height()));
  }
  x=0;
  for(int i=0; x>=0; ++i)
  {
    x=_center.x() - ((double)(i))*gridStep*_scale;
    painter.drawLine(QPoint(x, 0), QPoint(x, _canvasSize.height()));
  }
  double y=0;
  for(int i=0; y<_canvasSize.height(); ++i)
  {
    y=_center.y() + ((double)(i))*gridStep*_scale;
    painter.drawLine(QPoint(0, y), QPoint(_canvasSize.width(), y));
  }
  y=0;
  for(int i=0; y>=0; ++i)
  {
    y=_center.y() - ((double)(i))*gridStep*_scale;
    painter.drawLine(QPoint(0, y), QPoint(_canvasSize.width(), y));
  }
  // Create 0,0
  painter.setPen(QPen(QColor(0,0,0), 1, Qt::SolidLine));
  painter.drawLine(_center+QPoint(-1.0*_scale, 0), _center+QPoint(1.0*_scale, 0));
  painter.drawLine(_center+QPoint(0, -1.0*_scale), _center+QPoint(0, 1.0*_scale));
  // Create scale
  painter.drawLine(QPoint(2.0*_scale, 1.5*_scale), QPoint(2.0*_scale, 2.5*_scale));
  painter.drawLine(QPoint(12.0*_scale, 1.5*_scale), QPoint(12.0*_scale, 2.5*_scale));
  painter.drawLine(QPoint(2.0*_scale, 2.0*_scale), QPoint(12.0*_scale, 2.0*_scale));
  painter.drawText(2.0*_scale, 2.0*_scale, 10.0*_scale, 2.0*_scale, Qt::AlignCenter, "10m");
  // End
  painter.end();
}


// ------------------------------------------------------
void OdomScanWidget::updateView()
{
  if (_emptyOverlay)
    _dispLabel->setPixmap(*_canvas);
  else
  {
    QPixmap p(*_canvas);
    QPainter pt;
    pt.begin(&p);
    pt.setCompositionMode(QPainter::CompositionMode_SourceOver);
    pt.drawPixmap(0,0,*_overlay);
    pt.end();
    _dispLabel->setPixmap(p);
  }
}


// ------------------------------------------------------
void OdomScanWidget::plotPathNode(double x, double y)
{
  QPainter painter;
  painter.begin(_canvas);
  painter.setPen(QPen(QColor(255,0,0), 2, Qt::SolidLine));
  QPoint pt(x*_scale, -y*_scale);
  pt+=_center;

  if (_lastPathNode!=invalidPoint)
    painter.drawLine(_lastPathNode, pt);

  _lastPathNode=pt;
  painter.end();
}


// ------------------------------------------------------
void OdomScanWidget::plotScan(double x, double y, double startAngle, QVector<double> ranges, double angleStep, double maxRange)
{
  QPainter painter;
  painter.begin(_canvas);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(QPen(QColor(0,0,0,50), 1, Qt::SolidLine));
//  painter.setPen(QPen(QColor(0,0,0), 1, Qt::SolidLine));

  double angle=startAngle;
  for(int i=0; i<ranges.size(); ++i)
  {
    double r = ranges[i];
    if (r < maxRange)
    {
      double xp=r*cos(angle)+x;
      double yp=r*sin(angle)+y;
      painter.drawPoint(_center+QPoint(xp*_scale,-yp*_scale));
    }
    angle+=angleStep;
  }

  painter.end();
}


// ------------------------------------------------------
void OdomScanWidget::plotScanOverlay(double x, double y, double startAngle, QVector<double> ranges, double angleStep, double maxRange)
{
  QPainter painter;
  painter.begin(_overlay);
  painter.setPen(QPen(QColor(0,255,0), 2, Qt::SolidLine));

  if (_scanOverlayStyle==SOS_LINES)
  {
    double prevAngle=startAngle;
    int prev=0;
    for (int i=0; i<ranges.size(); ++i)
    {
      if (ranges[i] < maxRange)
      {
        prev=i;
        break;
      }
      prevAngle+=angleStep;
    }

    double angle=prevAngle+angleStep;
    for(int i=prev+1; i<ranges.size(); ++i)
    {
      if (ranges[i] < maxRange)
      {
        double r1=ranges[prev];
        double r2=ranges[i];
        double a1=prevAngle;
        double a2=angle;
        double x1=r1*cos(a1)+x;
        double y1=r1*sin(a1)+y;
        double x2=r2*cos(a2)+x;
        double y2=r2*sin(a2)+y;

        prevAngle=angle;
        prev=i;

        painter.drawLine(_center+QPoint(x1*_scale,-y1*_scale),_center+QPoint(x2*_scale,-y2*_scale));
      }
      angle+=angleStep;
    }
  }
  else if (_scanOverlayStyle==SOS_DOTS)
  {
    double angle=startAngle;
    for(int i=0; i<ranges.size(); ++i)
    {
      double r = ranges[i];
      if (r < maxRange)
      {
        double xp=r*cos(angle)+x;
        double yp=r*sin(angle)+y;
        painter.drawPoint(_center+QPoint(xp*_scale,-yp*_scale));
      }
      angle+=angleStep;
    }
  }

  painter.end();
  _emptyOverlay=false;
}


// ------------------------------------------------------
void OdomScanWidget::plotPointOverlay(double x, double y)
{
  QPainter painter;
  painter.begin(_overlay);
  painter.setPen(QPen(QColor(0,0,255), 0.3*_scale, Qt::SolidLine));
  QPoint pt(x*_scale, -y*_scale);
  pt+=_center;

  if (_lastPathNode!=invalidPoint)
    painter.drawPoint(pt);

  painter.end();
  _emptyOverlay=false;
}


// ------------------------------------------------------
bool OdomScanWidget::saveCanvas(QString filePath, const char *format)
{
  return _canvas->save(filePath, format);
}


// ------------------------------------------------------
bool OdomScanWidget::saveCanvasAndOverlay(QString filePath, const char *format)
{
  if (_emptyOverlay)
    return _canvas->save(filePath, format);
  else
  {
    QPixmap p(*_canvas);
    QPainter pt;
    pt.begin(&p);
    pt.setCompositionMode(QPainter::CompositionMode_SourceOver);
    pt.drawPixmap(0,0,*_overlay);
    pt.end();
    return p.save(filePath, format);
  }
}


// ------------------------------------------------------
void OdomScanWidget::ensureVisiblePoint(double x, double y)
{
  ensureVisible(x*_scale+_center.x(), _center.y()-y*_scale);
}
