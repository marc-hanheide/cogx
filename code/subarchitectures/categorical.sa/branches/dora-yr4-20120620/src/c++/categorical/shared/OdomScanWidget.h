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
 * \file OdomScanWidget.h
 * \author Andrzej Pronobis
 * \date 2008-08-28
 */

#ifndef __PLACE_ODOM_SCAN_WIDGET__
#define __PLACE_ODOM_SCAN_WIDGET__

#include<QScrollArea>
#include<QLabel>

namespace categorical
{

/**
 *
 */
class OdomScanWidget : public QScrollArea
{
  Q_OBJECT

public:

  enum ScanOverlayStyle {SOS_DOTS, SOS_LINES};


public:

  OdomScanWidget(QWidget *parent = 0, double scale=10.0, QSize canvasSize = QSize(1000, 1000));
  ~OdomScanWidget();

  void plotPathNode(double x, double y);

  void plotScan(double x, double y, double startAngle, QVector<double> ranges, double angleStep, double maxRange);

  void plotScanOverlay(double x, double y, double startAngle, QVector<double> ranges, double angleStep, double maxRange);

  void plotPointOverlay(double x, double y);

  void clearOverlay();

  void clearCanvas();

  void updateView();

  void setScale(double scale)
  { _scale=scale; clearCanvas(); clearOverlay(); }

  double getScale()
  { return _scale; }

  bool saveCanvas(QString filePath, const char *format);
  bool saveCanvasAndOverlay(QString filePath, const char *format);

  void setScanOverlayStyle(ScanOverlayStyle style)
  {
    _scanOverlayStyle=style;
  }

  void ensureVisiblePoint(double x, double y);

private:

  ScanOverlayStyle _scanOverlayStyle;
  bool _emptyOverlay;
  QSize _canvasSize;
  QPixmap *_canvas;
  QPixmap *_overlay;
  QPoint _center;
  QLabel *_dispLabel;
  double _scale;
  QPoint _lastPathNode;
};

}

#endif


