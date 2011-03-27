// ==================================================================
// DataEditor - Data Annotation Tool
// Copyright (C) 2008, 2009  Andrzej Pronobis
//
// This file is part of DataEditor.
//
// DataEditor is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License,
// or (at your option) any later version.
//
// DataEditor is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with DataEditor. If not, see <http://www.gnu.org/licenses/>.
// ==================================================================

/**
 * MainWindow class.
 * \file MainWindow.h
 * \author Andrzej Pronobis
 * \date 2008-08-28
 */

#ifndef __MAINWINDOW_H__
#define __MAINWINDOW_H__

#include "ui_MainWindow.h"
#include "CategoricalData.hpp"


class ProgressDialog;
namespace categorical
{
  class OdomScanWidget;
}

/**
 * Implements the main window of the application.
 */
class MainWindow : public QMainWindow, public Ui::MainWindow
{
  Q_OBJECT

public:

  /** Constructor. */
  MainWindow( QWidget* = 0 );


private slots:

  void actionOpenTriggered();
  void actionOpenLabelsTriggered();
  void actionSaveDataAsTriggered();
  void actionPlotScansUsingDotsTriggered();
  void actionPlotScansUsingLinesTriggered();
  void actionSaveMapTriggered();
  void actionSaveMapWithOverlayTriggered();
  void actionChangeLabelTriggered();
  void actionMarkForRemovalTriggered();
  void actionUnmarkTriggered();
  void actionChangeLabelTo1Triggered();
  void actionChangeLabelTo2Triggered();

  void currentItemChanged(QTreeWidgetItem * current, QTreeWidgetItem * previous);


private:

  bool readData(QString dataFilePath);
  void clearAll();


private:

  categorical::OdomScanWidget *_osWidget;


private:

  struct Image
  {
    CategoricalData::ImagePtr image;
    QString path;
  };

  QVector<Image> _images;
  QVector<CategoricalData::LaserScanPtr> _scans;
  QVector<CategoricalData::OdometryPtr> _odoms;
  QVector<CategoricalData::TargetPtr> _targets;
  QMap<int, QString> _labels;

  ProgressDialog *_dataLoadingProgress;

};

#endif // __MAINWINDOW_H__
