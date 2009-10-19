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
 * MonitorDialog class.
 * \file MonitorDialog.h
 * \author Andrzej Pronobis
 * \date 2008-08-12
 */

#ifndef __PLACE_MONITOR_DIALOG__
#define __PLACE_MONITOR_DIALOG__

#include "ui_MonitorDialog.h"
#include <PlaceData.hpp>
//#include <cast/architecture/ManagedComponent.hpp>
#include <QDialog>

class PlaceMonitor;

namespace place
{

class PullNodePlaceInfoDialog;
class PreviewDialog;
class SaveDataDialog;
class MapDialog;
class StatsDialog;
class NodesDialog;
class WMDialog;
class ResultsDialog;

class MonitorDialog : public QDialog, public Ui_MonitorDialog
{
  Q_OBJECT

public:

  /** Constructor. */
  MonitorDialog(PlaceMonitor *monitor, QWidget *parent = 0);


public: // Public interface of the dialog

  void addWmChange(cast::cdl::WorkingMemoryOperation operation,
      std::string id, std::string src, std::string type, double time);
  void newImage(const PlaceData::ImagePtr image);
  void newLaserScan(const PlaceData::LaserScanPtr scan);
  void newOdometry(const PlaceData::OdometryPtr odom);
  void newTarget(const PlaceData::TargetPtr target);
  void newDataSaverStatus(bool vision, bool laser, bool odometry, long framesSaved, long frameNo, bool error);
  void newCommand(std::string cmd, std::string params, std::string src, double time);
  void newVisualProcessorStatus(const PlaceData::VisualProcessorStatusPtr stat);
  void newLaserProcessorStatus(const PlaceData::LaserProcessorStatusPtr stat);
  void newVisualResults(const PlaceData::VisualResultsPtr res);
  void newLaserResults(const PlaceData::LaserResultsPtr res);
  void newIntegratedResults(const PlaceData::IntegratedResultsPtr res);
  void newNodeLabellerData(const PlaceData::NodeLabellerDataPtr res);


signals:

  void updateWmInfoSignal(int operation, QString id, QString src,
      QString type, double time);
  void updatePreviewImageSignal(QImage image, long frameNo, double time);
  void updatePreviewLaserScanSignal(double startAngle, double angleStep,
      double maxRange, QVector<double> range, long frameNo, double time);
  void updatePreviewOdometrySignal(double x, double y, double theta,
      long frameNo, double time);
  void updatePreviewTargetSignal(long targetNo, QString targetName, long frameNo);
  void updateDataSavedInfoSignal(bool vision, bool laser, bool odometry,
      long framesSaved, long frameNo, bool error);
  void updateCommandLogSignal(QString cmd, QString params, QString src, double time);
  void addMapLaserScanSignal(double startAngle, double angleStep,
      double maxRange, QVector<double> range, long frameNo, double time);
  void addMapOdometrySignal(double x, double y, double theta, long frameNo, double time);
  void updateImageStatsSignal(long frameNo, double time, double wmTime);
  void updateLaserScanStatsSignal(long frameNo, double time, double wmTime);
  void updateOdometryStatsSignal(long frameNo, double time, double wmTime);
  void updateVisualProcessorStatsSignal(double startTime, double extractionTime,
      double classificationTime);
  void updateVisualResultsSignal(long frameNo,
      QVector<PlaceData::ClassifierResult> results);
  void updateLaserProcessorStatsSignal(double startTime, double extractionTime,
      double classificationTime);
  void updateLaserResultsSignal(long frameNo,
      QVector<PlaceData::ClassifierResult> results);
  void updateResultsTargetSignal(long targetNo, QString targetName, long frameNo);
  void updateIntegratedResultsSignal(long frameNo, bool usedVision,
      bool usedLaser, QVector<PlaceData::ClassifierResult> results);
  void updateNodeLabellerDataSignal(PlaceData::NodeLabellerDataPtr nodeLabellerData);


private slots:

  void startPlaceButtonClicked();
  void stopPlaceButtonClicked();
  void updateButtonClicked();
  void startOnReadVPButtonClicked();
  void startOnReadDSButtonClicked();
  void stopVPButtonClicked();
  void stopDSButtonClicked();
  void killButtonClicked();
  void dataPreviewButtonClicked();
  void mapButtonClicked();
  void resultsButtonClicked();
  void statsButtonClicked();
  void wmButtonClicked();
  void saveDataButtonClicked();
  void stopSavingDataButtonClicked();
  void pauseSavingDataButtonClicked(bool checked);
  void nodesButtonClicked();
  void pullNodePlaceInfoButtonClicked();

  void targetComboItemActivated(int index);
  void sendNodePlaceInfoPullQueryButtonClicked();

  /** This function checks if the dialog should be closed. */
  void exitTimerFired();


private:

  PlaceMonitor *_monitor;
  place::PullNodePlaceInfoDialog *_pullNodePlaceInfoDialog;
  place::NodesDialog *_nodesDialog;
  place::PreviewDialog *_previewDialog;
  place::MapDialog *_mapDialog;
  place::SaveDataDialog *_saveDataDialog;
  place::StatsDialog *_statsDialog;
  place::WMDialog *_wmDialog;
  place::ResultsDialog *_resultsDialog;

  QVector<QRgb> _colorMap;

};


}

#endif

