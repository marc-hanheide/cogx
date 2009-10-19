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
 * \file MonitorDialog.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-12
 */

// Place.SA
#include "Monitor.h"
#include "MonitorDialog.h"
#include "PreviewDialog.h"
#include "SaveDataDialog.h"
#include "ResultsDialog.h"
#include "PullNodePlaceInfoDialog.h"
#include "NodesDialog.h"
#include "MapDialog.h"
#include "StatsDialog.h"
#include "WMDialog.h"
#include <PlaceData.hpp>
// Qt
#include <QMessageBox>
#include <QFileDialog>
#include <QTimer>
// STD
#include <iostream>
#include <math.h>

using namespace place;
using namespace std;


// ------------------------------------------------------
MonitorDialog::MonitorDialog(PlaceMonitor *monitor, QWidget *parent): 
    QDialog(parent), _monitor(monitor)
{
  // Setup ui
  setupUi(this);

  // Create dialogs
  _previewDialog = new PreviewDialog(this);
  _saveDataDialog = new SaveDataDialog(this);
  _mapDialog = new MapDialog(this);
  _statsDialog = new StatsDialog(this);
  _wmDialog = new WMDialog(this);
  _nodesDialog = new NodesDialog(this);
  _pullNodePlaceInfoDialog = new PullNodePlaceInfoDialog(this);
  _resultsDialog = new ResultsDialog(this);
  const LabelFile &labels = _monitor->getLabels();
  for (LabelFile::const_iterator l = labels.begin(); l!=labels.end(); l++)
    _saveDataDialog->targetComboBox->addItem(l->name.c_str(), QVariant(l->number));

  // Connect signals to slots
  connect(startPlaceButton, SIGNAL(clicked()), this, SLOT(startPlaceButtonClicked()));
  connect(stopPlaceButton, SIGNAL(clicked()), this, SLOT(stopPlaceButtonClicked()));
  connect(updateButton, SIGNAL(clicked()), this, SLOT(updateButtonClicked()));
  connect(startOnReadVPButton, SIGNAL(clicked()), this, SLOT(startOnReadVPButtonClicked()));
  connect(startOnReadDSButton, SIGNAL(clicked()), this, SLOT(startOnReadDSButtonClicked()));
  connect(stopVPButton, SIGNAL(clicked()), this, SLOT(stopVPButtonClicked()));
  connect(stopDSButton, SIGNAL(clicked()), this, SLOT(stopDSButtonClicked()));
  connect(killButton, SIGNAL(clicked()), this, SLOT(killButtonClicked()));
  connect(saveDataButton, SIGNAL(clicked()), this, SLOT(saveDataButtonClicked()));
  connect(dataPreviewButton, SIGNAL(clicked()), this, SLOT(dataPreviewButtonClicked()));
  connect(mapButton, SIGNAL(clicked()), this, SLOT(mapButtonClicked()));
  connect(wmButton, SIGNAL(clicked()), this, SLOT(wmButtonClicked()));
  connect(statsButton, SIGNAL(clicked()), this, SLOT(statsButtonClicked()));
  connect(resultsButton, SIGNAL(clicked()), this, SLOT(resultsButtonClicked()));
  connect(nodesButton, SIGNAL(clicked()), this, SLOT(nodesButtonClicked()));
  connect(pullNodePlaceInfoButton, SIGNAL(clicked()), this,
      SLOT(pullNodePlaceInfoButtonClicked()));

  // PullNodePlaceInfoDialog
  connect(_pullNodePlaceInfoDialog->sendQueryButton, SIGNAL(clicked()),
      this, SLOT(sendNodePlaceInfoPullQueryButtonClicked()));

  // NodesDialog
  connect(this, SIGNAL(updateNodeLabellerDataSignal(PlaceData::NodeLabellerDataPtr)),
      _nodesDialog, SLOT(updateNodeLabellerData(PlaceData::NodeLabellerDataPtr)));

  // ResultsDialog
  connect(this,
      SIGNAL(updateVisualResultsSignal(long, QVector<PlaceData::ClassifierResult>)),
      _resultsDialog,
      SLOT(updateVisualResults(long, QVector<PlaceData::ClassifierResult>)));
  connect(this, SIGNAL(updateResultsTargetSignal(long, QString, long)),
      _resultsDialog, SLOT(updateTarget(long, QString, long)));
  connect(this,
      SIGNAL(updateLaserResultsSignal(long, QVector<PlaceData::ClassifierResult>)),
      _resultsDialog,
      SLOT(updateLaserResults(long, QVector<PlaceData::ClassifierResult>)));
  connect(this,
      SIGNAL(updateIntegratedResultsSignal(long, bool, bool, QVector<PlaceData::ClassifierResult>)),
      _resultsDialog,
      SLOT(updateIntegratedResults(long, bool, bool, QVector<PlaceData::ClassifierResult>)));

  // WMDialog
  connect(this, SIGNAL(updateWmInfoSignal(int, QString, QString, QString, double)),
      _wmDialog, SLOT(updateWmInfo(int, QString, QString, QString, double)));
  connect(this, SIGNAL(updateCommandLogSignal(QString, QString, QString, double)),
      _wmDialog, SLOT(updateCommandLog(QString, QString, QString, double)));

  // StatsDialog
  connect(this, SIGNAL(updateImageStatsSignal(long, double, double)),
      _statsDialog, SLOT(updateImageStats(long, double, double)));
  connect(this, SIGNAL(updateLaserScanStatsSignal(long, double, double)),
      _statsDialog, SLOT(updateLaserScanStats(long, double, double)));
  connect(this, SIGNAL(updateOdometryStatsSignal(long, double, double)),
      _statsDialog, SLOT(updateOdometryStats(long, double, double)));
  connect(this, SIGNAL(updateVisualProcessorStatsSignal(double, double, double)),
      _statsDialog, SLOT(updateVisualProcessorStats(double, double, double)));
  connect(this, SIGNAL(updateLaserProcessorStatsSignal(double, double, double)),
      _statsDialog, SLOT(updateLaserProcessorStats(double, double, double)));

  // PreviewDialog
  connect(this, SIGNAL(updatePreviewImageSignal(QImage, long, double)),
      _previewDialog, SLOT(updateImage(QImage, long, double)));
  connect(this,
      SIGNAL(updatePreviewLaserScanSignal(double, double, double, QVector<double>, long, double)),
      _previewDialog,
      SLOT(updateLaserScan(double, double, double, QVector<double>, long, double)));
  connect(this,
      SIGNAL(updatePreviewOdometrySignal(double, double, double, long, double)),
      _previewDialog,
      SLOT(updateOdometry(double, double, double, long, double)));
  connect(this,
      SIGNAL(updatePreviewTargetSignal(long, QString, long)),
      _previewDialog,
      SLOT(updateTarget(long, QString, long)));

  // MapDialog
  connect(this,
      SIGNAL(addMapLaserScanSignal(double, double, double, QVector<double>, long, double)),
      _mapDialog,
      SLOT(addLaserScan(double, double, double, QVector<double>, long, double)));
  connect(this,
      SIGNAL(addMapOdometrySignal(double, double, double, long, double)),
      _mapDialog,
      SLOT(addOdometry(double, double, double, long, double)));

  // SaveDataDialog
  connect(_saveDataDialog->pauseButton, SIGNAL(clicked(bool)),
      this, SLOT(pauseSavingDataButtonClicked(bool)));
  connect(_saveDataDialog->stopButton, SIGNAL(clicked()),
      this, SLOT(stopSavingDataButtonClicked()));
  connect(_saveDataDialog, SIGNAL(closeEventHappened()),
      this, SLOT(stopSavingDataButtonClicked()));
  connect(_saveDataDialog->targetComboBox, SIGNAL(activated(int)),
      this, SLOT(targetComboItemActivated(int)));
  connect(this, SIGNAL(updateDataSavedInfoSignal(bool, bool, bool, long, long, bool)),
      _saveDataDialog, SLOT(updateDataSavedInfo(bool, bool, bool, long, long, bool)));

  qRegisterMetaType< QVector<double> >("QVector<double>");
  qRegisterMetaType< PlaceData::NodeLabellerDataPtr >("PlaceData::NodeLabellerDataPtr");
  qRegisterMetaType< QVector<PlaceData::ClassifierResult> >("QVector<PlaceData::ClassifierResult>");
  // Color map for images
  for (int i=0; i<256; ++i)
    _colorMap.append(qRgb(i, i, i));

  // Exit timer, checks if the dialog should be closed
  QTimer *exitTimer = new QTimer(this);
  connect(exitTimer, SIGNAL(timeout()), this, SLOT(exitTimerFired()));
  exitTimer->start(100);
}


// ------------------------------------------------------
void MonitorDialog::addWmChange(cast::cdl::WorkingMemoryOperation operation,
    std::string id, std::string src, std::string type, double time)
{
  if (_wmDialog->isVisible())
  {
    emit updateWmInfoSignal((int)operation, QString::fromStdString(id),
        QString::fromStdString(src), QString::fromStdString(type), time);
  }
}


// ------------------------------------------------------
void MonitorDialog::newImage(const PlaceData::ImagePtr image)
{
  dataFrameLabel->setText(QString::number(image->frameNo));

  if (image->status==PlaceData::DsValid)
  {
    if (_previewDialog->isVisible())
    {
      // Convert image to pixmap
      int w=image->imageBuffer.width;
      int h=image->imageBuffer.height;
      QImage tmpImage(w, h, QImage::Format_Indexed8);
      tmpImage.setColorTable(_colorMap);

      for (int i=0; i<h; ++i)
        for (int j=0; j<w; ++j)
          tmpImage.setPixel(j, i, (unsigned char)(image->imageBuffer.data[i*w+j]) );

      // Emit signal
      emit updatePreviewImageSignal(tmpImage, image->frameNo,
          image->realTimeStamp.s + 10e-6*image->realTimeStamp.us);
    }
  }

  if (_statsDialog->isVisible())
  {
    emit updateImageStatsSignal(image->frameNo,
        image->realTimeStamp.s + 10e-6*image->realTimeStamp.us,
        image->wmTimeStamp.s + 10e-6*image->wmTimeStamp.us );
  }
}


// ------------------------------------------------------
void MonitorDialog::newLaserScan(const PlaceData::LaserScanPtr scan)
{
  dataFrameLabel->setText(QString::number(scan->frameNo));

  if (scan->status==PlaceData::DsValid)
  {
    // Convert scan info
    QVector<double> range(scan->scanBuffer.ranges.size());
    for(unsigned int i=0; i<scan->scanBuffer.ranges.size(); ++i)
      range[i]=scan->scanBuffer.ranges[i];
    double max=scan->scanBuffer.maxRange;
    double startAngle=scan->scanBuffer.startAngle;
    double angleStep=scan->scanBuffer.angleStep;

    if (_previewDialog->isVisible())
      emit updatePreviewLaserScanSignal(startAngle, angleStep, max, range,
          scan->frameNo, scan->realTimeStamp.s + 10e-6*scan->realTimeStamp.us);
    if (_mapDialog->isVisible())
      emit addMapLaserScanSignal(startAngle, angleStep, max, range,
          scan->frameNo, scan->realTimeStamp.s + 10e-6*scan->realTimeStamp.us);
  }

  if (_statsDialog->isVisible())
  {
    emit updateLaserScanStatsSignal(scan->frameNo,
        scan->realTimeStamp.s + 10e-6*scan->realTimeStamp.us,
        scan->wmTimeStamp.s + 10e-6*scan->wmTimeStamp.us);
  }
}


// ------------------------------------------------------
void MonitorDialog::newOdometry(const PlaceData::OdometryPtr odom)
{
  dataFrameLabel->setText(QString::number(odom->frameNo));

  if (odom->status==PlaceData::DsValid)
  {
    // Convert odom info
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    if (!odom->odometryBuffer.odompose.empty())
    {
      x=odom->odometryBuffer.odompose[0].x;
      y=odom->odometryBuffer.odompose[0].y;
      theta= odom->odometryBuffer.odompose[0].theta;
    }

    if (_previewDialog->isVisible())
      emit updatePreviewOdometrySignal(x, y, theta,
          odom->frameNo, odom->realTimeStamp.s + 10e-6*odom->realTimeStamp.us);
    if (_mapDialog->isVisible())
      emit addMapOdometrySignal(x, y, theta, odom->frameNo,
          odom->realTimeStamp.s + 10e-6 * odom->realTimeStamp.us);
  }

  if (_statsDialog->isVisible())
  {
    emit updateOdometryStatsSignal(odom->frameNo,
        odom->realTimeStamp.s + 10e-6*odom->realTimeStamp.s,
        odom->wmTimeStamp.s + 10e-6*odom->wmTimeStamp.us);
  }
}


// ------------------------------------------------------
void MonitorDialog::newTarget(const PlaceData::TargetPtr target)
{
  if (target->status==PlaceData::DsValid)
  {
    if (_previewDialog->isVisible())
    {
      emit updatePreviewTargetSignal(target->targetNo,
          QString::fromStdString(target->targetName), target->frameNo);
    }
    if (_resultsDialog->isVisible())
    {
      emit updateResultsTargetSignal(target->targetNo,
          QString::fromStdString(target->targetName), target->frameNo);
    }
  }
}


// ------------------------------------------------------
void MonitorDialog::newVisualProcessorStatus(const PlaceData::VisualProcessorStatusPtr stat)
{
  if (stat->status==PlaceData::DsValid)
  {
    if (_statsDialog->isVisible())
    {
      emit updateVisualProcessorStatsSignal(
          stat->processingStartTimeStamp.s + 10e-6*stat->processingStartTimeStamp.us,
          stat->extractionEndTimeStamp.s + 10e-6*stat->extractionEndTimeStamp.us,
          stat->classificationEndTimeStamp.s + 10e-6*stat->classificationEndTimeStamp.us );
    }
  }
}


// ------------------------------------------------------
void MonitorDialog::newVisualResults(const PlaceData::VisualResultsPtr res)
{
  if (res->status==PlaceData::DsValid)
  {
    if (_resultsDialog->isVisible())
    {
      QVector<PlaceData::ClassifierResult> results(res->results.size());
      for (unsigned int i=0; i<res->results.size(); ++i)
        results[i]=res->results[i];
      emit updateVisualResultsSignal(res->frameNo, results);
    }
  }
}


// ------------------------------------------------------
void MonitorDialog::newLaserProcessorStatus(const PlaceData::LaserProcessorStatusPtr stat)
{
  if (stat->status==PlaceData::DsValid)
  {
    if (_statsDialog->isVisible())
    {
      emit updateLaserProcessorStatsSignal(
          stat->processingStartTimeStamp.s + 10e-6*stat->processingStartTimeStamp.us,
          stat->extractionEndTimeStamp.s + 10e-6*stat->extractionEndTimeStamp.us,
          stat->classificationEndTimeStamp.s + 10e-6*stat->classificationEndTimeStamp.us );
    }
  }
}


// ------------------------------------------------------
void MonitorDialog::newLaserResults(const PlaceData::LaserResultsPtr res)
{
  if (res->status==PlaceData::DsValid)
  {
    if (_resultsDialog->isVisible())
    {
      QVector<PlaceData::ClassifierResult> results(res->results.size());
      for (unsigned int i=0; i<res->results.size(); ++i)
        results[i]=res->results[i];
      emit updateLaserResultsSignal(res->frameNo, results);
    }
  }
}


// ------------------------------------------------------
void MonitorDialog::newIntegratedResults(const PlaceData::IntegratedResultsPtr res)
{
  intgResFrameLabel->setText(QString::number(res->frameNo));

  if (res->status==PlaceData::DsValid)
  {
    if (_resultsDialog->isVisible())
    {
      QVector<PlaceData::ClassifierResult> results(res->results.size());
      for (unsigned int i=0; i<res->results.size(); ++i)
        results[i]=res->results[i];
      emit updateIntegratedResultsSignal(res->frameNo,
          res->usedVision, res->usedLaser, results);
    }
  }
}


// ------------------------------------------------------
void MonitorDialog::newNodeLabellerData(const PlaceData::NodeLabellerDataPtr info)
{
  if (_nodesDialog->isVisible())
  {
    const PlaceData::NodeLabellerDataPtr info2 =
        new PlaceData::NodeLabellerData(*info);
    emit updateNodeLabellerDataSignal(info2);
  }
}


// ------------------------------------------------------
void MonitorDialog::newDataSaverStatus(bool vision, bool laser,
    bool odometry, long framesSaved, long frameNo, bool error)
{
  if (_saveDataDialog->isVisible())
  {
    emit updateDataSavedInfoSignal(vision, laser, odometry,
        framesSaved, frameNo, error);
  }
}


// ------------------------------------------------------
void MonitorDialog::newCommand(std::string cmd, std::string params,
    std::string src, double time)
{
  if (_wmDialog->isVisible())
  {
    emit updateCommandLogSignal(QString::fromStdString(cmd),
        QString::fromStdString(params), QString::fromStdString(src), time);
  }
}


// ------------------------------------------------------
void MonitorDialog::updateButtonClicked()
{
  _monitor->sendDataProviderCommand(PlaceData::DpCmdUpdate);
}


// ------------------------------------------------------
void MonitorDialog::startPlaceButtonClicked()
{
  _monitor->sendPlaceCommand(PlaceData::CmdStart);
}


// ------------------------------------------------------
void MonitorDialog::stopPlaceButtonClicked()
{
  _monitor->sendPlaceCommand(PlaceData::CmdStop);
}


// ------------------------------------------------------
void MonitorDialog::startOnReadVPButtonClicked()
{
  _monitor->sendVisualProcessorCommand(PlaceData::VpCmdUpdateOnReadStart);
}


// ------------------------------------------------------
void MonitorDialog::startOnReadDSButtonClicked()
{
  _monitor->sendDataSaverCommand(PlaceData::DsCmdUpdateStart, "", "", -1, "");
}


// ------------------------------------------------------
void MonitorDialog::stopVPButtonClicked()
{
  _monitor->sendVisualProcessorCommand(PlaceData::VpCmdUpdateStop);
}


// ------------------------------------------------------
void MonitorDialog::stopDSButtonClicked()
{
  _monitor->sendDataSaverCommand(PlaceData::DsCmdUpdateStop, "", "", -1, "");
}


// ------------------------------------------------------
void MonitorDialog::dataPreviewButtonClicked()
{
  _previewDialog->show();
}


// ------------------------------------------------------
void MonitorDialog::mapButtonClicked()
{
  _mapDialog->clearData();
  _mapDialog->show();
}


// ------------------------------------------------------
void MonitorDialog::statsButtonClicked()
{
  _statsDialog->show();
}


// ------------------------------------------------------
void MonitorDialog::wmButtonClicked()
{
  _wmDialog->show();
}


// ------------------------------------------------------
void MonitorDialog::resultsButtonClicked()
{
  _resultsDialog->show();
}


// ------------------------------------------------------
void MonitorDialog::nodesButtonClicked()
{
  _nodesDialog->show();
}


// ------------------------------------------------------
void MonitorDialog::pullNodePlaceInfoButtonClicked()
{
  _pullNodePlaceInfoDialog->show();
}


// ------------------------------------------------------
void MonitorDialog::saveDataButtonClicked()
{
  // Are we already saving?
  if (_saveDataDialog->isVisible())
    return;

  // Get the paths
  QString filePath = QFileDialog::getSaveFileName(this, "Select Data Config File",  "",
                                                  "Data Config Files (*.data)", 0,
                                                   QFileDialog::DontConfirmOverwrite);
  if (filePath.isEmpty())
    return;

  // Extract dir and filename
  QFileInfo fInfo(filePath);
  QString baseName=fInfo.baseName();
  QString dirPath=fInfo.absolutePath();

  QFileInfo f2Info(QDir(dirPath), baseName+".data");
  if (f2Info.exists())
  {
    QMessageBox::warning(this, "File Exists", "File exists and cannot be overwritten!");
    return;
  }


  // Make the button disabled
  saveDataButton->setEnabled(false);

  // Open dialog and start saving
  _saveDataDialog->clearStats();
  _saveDataDialog->show();
  _monitor->sendDataSaverCommand(PlaceData::DsCmdStart, dirPath.toStdString(),
      baseName.toStdString(), _monitor->getLabels().begin()->number,
      _monitor->getLabels().begin()->name);
}


// ------------------------------------------------------
void MonitorDialog::stopSavingDataButtonClicked()
{
  _saveDataDialog->_stopOnClose=false;
  _saveDataDialog->close();
  _monitor->sendDataSaverCommand(PlaceData::DsCmdStop, "", "", -1, "");
  saveDataButton->setEnabled(true);
}


// ------------------------------------------------------
void MonitorDialog::pauseSavingDataButtonClicked(bool checked)
{
  if (checked)
    _monitor->sendDataSaverCommand(PlaceData::DsCmdPause, "", "", -1, "");
  else
    _monitor->sendDataSaverCommand(PlaceData::DsCmdUnpause, "", "", -1, "");
}


// ------------------------------------------------------
void MonitorDialog::exitTimerFired()
{
  if (!_monitor->isRunning())
  {
    done(0);
  }
}


// ------------------------------------------------------
void MonitorDialog::targetComboItemActivated(int index)
{
  int targetNo=_saveDataDialog->targetComboBox->itemData(index).toInt();
  std::string targetName=_monitor->getLabels().labelNoToName(targetNo);

  if (targetName.empty())
  {
    QMessageBox::critical(this, "Target Not Found",
        "Target number was not found in the list!");
    return;
  }

  _monitor->sendDataSaverCommand(PlaceData::DsCmdNewTarget, "", "",
      targetNo, targetName);
}


// ------------------------------------------------------
void MonitorDialog::killButtonClicked()
{
  exit(0);
}


// ------------------------------------------------------
void MonitorDialog::sendNodePlaceInfoPullQueryButtonClicked()
{
  /* Commented out for now since, this part of code might not be used
  if (_monitor->isConnectedToNodeLabeller())
  {
    // Prepare query
    bool gateway=_pullNodePlaceInfoDialog->yesRadioButton->isChecked();
    PlaceData::NodeLabellerQueryType queryType = PlaceData::NlQtInfo;
    if (_pullNodePlaceInfoDialog->infoRadioButton->isChecked())
      queryType = PlaceData::NlQtInfo;
    else if (_pullNodePlaceInfoDialog->newRadioButton->isChecked())
      queryType = PlaceData::NlQtNew;
    else if (_pullNodePlaceInfoDialog->updateRadioButton->isChecked())
      queryType = PlaceData::NlQtUpdate;

    // Pull

    PlaceData::NodePlaceInfo nodePlaceInfo =
        _monitor->pullNodePlaceInfo(_pullNodePlaceInfoDialog->nodeIdSpinBox->value(), queryType, gateway);

    _pullNodePlaceInfoDialog->nodeIdLabel->setText(QString::number(nodePlaceInfo.nodeId));
    _pullNodePlaceInfoDialog->nodeClassLabel->setText(QString::number(nodePlaceInfo.nodeClassNo)+" - "+
                                                      QString(nodePlaceInfo.nodeClassName));
    _pullNodePlaceInfoDialog->areaClassLabel->setText(QString::number(nodePlaceInfo.areaClassNo)+" - "+
                                                      QString(nodePlaceInfo.areaClassName));

  }
  else
  {
    QMessageBox::critical(this, "Error!", "NodeLabeller not connected to Monitor!");
  }
  */
}


