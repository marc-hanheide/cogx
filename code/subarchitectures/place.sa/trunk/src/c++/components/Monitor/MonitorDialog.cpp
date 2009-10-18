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
#include <place/idl/PlaceData.hh>
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
  connect(pullNodePlaceInfoButton, SIGNAL(clicked()), this, SLOT(pullNodePlaceInfoButtonClicked()));

  // PullNodePlaceInfoDialog
  connect(_pullNodePlaceInfoDialog->sendQueryButton, SIGNAL(clicked()), this, SLOT(sendNodePlaceInfoPullQueryButtonClicked()));

  // NodesDialog
  connect(this, SIGNAL(updateNodeLabellerDataSignal(PlaceData::NodeLabellerData)), _nodesDialog, SLOT(updateNodeLabellerData(PlaceData::NodeLabellerData)));

  // ResultsDialog
  connect(this, SIGNAL(updateVisualResultsSignal(long, QVector<PlaceData::ClassifierResult>)), _resultsDialog, SLOT(updateVisualResults(long, QVector<PlaceData::ClassifierResult>)));
  connect(this, SIGNAL(updateResultsTargetSignal(long, QString, long)), _resultsDialog, SLOT(updateTarget(long, QString, long)));
  connect(this, SIGNAL(updateLaserResultsSignal(long, QVector<PlaceData::ClassifierResult>)), _resultsDialog, SLOT(updateLaserResults(long, QVector<PlaceData::ClassifierResult>)));
  connect(this, SIGNAL(updateIntegratedResultsSignal(long, bool, bool, QVector<PlaceData::ClassifierResult>)), _resultsDialog, SLOT(updateIntegratedResults(long, bool, bool, QVector<PlaceData::ClassifierResult>)));

  // WMDialog
  connect(this, SIGNAL(updateWmInfoSignal(int, QString, QString, QString, double)), _wmDialog, SLOT(updateWmInfo(int, QString, QString, QString, double)));
  connect(this, SIGNAL(updateCommandLogSignal(QString, QString, QString, double)), _wmDialog, SLOT(updateCommandLog(QString, QString, QString, double)));

  // StatsDialog
  connect(this, SIGNAL(updateImageStatsSignal(long, double, double)), _statsDialog, SLOT(updateImageStats(long, double, double)));
  connect(this, SIGNAL(updateLaserScanStatsSignal(long, double, double)), _statsDialog, SLOT(updateLaserScanStats(long, double, double)));
  connect(this, SIGNAL(updateOdometryStatsSignal(long, double, double)), _statsDialog, SLOT(updateOdometryStats(long, double, double)));
  connect(this, SIGNAL(updateVisualProcessorStatsSignal(double, double, double)), _statsDialog, SLOT(updateVisualProcessorStats(double, double, double)));
  connect(this, SIGNAL(updateLaserProcessorStatsSignal(double, double, double)), _statsDialog, SLOT(updateLaserProcessorStats(double, double, double)));

  // PreviewDialog
  connect(this, SIGNAL(updatePreviewImageSignal(QImage, long, double)), _previewDialog, SLOT(updateImage(QImage, long, double)));
  connect(this, SIGNAL(updatePreviewLaserScanSignal(double, double, double, QVector<double>, long, double)), _previewDialog, SLOT(updateLaserScan(double, double, double, QVector<double>, long, double)));
  connect(this, SIGNAL(updatePreviewOdometrySignal(double, double, double, long, double)), _previewDialog, SLOT(updateOdometry(double, double, double, long, double)));
  connect(this, SIGNAL(updatePreviewTargetSignal(long, QString, long)), _previewDialog, SLOT(updateTarget(long, QString, long)));

  // MapDialog
  connect(this, SIGNAL(addMapLaserScanSignal(double, double, double, QVector<double>, long, double)), _mapDialog, SLOT(addLaserScan(double, double, double, QVector<double>, long, double)));
  connect(this, SIGNAL(addMapOdometrySignal(double, double, double, long, double)), _mapDialog, SLOT(addOdometry(double, double, double, long, double)));

  // SaveDataDialog
  connect(_saveDataDialog->pauseButton, SIGNAL(clicked(bool)), this, SLOT(pauseSavingDataButtonClicked(bool)));
  connect(_saveDataDialog->stopButton, SIGNAL(clicked()), this, SLOT(stopSavingDataButtonClicked()));
  connect(_saveDataDialog, SIGNAL(closeEventHappened()), this, SLOT(stopSavingDataButtonClicked()));
  connect(_saveDataDialog->targetComboBox, SIGNAL(activated(int)), this, SLOT(targetComboItemActivated(int)));
  connect(this, SIGNAL(updateDataSavedInfoSignal(bool, bool, bool, long, long, bool)), _saveDataDialog, SLOT(updateDataSavedInfo(bool, bool, bool, long, long, bool)));

  qRegisterMetaType< QVector<double> >("QVector<double>");
  qRegisterMetaType< PlaceData::NodeLabellerData >("PlaceData::NodeLabellerData");
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
void MonitorDialog::addWmChange(cast::cdl::WorkingMemoryOperation operation, const char *id, const char *src, const char *type, double time)
{
  if (_wmDialog->isVisible())
  {
    emit updateWmInfoSignal((int)operation, QString(id), QString(src), QString(type), time);
  }
}


// ------------------------------------------------------
void MonitorDialog::newImage(const PlaceData::Image &image)
{
  dataFrameLabel->setText(QString::number(image.frameNo));

  if (image.status==PlaceData::DS_VALID)
  {
    if (_previewDialog->isVisible())
    {
      // Convert image to pixmap
      int w=image.imageBuffer.m_width;
      int h=image.imageBuffer.m_height;
      QImage tmpImage(w, h, QImage::Format_Indexed8);
      tmpImage.setColorTable(_colorMap);

      for (int i=0; i<h; ++i)
        for (int j=0; j<w; ++j)
          tmpImage.setPixel(j, i, (unsigned char)(image.imageBuffer.m_image[i*w+j]) );

      // Emit signal
      emit updatePreviewImageSignal(tmpImage, image.frameNo, BALTTimer::toSeconds(image.realTimeStamp));
    }
  }

  if (_statsDialog->isVisible())
  {
    emit updateImageStatsSignal(image.frameNo, BALTTimer::toSeconds(image.realTimeStamp), 
                                BALTTimer::toSeconds(image.wmTimeStamp));
  }
}


// ------------------------------------------------------
void MonitorDialog::newLaserScan(const PlaceData::LaserScan &scan)
{
  dataFrameLabel->setText(QString::number(scan.frameNo));

  if (scan.status==PlaceData::DS_VALID)
  {
    // Convert scan info
    QVector<double> range(scan.scanBuffer.m_numPts);
    for(int i=0; i<scan.scanBuffer.m_numPts; ++i)
      range[i]=scan.scanBuffer.m_range[i];
    double max=scan.scanBuffer.m_max_range;
    double startAngle=scan.scanBuffer.m_start_angle;
    double angleStep=scan.scanBuffer.m_angle_step;

    if (_previewDialog->isVisible())
      emit updatePreviewLaserScanSignal(startAngle, angleStep, max, range, scan.frameNo, BALTTimer::toSeconds(scan.realTimeStamp));
    if (_mapDialog->isVisible())
      emit addMapLaserScanSignal(startAngle, angleStep, max, range, scan.frameNo, BALTTimer::toSeconds(scan.realTimeStamp));
  }

  if (_statsDialog->isVisible())
  {
    emit updateLaserScanStatsSignal(scan.frameNo, BALTTimer::toSeconds(scan.realTimeStamp),
                                    BALTTimer::toSeconds(scan.wmTimeStamp));
  }
}


// ------------------------------------------------------
void MonitorDialog::newOdometry(const PlaceData::Odometry &odom)
{
  dataFrameLabel->setText(QString::number(odom.frameNo));

  if (odom.status==PlaceData::DS_VALID)
  {
    // Convert odom info
    double x = odom.odometryBuffer.m_x;
    double y = odom.odometryBuffer.m_y;
    double theta = odom.odometryBuffer.m_theta;
    if (_previewDialog->isVisible())
      emit updatePreviewOdometrySignal(x, y, theta, odom.frameNo, BALTTimer::toSeconds(odom.realTimeStamp));
    if (_mapDialog->isVisible())
      emit addMapOdometrySignal(x, y, theta, odom.frameNo, BALTTimer::toSeconds(odom.realTimeStamp));
  }

  if (_statsDialog->isVisible())
  {
    emit updateOdometryStatsSignal(odom.frameNo, BALTTimer::toSeconds(odom.realTimeStamp), 
                                   BALTTimer::toSeconds(odom.wmTimeStamp));
  }
}


// ------------------------------------------------------
void MonitorDialog::newTarget(const PlaceData::Target &target)
{
  if (target.status==PlaceData::DS_VALID)
  {
    if (_previewDialog->isVisible())
    {
      emit updatePreviewTargetSignal(target.targetNo, QString(target.targetName), target.frameNo);
    }
    if (_resultsDialog->isVisible())
    {
      emit updateResultsTargetSignal(target.targetNo, QString(target.targetName), target.frameNo);
    }
  }
}


// ------------------------------------------------------
void MonitorDialog::newVisualProcessorStatus(const PlaceData::VisualProcessorStatus &stat)
{
  if (stat.status==PlaceData::DS_VALID)
  {
    if (_statsDialog->isVisible())
    {
      emit updateVisualProcessorStatsSignal(BALTTimer::toSeconds(stat.processingStartTimeStamp),
                                            BALTTimer::toSeconds(stat.extractionEndTimeStamp),
                                            BALTTimer::toSeconds(stat.classificationEndTimeStamp) );
    }
  }
}


// ------------------------------------------------------
void MonitorDialog::newVisualResults(const PlaceData::VisualResults &res)
{
  if (res.status==PlaceData::DS_VALID)
  {
    if (_resultsDialog->isVisible())
    {
      QVector<PlaceData::ClassifierResult> results(res.results.length());
      for (unsigned int i=0; i<res.results.length(); ++i)
        results[i]=res.results[i];
      emit updateVisualResultsSignal(res.frameNo, results);
    }
  }
}


// ------------------------------------------------------
void MonitorDialog::newLaserProcessorStatus(const PlaceData::LaserProcessorStatus &stat)
{
  if (stat.status==PlaceData::DS_VALID)
  {
    if (_statsDialog->isVisible())
    {
      emit updateLaserProcessorStatsSignal(BALTTimer::toSeconds(stat.processingStartTimeStamp),
                                           BALTTimer::toSeconds(stat.extractionEndTimeStamp),
                                           BALTTimer::toSeconds(stat.classificationEndTimeStamp) );
    }
  }
}


// ------------------------------------------------------
void MonitorDialog::newLaserResults(const PlaceData::LaserResults &res)
{
  if (res.status==PlaceData::DS_VALID)
  {
    if (_resultsDialog->isVisible())
    {
      QVector<PlaceData::ClassifierResult> results(res.results.length());
      for (unsigned int i=0; i<res.results.length(); ++i)
        results[i]=res.results[i];
      emit updateLaserResultsSignal(res.frameNo, results);
    }
  }
}


// ------------------------------------------------------
void MonitorDialog::newIntegratedResults(const PlaceData::IntegratedResults &res)
{
  intgResFrameLabel->setText(QString::number(res.frameNo));

  if (res.status==PlaceData::DS_VALID)
  {
    if (_resultsDialog->isVisible())
    {
      QVector<PlaceData::ClassifierResult> results(res.results.length());
      for (unsigned int i=0; i<res.results.length(); ++i)
        results[i]=res.results[i];
      emit updateIntegratedResultsSignal(res.frameNo, res.usedVision, res.usedLaser, results);
    }
  }
}


// ------------------------------------------------------
void MonitorDialog::newNodeLabellerData(const PlaceData::NodeLabellerData &info)
{
  if (_nodesDialog->isVisible())
  {
    emit updateNodeLabellerDataSignal(info);
  }
}


// ------------------------------------------------------
void MonitorDialog::newDataSaverStatus(bool vision, bool laser, bool odometry, long framesSaved, long frameNo, bool error)
{
  if (_saveDataDialog->isVisible())
  {
    emit updateDataSavedInfoSignal(vision, laser, odometry, framesSaved, frameNo, error);
  }
}


// ------------------------------------------------------
void MonitorDialog::newCommand(const char *cmd, const char *params, const char *src, double time)
{
  if (_wmDialog->isVisible())
  {
    emit updateCommandLogSignal(QString(cmd), QString(params), QString(src), time);
  }
}


// ------------------------------------------------------
void MonitorDialog::updateButtonClicked()
{
  _monitor->sendDataProviderCommand(PlaceData::DP_CMD_UPDATE);
}


// ------------------------------------------------------
void MonitorDialog::startPlaceButtonClicked()
{
  _monitor->sendPlaceCommand(PlaceData::CMD_START);
}


// ------------------------------------------------------
void MonitorDialog::stopPlaceButtonClicked()
{
  _monitor->sendPlaceCommand(PlaceData::CMD_STOP);
}


// ------------------------------------------------------
void MonitorDialog::startOnReadVPButtonClicked()
{
  _monitor->sendVisualProcessorCommand(PlaceData::VP_CMD_UPDATE_ON_READ_START);
}


// ------------------------------------------------------
void MonitorDialog::startOnReadDSButtonClicked()
{
  _monitor->sendDataSaverCommand(PlaceData::DS_CMD_UPDATE_START, "", "", -1, "");
}


// ------------------------------------------------------
void MonitorDialog::stopVPButtonClicked()
{
  _monitor->sendVisualProcessorCommand(PlaceData::VP_CMD_UPDATE_STOP);
}


// ------------------------------------------------------
void MonitorDialog::stopDSButtonClicked()
{
  _monitor->sendDataSaverCommand(PlaceData::DS_CMD_UPDATE_STOP, "", "", -1, "");
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
  _monitor->sendDataSaverCommand(PlaceData::DS_CMD_START, dirPath.toStdString(), baseName.toStdString(), 
                                 _monitor->getLabels().begin()->number, _monitor->getLabels().begin()->name);
}


// ------------------------------------------------------
void MonitorDialog::stopSavingDataButtonClicked()
{
  _saveDataDialog->_stopOnClose=false;
  _saveDataDialog->close();
  _monitor->sendDataSaverCommand(PlaceData::DS_CMD_STOP, "", "", -1, "");
  saveDataButton->setEnabled(true);
}


// ------------------------------------------------------
void MonitorDialog::pauseSavingDataButtonClicked(bool checked)
{
  if (checked)
    _monitor->sendDataSaverCommand(PlaceData::DS_CMD_PAUSE, "", "", -1, "");
  else
    _monitor->sendDataSaverCommand(PlaceData::DS_CMD_UNPAUSE, "", "", -1, "");
}


// ------------------------------------------------------
void MonitorDialog::exitTimerFired()
{
  if (!_monitor->isStatusRun())
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
    QMessageBox::critical(this, "Target Not Found", "Target number was not found in the list!");
    return;
  }

  _monitor->sendDataSaverCommand(PlaceData::DS_CMD_NEW_TARGET, "", "", targetNo, 
                                 targetName);
}


// ------------------------------------------------------
void MonitorDialog::killButtonClicked()
{
  exit(0);
}


// ------------------------------------------------------
void MonitorDialog::sendNodePlaceInfoPullQueryButtonClicked()
{
  if (_monitor->isConnectedToNodeLabeller())
  {
    // Prepare query
    bool gateway=_pullNodePlaceInfoDialog->yesRadioButton->isChecked();
    PlaceData::NodeLabellerQueryType queryType = PlaceData::NL_QT_INFO;
    if (_pullNodePlaceInfoDialog->infoRadioButton->isChecked())
      queryType = PlaceData::NL_QT_INFO;
    else if (_pullNodePlaceInfoDialog->newRadioButton->isChecked())
      queryType = PlaceData::NL_QT_NEW;
    else if (_pullNodePlaceInfoDialog->updateRadioButton->isChecked())
      queryType = PlaceData::NL_QT_UPDATE;

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
}


