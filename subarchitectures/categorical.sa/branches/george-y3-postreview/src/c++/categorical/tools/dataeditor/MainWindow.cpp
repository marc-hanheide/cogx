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
 * \file MainWindow.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-28
 */

#include "MainWindow.h"
#include "ProgressDialog.h"
#include "LabelDialog.h"
#include "shared/OdomScanWidget.h"
#include "shared/DataReader.h"
#include "shared/DataWriter.h"
#include "shared/LabelFile.h"
#include <cast/core/CASTTimer.hpp>

#include <QHeaderView>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QMessageBox>

using namespace categorical;
using namespace std;


// ------------------------------------------------------
MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent)
{
  // Setup UI
  setupUi( this );
  dataTreeWidget->header()->setResizeMode(0, QHeaderView::ResizeToContents);
  dataTreeWidget->header()->setResizeMode(1, QHeaderView::ResizeToContents);
  dataTreeWidget->header()->setResizeMode(2, QHeaderView::ResizeToContents);

  // Create the odomscanwidget
  QVBoxLayout *vbl = new QVBoxLayout(mapGroupBox);
  _osWidget = new OdomScanWidget(mapGroupBox, 35.0, QSize(2500, 2500));
  vbl->addWidget(_osWidget);
  vbl->setContentsMargins(2,2,2,2);
  _osWidget->setFrameShape(QFrame::StyledPanel);
  _osWidget->setFrameShadow(QFrame::Plain);

  // Actions
  connect(actionOpen, SIGNAL(triggered()), this, SLOT(actionOpenTriggered()));
  connect(actionOpenLabels, SIGNAL(triggered()), this, SLOT(actionOpenLabelsTriggered()));
  connect(actionSaveDataAs, SIGNAL(triggered()), this, SLOT(actionSaveDataAsTriggered()));
  connect(actionPlotScansUsingDots, SIGNAL(triggered()), this, SLOT(actionPlotScansUsingDotsTriggered()));
  connect(actionPlotScansUsingLines, SIGNAL(triggered()), this, SLOT(actionPlotScansUsingLinesTriggered()));
  connect(actionSaveMap, SIGNAL(triggered()), this, SLOT(actionSaveMapTriggered()));
  connect(actionSaveMapWithOverlay, SIGNAL(triggered()), this, SLOT(actionSaveMapWithOverlayTriggered()));
  connect(actionChangeLabel, SIGNAL(triggered()), this, SLOT(actionChangeLabelTriggered()));
  connect(actionMarkForRemoval, SIGNAL(triggered()), this, SLOT(actionMarkForRemovalTriggered()));
  connect(actionUnmark, SIGNAL(triggered()), this, SLOT(actionUnmarkTriggered()));
  connect(actionChangeLabelTo1, SIGNAL(triggered()), this, SLOT(actionChangeLabelTo1Triggered()));
  connect(actionChangeLabelTo2, SIGNAL(triggered()), this, SLOT(actionChangeLabelTo2Triggered()));

  // Context menus
  dataTreeWidget->addAction(actionChangeLabel);
  dataTreeWidget->addAction(actionMarkForRemoval);
  dataTreeWidget->addAction(actionUnmark);

  // Connections
  connect(dataTreeWidget, SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)), this, SLOT(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)));

  // Empty Image
  QPixmap p(320,240);
  p.fill();
  imageLabel->setPixmap(p);

  //Other
  _dataLoadingProgress=0;
}


// ------------------------------------------------------
void MainWindow::actionOpenTriggered()
{
  // Get file paths
  QString dataFilePath = QFileDialog::getOpenFileName(this, "Open Data Config File", "",
                                                  "Data Config Files (*.data)");

  if (!dataFilePath.isEmpty())
  {
    // Open dialog box
    _dataLoadingProgress = new ProgressDialog(this);
    _dataLoadingProgress->show();
    _dataLoadingProgress->setLabel("Loading data...");
    _dataLoadingProgress->setProgress(0);

    // Clear all
    clearAll();

    if (!readData(dataFilePath))
    {
      QMessageBox::critical(this, "Error",
                            "Couldn't load the data config file '"+dataFilePath+"'");
      setWindowTitle("DataEditor");
      clearAll();
    }
    else
    {
      setWindowTitle("DataEditor - "+dataFilePath);
    }

    // Close dialog box
    delete _dataLoadingProgress;
    _dataLoadingProgress=0;
  }
}


// ------------------------------------------------------
void MainWindow::actionOpenLabelsTriggered()
{
  QString labelFilePath = QFileDialog::getOpenFileName(this, "Open Label File", "",
      "Label Files (*.lbl)");

  if (!labelFilePath.isEmpty())
  {
    _labels.clear();

    LabelFile lf;
    if (!lf.read(labelFilePath.toStdString()))
    {
      QMessageBox::critical(this, "Error",
                            "Couldn't load the label file '"+labelFilePath+"'");
      _labels.clear();
    }

    // Read the labels to map
    for (LabelFile::const_iterator i = lf.begin(); i!=lf.end(); ++i)
      _labels[i->number]=i->name.c_str();
  }
}


// ------------------------------------------------------
void MainWindow::actionSaveDataAsTriggered()
{
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

  // Open dialog box
  ProgressDialog *progress = new ProgressDialog(this);
  progress->show();
  progress->setLabel("Saving data...");
  progress->setProgress(0);

  // Do writing
  DataWriter dw(dirPath.toStdString(), baseName.toStdString());
  int savedFrames=0;
  for (int i=0; i<dataTreeWidget->topLevelItemCount(); ++i)
  {
    progress->setProgress(100.0*((double)i)/((double)dataTreeWidget->topLevelItemCount()));

    int idx = dataTreeWidget->topLevelItem(i)->data(1, Qt::UserRole).toInt();
    bool markedRemoval = dataTreeWidget->topLevelItem(i)->data(0, Qt::UserRole).toBool();
    int frameNo=_images[idx].image->frameNo;
    if (!markedRemoval)
    {
      dw.setFrameNo(frameNo);
      if (!dw.copyImage(_images[idx].path.toStdString(), _images[idx].image))
      {
        QMessageBox::critical(this, "Error", QString("Couldn't save image for frame number %1").arg(frameNo));
        break;
      }
      if (!dw.writeLaserScan(_scans[idx]))
      {
        QMessageBox::critical(this, "Error", QString("Couldn't save scan for frame number %1").arg(frameNo));
        break;
      }
      if (!dw.writeOdometry(_odoms[idx]))
      {
        QMessageBox::critical(this, "Error", QString("Couldn't save odometry for frame number %1").arg(frameNo));
        break;
      }
      if (!dw.writeTarget(_targets[idx]->targetNo, string(_targets[idx]->targetName)))
      {
        QMessageBox::critical(this, "Error", QString("Couldn't save target for frame number %1").arg(frameNo));
        break;
      }
      savedFrames++;
    }
  }

  if (!dw.writeDataConfigFile(true, true, true, savedFrames))
  {
    QMessageBox::critical(this, "Error", QString("Couldn't save data config file!"));
  }

  // Close dialog box
  delete progress;
}


// ------------------------------------------------------
bool MainWindow::readData(QString dataFilePath)
{
  try
  {
    DataReader dr;

    // Read config file
    if (!dr.readDataConfigFile(dataFilePath.toStdString()))
      return false;

    if ( (!dr.isUseVision()) || (!dr.isUseLaser()) || (!dr.isUseOdometry()) )
      return false;

    // Allocate space for data
    long framesNo=dr.getFramesNo();
    _odoms.resize(framesNo);
    _scans.resize(framesNo);
    _images.resize(framesNo);
    _targets.resize(framesNo);
    QList<QTreeWidgetItem *> twItems;

    // Read data
    for(int i=0; i<framesNo; ++i)
    {
      _dataLoadingProgress->setProgress(100.0*((double)i)/((double)framesNo));
      string p;
      // Allocate memory
      _images[i].image = new CategoricalData::Image;
      _scans[i] = new CategoricalData::LaserScan;
      _odoms[i] = new CategoricalData::Odometry;
      _targets[i] = new CategoricalData::Target;
      // Read data
      dr.readImageInfo(p, _images[i].image, false);
      _images[i].path=p.c_str();
      dr.readLaserScan(_scans[i], false);
      dr.readOdometry(_odoms[i], false);
      dr.readTarget(_targets[i], false);
      // Check
      if ((_images[i].image->frameNo!=_scans[i]->frameNo) ||
           (_images[i].image->frameNo!=_odoms[i]->frameNo) ||
           (_images[i].image->frameNo!=_targets[i]->frameNo))
        return false;
      // Save labels
      _labels[_targets[i]->targetNo] = QString::fromStdString(_targets[i]->targetName);
      // Add item to the list
      QTreeWidgetItem *twItem = new QTreeWidgetItem();
      twItem->setData(0, Qt::UserRole, false);
      twItem->setIcon(0, QIcon(":/DataEditor/unmarked_16.png"));
      twItem->setData(1, Qt::UserRole, i);
      twItem->setText(1, QString::number(_images[i].image->frameNo));
      twItem->setText(2, QString::number((_images[i].image->realTimeStamp.s + _images[i].image->realTimeStamp.us * 1e-06), 'f', 6));
      twItem->setText(3, QString::number(_targets[i]->targetNo)+" - "+ QString::fromStdString(_targets[i]->targetName));
      twItems.append(twItem);

      // Update the map
      int scanPts=_scans[i]->scanBuffer.ranges.size();
      QVector<double> scanV(scanPts);
      for (int j=0; j<scanPts; ++j)
        scanV[j]=_scans[i]->scanBuffer.ranges[j];

      // Plot scan
      _osWidget->plotScan(_odoms[i]->odometryBuffer.odompose[0].x, _odoms[i]->odometryBuffer.odompose[0].y,
                          _odoms[i]->odometryBuffer.odompose[0].theta+_scans[i]->scanBuffer.startAngle,
                          scanV, _scans[i]->scanBuffer.angleStep, _scans[i]->scanBuffer.maxRange);
      // Plot path
      _osWidget->plotPathNode(_odoms[i]->odometryBuffer.odompose[0].x, _odoms[i]->odometryBuffer.odompose[0].y);
    }
    // Add items to the list
    dataTreeWidget->insertTopLevelItems(0, twItems);
  }
  catch(...)
  {
    return false;
  }

  _osWidget->updateView();

  return true;
}


// ------------------------------------------------------
void MainWindow::clearAll()
{
  _labels.clear();
  _odoms.clear();
  _scans.clear();
  _images.clear();
  _targets.clear();
  dataTreeWidget->clear();
  _osWidget->clearCanvas();
}


// ------------------------------------------------------
void MainWindow::currentItemChanged(QTreeWidgetItem * current, QTreeWidgetItem * previous)
{
  if (current==0)
    return;

  int i=current->data(1, Qt::UserRole).toInt();

  // Load image
  QPixmap image(_images[i].path);
  imageLabel->setPixmap(image);

  // Highlight scan and position
  int scanPts=_scans[i]->scanBuffer.ranges.size();
  QVector<double> scanV(scanPts);
  for (int j=0; j<scanPts; ++j)
    scanV[j]=_scans[i]->scanBuffer.ranges[j];

   // Plot scan
  _osWidget->clearOverlay();
  _osWidget->plotScanOverlay(_odoms[i]->odometryBuffer.odompose[0].x, _odoms[i]->odometryBuffer.odompose[0].y,
                      _odoms[i]->odometryBuffer.odompose[0].theta+_scans[i]->scanBuffer.startAngle,
                      scanV, _scans[i]->scanBuffer.angleStep, _scans[i]->scanBuffer.maxRange);
   // Plot path
  _osWidget->plotPointOverlay(_odoms[i]->odometryBuffer.odompose[0].x, _odoms[i]->odometryBuffer.odompose[0].y);

  _osWidget->updateView();

  _osWidget->ensureVisiblePoint(_odoms[i]->odometryBuffer.odompose[0].x, _odoms[i]->odometryBuffer.odompose[0].y);
}


// ------------------------------------------------------
void MainWindow::actionPlotScansUsingDotsTriggered()
{
  actionPlotScansUsingLines->setChecked(false);
  actionPlotScansUsingDots->setChecked(true);
  _osWidget->setScanOverlayStyle(OdomScanWidget::SOS_DOTS);
}


// ------------------------------------------------------
void MainWindow::actionPlotScansUsingLinesTriggered()
{
  actionPlotScansUsingLines->setChecked(true);
  actionPlotScansUsingDots->setChecked(false);
  _osWidget->setScanOverlayStyle(OdomScanWidget::SOS_LINES);
}


// ------------------------------------------------------
void MainWindow::actionSaveMapTriggered()
{
  QString filePath = QFileDialog::getSaveFileName(this, "Select file",  "",
      "PNG Files (*.png)");
  if (!filePath.isEmpty())
    _osWidget->saveCanvas(filePath, "PNG");
}


// ------------------------------------------------------
void MainWindow::actionSaveMapWithOverlayTriggered()
{
  QString filePath = QFileDialog::getSaveFileName(this, "Select file",  "",
      "PNG Files (*.png)");
  if (!filePath.isEmpty())
    _osWidget->saveCanvasAndOverlay(filePath, "PNG");

}


// ------------------------------------------------------
void MainWindow::actionChangeLabelTriggered()
{
  QList<QTreeWidgetItem *> selItems=dataTreeWidget->selectedItems();

  if (selItems.count()==0)
    return;

  // Select new label
  LabelDialog dialog(this);
  for (QMap<int, QString>::iterator i=_labels.begin(); i!=_labels.end(); ++i)
  {
    QListWidgetItem *lwItem = new QListWidgetItem();
    lwItem->setData(Qt::UserRole, i.key());
    lwItem->setText(QString::number(i.key())+" - "+i.value());
    dialog.labelListWidget->addItem(lwItem);
  }
  if (dialog.exec()==QDialog::Accepted)
  {
    QListWidgetItem *curItem = dialog.labelListWidget->currentItem();
    if (curItem!=0)
    {
      int labelNo=curItem->data(Qt::UserRole).toInt();
      QString labelName=_labels[labelNo];

      for(int i=0; i<selItems.count(); ++i)
      {
        int idx=selItems[i]->data(1, Qt::UserRole).toInt();
        _targets[idx]->targetNo=labelNo;
        _targets[idx]->targetName=labelName.toStdString();
        selItems[i]->setText(3, QString::number(_targets[idx]->targetNo)+" - "+ QString::fromStdString(_targets[idx]->targetName));
      }
    }
  }
}


// ------------------------------------------------------
void MainWindow::actionChangeLabelTo1Triggered()
{
  QList<QTreeWidgetItem *> selItems=dataTreeWidget->selectedItems();

  if (selItems.count()==0)
    return;

  // Select new label
  int labelNo=1;
  QString labelName=_labels[labelNo];

  for(int i=0; i<selItems.count(); ++i)
  {
	  int idx=selItems[i]->data(1, Qt::UserRole).toInt();
	  _targets[idx]->targetNo=labelNo;
	  _targets[idx]->targetName=labelName.toStdString();
	  selItems[i]->setText(3, QString::number(_targets[idx]->targetNo)+" - "+ QString::fromStdString(_targets[idx]->targetName));
  }
}


// ------------------------------------------------------
void MainWindow::actionChangeLabelTo2Triggered()
{
  QList<QTreeWidgetItem *> selItems=dataTreeWidget->selectedItems();

  if (selItems.count()==0)
    return;

  // Select new label
  int labelNo=2;
  QString labelName=_labels[labelNo];

  for(int i=0; i<selItems.count(); ++i)
  {
	  int idx=selItems[i]->data(1, Qt::UserRole).toInt();
	  _targets[idx]->targetNo=labelNo;
	  _targets[idx]->targetName=labelName.toStdString();
	  selItems[i]->setText(3, QString::number(_targets[idx]->targetNo)+" - "+ QString::fromStdString(_targets[idx]->targetName));
  }
}


// ------------------------------------------------------
void MainWindow::actionMarkForRemovalTriggered()
{
  QList<QTreeWidgetItem *> selItems=dataTreeWidget->selectedItems();

  if (selItems.count()==0)
    return;

  for(int i=0; i<selItems.count(); ++i)
  {
    selItems[i]->setData(0, Qt::UserRole, true);
    selItems[i]->setIcon(0, QIcon(":/DataEditor/marked_removal_16.png"));
  }
}


// ------------------------------------------------------
void MainWindow::actionUnmarkTriggered()
{
  QList<QTreeWidgetItem *> selItems=dataTreeWidget->selectedItems();

  if (selItems.count()==0)
    return;

  for(int i=0; i<selItems.count(); ++i)
  {
    selItems[i]->setData(0, Qt::UserRole, false);
    selItems[i]->setIcon(0, QIcon(":/DataEditor/unmarked_16.png"));
  }
}

