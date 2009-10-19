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
 * NodesDialog class.
 * \file NodesDialog.cpp
 * \author Andrzej Pronobis
 * \date 2008-09-04
 */

// Place.SA
#include "NodesDialog.h"
// QT
#include <QDesktopWidget>
#include <QHeaderView>

using namespace place;


// ------------------------------------------------------
NodesDialog::NodesDialog(QWidget *parent):
    QDialog(parent, Qt::Tool) //|Qt::WindowStaysOnTopHint
{
  // Setup ui
  setupUi(this);

  // Center dialog
  move(QApplication::desktop()->availableGeometry(parent).center()-(frameGeometry().bottomRight()-frameGeometry().topLeft())
  /2);

  nodesTreeWidget->header()->setResizeMode(0, QHeaderView::ResizeToContents);
  nodesTreeWidget->header()->setResizeMode(1, QHeaderView::ResizeToContents);

  clearAll();
}


// ------------------------------------------------------
void NodesDialog::showEvent(QShowEvent * event)
{
  QDialog::showEvent(event);
  clearAll();
}


// ------------------------------------------------------
void NodesDialog::clearAll()
{
  nodesTreeWidget->clear();
}


// ------------------------------------------------------
QString NodesDialog::getOutputString(PlaceData::ClassifierOutputs outs, double outputsCount)
{
  if (outs.empty())
    return "";

  QString outStr = QString::fromStdString(outs[0].name) + ":" +
      QString::number(outs[0].value/outputsCount);
  for (unsigned int i=1; i<outs.size(); ++i)
  {
    outStr+=(QString(" ") + QString::fromStdString(outs[i].name) + ":" +
        QString::number(outs[i].value/outputsCount));
  }
  return outStr;
}


// ------------------------------------------------------
QString NodesDialog::getResultString(PlaceData::ClassifierResults res)
{
  if (res.empty())
    return "";

  QString resStr = QString::number(res[0].classNo) + "-" +
      QString::fromStdString(res[0].className) + ":" +
      QString::number(res[0].confidence);
  for (unsigned int i=1; i<res.size(); ++i)
  {
    resStr+=(QString(" ") + QString::number(res[i].classNo) + "-" +
        QString::fromStdString(res[i].className) + ":" +
        QString::number(res[i].confidence));
  }
  return resStr;
}


// ------------------------------------------------------
void NodesDialog::updateNodeLabellerData(PlaceData::NodeLabellerDataPtr nodeLabellerData)
{
  lastNodeLabel->setText(QString::number(nodeLabellerData->lastNodeId));
  nodesTreeWidget->clear();

  // Current node info
  QTreeWidgetItem *twItem = new QTreeWidgetItem();
  twItem->setText(0, "Cur");
  twItem->setText(1, ((nodeLabellerData->currentNode.gateway)?"1":"0") );
  twItem->setText(2, getOutputString(nodeLabellerData->currentNode.nodeAccumulatedOutputs, 1));
  twItem->setText(3, getOutputString(nodeLabellerData->currentNode.nodeAccumulatedOutputs,
                  nodeLabellerData->currentNode.nodeOutputCount));
  twItem->setText(4, getResultString(nodeLabellerData->currentNode.nodeResults));
  twItem->setText(5, getOutputString(nodeLabellerData->currentNode.areaAccumulatedOutputs, 1));
  twItem->setText(6, getOutputString(nodeLabellerData->currentNode.areaAccumulatedOutputs,
                  nodeLabellerData->currentNode.areaOutputCount));
  twItem->setText(7, getResultString(nodeLabellerData->currentNode.areaResults));
  nodesTreeWidget->addTopLevelItem(twItem);

  for (unsigned int i=0; i<nodeLabellerData->nodes.size(); ++i)
  {
    QTreeWidgetItem *twItem = new QTreeWidgetItem();
    twItem->setText(0, QString::number(nodeLabellerData->nodes[i].nodeId));
    twItem->setText(1, ((nodeLabellerData->nodes[i].gateway)?"1":"0") );
    twItem->setText(2, getOutputString(nodeLabellerData->nodes[i].nodeAccumulatedOutputs, 1));
    twItem->setText(3, getOutputString(nodeLabellerData->nodes[i].nodeAccumulatedOutputs,
                    nodeLabellerData->nodes[i].nodeOutputCount));
    twItem->setText(4, getResultString(nodeLabellerData->nodes[i].nodeResults));
    twItem->setText(5, getOutputString(nodeLabellerData->nodes[i].areaAccumulatedOutputs, 1));
    twItem->setText(6, getOutputString(nodeLabellerData->nodes[i].areaAccumulatedOutputs,
                    nodeLabellerData->nodes[i].areaOutputCount));
    twItem->setText(7, getResultString(nodeLabellerData->nodes[i].areaResults));
    if (nodeLabellerData->nodes[i].nodeId==nodeLabellerData->lastNodeId)
    {
      twItem->setBackground(0, QColor(230, 0, 0));
      twItem->setBackground(1, QColor(230, 0, 0));
      twItem->setBackground(2, QColor(230, 0, 0));
      twItem->setBackground(3, QColor(230, 0, 0));
      twItem->setBackground(4, QColor(230, 0, 0));
      twItem->setBackground(5, QColor(230, 0, 0));
      twItem->setBackground(6, QColor(230, 0, 0));
      twItem->setBackground(7, QColor(230, 0, 0));
    }
    nodesTreeWidget->addTopLevelItem(twItem);
  }
}


