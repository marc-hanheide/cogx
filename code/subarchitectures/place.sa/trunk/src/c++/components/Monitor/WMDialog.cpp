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
 * WMDialog class.
 * \file WMDialog.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-31
 */

// Place.SA
#include "WMDialog.h"
#include <place/idl/PlaceData.hh>
// CAST
#include <cast/architecture/ManagedProcess.hpp>
// Qt
#include <QHeaderView>
#include <QDesktopWidget>

using namespace place;


#define MAX_WM_LOG_ITEMS 40
#define MAX_CMD_LOG_ITEMS 30


// ------------------------------------------------------
WMDialog::WMDialog(QWidget *parent):
    QDialog(parent, Qt::Tool) // |Qt::WindowStaysOnTopHint
{
  // Setup ui
  setupUi(this);
  wmLogTreeWidget->header()->setResizeMode(0, QHeaderView::ResizeToContents);
  wmLogTreeWidget->header()->setResizeMode(1, QHeaderView::Fixed);
  wmLogTreeWidget->header()->resizeSection(1, 30);
  wmLogTreeWidget->header()->setResizeMode(2, QHeaderView::ResizeToContents);
  wmLogTreeWidget->header()->setResizeMode(3, QHeaderView::ResizeToContents);
  commandsTreeWidget->header()->setResizeMode(0, QHeaderView::ResizeToContents);
  commandsTreeWidget->header()->setResizeMode(1, QHeaderView::ResizeToContents);
  commandsTreeWidget->header()->setResizeMode(2, QHeaderView::ResizeToContents);
  commandsTreeWidget->header()->setResizeMode(3, QHeaderView::ResizeToContents);

  // Center dialog
  move(QApplication::desktop()->availableGeometry(parent).center()-(frameGeometry().bottomRight()-frameGeometry().topLeft())/2);

  // Icons
  _wmIcons<<QIcon(":/Monitor/resources/wm_unknown_16.png");
  _wmIcons<<QIcon(":/Monitor/resources/wm_add_16.png");
  _wmIcons<<QIcon(":/Monitor/resources/wm_delete_16.png");
  _wmIcons<<QIcon(":/Monitor/resources/wm_overwrite_16.png");
  _wmIcons<<QIcon(":/Monitor/resources/wm_get_16.png");
  _cmdIcons["DP_CMD_UPDATE"]=QIcon(":/Monitor/resources/dp_cmd_update_16.png");
  _cmdIcons["DS_CMD_UPDATE_START"]=QIcon(":/Monitor/resources/ds_cmd_start_16.png");
  _cmdIcons["DS_CMD_UPDATE_STOP"]=QIcon(":/Monitor/resources/ds_cmd_stop_16.png");
  _cmdIcons["DS_CMD_START"]=QIcon(":/Monitor/resources/ds_cmd_start_16.png");
  _cmdIcons["DS_CMD_STOP"]=QIcon(":/Monitor/resources/ds_cmd_stop_16.png");
  _cmdIcons["DS_CMD_PAUSE"]=QIcon(":/Monitor/resources/ds_cmd_pause_16.png");
  _cmdIcons["DS_CMD_UNPAUSE"]=QIcon(":/Monitor/resources/ds_cmd_unpause_16.png");
  _cmdIcons["DS_CMD_NEW_TARGET"]=QIcon(":/Monitor/resources/ds_cmd_new_target_16.png");
  _cmdIcons["VP_CMD_UPDATE_ON_READ_START"]=QIcon(":/Monitor/resources/ds_cmd_start_16.png");
  _cmdIcons["VP_CMD_UPDATE_STOP"]=QIcon(":/Monitor/resources/ds_cmd_stop_16.png");
  _typeIcons["PlaceData::Image"]=QIcon(":/Monitor/resources/type_image_32.png");
  _typeIcons["PlaceData::MonitorImage"]=QIcon(":/Monitor/resources/type_image_32.png");
  _typeIcons["PlaceData::Odometry"]=QIcon(":/Monitor/resources/type_odometry_32.png");
  _typeIcons["PlaceData::LaserScan"]=QIcon(":/Monitor/resources/type_scan_32.png");
  _typeIcons["PlaceData::Target"]=QIcon(":/Monitor/resources/type_target_32.png");
  _typeIcons["PlaceData::PlaceCommand"]=QIcon(":/Monitor/resources/type_command_32.png");
  _typeIcons["PlaceData::DataSaverStatus"]=QIcon(":/Monitor/resources/type_data_saver_status_32.png");
  _typeIcons["PlaceData::DataProviderCommand"]=QIcon(":/Monitor/resources/type_command_32.png");
  _typeIcons["PlaceData::DataSaverCommand"]=QIcon(":/Monitor/resources/type_command_32.png");
  _typeIcons["PlaceData::VisualProcessorCommand"]=QIcon(":/Monitor/resources/type_command_32.png");
  _typeIcons["PlaceData::LaserProcessorCommand"]=QIcon(":/Monitor/resources/type_command_32.png");
  _typeIcons["PlaceData::DataProviderCommandAck"]=QIcon(":/Monitor/resources/type_ack_32.png");
  _typeIcons["PlaceData::DataSaverCommandAck"]=QIcon(":/Monitor/resources/type_ack_32.png");
  _typeIcons["PlaceData::VisualProcessorCommandAck"]=QIcon(":/Monitor/resources/type_ack_32.png");
  _typeIcons["PlaceData::LaserProcessorCommandAck"]=QIcon(":/Monitor/resources/type_ack_32.png");
  _typeIcons["PlaceData::VisualResults"]=QIcon(":/Monitor/resources/type_results_32.png");
  _typeIcons["PlaceData::LaserResults"]=QIcon(":/Monitor/resources/type_results_32.png");
  _typeIcons["PlaceData::IntegratedResults"]=QIcon(":/Monitor/resources/type_results_32.png");
  _typeIcons["PlaceData::VisualProcessorStatus"]=QIcon(":/Monitor/resources/type_status_32.png");
  _typeIcons["PlaceData::LaserProcessorStatus"]=QIcon(":/Monitor/resources/type_status_32.png");

  // Clear things
  clearAll();
}



// ------------------------------------------------------
void WMDialog::updateWmInfo(int operation, QString id, QString src, QString type, double time)
{
  QIcon icon;
  if (operation==cast::cdl::ADD)
    icon=_wmIcons[1];
  else if (operation==cast::cdl::DELETE)
    icon=_wmIcons[2];
  else if (operation==cast::cdl::OVERWRITE)
    icon=_wmIcons[3];
  else if (operation==cast::cdl::GET)
    icon=_wmIcons[4];
  else
    icon=_wmIcons[0];

  // Update log
  QTreeWidgetItem *twItem = new QTreeWidgetItem();
  twItem->setText(0, QString::number(time));
  twItem->setIcon(1, icon);
  twItem->setText(2, src);
  twItem->setText(3, type);
  twItem->setText(4, id);
  wmLogTreeWidget->insertTopLevelItem(0, twItem);

  // Remove last if too man items
  if (wmLogTreeWidget->topLevelItemCount()>MAX_WM_LOG_ITEMS)
    delete wmLogTreeWidget->topLevelItem(wmLogTreeWidget->topLevelItemCount()-1);

  // Check if item like this exists in the contents view
  QListWidgetItem *existingItem=0;
  for (int i=0; i<wmListWidget->count(); ++i)
  {
    QListWidgetItem *lwItem=wmListWidget->item(i);
    if (lwItem->data(Qt::UserRole).toString()==id)
    {
      existingItem=lwItem;
    }
  }

  // Get item text and icon
  QString text=type+"\n("+id+")";
  if (_typeIcons.contains(type))
    icon=_typeIcons[type];
  else
    icon=QIcon(":/Monitor/resources/type_unknown_32.png");

  // Update the contents
  if (operation==cast::cdl::ADD)
  {
    if (!existingItem)
    {
      QListWidgetItem *lwItem = new QListWidgetItem();
      lwItem->setText(text);
      lwItem->setData(Qt::UserRole, id);
      lwItem->setIcon(icon);
      wmListWidget->addItem(lwItem);
    }
  }
  else if (operation==cast::cdl::DELETE)
  {
    if (existingItem)
    {
      delete existingItem;
    }
  }
  else if (operation==cast::cdl::OVERWRITE)
  {
    if (!existingItem)
    {
      QListWidgetItem *lwItem = new QListWidgetItem();
      lwItem->setText(text);
      lwItem->setData(Qt::UserRole, id);
      lwItem->setIcon(icon);
      wmListWidget->addItem(lwItem);
    }
  }
}


// ------------------------------------------------------
void WMDialog::updateCommandLog(QString cmd, QString params, QString src, double time)
{
  // Update log
  QTreeWidgetItem *twItem = new QTreeWidgetItem();
  twItem->setText(0, QString::number(time));
  if (_cmdIcons.contains(cmd))
    twItem->setIcon(1,_cmdIcons[cmd]);
  else
    twItem->setIcon(1, QIcon(":/Monitor/resources/cmd_unknown_16.png"));
  twItem->setText(1, cmd);
  twItem->setText(2, src);
  twItem->setText(3, params);
  commandsTreeWidget->insertTopLevelItem(0, twItem);

  // Remove last if too many items
  if (commandsTreeWidget->topLevelItemCount()>MAX_CMD_LOG_ITEMS)
    delete commandsTreeWidget->topLevelItem(commandsTreeWidget->topLevelItemCount()-1);
}



// ------------------------------------------------------
void WMDialog::showEvent(QShowEvent * event)
{
  QDialog::showEvent(event);
  clearAll();
}


// ------------------------------------------------------
void WMDialog::clearAll()
{
  wmListWidget->clear();
  wmLogTreeWidget->clear();
  commandsTreeWidget->clear();
}


