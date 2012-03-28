/*
 * Author: Marko Mahnič
 * Created: 2011-06-20
 *
 * © Copyright 2011 Marko Mahnič. 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef QCASTDIALOGFRAME_DYSTP55V
#define QCASTDIALOGFRAME_DYSTP55V

#include "ui_dialogwin.h"
#include <QTabWidget>
#include <QScriptEngine>
#include <QCloseEvent>
#include "../Model.hpp"

// This object is known in JS as 'dialogOwner'.
// It is used for interacrtion between the dialog and the remote client.
class QCastDialogProxy:
   public QObject,
   public cogx::display::CGuiDialogDisplayProxy
{
   Q_OBJECT

private:
   cogx::display::CGuiDialog *pDialog;
   QWidget* wdialog;
   QScriptEngine engine;  // one per dialog
   QScriptValue uiobject; // interaction?

public:
   QCastDialogProxy(cogx::display::CGuiDialog* pDialog_, QWidget* pDialogWidget_);
   bool isProxyFor(cogx::display::CGuiDialog* pDialog_);

   // The client executes some JavaScript in the dialog (eg. to set dialog values)
   void execute(const std::string& script);

public slots:
   // These slots are used by JavaScript (through dialogOwner)
   void testMe();
   void setValue(QString name, QScriptValue value);
   void call(QString name, QScriptValue value);
   void setHtml(QString objectId, QString partId, QScriptValue value);
   void dumpObject(QString objectId, QScriptValue value);

   // These interfaces are missing in JS objects
   void setComboBoxItems(QString cbObjectName, QScriptValue stringItems);

private slots:
   void doExecute(QString script);
   void handleEngineException(QScriptValue exception);

signals:
   void sigExecute(QString script);

private:
   void dumpUncaughtException(const std::string& title, const std::string& context, const std::string& extra="");
};

class QCastDialogFrame:
   public QTabWidget,
   public cogx::display::CDisplayModelObserver
{
   Q_OBJECT
private:
   Ui::DialogWin ui;

private:
   QList<QCastDialogProxy*> m_dialogs;

   QCastDialogProxy* findDialog(cogx::display::CGuiDialog *pDialog);

public:
   QCastDialogFrame( QWidget * parent = nullptr, Qt::WindowFlags flags = 0 /* unused */ );
   ~QCastDialogFrame();
   void addDialog(cogx::display::CGuiDialog *pDialog);
   void closeEvent(QCloseEvent *event);
};

#endif

