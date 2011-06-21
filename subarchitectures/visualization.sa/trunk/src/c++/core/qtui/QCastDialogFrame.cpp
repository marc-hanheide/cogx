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
#include "QCastDialogFrame.hpp"
#include "QCastMainFrame.hpp"

#ifdef DEBUG_TRACE
#undef DEBUG_TRACE
#endif
#include "../convenience.hpp"

#include <QUiLoader>
#include <QBuffer>
#include <QByteArray>

QCastDialogFrame::QCastDialogFrame( QWidget * parent, Qt::WindowFlags flags )
   : QTabWidget(parent)
{
   ui.setupUi(this);
   while (count() > 0) {
      QWidget* pw = widget(0);
      removeTab(0);
      if (pw) delete pw;
   }
}

QCastDialogFrame::~QCastDialogFrame()
{
   while (count() > 0) {
      QWidget* pw = widget(0);
      removeTab(0);
      if (pw) delete pw;
   }
   foreach(qpack* ppack, m_dialogs) {
      delete ppack;
   }
   m_dialogs.clear();
}

void QCastDialogFrame::closeEvent(QCloseEvent *event)
{
   DTRACE("QCastDialogFrame::closeEvent");
   setVisible(false);
   FrameManager.dialogWindowHidden(this);
   event->accept();
}

void QCastDialogFrame::addDialog(cogx::display::CGuiDialog *pDialog)
{
   DTRACE("QCastDialogFrame::addDialog");
   if (!pDialog)
      return;

   foreach(qpack* ppack, m_dialogs) {
      if (ppack->pDialog == pDialog)
         return;
   }

   qpack* pqpack = new qpack();
   pqpack->pDialog = pDialog;

   QUiLoader loader;
   QByteArray ba(pDialog->m_designCode.c_str());
   QBuffer uiBuf(&ba);
   QWidget *wui = loader.load(&uiBuf);
   pqpack->wdialog = wui;

   pqpack->engine.evaluate(QString::fromStdString(pDialog->m_scriptCode));

   QScriptValue ctor = pqpack->engine.evaluate(QString::fromStdString(pDialog->m_ctorName));
   QScriptValue scriptUi = pqpack->engine.newQObject(wui, QScriptEngine::ScriptOwnership);
   pqpack->uiobject = ctor.construct(QScriptValueList() << scriptUi);

   int idx = addTab(wui, QString::fromStdString(pDialog->m_id));

   m_dialogs << pqpack;
}

