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
// #undef DEBUG_TRACE
#endif
#include "../convenience.hpp"

#include <QUiLoader>
#include <QBuffer>
#include <QByteArray>
#include <QComboBox>

QCastDialogProxy::QCastDialogProxy(cogx::display::CGuiDialog* pDialog_, QWidget* pDialogWidget_)
{
   this->pDialog = pDialog_;
   this->wdialog = pDialogWidget_;
   pDialog_->m_pDialogView = this;

   connect(&engine, SIGNAL(signalHandlerException(QScriptValue)),
         this, SLOT(handleEngineException(QScriptValue)),
         Qt::QueuedConnection);

   QScriptValue glob = engine.globalObject();
   QScriptValue proxy = engine.newQObject(this, QScriptEngine::QtOwnership);
   //           , QScriptEngine::PreferExistingWrapperObject | QScriptEngine::ExcludeSuperClassContents);
   glob.setProperty("dialogOwner", proxy);

   QString s = QString::fromStdString(pDialog->m_ctorName);
   s.replace(QRegExp("[^a-zA-Z0-9_]"), " ");
   QStringList lsConstruct = s.split(' ', QString::SkipEmptyParts);
   QString sName;
   if (lsConstruct.size() > 1)
      sName = lsConstruct[1].trimmed();
   if (lsConstruct.size() < 1) {
      // The type was not provided. Invent one.
      sName = "_fake_object_";
      engine.evaluate("function _Fake_Object_Type_(ui) { this.ui = ui }");
      lsConstruct << "_Fake_Object_Type_";
   }
   if (sName.length() < 1)
      sName = "_" + lsConstruct[0].trimmed() + "_";

   engine.evaluate(QString::fromStdString(pDialog->m_scriptCode), QString::fromStdString(pDialog->m_id));
   if (engine.hasUncaughtException()) {
      dumpUncaughtException("Exception while loading script", "loading");
   }

   QScriptValue ctor = engine.evaluate(lsConstruct[0]);
   if (engine.hasUncaughtException()) {
      dumpUncaughtException("Exception while creating proxy", "evaltype");
   }
   QScriptValue scriptUi = engine.newQObject(this->wdialog, QScriptEngine::ScriptOwnership);
   if (engine.hasUncaughtException()) {
      dumpUncaughtException("Exception while creating proxy", "newobject");
   }
   uiobject = ctor.construct(QScriptValueList() << scriptUi);
   if (engine.hasUncaughtException()) {
      dumpUncaughtException("Exception while creating proxy", "construct");
   }

   if (sName.length() > 0) {
      glob.setProperty(sName, uiobject);
      DMESSAGE("JS Object of type '" << lsConstruct[0].toStdString() <<
           "' is konwn as '" << sName.toStdString() << "'");
   }

   connect(this, SIGNAL(sigExecute(QString)),
         this, SLOT(doExecute(QString)),
         Qt::QueuedConnection);
}

bool QCastDialogProxy::isProxyFor(cogx::display::CGuiDialog* pDialog_)
{
   return this->pDialog == pDialog_;
}

void QCastDialogProxy::testMe()
{
   DMESSAGE(" ****** I'VE BEEN TESTED **** ");
}

void QCastDialogProxy::setValue(QString name, QScriptValue value)
{
   DMESSAGE("setValue(" << name.toStdString() << ", " << value.toString().toStdString() << ")");
   // TODO: convert value to JSON
   pDialog->notify_setValue(name.toStdString(), value.toString().toStdString());
}

void QCastDialogProxy::call(QString name, QScriptValue value)
{
   DMESSAGE("call: " << name.toStdString() << "(" << value.toString().toStdString() << ")");
   // TODO: convert value to JSON
   pDialog->notify_call(name.toStdString(), value.toString().toStdString());
}

void QCastDialogProxy::setHtml(QString objectId, QString partId, QScriptValue value)
{
   DMESSAGE("setHtml: " << objectId.toStdString() << "." << partId.toString());
   pDialog->notify_setHtml(objectId.toStdString(), partId.toStdString(), value.toString().toStdString());
}

void QCastDialogProxy::setComboBoxItems(QString cbObjectName, QScriptValue stringItems)
{
   DTRACE("QCastDialogProxy::setComboBoxItems");
   //QComboBox* pcb = dynamic_cast<QComboBox*>(cbObject.toQObject());

   QComboBox* pcb = wdialog->findChild<QComboBox*>(cbObjectName);
   if (pcb) {
      pcb->blockSignals(true);
      pcb->clear();
      QVariant qitems = stringItems.toVariant();
      QStringList items = qitems.toStringList();
      foreach(QString value, items) {
         pcb->addItem(value);
      }
      pcb->blockSignals(false);
   }
}

void QCastDialogProxy::execute(const std::string& script)
{
   DTRACE("QCastDialogProxy::execute");
   // engine.evaluate(QString::fromStdString(script));
   emit sigExecute(QString::fromStdString(script));
}

void QCastDialogProxy::doExecute(QString script)
{
   DTRACE("QCastDialogProxy::doExecute");
   engine.evaluate(script, "execInDialog");
   if (engine.hasUncaughtException()) {
      dumpUncaughtException("Exception in doExecute()", "doExecute", script.toStdString());
   }
}

void QCastDialogProxy::handleEngineException(QScriptValue exception)
{
   dumpUncaughtException("Signal Handler Exception", "sigexception");
}

void QCastDialogProxy::dumpUncaughtException(const std::string& title, const std::string& context,
      const std::string& extra)
{
   std::string htmlobj = "@scripterror"; 
   std::ostringstream ss;
   ss << "<h3>" << title << " (" << pDialog->m_id << ")</h3>";
   ss << engine.uncaughtException().toString().toStdString();
   ss << "<br>---- trace ----</br>";
   QStringList items = engine.uncaughtExceptionBacktrace();
   foreach(QString value, items) {
      ss << value.toStdString() << "<br>";
   }
   if (extra != "") {
      ss << "---- ----<br><pre>" << extra << "</pre>";
   }
   pDialog->notify_setHtml(htmlobj, pDialog->m_id + "-" + context, ss.str());
}

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
      if (pw)
         pw->deleteLater();
   }
   foreach(QCastDialogProxy* ppack, m_dialogs) {
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

   if (findDialog(pDialog))
      return;

   QUiLoader loader;
   QByteArray ba(pDialog->m_designCode.c_str());
   QBuffer uiBuf(&ba);
   QWidget *wui = loader.load(&uiBuf);

   QCastDialogProxy* pqpack = new QCastDialogProxy(pDialog, wui);

   int idx = addTab(wui, wui->windowTitle());
   setTabToolTip(idx, wui->toolTip() + "\nform.id:" + QString::fromStdString(pDialog->m_id));
   //pqpack->engine.evaluate("dialogOwner.testMe();");

   m_dialogs << pqpack;
}

QCastDialogProxy* QCastDialogFrame::findDialog(cogx::display::CGuiDialog *pDialog)
{
   foreach(QCastDialogProxy* ppack, m_dialogs) {
      if (ppack->isProxyFor(pDialog))
         return ppack;
   }
   return 0;
}
