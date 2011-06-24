
#include "formcap.hpp"
#include "v11n_jscode.inc"
#include <QStringList>
#include <QMessageBox>

#include "../QCastMainFrame.hpp"

#ifdef DEBUG_TRACE
#undef DEBUG_TRACE
#endif
#include "convenience.hpp"

namespace cxd = cogx::display;

QCastFormProxy::QCastFormProxy()
{
}

QCastFormProxy::~QCastFormProxy()
{
   for(TFormMapIterator it = m_Forms.begin(); it != m_Forms.end(); it++) {
      cxd::CHtmlChunk* pForm = it->second;
      pForm->Observers.removeObserver(this);
   }
}

void QCastFormProxy::registerChunk(cogx::display::CHtmlChunk* pChunk)
{
   if (! pChunk) return;
   if (! (pChunk->type() & (cxd::CHtmlChunk::form | cxd::CHtmlChunk::activehtml))) return;

   m_Forms[QString::fromStdString(pChunk->htmlid())] = pChunk;
   if (pChunk->type() == cxd::CHtmlChunk::form) {
      pChunk->Observers.addObserver(this);
   }
}

void QCastFormProxy::removeChunk(cogx::display::CHtmlChunk* pChunk)
{
   if (!pChunk) return;
   pChunk->Observers.removeObserver(this);
   m_Forms.erase(QString::fromStdString(pChunk->htmlid()));
}

void convertValues(const QMap<QString,QVariant>& object, cxd::TFormValues& vals)
{
   DTRACE("convertValues");
   foreach (QString str, object.keys()) {
      QVariant val = object.value(str);
      std::string sval;
      if (val.type() == QVariant::String)
         sval = val.toString().toStdString();
      else if (val.type() == QVariant::List) {
         QVariantList lst = val.toList();
         sval = "";
         foreach(QVariant item, lst) {
            QString s = item.toString();
            s.replace('\n', '\r');
            s.replace(QRegExp("\r+"), " ");
            if (sval.size() == 0) sval = s.toStdString();
            else sval += std::string("\n") + s.toStdString();
         }
      }
      vals[str.toStdString()] = sval;
      DMESSAGE(val.type() << " " << str.toStdString() << ":" << sval);
   }
}

void QCastFormProxy::sendValues(const QString& formid, const QMap<QString,QVariant>& object)
{
   DTRACE("QCastFormProxy::sendValues");
   cxd::CHtmlChunk* pForm = NULL;
   
   // std::string id = formid.mid(1).toStdString(); // remove leading '#' from id
   TFormMapIterator it = m_Forms.find(formid.mid(1));
   // DMESSAGE("Looking for " << id << " among " << m_Forms.size() << " forms");
   if (it == m_Forms.end()) pForm = NULL;
   else pForm = it->second;

   if (pForm) {
      cxd::TFormValues vals;
      convertValues(object, vals);
      pForm->notifyFormSubmit(vals, this);
   }
   else {
      DMESSAGE("FORM NOT ATTACHED");
      foreach (QString str, object.keys()) {
         DMESSAGE(str.toStdString() << ":" << object.value(str).toString().toStdString());
      }
   }
}

QMap<QString, QVariant> QCastFormProxy::getValues(const QString& formid)
{
   DTRACE("QCastFormProxy::getValues " << formid.toStdString());
   cxd::CHtmlChunk* pForm = NULL;
   
   TFormMapIterator it = m_Forms.find(formid.mid(1));
   if (it == m_Forms.end()) pForm = NULL;
   else pForm = it->second;

   if (pForm && pForm->type() == cxd::CHtmlChunk::form) {
      DMESSAGE("Fields: " << pForm->m_formData.size());
      QMap<QString, QVariant> data;
      typeof(pForm->m_formData.begin()) it;
      for(it = pForm->m_formData.begin(); it != pForm->m_formData.end(); it++) {
         size_t pos = it->second.find("\n");
         if (pos == std::string::npos) {
            data[QString::fromStdString(it->first)] = QVariant(QString::fromStdString(it->second));
            //DMESSAGE(it->first << ":" << it->second);
         }
         else {
            //DMESSAGE(it->first << ":LIST");
            QVariantList list;
            std::istringstream ss(it->second);
            std::string item;
            while(std::getline(ss, item, '\n')) {
               if (item == "") continue;
               //DMESSAGE(item);
               list << QVariant(QString::fromStdString(item));
            }
            data[QString::fromStdString(it->first)] = QVariant(list);
         }
      }
      return data;
   }
   else {
      DMESSAGE("NO SUCH FORM: " << formid.toStdString());
      return QMap<QString, QVariant>();
   }
}

void QCastFormProxy::setPost(const QString& formid, const QMap<QString,QVariant>& object)
{
   DTRACE("QCastFormProxy::setPost " << formid.toStdString());
   _post = object;
   sendValues(formid, object);
}

QMap<QString, QVariant> QCastFormProxy::getPost()
{
   DTRACE("QCastFormProxy::getPost");
   return _post;
}

void QCastFormProxy::setGet(const QString& formid, const QMap<QString,QVariant>& object)
{
   DTRACE("QCastFormProxy::setGet " << formid.toStdString());
   _get = object;
   sendValues(formid, object);
}

QMap<QString, QVariant> QCastFormProxy::getGet()
{
   DTRACE("QCastFormProxy::getGet");
   return _get;
}

void QCastFormProxy::onClick(const QString& htmlId, const QString& ctrlId)
{
   DTRACE("QCastFormProxy::onClick " << ctrlId.toStdString());

   cxd::CHtmlChunk* pChunk = NULL;
   TFormMapIterator it = m_Forms.find(htmlId.mid(1));
   if (it == m_Forms.end()) pChunk = NULL;
   else pChunk = it->second;

   if (pChunk) {
      pChunk->notifyChunkEvent(cxd::CHtmlChunk::onClick, ctrlId.toStdString(), "", this);
   }
   else {
      DMESSAGE("Chunk not registered: " << htmlId.toStdString());
   }
}

void QCastFormProxy::onSendValue(const QString& formid, const QString& ctrlId, const QString& valueId,
      const QMap<QString,QVariant>& object)
{
   DTRACE("QCastFormProxy::onSendValue " << ctrlId.toStdString());

   cxd::CHtmlChunk* pChunk = NULL;
   TFormMapIterator it = m_Forms.find(formid.mid(1));
   if (it == m_Forms.end()) pChunk = NULL;
   else pChunk = it->second;

   if (pChunk) {
      cxd::TFormValues vals;
      convertValues(object, vals);
      std::string value;
      typeof(vals.begin()) it = vals.find(valueId.toStdString());
      if (it != vals.end()) value = it->second;
      pChunk->notifyChunkEvent(cxd::CHtmlChunk::onSendValue, ctrlId.toStdString(), value, this);
   }
   else {
      DMESSAGE("Chunk not registered: " << formid.toStdString());
   }
}

void QCastFormProxy::saveFormData(const QString& formid, const QMap<QString,QVariant>& object)
{
   DTRACE("QCastFormProxy::saveFormData " << formid.toStdString());
   QCastMainFrame *pWin = dynamic_cast<QCastMainFrame*>(QApplication::activeWindow());
   if (! pWin) {
      DMESSAGE("FAILED TO GET MAIN WINDOW");
      return;
   }
   std::auto_ptr<QSettings> pSettings(pWin->getPersistentStorage());
   if (! pSettings.get()) {
      DMESSAGE("FAILED TO GET PERSISTENT STORAGE");
      return;
   }

   pSettings->beginGroup(QString("FormData/") + formid);
   pSettings->remove(""); // remove all existing values
   foreach (QString skey, object.keys()) {
      QVariant val = object.value(skey);
      if (val.type() == QVariant::List) {
         QVariantList lst = val.toList();
         QString slist;
         foreach(QVariant item, lst) {
            QString s = item.toString();
            s.replace(QRegExp("[\n\r]+"), " ");
            slist = slist + "\n" + s;
         }
         pSettings->setValue(skey, slist);
      }
      else pSettings->setValue(skey, val);
   }
   pSettings->endGroup();
   QMessageBox msgBox;
   msgBox.setText("Form data have been saved to the registry.");
   msgBox.exec();
}

QMap<QString, QVariant> QCastFormProxy::getSavedFormData(const QString& formid)
{
   DTRACE("QCastFormProxy::getSavedFormData");
   QCastMainFrame *pWin = dynamic_cast<QCastMainFrame*>(QApplication::activeWindow());
   if (! pWin) {
      DMESSAGE("FAILED TO GET MAIN WINDOW");
      return QMap<QString, QVariant>();
   }
   std::auto_ptr<QSettings> pSettings(pWin->getPersistentStorage());
   if (! pSettings.get()) {
      DMESSAGE("FAILED TO GET PERSISTENT STORAGE");
      return QMap<QString, QVariant>();
   }

   QMap<QString, QVariant> formData;
   pSettings->beginGroup(QString("FormData/") + formid);
   QStringList fields = pSettings->childKeys();
   foreach(QString fld, fields) {
      QString val = pSettings->value(fld, "").toString();
      DMESSAGE(fld.toStdString() << ":" << val.toStdString());
      if (val.indexOf('\n') < 0) 
         formData[fld] = QVariant(val);
      else {
         QStringList lst = val.split('\n', QString::SkipEmptyParts);
         formData[fld] = QVariant(lst);
      }
   }
   pSettings->endGroup();

   return formData;
}

QString QCastFormProxy::getJavaScript(const QString& jsObjectName, bool htmlScriptBlock)
{
   DTRACE("QCastFormProxy::getJavaScript");
   QStringList str;
   if (htmlScriptBlock) str << "<script type=\"text/javascript\">\n";
   QString jsc = jscode_formcap_js;
   str << jsc.replace("MyQObject", jsObjectName);
   if (htmlScriptBlock) str << "\n</script>";
   return str.join("");
}

// This function can only be called if the form is displayed in multiple windows and
// is submitted in another window.
void QCastFormProxy::onFormSubmitted(cogx::display::CHtmlChunk *pForm,
      const cogx::display::TFormValues& newValues)
{
   emit signalOwnerDataChanged(QString::fromStdString(pForm->htmlid()));
}

void QCastFormProxy::onForm_OwnerDataChanged(cogx::display::CHtmlChunk *pForm,
      const cogx::display::TFormValues& newValues)
{
   DTRACE("QCastFormProxy::onForm_OwnerDataChanged" << this);
   emit signalOwnerDataChanged(QString::fromStdString(pForm->htmlid()));
}
