
#include "formcap.hpp"
#include "v11n_jscode.inc"
#include <QStringList>

#ifdef DEBUG_TRACE
// #undef DEBUG_TRACE
#endif
#include "convenience.hpp"

QCastFormProxy::QCastFormProxy()
{
}

QCastFormProxy::~QCastFormProxy()
{
   for(TFormMapIterator it = m_Forms.begin(); it != m_Forms.end(); it++) {
      cogx::display::CHtmlChunk* pForm = it->second;
      pForm->Observers.removeObserver(this);
   }
}

void QCastFormProxy::registerForm(cogx::display::CHtmlChunk* pForm)
{
   if (!pForm) return;
   m_Forms[QString::fromStdString(pForm->htmlid())] = pForm;
   pForm->Observers.addObserver(this);
}

void QCastFormProxy::removeForm(cogx::display::CHtmlChunk* pForm)
{
   if (!pForm) return;
   pForm->Observers.removeObserver(this);
   m_Forms.erase(QString::fromStdString(pForm->htmlid()));
}

void QCastFormProxy::sendValues(const QString& formid, const QMap<QString,QVariant>& object)
{
   DTRACE("QCastFormProxy::sendValues");
   cogx::display::CHtmlChunk* pForm = NULL;
   
   // std::string id = formid.mid(1).toStdString(); // remove leading '#' from id
   TFormMapIterator it = m_Forms.find(formid.mid(1));
   // DMESSAGE("Looking for " << id << " among " << m_Forms.size() << " forms");
   if (it == m_Forms.end()) pForm = NULL;
   else pForm = it->second;

   if (pForm) {
      cogx::display::TFormValues vals;
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
   DTRACE("QCastFormProxy::getValues");
   cogx::display::CHtmlChunk* pForm = NULL;
   
   // std::string id = formid.mid(1).toStdString(); // remove leading '#' from id
   TFormMapIterator it = m_Forms.find(formid.mid(1));
   // DMESSAGE("Looking for " << id << " among " << m_Forms.size() << " forms");
   if (it == m_Forms.end()) pForm = NULL;
   else pForm = it->second;

   if (pForm) {
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

void QCastFormProxy::onOwnerDataChanged(cogx::display::CHtmlChunk *pForm,
      const cogx::display::TFormValues& newValues)
{
   DTRACE("QCastFormProxy::onOwnerDataChanged" << this);
   emit signalOwnerDataChanged(QString::fromStdString(pForm->htmlid()));
}
