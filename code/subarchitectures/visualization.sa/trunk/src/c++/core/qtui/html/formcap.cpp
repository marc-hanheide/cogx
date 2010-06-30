
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
   m_Forms[pForm->htmlid()] = pForm;
   pForm->Observers.addObserver(this);
}

void QCastFormProxy::removeForm(cogx::display::CHtmlChunk* pForm)
{
   if (!pForm) return;
   pForm->Observers.removeObserver(this);
   m_Forms.erase(pForm->htmlid());
}

void QCastFormProxy::sendValues(const QString& formid, const QMap<QString,QVariant>& object)
{
   DTRACE("QCastFormProxy::sendValues");
   cogx::display::CHtmlChunk* pForm = NULL;
   
   std::string id = formid.mid(1).toStdString(); // remove leading '#' from id
   TFormMapIterator it = m_Forms.find(id);
   DMESSAGE("Looking for " << id << " among " << m_Forms.size() << " forms");
   if (it == m_Forms.end()) pForm = NULL;
   else pForm = it->second;

   // TODO: find form with formid
   if (pForm) {
      cogx::display::TFormValues vals;
      foreach (QString str, object.keys()) {
         vals[str.toStdString()] = object.value(str).toString().toStdString();
         DMESSAGE(str.toStdString() << ":" << object.value(str).toString().toStdString());
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
   // TODO: pass the data to the form
}

void QCastFormProxy::onOwnerDataChanged(cogx::display::CHtmlChunk *pForm,
      const cogx::display::TFormValues& newValues)
{
   // TODO: (important) pass the data to the form
}

