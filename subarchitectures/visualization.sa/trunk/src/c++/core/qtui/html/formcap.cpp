
#include "formcap.hpp"
#include "v11n_jscode.inc"
#include <QStringList>

#ifdef DEBUG_TRACE
// #undef DEBUG_TRACE
#endif
#include "convenience.hpp"

QCastFormProxy::QCastFormProxy(/*CHtmlChunk* pForm*/)
{
   //m_pForm = pForm;
   m_pForm = NULL;
   if (m_pForm) {
      m_pForm->Observers.addObserver(this);
   }
}

QCastFormProxy::~QCastFormProxy()
{
   if (m_pForm) {
      m_pForm->Observers.removeObserver(this);
   }
}

void QCastFormProxy::sendValues(const QMap<QString,QVariant>& object)
{
   DTRACE("QCastFormProxy::sendValues");
   if (m_pForm) {
      cogx::display::TFormValues vals;
      foreach (QString str, object.keys()) {
         vals[str.toStdString()] = object.value(str).toString().toStdString();
         DMESSAGE(str.toStdString() << ":" << object.value(str).toString().toStdString());
      }
      m_pForm->notifyFormSubmit(vals, this);
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
   sendValues(object);
}

QMap<QString, QVariant> QCastFormProxy::getPost()
{
   DTRACE("QCastFormProxy::getPost");
   return _post;
}

void QCastFormProxy::setGet(const QString& formid, const QMap<QString,QVariant>& object)
{
   DTRACE("QCastFormProxy::setGet");
   _get = object;
   sendValues(object);
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

