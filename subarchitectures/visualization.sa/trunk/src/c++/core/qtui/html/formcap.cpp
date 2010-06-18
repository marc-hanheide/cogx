
#include "formcap.hpp"
#include "v11n_jscode.inc"
#include <QStringList>

#ifdef DEBUG_TRACE
// #undef DEBUG_TRACE
#endif
#include "convenience.hpp"

void QCastFormObserver::setPost(const QMap<QString,QVariant>& object)
{
   DTRACE("QCastFormObserver::setPost");
   _post = object;
#ifdef DEBUG_TRACE
   foreach (QString str, object.keys()) {
      DMESSAGE(str.toStdString() << ":" << object.value(str).toString().toStdString());
   }
#endif
}

QMap<QString, QVariant> QCastFormObserver::getPost()
{
   DTRACE("QCastFormObserver::getPost");
   return _post;
}

void QCastFormObserver::setGet(const QMap<QString,QVariant>& object)
{
   DTRACE("QCastFormObserver::setGet");
   _get = object;
}

QMap<QString, QVariant> QCastFormObserver::getGet()
{
   DTRACE("QCastFormObserver::getGet");
   return _get;
}

QString QCastFormObserver::getJavaScript(const QString& jsObjectName, bool htmlScriptBlock)
{
   DTRACE("QCastFormObserver::getJavaScript");
   QStringList str;
   if (htmlScriptBlock) str << "<script type=\"text/javascript\">\n";
   QString jsc = jscode_formcap_js;
   str << jsc.replace("MyQObject", jsObjectName);
   if (htmlScriptBlock) str << "\n</script>";
   return str.join("");
}


