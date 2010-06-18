
#ifndef FORMCAP_QF5IUX3L
#define FORMCAP_QF5IUX3L

#include <QObject>
#include <QString>
#include <QVariant>
#include <QMap>

// An object of this instance is added to a web page with
//    QWebFrame::addToJavaScriptWindowObject("MyQFormObserver", obj)
// The string produced with getJavaScript("MyQFormObserver", true) is added
// to the page head.
// The form to be processed starts with:
//    <form id="myform" onsubmit="return MyLibSubmit('#myform')" >
class QCastFormObserver: public QObject
{
   Q_OBJECT
private:
   QMap<QString, QVariant> _post;
   QMap<QString, QVariant> _get;

public slots:
   void setPost(const QMap<QString,QVariant>& object);
   QMap<QString, QVariant> getPost();
   void setGet(const QMap<QString,QVariant>& object);
   QMap<QString, QVariant> getGet();

   static QString getJavaScript(const QString& jsObjectName, bool htmlScriptBlock = false);
};

#endif /* end of include guard: FORMCAP_QF5IUX3L */
