
#ifndef FORMCAP_QF5IUX3L
#define FORMCAP_QF5IUX3L

#include "../../HtmlElements.hpp"
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
class QCastFormProxy:
   public QObject,
   public cogx::display::CHtmlFormObserver
{
   Q_OBJECT
private:
   QMap<QString, QVariant> _post;
   QMap<QString, QVariant> _get;
   cogx::display::CHtmlChunk* m_pForm;

private:
   void sendValues(const QMap<QString,QVariant>& object);

public:
   // TODO: redo from scratch;
   //    - maybe: a list of forms instd. of m_pForm
   //    - when sending verify if pForm.m_htmlid is the same as in setPost
   QCastFormProxy(/*CHtmlChunk* pForm=NULL*/);
   ~QCastFormProxy();

public slots:
   void setPost(const QString& formid, const QMap<QString,QVariant>& object);
   QMap<QString, QVariant> getPost();
   void setGet(const QString& formid, const QMap<QString,QVariant>& object);
   QMap<QString, QVariant> getGet();

   static QString getJavaScript(const QString& jsObjectName, bool htmlScriptBlock = false);

// CHtmlFormObserver
public:
   virtual void onFormSubmitted(cogx::display::CHtmlChunk *pForm,
         const cogx::display::TFormValues& newValues);
   virtual void onOwnerDataChanged(cogx::display::CHtmlChunk *pForm,
         const cogx::display::TFormValues& newValues);
};

#endif /* end of include guard: FORMCAP_QF5IUX3L */
// vim:sw=3:ts=8:et

// TODO: one object per form or parameter formid in getPost/setPost
// TODO: pointer to pModel (or better an intermediate object) that will handle form submits etc.
//       - the link between the two is created in QCastViewHtml::createJsObjects() which
//         is after the page has been reloaded
//       - QCastFormObserver must be an observer of the intermediate object so that
//         it can receive form data from owner component
//       - the intermediate object is an observer of the QCastFormObserver: a single form can
//         be displayed in multiple views so a simple pointer to interm. obj. doesn't suffice
// TODO: pModel must know about each QCastFormObserver so that it can send
//       to the form its content received from the owner cast component
//       ! QCastFormObserver should be a CHtmlFormObserver and should implement onOwnerDataChanged()
