
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
   typedef std::map<QString, cogx::display::CHtmlChunk*> TFormMap;
   typedef TFormMap::iterator TFormMapIterator;
   TFormMap m_Forms;

private:
   void sendValues(const QString& formid, const QMap<QString,QVariant>& object);

public:
   // TODO: redo from scratch;
   //    - maybe: a list of forms instd. of m_pForm
   //    - when sending verify if pForm.m_htmlid is the same as in setPost
   QCastFormProxy();
   ~QCastFormProxy();
   void registerChunk(cogx::display::CHtmlChunk* pChunk);
   void removeChunk(cogx::display::CHtmlChunk* pChunk);

public slots:
   // formid conatins a leading #
   void setPost(const QString& formid, const QMap<QString,QVariant>& object);
   QMap<QString, QVariant> getPost();
   void setGet(const QString& formid, const QMap<QString,QVariant>& object);
   QMap<QString, QVariant> getGet();
   QMap<QString, QVariant> getValues(const QString& formid);
   void saveFormData(const QString& formid, const QMap<QString,QVariant>& object);
   QMap<QString, QVariant> getSavedFormData(const QString& formid);
   // htmlId doesn't contain a leading #
   void onClick(const QString& htmlId, const QString& ctrlId);
   void onSendValue(const QString& formid, const QString& ctrlId, const QString& valueId,
         const QMap<QString,QVariant>& object);

public:
   static QString getJavaScript(const QString& jsObjectName, bool htmlScriptBlock = false);

public:
   // CHtmlFormObserver
   // Both functions emit signalOwnerDataChanged and QCastViewHtml handles it.
   // (Note: another solution would be to make QCastViewHtml a CHtmlFormObserver)
   virtual void onFormSubmitted(cogx::display::CHtmlChunk *pForm,
         const cogx::display::TFormValues& newValues);
   virtual void onForm_OwnerDataChanged(cogx::display::CHtmlChunk *pForm,
         const cogx::display::TFormValues& newValues);
   virtual void onHtmlClick(cogx::display::CHtmlChunk *pChunk,
         const std::string& ctrlId) { /*unused*/ }
   virtual void onHtmlSendValue(cogx::display::CHtmlChunk *pChunk,
         const std::string& ctrlId, const std::string& value) { /*unused*/ }

signals:
   // Transfer onOwnerDataChanged to Qt UI thread; connected to QCastViewHtml
   // see <url:../QCastViewHtml.cpp#tn=::createJsObjects>
   // see <url:../QCastViewHtml.cpp#tn=::doFillHtmlFrom>
   // Operation:
   //    1. this.onForm_OwnerDataChanged(pForm); current data is stored with the form
   //    2. this.emit signalOwnerDataChanged(formid)
   //    3. QCastViewHtml::doFillHtmlFrom (connected to signalOwnerDataChanged, queued)
   //    4. QCastViewHtml::exec_JS: CogxJsFillForm(formid)
   //    5. this.getValues(formid)
   void signalOwnerDataChanged(const QString& formid);
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
