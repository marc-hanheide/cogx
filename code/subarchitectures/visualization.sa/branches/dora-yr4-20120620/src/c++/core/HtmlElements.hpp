/*
 * Author: Marko Mahnič
 * Created: 2010-06-29
 *
 * © Copyright 2010 Marko Mahnič. 
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
#ifndef HTMLELEMENTS_Y03BCHFJ
#define HTMLELEMENTS_Y03BCHFJ

#include <string>
#include <Ice/Ice.h>
#include "observer.hpp"
#include "Model.hpp"

// Using QString to store HTML to reduce the amount of conversions from std::string.
#include <QtCore>

class QCastFormProxy;

namespace cogx { namespace display {

typedef std::map<std::string, std::string> TFormValues;
class CHtmlChunk;
class CHtmlFormObserver
{
public:
   // Pass the POST data to the owner component
   virtual void onFormSubmitted(CHtmlChunk *pForm, const TFormValues& newValues) = 0;

   // onClick(pChunk, id) for active chunks; XXX: CHtmlFormObserver --> rename to CHtmlChunkObserver.
   // CDisplayServer executes a button click.
   // @param ctrlId is the parameter passed to @@ONCLICK@@
   // (see <url:object/CHtmlObject.cpp#tn=CHtmlTransformer>)
   virtual void onHtmlClick(CHtmlChunk *pChunk, const std::string& ctrlId) = 0;
   virtual void onHtmlSendValue(CHtmlChunk *pChunk, const std::string& ctrlId, const std::string& value) = 0;

   // Pass the changes from the owner to the form.
   // This function shoul only be implemented in QCastFormObserver.
   // The CDisplayServer should not respond to this notification since it is the
   // CDisplayServer that causes the change.
   virtual void onForm_OwnerDataChanged(CHtmlChunk *pForm, const TFormValues& newValues) {}
};

class CHtmlChunk: public CDisplayObjectPart
{
public:
   enum EChunkType { head = 0x01, html = 0x02, activehtml = 0x04, form = 0x08 };
   enum EChunkEventType { onClick, onSendValue };
   static const std::string JavascriptObjectName;

private:
   EChunkType m_type;
   std::string m_parentId;     // Id of the owner CDisplayObject
   //std::string m_partId;
   std::string m_htmlId; // a valid hmtl id generated from type, m_id and m_partId

public:
   Ice::Identity m_dataOwner; // the ID of the component that receives data from a form
   QString m_html;
   TFormValues m_formData; // reused when the HTML document is reloaded

   CObserverList<CHtmlFormObserver> Observers;

public:
   static std::string makeHtmlId(EChunkType type, const std::string& objectId, const std::string& partId);

public:
   CHtmlChunk(const std::string& id, const std::string& partId, EChunkType type=html,
         const Ice::Identity& ident=Ice::Identity());
   void setContent(const std::string& content);
   EChunkType type() { return m_type; }
   const std::string& htmlid() { return m_htmlId; }
   const std::string& id() { return m_parentId; }
   const std::string& partId() { return m_id; }

   // (normally) called after a submit subscribed observes.
   void notifyFormSubmit(TFormValues& formData, const QCastFormProxy* changeSource);

   // some components may want to receive button clicks, etc.
   void notifyChunkEvent(EChunkEventType type, const std::string& sourceId, const std::string& value,
         const QCastFormProxy* changeSource);

   // Synchronize the control state after a change in the (remote) component that 
   // created the form (the owner). This function is called as a result of
   // a getFormData() query.
   // TODO: syncFormData can also be called by the owner after a change in its
   // internal state. This requires another function in the ICE interface:
   // DisplayInterface::setFormData.
   void syncFormData(const TFormValues& formData, bool notify=false);
};


}} // namespace
#endif /* end of include guard: HTMLELEMENTS_Y03BCHFJ */
// vim:sw=3:ts=8:et
