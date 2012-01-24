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
#include "HtmlElements.hpp"
#include "qtui/html/formcap.hpp"

#ifdef DEBUG_TRACE
#undef DEBUG_TRACE
#endif
#include "convenience.hpp"

namespace cogx { namespace display {

const std::string CHtmlChunk::JavascriptObjectName = "CastQFormProxy";

std::string CHtmlChunk::makeHtmlId(EChunkType type, const std::string& objectId, const std::string& partId)
{
   std::string htmlId;
   if (type == form) htmlId = "form";
   else htmlId = "chunk";

   std::string str = "_0" + objectId + "_1" + partId;
   int len = str.size();
   char prevch = ' ';
   for(int i = 0; i < len; i++) {
      char ch = str[i];
      if (isalnum(ch)) htmlId += ch;
      else {
         ch = '_';
         if (prevch != ch) htmlId += ch;
      }
      prevch = ch;
   }
   return htmlId;
}

CHtmlChunk::CHtmlChunk(const std::string& id, const std::string& partId, EChunkType type,
      const Ice::Identity& ident)
{
   m_parentId = id;
   m_id = partId;
   m_type = type;
   m_dataOwner = ident;
   m_htmlId = makeHtmlId(m_type, id, partId);
}

void CHtmlChunk::setContent(const std::string& content)
{
   m_html = QString::fromStdString(content);
}

void CHtmlChunk::notifyFormSubmit(TFormValues& formData, const QCastFormProxy* changeSource)
{
   DTRACE("CHtmlChunk::notifyFormSubmit");
   m_formData = formData;

   CHtmlFormObserver *pObsrvr;
   CObserverList<CHtmlFormObserver>::ReadLock lock(Observers); // XXX: the loop could be long for locking
   FOR_EACH(pObsrvr, Observers) {
      // The source of the data change should already be aware of the change
      // so we don't need to notify it.
      if (pObsrvr == (CHtmlFormObserver*)changeSource) continue;
      //DMESSAGE("Notifying " << pObsrvr << " source=" << changeSource);
      pObsrvr->onFormSubmitted(this, formData);
   }
}

void CHtmlChunk::notifyChunkEvent(EChunkEventType type, const std::string& sourceId,
      const std::string& value, const QCastFormProxy* changeSource)
{
   CHtmlFormObserver *pObsrvr;
   CObserverList<CHtmlFormObserver>::ReadLock lock(Observers); // XXX: the loop could be long for locking
   FOR_EACH(pObsrvr, Observers) {
      switch (type) {
         case onClick: pObsrvr->onHtmlClick(this, sourceId); break;
         case onSendValue: pObsrvr->onHtmlSendValue(this, sourceId, value); break;
         default: break;
      }
   }
}

void CHtmlChunk::syncFormData(const TFormValues& formData, bool notify)
{
   DTRACE("CHtmlChunk::syncFormData");
   m_formData = formData;

   CHtmlFormObserver *pObsrvr;
   CObserverList<CHtmlFormObserver>::ReadLock lock(Observers); // XXX: the loop could be long for locking
   FOR_EACH(pObsrvr, Observers) {
      pObsrvr->onForm_OwnerDataChanged(this, formData);
   }
}

}} // namespace
// vim:sw=3:ts=8:et
