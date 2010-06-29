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

namespace cogx { namespace display {

CHtmlChunk::CHtmlChunk(const std::string& id, EChunkType type, const Ice::Identity& ident)
{
   m_id = id;
   m_type = type;
   m_dataOwner = ident;
}

void CHtmlChunk::setContent(const std::string& content)
{
   m_html = QString::fromStdString(content);
}

void CHtmlChunk::notifyFormSubmit(TFormValues& formData, const QCastFormProxy* changeSource)
{
   //DTRACE("CHtmlChunk::notifyFormSubmit");
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

void CHtmlChunk::syncFormData(TFormValues& formData, bool notify)
{
   //DTRACE("CHtmlChunk::syncFormData");
   CHtmlFormObserver *pObsrvr;
   CObserverList<CHtmlFormObserver>::ReadLock lock(Observers); // XXX: the loop could be long for locking
   FOR_EACH(pObsrvr, Observers) {
      pObsrvr->onOwnerDataChanged(this, formData);
   }
}

}} // namespace
// vim:sw=3:ts=8:et
