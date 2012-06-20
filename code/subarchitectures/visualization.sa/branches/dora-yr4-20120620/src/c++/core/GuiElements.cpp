/*
 * @author:  Marko Mahnič
 * @created: 2010-06-08
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
   
#include "GuiElements.hpp"
#include "qtui/ChangeSlot.hpp"
#include "convenience.hpp"

namespace cogx { namespace display {

void CGuiElement::notifyDataChange(const std::string& newValue, CChangeSlot* changeSource)
{
   DTRACE("CGuiElement::notifyDataChange");
   CGuiElementObserver *pObsrvr;
   CObserverList<CGuiElementObserver>::ReadLock lock(Observers); // XXX: the loop could be long for locking
   FOR_EACH(pObsrvr, Observers) {
      // The source of the data change should already be aware of the change
      // so we don't need to notify it.
      if (pObsrvr == changeSource) continue;
      DMESSAGE("Notifying (" << pObsrvr->m_sGuiObserverName << ") source=" << changeSource);
      pObsrvr->onGuiElement_CtrlDataChanged(this, newValue);
   }
}

// Synchronize the control state after a change in the (remote) component that 
// created the control (the owner). This function is called as a result of
// a getControlState() query.
// TODO: syncControlState can also be called by the owner after a change in its
// internal state. This requires another function in the ICE interface:
// DisplayInterface::setControlState.
void CGuiElement::syncControlState(const std::string& newValue, bool notify)
{
   DTRACE("CGuiElement::syncControlState");
   CGuiElementObserver *pObsrvr;
   CObserverList<CGuiElementObserver>::ReadLock lock(Observers); // XXX: the loop could be long for locking
   FOR_EACH(pObsrvr, Observers) {
      pObsrvr->onGuiElement_OwnerDataChanged(this, newValue);
   }
}

void CGuiDialog::notify_setValue(const std::string& name, const std::string& value)
{
   DTRACE("CGuiDialog::notify_setValue");
   CGuiDialogObserver *pObsrvr;
   CObserverList<CGuiDialogObserver>::ReadLock lock(Observers); // XXX: the loop could be long for locking
   FOR_EACH(pObsrvr, Observers) {
      pObsrvr-> onGuiDialog_setValue(this, name, value);
   }
}

void CGuiDialog::notify_call(const std::string& name, const std::string& value)
{
   DTRACE("CGuiDialog::notify_call");
   CGuiDialogObserver *pObsrvr;
   CObserverList<CGuiDialogObserver>::ReadLock lock(Observers); // XXX: the loop could be long for locking
   FOR_EACH(pObsrvr, Observers) {
      pObsrvr-> onGuiDialog_call(this, name, value);
   }
}

void CGuiDialog::notify_setHtml(const std::string& objectId, const std::string& partId, const std::string& value)
{
   DTRACE("CGuiDialog::notify_setHtml");
   CGuiDialogObserver *pObsrvr;
   CObserverList<CGuiDialogObserver>::ReadLock lock(Observers); // XXX: the loop could be long for locking
   FOR_EACH(pObsrvr, Observers) {
      pObsrvr-> onGuiDialog_setHtmlChunk(this, objectId, partId, value);
   }
}

void CGuiDialog::execute(const std::string& script)
{
   DTRACE("CGuiDialog::execute");
   if (m_pDialogView)
      m_pDialogView->execute(script);
}

}} // namespace
// vim:sw=3:ts=8:et
