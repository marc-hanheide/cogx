/*
 * Author: Marko Mahnič
 * Created: 2010-04-12
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
#ifndef GUIELEMENTS_6I8U02AA
#define GUIELEMENTS_6I8U02AA

#include <string>
#include <Ice/Ice.h>
#include "observer.hpp"

class CChangeSlot;

namespace cogx { namespace display {

class CGuiElement;
class CGuiElementObserver
{
public:
   // Pass the data change caused by UI interaction to the observer
   virtual void onUiDataChanged(CGuiElement *pElement, const std::string& newValue) = 0;

   // Pass the changes from the owner to the GUI element
   // The CDisplayServer should not respond to this notification since it is the
   // CDisplayServer that causes the change.
   virtual void onOwnerDataChanged(CGuiElement *pElement, const std::string& newValue) {}
};

// An instance of this class describes a dynamically created UI widget which
// is displayed when an object associated with the widget is displayed in a
// view.
// When a view is activated, a list of widgets for the view is obtained from
// the model.
//
// Events:
// Instances of this class are created in CDisplayServer which is an observer
// for each instance. When a change happens in a widget, the associated
// CGuiElement instance is notified, which notifies all its observers including
// CDisplayServer which finally notifies the remote client(s).
class CGuiElement
{
public:
   enum TWidgetType { wtAction, wtButton, wtCheckBox, wtDropList };
   TWidgetType m_type;
   // TODO: m_viewId could be a list of views that should show the widget
   std::string m_viewId; 
   std::string m_id;
   std::string m_label;
   std::string m_iconLabel;
   std::string m_iconSvg;
   bool m_checkable;
   Ice::Identity m_dataOwner; // the ID of the component that holds data for the element
   // TODO: list of subscribed clients (CAST components); notify on change
   // TODO: wtDropList has a list of items

   CObserverList<CGuiElementObserver> Observers;

   bool isSameElement(CGuiElement *pGuiElement) {
      if (! pGuiElement) return false;
      if (m_type != pGuiElement->m_type) return false;
      if (m_id != pGuiElement->m_id) return false;
      if (m_dataOwner != pGuiElement->m_dataOwner) return false;
      return true;
   }

   // (normally) called after a change in GUI to notify subscribed observes.
   void notifyDataChange(const std::string& newValue, CChangeSlot* changeSource);

   // Synchronize the control state after a change in the (remote) component that 
   // created the control (the owner). This function is called as a result of
   // a getControlState() query.
   // TODO: syncControlState can also be called by the owner after a change in its
   // internal state. This requires another function in the ICE interface:
   // DisplayInterface::setControlState.
   void syncControlState(const std::string& newValue, bool notify=false);
};

// CGuiElementValue is used for ICE communication: instances are stored in
// a queue in hIceDisplayServer.
struct CGuiElementValue
{
   enum { get, set };
   CGuiElementValue(CGuiElement *pElement, const std::string& value, int direction=set) {
      this->pElement = pElement;
      this->value = value;
      this->mode = direction;
   }
   CGuiElement *pElement;
   std::string value;
   int mode; 
};

}} // namespace
#endif /* end of include guard: GUIELEMENTS_6I8U02AA */
// vim:sw=3:ts=8:et
