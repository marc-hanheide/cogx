/*
 * Author: Marko Mahnič
 * Created: 2010-04-12
 */
#ifndef GUIELEMENTS_6I8U02AA
#define GUIELEMENTS_6I8U02AA

#include <string>
#include <Ice/Ice.h>
#include "observer.hpp"

namespace cogx { namespace display {

class CGuiElement;
class CGuiElementObserver
{
public:
   virtual void onUiDataChanged(CGuiElement *pElement, const std::string& newValue) = 0;
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
// 
// TODO: ATM there is no way to retrieve the data for a widget from a (remote) component
class CGuiElement
{
public:
   enum TWidgetType { wtAction, wtButton, wtCheckBox, wtDropList };
   TWidgetType m_type;
   std::string m_viewId;  // TODO: list of views (or objects?) that show the widget
   std::string m_id;
   std::string m_label;
   std::string m_iconLabel;
   std::string m_iconSvg;
   bool m_checkable;
   Ice::Identity m_dataOwner; // the ID of the component that holds data for the element
   // TODO: list of subscribed clients (CAST components); notify on change
   // TODO: wtDropList has a list of items

   CObserver<CGuiElementObserver> Observers;

   bool isSameElement(CGuiElement *pGuiElement) {
      if (! pGuiElement) return false;
      if (m_type != pGuiElement->m_type) return false;
      if (m_id != pGuiElement->m_id) return false;
      if (m_dataOwner != pGuiElement->m_dataOwner) return false;
      return true;
   }

   void notifyDataChange(const std::string& newValue, void* changeSource) {
      CGuiElementObserver *pObsrvr;
      FOR_EACH(pObsrvr, Observers) {
         // The source of the data change should already be aware of the change
         // so we don't need to notify it.
         if (pObsrvr == changeSource) continue;
         pObsrvr->onUiDataChanged(this, newValue);
      }
   }
};

struct CGuiElementValue
{
   CGuiElementValue(CGuiElement *pElement, const std::string& value) {
      this->pElement = pElement;
      this->value = value;
   }
   CGuiElement *pElement;
   std::string value;
};

}} // namespace
#endif /* end of include guard: GUIELEMENTS_6I8U02AA */
