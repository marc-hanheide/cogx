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
   virtual void onDataChanged(CGuiElement *pElement, const std::string& newValue) = 0;
};


class CGuiElement
{
public:
   enum TWidgetType { wtButton, wtCheckBox, wtDropList };
   TWidgetType m_type;
   std::string m_viewId;  // TODO: list of views (or objects?) that show the widget
   std::string m_id;
   std::string m_label;
   Ice::Identity m_dataOwner; // the ID of the component that holds data for the element
   // TODO: list of subscribed clients
   // TODO: wtDropList has a list of items

   CObserver<CGuiElementObserver> Observers;

   bool isSameElement(CGuiElement *pGuiElement) {
      if (! pGuiElement) return false;
      if (m_type != pGuiElement->m_type) return false;
      if (m_id != pGuiElement->m_id) return false;
      if (m_dataOwner != pGuiElement->m_dataOwner) return false;
      return true;
   }

   void notifyDataChange(const std::string& newValue) {
      CGuiElementObserver *pObsrvr;
      FOR_EACH(pObsrvr, Observers) {
         pObsrvr->onDataChanged(this, newValue);
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
