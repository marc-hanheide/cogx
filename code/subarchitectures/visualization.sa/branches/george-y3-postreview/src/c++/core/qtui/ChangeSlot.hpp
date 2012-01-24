/*
 * Author: Marko Mahnič
 * Created: 2010-04-15
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

#ifndef CHANGESLOT_SPOFBGTO
#define CHANGESLOT_SPOFBGTO

#include <QObject>
#include "../GuiElements.hpp"
#include "../Model.hpp"
#include "../convenience.hpp"

// An instance of CChangeSlot is created for each dynamic Qt widget.  When a
// signal is trigerred by the widget, it is passed to one of the slots below
// and the slot notifies the CGuiElement about the data change. CGuiElement
// dispatches the notification to all its observers.
// @see <url:../GuiElements.hpp#tn=CGuiElement>
//
// A CGuiElement can be displayed in multiple views, with a different instance
// of a Qt widget in each. All the views are observers of the CGuiElement, but
// the source of the data change should not be notified about the change.
// 
// Each instance MUST be owned by the matching dynamic Qt widget, otherwise
// the updates from the remote owner won't work.
// See <url:QCustomGuiPanel.cpp#tn=::doUiDataChanged>, findChildren.
class CChangeSlot: public QObject,
   public cogx::display::CGuiElementObserver   
{
   Q_OBJECT
private:
   cogx::display::CDisplayView *m_pView;
   cogx::display::CGuiElement *m_pGuiElement;

public:
   CChangeSlot(cogx::display::CGuiElement* pGuiElement, cogx::display::CDisplayView* pView, QObject* parent)
      : QObject(parent)
   {
      m_pGuiElement = pGuiElement;
      m_pView = pView;
 
      // The view needs to be notified about data changes related to pGuiElement
      // A button has no data, so it doesn't need to be notified.
      if (m_pGuiElement) {
        switch(m_pGuiElement->m_type) {
           case cogx::display::CGuiElement::wtButton:
              break;
            case cogx::display::CGuiElement::wtAction:
              if (!m_pGuiElement->m_bCheckable)
                 break;
              // else: fall-through
            default:
              DMESSAGE("CChangeSlot: Add obsevers for " << pGuiElement->m_id);
              if (m_pView) m_pGuiElement->Observers += m_pView;
              m_pGuiElement->Observers += this;
              break;
        }
      }
      m_sGuiObserverName = std::string("CChangeSlot(") + pGuiElement->m_id + ")";
   }
   ~CChangeSlot() {
      DMESSAGE("~Destroying CChangeSlot");
      if (m_pGuiElement) {
         if (m_pView) m_pGuiElement->Observers -= m_pView;
         m_pGuiElement->Observers -= this;
      }
   }
   cogx::display::CGuiElement* guiElementPtr() {
      return m_pGuiElement;
   }

public:
   // CGuiElementObserver
   void onGuiElement_CtrlDataChanged(cogx::display::CGuiElement *pElement, const std::string& newValue);

public slots:
   void onCheckBoxChange(int value);
   void onButtonClick(bool checked);
};

#endif /* end of include guard: CHANGESLOT_SPOFBGTO */
// vim:sw=3:ts=8:et
