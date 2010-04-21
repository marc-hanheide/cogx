/*
 * Author: Marko Mahnič
 * Created: 2010-04-15
 */

#ifndef CHANGESLOT_SPOFBGTO
#define CHANGESLOT_SPOFBGTO

#include <QObject>
#include "../GuiElements.hpp"
#include "../Model.hpp"

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
// Each instance should be owned by the matching dynamic Qt widget.
class CChangeSlot: public QObject
{
   Q_OBJECT
private:
   cogx::display::CDisplayView *m_pView;
public:
   cogx::display::CGuiElement *m_pGuiElement;
   CChangeSlot(cogx::display::CGuiElement* pGuiElement, cogx::display::CDisplayView* pView, QObject* parent)
      : QObject(parent)
   {
      m_pGuiElement = pGuiElement;
      m_pView = pView;
 
      // The view needs to be notified about data changes related to pGuiElement
      // A button has no data, so it doesn't need to be notified.
      if (m_pGuiElement && m_pView && m_pGuiElement->m_type != cogx::display::CGuiElement::wtButton)
         m_pGuiElement->Observers += m_pView;
   }
   ~CChangeSlot() {
      if (m_pGuiElement && m_pView) m_pGuiElement->Observers -= m_pView;
   }

public slots:
   void onCheckBoxChange(int value);
   void onButtonClick(bool checked);
};

#endif /* end of include guard: CHANGESLOT_SPOFBGTO */
