/*
 * Author: Marko Mahnič
 * Created: 2010-04-15
 */

#ifndef CHANGESLOT_SPOFBGTO
#define CHANGESLOT_SPOFBGTO

#include <QObject>
#include "../GuiElements.hpp"

// An instance of CChangeSlot is created for each dynamic Qt widget.  When a
// signal is trigerred by the widget, it is passed to one of the slots below
// and the slot forwards the event (using Ice) to the component with the id set
// in m_pGuiElement->m_dataOwner.
// 
// Each instance should be owned by the matching dynamic Qt widget.
class CChangeSlot: public QObject
{
   Q_OBJECT
public:
   cogx::display::CGuiElement *m_pGuiElement;
   CChangeSlot(cogx::display::CGuiElement* pGuiElement, QObject* parent)
      : QObject(parent)
   {
      m_pGuiElement  = pGuiElement;
   }

public slots:
   void onCheckBoxChange(int value);
   void onButtonClick(bool checked);
};

#endif /* end of include guard: CHANGESLOT_SPOFBGTO */
