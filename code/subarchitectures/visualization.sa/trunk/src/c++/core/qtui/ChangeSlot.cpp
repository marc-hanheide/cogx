/*
 * Author: Marko Mahnič
 * Created: 2010-04-15
 */

#include "ChangeSlot.hpp"
#include "../convenience.hpp"

void CChangeSlot::onCheckBoxChange(int value)
{
   DTRACE("CChangeSlot::onCheckBoxChange");
   std::stringstream sval;
   sval << value;
   if (m_pGuiElement) m_pGuiElement->notifyDataChange(sval.str(), m_pView);
}

void CChangeSlot::onButtonClick(bool checked)
{
   DTRACE("CChangeSlot::onButtonClick");
   std::string sval = checked ? "1" : "0";
   if (m_pGuiElement) m_pGuiElement->notifyDataChange(sval, m_pView);
}
