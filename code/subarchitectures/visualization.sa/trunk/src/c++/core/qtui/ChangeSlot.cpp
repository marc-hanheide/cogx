/*
 * Author: Marko Mahnič
 * Created: 2010-04-15
 */

#include "ChangeSlot.hpp"

void CChangeSlot::onCheckBoxChange(int value)
{
   std::stringstream sval;
   sval << value;
   if (m_pGuiElement) m_pGuiElement->notifyDataChange(sval.str());
}

void CChangeSlot::onButtonClick(bool checked)
{
   std::string sval = checked ? "1" : "0";
   if (m_pGuiElement) m_pGuiElement->notifyDataChange(sval);
}
