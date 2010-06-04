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
