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
#include <QCheckBox>
#include <QAction>

#include "../convenience.hpp"

void CChangeSlot::onCheckBoxChange(int value)
{
   DTRACE("CChangeSlot::onCheckBoxChange " << this);
   std::stringstream sval;
   sval << value;
   if (m_pGuiElement) m_pGuiElement->notifyDataChange(sval.str(), this);
}

void CChangeSlot::onButtonClick(bool checked)
{
   DTRACE("CChangeSlot::onButtonClick " << this);
   std::string sval = checked ? "1" : "0";
   if (m_pGuiElement) m_pGuiElement->notifyDataChange(sval, this);
}

void CChangeSlot::onGuiElement_CtrlDataChanged(cogx::display::CGuiElement *pElement, const std::string& newValue)
{
   DTRACE("CChangeSlot::onElementUiDataChanged " << this << " " << m_sGuiObserverName);
   if (m_pGuiElement != pElement) { DMESSAGE("WE HAVE A MIXUP"); }

   QCheckBox *pBox;
   switch (m_pGuiElement->m_type) {
      case cogx::display::CGuiElement::wtCheckBox:
         pBox = dynamic_cast<QCheckBox*>(parent());
         if (pBox) {
            pBox->blockSignals(true);
            if (newValue == "0") pBox->setCheckState(Qt::Unchecked);
            else pBox->setCheckState(Qt::Checked);
            pBox->blockSignals(false);
         }
         break;
      case cogx::display::CGuiElement::wtAction:
         DMESSAGE("wtAction");
         if (m_pGuiElement->m_bCheckable) {
            QAction* pAction = dynamic_cast<QAction*>(parent());
            if (pAction) {
               DMESSAGE("Setting a checkable action value");
               pAction->blockSignals(true);
               pAction->setChecked(newValue != "0");
               pAction->blockSignals(false);
            }
         }
      default: break;
   };
}
