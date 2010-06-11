/*
 * Author: Marko Mahnič
 * Created: 2010-06-10
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
#include "CHtmlObject.hpp"
#include <QStringList>

namespace cogx { namespace display {

std::auto_ptr<CRenderer> CHtmlObject::renderHtml(new CHtmlObject_RenderHtml());

CHtmlObject::CHtmlObject()
{
}

CHtmlObject::~CHtmlObject()
{
   typeof(m_Parts.begin()) itmap;
   for(itmap = m_Parts.begin(); itmap != m_Parts.end(); itmap++) {
      if (itmap->second) delete itmap->second;
   }
   m_Parts.clear();
}

bool CHtmlObject::isBitmap()
{
   return false;
}

ERenderContext CHtmlObject::getPreferredContext()
{
   return ContextHtml;
}

CRenderer* CHtmlObject::getRenderer(ERenderContext context)
{
   switch(context) {
      case ContextHtml: return renderHtml.get();
   }
   return NULL;
}

void CHtmlObject::setHtml(const std::string& partId, const std::string& text)
{
   QString* pPart = NULL;
   if (m_Parts.find(partId)->second != NULL) {
      pPart = m_Parts[partId];
   }

   if (pPart == NULL) {
      pPart = new QString();
      m_Parts[partId] = pPart;
   }

   if (pPart) {
      *pPart = QString::fromStdString(text);
   }
}

void CHtmlObject::setHead(const std::string& partId, const std::string& text)
{
   QString* pPart = NULL;
   if (m_HeadParts.find(partId)->second != NULL) {
      pPart = m_HeadParts[partId];
   }

   if (pPart == NULL) {
      pPart = new QString();
      m_HeadParts[partId] = pPart;
   }

   if (pPart) {
      *pPart = QString::fromStdString(text);
   }
}

void CHtmlObject::removePart(const std::string& partId)
{
   typeof(m_Parts.begin()) itmap = m_Parts.find(partId);
   if (itmap->second != NULL) {
      QString* pPart = m_Parts[partId];
      m_Parts.erase(itmap);
      delete pPart;
   }

   itmap = m_HeadParts.find(partId);
   if (itmap->second != NULL) {
      QString* pPart = m_HeadParts[partId];
      m_HeadParts.erase(itmap);
      delete pPart;
   }
}

void CHtmlObject_RenderHtml::draw(CDisplayObject *pObject, void *pContext)
{
   if (pObject == NULL) return;
   CHtmlObject *pHtml = dynamic_cast<CHtmlObject*>(pObject);
   if (pHtml->m_Parts.size() < 1) return;

   QStringList *pList = (QStringList*) pContext;

   typeof(pHtml->m_Parts.begin()) itmap;
   for(itmap = pHtml->m_Parts.begin(); itmap != pHtml->m_Parts.end(); itmap++) {
      if (itmap->second) 
         pList->append(*(itmap->second));
   }
}

void CHtmlObject_RenderHtml::draw(const std::string& info, CDisplayObject *pObject, void *pContext)
{
   if (pObject == NULL) return;
   CHtmlObject *pHtml = dynamic_cast<CHtmlObject*>(pObject);
   if (pHtml->m_Parts.size() < 1) return;

   QStringList *pList = (QStringList*) pContext;

   if (info == "head") {
      typeof(pHtml->m_HeadParts.begin()) itmap;
      for(itmap = pHtml->m_HeadParts.begin(); itmap != pHtml->m_HeadParts.end(); itmap++) {
         if (itmap->second) 
            pList->append(*(itmap->second));
      }
   }

   if (info == "body") {
      typeof(pHtml->m_Parts.begin()) itmap;
      for(itmap = pHtml->m_Parts.begin(); itmap != pHtml->m_Parts.end(); itmap++) {
         if (itmap->second) 
            pList->append(*(itmap->second));
      }
   }
}

}} // namespace
