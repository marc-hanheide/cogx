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
   typeof(m_HeadParts.begin()) ithdr;
   for(ithdr = m_HeadParts.begin(); ithdr != m_HeadParts.end(); ithdr++) {
      if (ithdr->second) delete ithdr->second;
   }
   m_HeadParts.clear();

   typeof(m_Parts.begin()) itp;
   for(itp = m_Parts.begin(); itp != m_Parts.end(); itp++) {
      if (itp->second) delete itp->second;
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
   CHtmlChunk* pPart = NULL;
   if (m_Parts.find(partId)->second != NULL) {
      pPart = m_Parts[partId];
   }

   if (pPart == NULL) {
      pPart = new CHtmlChunk(m_id, partId, CHtmlChunk::html);
      m_Parts[partId] = pPart;
   }

   if (pPart) {
      pPart->setContent(text);
   }
}

CHtmlChunk* CHtmlObject::setForm(const Ice::Identity& ident, const std::string& partId, const std::string& text)
{
   CHtmlChunk* pPart = NULL;
   if (m_Parts.find(partId)->second != NULL) {
      pPart = m_Parts[partId];
   }

   if (pPart == NULL) {
      pPart = new CHtmlChunk(m_id, partId, CHtmlChunk::form, ident);
      m_Parts[partId] = pPart;
   }

   if (pPart) {
      std::string formtag;
      // see: <url:../qtui/html/formcap.js>
      formtag += "<form id=\"" + pPart->htmlid() + "\" method=\"post\" action=\"javascript:history.go(-1)\" ";
      formtag += "onsubmit=\"return CogxJsSubmit('#" + pPart->htmlid() + "')\" >\n";
      pPart->setContent(formtag + text + "\n</form>");
   }

   return pPart;
}

void CHtmlObject::setHead(const std::string& partId, const std::string& text)
{
   CHtmlChunk* pPart = NULL;
   if (m_HeadParts.find(partId)->second != NULL) {
      pPart = m_HeadParts[partId];
   }

   if (pPart == NULL) {
      pPart = new CHtmlChunk(m_id, partId, CHtmlChunk::head);
      m_HeadParts[partId] = pPart;
   }

   if (pPart) {
      pPart->m_html = QString::fromStdString(text);
   }
}

int CHtmlObject::getHtmlForms(CPtrVector<CHtmlChunk>& forms)
{
   typeof(m_Parts.begin()) itmap;
   int count = 0;
   for(itmap = m_Parts.begin(); itmap != m_Parts.end(); itmap++) {
      CHtmlChunk *pChunk = itmap->second;
      if (pChunk->type() == CHtmlChunk::form) {
         forms.push_back(pChunk);
         count++;
      }
   }
   return count;
}

void CHtmlObject::removePart(const std::string& partId)
{
   typeof(m_Parts.begin()) itpart = m_Parts.find(partId);
   if (itpart->second != NULL) {
      CHtmlChunk* pPart = itpart->second;
      m_Parts.erase(itpart);
      delete pPart;
   }

   typeof(m_HeadParts.begin()) ithd = m_HeadParts.find(partId);
   if (ithd->second != NULL) {
      CHtmlChunk* pPart = m_HeadParts[partId];
      m_HeadParts.erase(ithd);
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
      CHtmlChunk *pChunk = itmap->second;
      pList->append(pChunk->m_html);
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
         CHtmlChunk *pChunk = itmap->second;
         pList->append(pChunk->m_html);
      }
   }

   if (info == "body") {
      typeof(pHtml->m_Parts.begin()) itmap;
      for(itmap = pHtml->m_Parts.begin(); itmap != pHtml->m_Parts.end(); itmap++) {
         CHtmlChunk *pChunk = itmap->second;
         pList->append(pChunk->m_html);
      }
   }
}

}} // namespace
// vim:sw=3:ts=8:et
