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

struct _s_ 
{
   static void replace(std::string &str, const std::string &what,
         const std::string &newval, int maxreplace=0)
   {
      size_t findlen = what.length();
      size_t replen = newval.length();
      size_t pos = 0;
      int count = 0;
      while((pos = str.find(what, pos)) != std::string::npos) {
         str.replace(pos, findlen, newval);
         pos += replen;
         if (maxreplace > 0) {
            count++;
            if (count >= maxreplace) break;
         }
      }
   }
};

namespace cogx { namespace display {

struct CHtmlTransformer
{
   static std::string stripchars(const std::string& str, const std::string& chars)
   {
      size_t ib = str.find_first_not_of(chars);
      if (ib == std::string::npos) return "";
      size_t ie = str.find_last_not_of(chars);
      return str.substr(ib, ie-ib+1);
   }
   static std::string escape(const std::string& str, const std::string& chars)
   {
      std::ostringstream ss;
      char esc = chars[0];
      size_t pos = 0;
      size_t len = str.size();
      while(pos < len) {
         char ch = str[pos];
         if (chars.find(ch) == std::string::npos) ss << ch;
         else ss << esc << ch;
         pos++;
      }
      return ss.str();
   }
   static void skipspace(size_t& pos, size_t len, const std::string& text)
   {
      while(pos < len && (text[pos] == ' ' || text[pos] == '\t')) pos++;
   }
   static void skipto(size_t& pos, size_t len, const std::string& text, char end, const std::string& bad)
   {
      while(pos < len) {
         if (text[pos] == end) return;
         if (bad.find(text[pos]) != std::string::npos) {
            pos = std::string::npos;
            return;
         }
         pos++;
      }
   }

   void transform(const std::string& htmlId, const std::string& text, std::ostream& ss)
   {
      std::string jsOnClick = std::string("onclick=\"")
         + CHtmlChunk::JavascriptObjectName + ".onClick('#"
         + escape(htmlId, "\\'\"") + "', '";

      const char* data = text.data();
      size_t pos = 0;
      size_t len = text.size();
      size_t posTag = text.find("@@ONCLICK@@");
      while (posTag != std::string::npos) {
         size_t start = posTag;
         ss.write(data + pos, posTag - pos);
         pos = posTag + 11;
         // extract param
         skipspace(pos, len, text);
         if (text[pos] != '(') posTag = std::string::npos;
         else {
            pos++;
            posTag = pos;
            skipto(posTag, len, text, ')', "\n\r");
         }
         if (posTag == std::string::npos) {
            pos = start + 2;
            posTag = text.find("@@ONCLICK@@", pos);
            continue;
         }
         else {
            std::string ctrlid(data + pos, posTag - pos);
            ctrlid = stripchars(ctrlid, " \'\"");
            ctrlid = escape(ctrlid, "\\'\"");
            ss << jsOnClick << ctrlid << "');\"";
            pos = posTag + 1;
         }
         posTag = text.find("@@ONCLICK@@", pos);
      }
      if (pos < text.size()) {
         ss.write(data + pos, text.size() - pos);
      }
   }
};

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
   //if (m_Parts.find(partId)->second != NULL) {
   typeof(m_Parts.begin()) itExtng = m_Parts.find(partId);
   if (itExtng != m_Parts.end()) {
      pPart = m_Parts[partId];
      // TODO: if type != html, delete
   }

   CDisplayObject::WriteLock lock(*this);
   if (pPart == NULL) {
      pPart = new CHtmlChunk(m_id, partId, CHtmlChunk::html);
      m_Parts[partId] = pPart;
   }

   if (pPart) {
      pPart->setContent(text);
   }
}

CHtmlChunk* CHtmlObject::setActiveHtml(const Ice::Identity& ident, const std::string& partId,
      const std::string& text)
{
   CHtmlChunk* pPart = NULL;
   // if (m_Parts.find(partId)->second != NULL) {
   typeof(m_Parts.begin()) itExtng = m_Parts.find(partId);
   if (itExtng != m_Parts.end()) {
      pPart = m_Parts[partId];
      // pPart->m_dataOwner = ident; // just in case it changed
      // TODO: if type != activehtml, delete
   }

   CDisplayObject::WriteLock lock(*this);
   if (pPart == NULL) {
      pPart = new CHtmlChunk(m_id, partId, CHtmlChunk::activehtml, ident);
      m_Parts[partId] = pPart;
   }

   if (pPart) {
      CHtmlTransformer trans;
      std::ostringstream ss;
      trans.transform(pPart->htmlid(), text, ss);
      pPart->setContent(ss.str());
   }

   return pPart;
}

CHtmlChunk* CHtmlObject::setForm(const Ice::Identity& ident, const std::string& partId, const std::string& text)
{
   CHtmlChunk* pPart = NULL;
   // if (m_Parts.find(partId)->second != NULL) {
   typeof(m_Parts.begin()) itExtng = m_Parts.find(partId);
   if (itExtng != m_Parts.end()) {
      pPart = m_Parts[partId];
      // pPart->m_dataOwner = ident; // just in case it changed
      // TODO: if type != form, delete
   }

   CDisplayObject::WriteLock lock(*this);
   if (pPart == NULL) {
      pPart = new CHtmlChunk(m_id, partId, CHtmlChunk::form, ident);
      m_Parts[partId] = pPart;
   }

   if (pPart) {
      std::string formtag;
      // see: <url:../qtui/html/formcap.js>
      formtag += "<form id=\"" + pPart->htmlid() + "\" method=\"post\" action=\"javascript:history.go(-1)\" ";
      formtag += "onsubmit=\"return CogxJsSubmit('#" + pPart->htmlid() + "')\" >\n";

      // style set in <url:../qtui/QCastViewHtml.cpp#tn=::doupdatecontent>
      formtag += "<div class=\"v11nformbar\"><table><tr><td class=\"v11nformtitle\">"
         + pPart->partId();
      //formtag += "<td><input type=\"submit\" value=\"Apply\" /></td>";
      formtag += "<td>Registry:</td>";
      formtag += "<td><span class='v11nformbutton' onclick=\"CogxJsSave('#"
         + pPart->htmlid() + "')\" title='Save form data to registry'>Save</span></td>\n";
      formtag += "<td><span class='v11nformbutton' onclick=\"CogxJsLoad('#"
         + pPart->htmlid() + "')\" title='Load form data from registry'>Load</span></td>\n";
      formtag += "<td width='32px'>&nbsp;</td></tr></table></div>";

      CHtmlTransformer trans;
      std::ostringstream ss;
      trans.transform(pPart->htmlid(), text, ss);
 
      // XXX: this was a quick fix; move to transform
      std::string s = ss.str();
      _s_::replace(s, "@@FORMID@@", "#" + CHtmlTransformer::escape(pPart->htmlid(), "\\'\""));

      pPart->setContent(formtag + s + "\n</form>");
   }

   return pPart;
}

void CHtmlObject::setHead(const std::string& partId, const std::string& text)
{
   CHtmlChunk* pPart = NULL;
   //if (m_HeadParts.find(partId)->second != NULL) {
   typeof(m_HeadParts.begin()) itExtng = m_HeadParts.find(partId);
   if (itExtng != m_HeadParts.end()) {
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

int CHtmlObject::getHtmlChunks(CPtrVector<CHtmlChunk>& chunks, int typeMask)
{
   CDisplayObject::ReadLock lock(*this);
   typeof(m_Parts.begin()) itmap;
   int count = 0;
   for(itmap = m_Parts.begin(); itmap != m_Parts.end(); itmap++) {
      CHtmlChunk *pChunk = itmap->second;
      if (pChunk->type() & typeMask) {
         chunks.push_back(pChunk);
         count++;
      }
   }
   return count;
}

CHtmlChunk* CHtmlObject::getPart(const std::string& partId)
{
   CDisplayObject::ReadLock lock(*this);
   typeof(m_Parts.begin()) itpart = m_Parts.find(partId);
   if (itpart != m_Parts.end()) return itpart->second;

   itpart = m_HeadParts.find(partId);
   if (itpart != m_HeadParts.end()) return itpart->second;

   return NULL;
}

bool CHtmlObject::removePart(const std::string& partId, CPtrVector<CDisplayObjectPart>& parts)
{
   typeof(m_Parts.begin()) itpart = m_Parts.find(partId);
   bool removed = false;
   //if (itpart->second != NULL) {
   if (itpart != m_Parts.end()) {
      CHtmlChunk* pPart = itpart->second;
      CDisplayObject::WriteLock lock(*this);
      m_Parts.erase(itpart);
      parts.push_back(pPart);
      removed = true;
   }

   typeof(m_HeadParts.begin()) ithd = m_HeadParts.find(partId);
   //if (ithd->second != NULL) {
   if (ithd != m_HeadParts.end()) {
      CHtmlChunk* pPart = ithd->second; // m_HeadParts[partId];
      // XXX: This is where a segfault occurs occasionally
      //    setHtml() -> (when data is empty) -> removePart()
      CDisplayObject::WriteLock lock(*this);
      m_HeadParts.erase(ithd);
      parts.push_back(pPart);
      removed = true;
   }
   return removed;
}

void CHtmlObject::getParts(CPtrVector<CDisplayObjectPart>& parts, bool bOrdered)
{
   CDisplayObject::ReadLock lock(*this);
   CHtmlObject *pHtml;
   typeof(m_HeadParts.begin()) itmap;
   for(itmap = m_HeadParts.begin(); itmap != m_HeadParts.end(); itmap++) {
      CHtmlChunk *pChunk = itmap->second;
      if (pChunk) parts.push_back(pChunk);
   }

   for(itmap = m_Parts.begin(); itmap != m_Parts.end(); itmap++) {
      CHtmlChunk *pChunk = itmap->second;
      if (pChunk) parts.push_back(pChunk);
   }
}

void CHtmlObject_RenderHtml::draw(CDisplayView *pView, CDisplayObject *pObject, void *pContext)
{
   if (pObject && pContext)
      draw(pView, "body", pObject, pContext);
}

void CHtmlObject_RenderHtml::draw(CDisplayView *pView, const std::string& info, CDisplayObject *pObject,
      void *pContext)
{
   if (! pObject || !pContext) return;
   CHtmlObject *pHtml = dynamic_cast<CHtmlObject*>(pObject);
   if (! pHtml) return;
   CViewedObjectState *pState = pView->getObjectState(pHtml->m_id);

   QStringList *pList = (QStringList*) pContext;

   if (info == "head") {
      CDisplayObject::ReadLock lock(*pObject);
      typeof(pHtml->m_HeadParts.begin()) itmap;
      for(itmap = pHtml->m_HeadParts.begin(); itmap != pHtml->m_HeadParts.end(); itmap++) {
         CHtmlChunk *pChunk = itmap->second;
         if (!pChunk || !pState->m_childState[pChunk->m_id].m_bVisible) continue;
         pList->append(pChunk->m_html);
      }
   }

   if (info == "body") {
      CDisplayObject::ReadLock lock(*pObject);
      typeof(pHtml->m_Parts.begin()) itmap;
      for(itmap = pHtml->m_Parts.begin(); itmap != pHtml->m_Parts.end(); itmap++) {
         CHtmlChunk *pChunk = itmap->second;
         if (!pChunk || !pState->m_childState[pChunk->m_id].m_bVisible) continue;
         pList->append(pChunk->m_html);
      }
   }
}

}} // namespace
// vim:sw=3:ts=8:et
