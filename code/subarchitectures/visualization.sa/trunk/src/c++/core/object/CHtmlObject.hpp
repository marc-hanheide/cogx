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
#ifndef CHTMLOBJECT_FJCZERMX
#define CHTMLOBJECT_FJCZERMX

#include "../Model.hpp"
#include "../HtmlElements.hpp"

namespace cogx { namespace display {

class CHtmlObject: public CDisplayObject
{
   friend class CHtmlObject_RenderHtml;
   static std::auto_ptr<CRenderer> renderHtml;

private:
   std::map<std::string, CHtmlChunkPtr> m_HeadParts;
   std::map<std::string, CHtmlChunkPtr> m_Parts;

public:
   CHtmlObject();
   ~CHtmlObject();
   bool isBitmap(); /*override*/
   virtual ERenderContext getPreferredContext(); /*override*/
   virtual CRenderer* getRenderer(ERenderContext context); /*override*/
   virtual int getHtmlChunks(std::vector<CHtmlChunkPtr>& chunks, int typeMask); /*override*/
   void setHtml(const std::string& partId, const std::string& text);
   CHtmlChunkPtr setActiveHtml(const Ice::Identity& ident, const std::string& partId, const std::string& text);
   CHtmlChunkPtr setForm(const Ice::Identity& ident, const std::string& partId, const std::string& text);
   void setHead(const std::string& partId, const std::string& text);
   CHtmlChunkPtr getPart(const std::string& partId);
   virtual bool removePart(const std::string& partId); /*override*/
   void getParts(std::vector<CDisplayObjectPartPtr>& parts, bool bOrdered=false); /*override*/
};

class CHtmlObject_RenderHtml: public CRenderer
{
public:
   virtual void draw(CDisplayView *pView, CDisplayObject *pObject, void *pContext); /*override*/
   virtual void draw(CDisplayView *pView, const std::string& info, CDisplayObject *pObject, void *pContext); /*override*/
};

}} // namespace
#endif /* end of include guard: CHTMLOBJECT_FJCZERMX */
// vim:sw=3:ts=8:et
