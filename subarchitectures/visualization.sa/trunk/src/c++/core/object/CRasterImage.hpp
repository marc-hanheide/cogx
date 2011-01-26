/*
 * Author: Marko Mahnič
 * Created: 2010-03-31
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
#ifndef CRASTERIMAGE_4RYC90L
#define CRASTERIMAGE_4RYC90L

#include "../Model.hpp"
#include <QImage>

namespace cogx { namespace display {

class CRasterImage: public CDisplayObject
{
   friend class CRasterImage_Render2D;
   static std::auto_ptr<CRenderer> render2D;
   friend class CRasterImage_RenderScene;
   static std::auto_ptr<CRenderer> renderScene;

public:
   QImage* m_pImage;

public:
   CRasterImage();
   ~CRasterImage();
   bool isBitmap(); /*override*/
   virtual CRenderer* getRenderer(ERenderContext context); /*override*/
   virtual ERenderContext getPreferredContext()
   {
      return ContextGraphics;
   }
   virtual bool removePart(const std::string& partId, CPtrVector<CDisplayObjectPart>& parts) /*override*/
   {
      return false;
   }
};

class CRasterImage_Render2D: public CRenderer
{
public:
   virtual void draw(CDisplayView *pView, CDisplayObject *pObject, void *pContext); /*override*/
};

class CRasterImage_RenderScene: public CRenderer
{
public:
   virtual void draw(CDisplayView *pView, CDisplayObject *pObject, void *pContext); /*override*/
};

}} // namespace
#endif /* end of include guard: CRASTERIMAGE_4RYC90L */
