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

#include "CRasterImage.hpp"

using namespace std;

namespace cogx { namespace display {

std::auto_ptr<CRenderer> CRasterImage::render2D(new CRasterImage_Render2D());

CRasterImage::CRasterImage()
{
   m_pImage = NULL;
}

CRasterImage::~CRasterImage()
{
   if (m_pImage != NULL) delete m_pImage;
   m_pImage = NULL;
}

bool CRasterImage::isBitmap()
{
   return true;
}
   
CRenderer* CRasterImage::getRenderer(ERenderContext context)
{
   switch(context) {
      case Context2D: return render2D.get();
   }
   return NULL;
}

void CRasterImage_Render2D::draw(CDisplayObject *pObject, void *pContext)
{
   if (pObject == NULL || pContext == NULL) return;
   CRasterImage *pImage = (CRasterImage*) pObject;
   QPainter *pPainter = (QPainter*) pContext;
   if (pImage->m_pImage) {
      pPainter->drawImage(0, 0, *(pImage->m_pImage));
   }
}

}} // namespace
