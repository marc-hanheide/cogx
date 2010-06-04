/*
 * Author: Marko Mahnič
 * Created: 2010-04-16
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
#include "CSvgImage.hpp"

#ifdef DEBUG_TRACE
#undef DEBUG_TRACE
#endif
#include "../convenience.hpp"

using namespace std;

namespace cogx { namespace display {

std::auto_ptr<CRenderer> CSvgImage::render2D(new CSvgImage_Render2D());

CSvgImage::CSvgImage()
{
}

CSvgImage::~CSvgImage()
{
   SPart* pPart;
   FOR_EACH(pPart, m_Parts) {
      if (pPart) delete pPart;
   }
   m_Parts.erase(m_Parts.begin(), m_Parts.end());
}

CSvgImage::SPart* CSvgImage::findPart(const std::string& partId)
{
   SPart* pPart;
   FOR_EACH(pPart, m_Parts) {
      if (pPart && pPart->id == partId) return pPart;
   }
   return NULL;
}

void CSvgImage::setPart(const std::string& partId, const std::string& xmlData)
{
   SPart* pPart = findPart(partId);
   if (! pPart) {
      pPart = new SPart(partId);
      pPart->setIdentity();
      m_Parts.push_back(pPart);
   }
   if (pPart) {
      pPart->setData(xmlData);
   }
}

void CSvgImage::setTransform2D(const std::string& partId, const std::vector<double> &matrix)
{
   DTRACE("CSvgImage::setTransform2D");
   assert (matrix.size() == 9 || matrix.size() == 0);
   SPart* pPart = findPart(partId);
   if (! pPart) {
      pPart = new SPart(partId);
      m_Parts.push_back(pPart);
   }
   if (pPart) {
      if (matrix.size() != 9) pPart->setIdentity();
      else pPart->trmatrix = matrix;
   }
}

CRenderer* CSvgImage::getRenderer(ERenderContext context)
{
   switch(context) {
      case Context2D: return render2D.get();
   }
   return NULL;
}

void CSvgImage_Render2D::draw(CDisplayObject *pObject, void *pContext)
{
   if (pObject == NULL || pContext == NULL) return;
   CSvgImage *pImage = (CSvgImage*) pObject;
   QPainter *pPainter = (QPainter*) pContext;

   CSvgImage::SPart* pPart;
   FOR_EACH(pPart, pImage->m_Parts) {
      if (! pPart) continue;
      if (pPart->data.size() < 16) continue;

      pPainter->save();
      try {
         // QSvgRenderer doc(QByteArray::fromRawData(pPart->data.c_str(), pPart->data.length()), NULL);
         QSvgRenderer& doc = pPart->getSvgDoc();
         QSize size = doc.defaultSize();
         if (pPart->trmatrix.size() == 9) {
            std::vector<double>& trmatrix = pPart->trmatrix;
            QTransform trans;
            trans.setMatrix(
                 trmatrix[0], trmatrix[1], trmatrix[2],
                 trmatrix[3], trmatrix[4], trmatrix[5],
                 trmatrix[6], trmatrix[7], trmatrix[8]);
            pPainter->setWorldTransform(trans, true);
         }
         doc.render(pPainter, QRectF(0, 0, size.width(), size.height()));
      }
      catch (...) {
      }
      pPainter->restore();
   }
}

}} // namespace
