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
#include <QGraphicsItemGroup>
#include <QGraphicsScene>
#include <QGraphicsSvgItem>

#ifdef DEBUG_TRACE
// #undef DEBUG_TRACE
#endif
#include "../convenience.hpp"

using namespace std;

namespace cogx { namespace display {

std::auto_ptr<CRenderer> CSvgImage::render2D(new CSvgImage_Render2D());
std::auto_ptr<CRenderer> CSvgImage::renderScene(new CSvgImage_RenderScene());
std::auto_ptr<CRenderer> CSvgImage::renderHtml(new CSvgImage_RenderHtml());

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
   CDisplayObject::ReadLock lock(*this);
   FOR_EACH(pPart, m_Parts) {
      if (pPart && pPart->m_id == partId) return pPart;
   }
   return nullptr;
}

bool CSvgImage::setPart(const std::string& partId, const std::string& xmlData)
{
   SPart* pPart = findPart(partId);
   bool exists = pPart != nullptr;
   CDisplayObject::WriteLock lock(*this);
   if (! exists) {
      pPart = new SPart(partId);
      m_Parts.push_back(pPart);
   }
   if (pPart) {
      pPart->setData(xmlData);
   }
   return !exists;
}

bool CSvgImage::removePart(const std::string& partId, CPtrVector<CDisplayObjectPart>& parts)
{
   bool removed = false;
   for (auto itpart = m_Parts.begin(); itpart != m_Parts.end(); itpart++) {
      SPart* pPart = *itpart;
      if (! pPart) continue;
      if (pPart->m_id == partId) {
         CDisplayObject::WriteLock lock(*this);
         m_Parts.erase(itpart);
         parts.push_back(pPart);
         removed = true;
         break;
      }
   }
   return removed;
}

void CSvgImage::getParts(CPtrVector<CDisplayObjectPart>& objects, bool bOrdered)
{
   // TODO: bOrdered
   for (auto itpart = m_Parts.begin(); itpart != m_Parts.end(); itpart++) {
      SPart* pPart = *itpart;
      if (pPart) objects.push_back(pPart);
   }
}

void CSvgImage::setTransform2D(const std::string& partId, const std::vector<double> &matrix)
{
   DTRACE("CSvgImage::setTransform2D");
   assert (matrix.size() == 9 || matrix.size() == 0);

   SPart* pPart = findPart(partId);
   CDisplayObject::WriteLock lock(*this);
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
      case ContextGraphics: return renderScene.get();
      case ContextHtml: return renderHtml.get();
   }
   return nullptr;
}

void CSvgImage_Render2D::draw(CDisplayView *pView, CDisplayObject *pObject, void *pContext)
{
   if (pObject == nullptr || pContext == nullptr) return;
   CSvgImage *pImage = (CSvgImage*) pObject;
   QPainter *pPainter = (QPainter*) pContext;
   CViewedObjectState *pState = pView->getObjectState(pImage->m_id);

   CSvgImage::SPart* pPart;
   CDisplayObject::ReadLock lock(*pObject);
   FOR_EACH(pPart, pImage->m_Parts) {
      if (! pPart) continue;
      if (! pState->m_childState[pPart->m_id].m_bVisible) continue;
      if (pPart->data.size() < 16) continue;

      pPainter->save();
      try {
         // QSvgRenderer doc(QByteArray::fromRawData(pPart->data.c_str(), pPart->data.length()), NULL);
         QSvgRenderer& doc = pPart->getSvgDoc();
         QRect rect = doc.viewBox();
         //DMESSAGE("SVG size " << size.width() << "x" << size.height());
         //DMESSAGE("SVG rect @" << rect.x() << "," << rect.y() << "; " << rect.width() << "x" << rect.height());
         if (pPart->trmatrix.size() == 9) {
            std::vector<double>& trmatrix = pPart->trmatrix;
            QTransform trans;
            trans.setMatrix(
                 trmatrix[0], trmatrix[1], trmatrix[2],
                 trmatrix[3], trmatrix[4], trmatrix[5],
                 trmatrix[6], trmatrix[7], trmatrix[8]);
            pPainter->setWorldTransform(trans, true);
         }
         doc.render(pPainter, QRectF(rect));
      }
      catch (...) {
      }
      pPainter->restore();
   }
}

void CSvgImage_RenderScene::draw(CDisplayView *pView, CDisplayObject *pObject, void *pContext)
{
   if (pObject == nullptr || pContext == nullptr) return;
   CSvgImage *pImage = (CSvgImage*) pObject;
   QGraphicsItemGroup *pGroup = (QGraphicsItemGroup*) pContext;
   QGraphicsScene *pScene = pGroup->scene();
   CViewedObjectState *pState = pView->getObjectState(pImage->m_id);

   CSvgImage::SPart* pPart;
   CDisplayObject::ReadLock lock(*pObject);
   FOR_EACH(pPart, pImage->m_Parts) {
      if (! pPart) continue;
      if (! pState->m_childState[pPart->m_id].m_bVisible) continue;
      if (pPart->data.size() < 16) continue;

      QGraphicsSvgItem* pSvgItem = new QGraphicsSvgItem(pGroup);
      pSvgItem->setFlags(QGraphicsItem::ItemClipsToShape);
      pSvgItem->setCacheMode(QGraphicsItem::NoCache);

      QTransform trans;

      QSvgRenderer* pRndr = &pPart->getSvgDoc();
      // TODO: translation of the origin to 0,0 should be optional in CDisplayObject/SPart
      // eg. setPartOption("name", "value");
      QRectF vb = pRndr->viewBox();
      trans.translate(vb.left(), vb.top());

      // XXX Unsafe when pPart is deleted !!! COULD CRASH.
      pSvgItem->setSharedRenderer(pRndr);

      if (pPart->trmatrix.size() == 9) {
         std::vector<double>& trmatrix = pPart->trmatrix;
         QTransform t2;
         t2.setMatrix(
               trmatrix[0], trmatrix[1], trmatrix[2],
               trmatrix[3], trmatrix[4], trmatrix[5],
               trmatrix[6], trmatrix[7], trmatrix[8]);
         trans = trans * t2;
      }
      pSvgItem->setTransform(trans, true);

      pGroup->addToGroup(pSvgItem);
   }
}

void CSvgImage_RenderHtml::draw(CDisplayView *pView, CDisplayObject *pObject, void *pContext)
{
   if (pObject && pContext)
      draw(pView, "body", pObject, pContext);
}

void CSvgImage_RenderHtml::draw(CDisplayView *pView, const std::string& info,
      CDisplayObject *pObject, void *pContext)
{
   if (pObject == nullptr || pContext == nullptr) return;
   CSvgImage *pImage = (CSvgImage*) pObject;
   CViewedObjectState *pState = pView->getObjectState(pImage->m_id);

   QStringList *pList = (QStringList*) pContext;

   if (info == "body") {
      CDisplayObject::ReadLock lock(*pObject);
      CSvgImage::SPart* pPart;
      FOR_EACH(pPart, pImage->m_Parts) {
         if (! pPart) continue;
         if (! pState->m_childState[pPart->m_id].m_bVisible) continue;
         if (pPart->data.size() < 16) continue;

         pList->append("<div class='svgpart' >");
         pList->append(QString::fromStdString(pPart->data));
         pList->append("</div>");
      }
   }
}

}} // namespace
