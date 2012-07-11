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
}

CSvgImage::SPartPtr CSvgImage::findPart(const std::string& partId)
{
   CDisplayObject::ReadLock lock(*this);
   auto ipart = m_Parts.find(partId);
   if (ipart != m_Parts.end()) {
      return ipart->second;
   }
   return nullptr;
}

bool CSvgImage::setPart(const std::string& partId, const std::string& xmlData)
{
   SPartPtr pPart = findPart(partId);
   bool exists = pPart != nullptr;

   CDisplayObject::WriteLock lock(*this);
   if (! exists) {
      pPart = SPartPtr(new SPart(partId));
      m_Parts[partId] = pPart;
   }
   if (pPart) {
      pPart->setData(xmlData);
   }
   return !exists;
}

bool CSvgImage::removePart(const std::string& partId)
{
   SPartPtr ipart = findPart(partId);
   if (!ipart) {
      return false;
   }

   CDisplayObject::WriteLock lock(*this);
   m_Parts.erase(partId);
   return true;
}

void CSvgImage::getParts(std::vector<CDisplayObjectPartPtr>& objects, bool bOrdered)
{
   for (auto ipart : m_Parts) {
      objects.push_back(ipart.second);
   }
}

void CSvgImage::setTransform2D(const std::string& partId, const std::vector<double> &matrix)
{
   DTRACE("CSvgImage::setTransform2D");
   assert (matrix.size() == 9 || matrix.size() == 0);

   SPartPtr pPart = findPart(partId);
   CDisplayObject::WriteLock lock(*this);
   if (! pPart) {
      pPart = SPartPtr (new SPart(partId));
      m_Parts[partId] = pPart;
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

   CDisplayObject::ReadLock lock(*pObject);
   for (auto ipart : pImage->m_Parts) {
      CSvgImage::SPartPtr& pPart = ipart.second;
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

   CDisplayObject::ReadLock lock(*pObject);
   for (auto ipart : pImage->m_Parts) {
      CSvgImage::SPartPtr& pPart = ipart.second;
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
      for (auto ipart : pImage->m_Parts) {
         CSvgImage::SPartPtr& pPart = ipart.second;
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
