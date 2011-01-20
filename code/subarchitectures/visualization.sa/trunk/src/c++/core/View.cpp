/*
 * Author: Marko Mahnič
 * Created: 2010-03-11
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
#include "Model.hpp"
#include "HtmlElements.hpp"
#include <QList>
#include <QGraphicsItem>
#include <QGraphicsItemGroup>

#ifdef DEBUG_TRACE
// #undef DEBUG_TRACE
#endif
#include "convenience.hpp"

using namespace std;

namespace cogx { namespace display {

CDisplayView::CDisplayView()
{
   m_preferredContext = ContextGraphics;
   m_bDefaultView = false;
}

CDisplayView::~CDisplayView()
{
   m_Objects.clear(); // do not delete, owned by the model
}

bool CDisplayView::hasObject(const std::string &id)
{
   return m_Objects.count(id) ? true : false;
}

bool CDisplayView::waitsForObject(const std::string &id)
{
   return m_SubscribedObjects.count(id) ? true : false;
}

void CDisplayView::addObject(CDisplayObject *pObject)
{
   if (! pObject) return;
   DMESSAGE(m_id << ": Adding object: " << pObject->m_id);

   TObjectMap::iterator existing = m_Objects.find(pObject->m_id);
   if (existing != m_Objects.end()) {
      DMESSAGE(m_id << ": *** REPLACING an existing object ***");
      m_Objects.erase(existing);
   }

   m_Objects[pObject->m_id] = pObject;

   // Add object id to object-order, if necessary
   typeof(m_ObjectOrder.begin()) itorder;
   for (itorder = m_ObjectOrder.begin(); itorder != m_ObjectOrder.end(); itorder++) {
      if (*itorder == pObject->m_id) break;
   }
   if (itorder == m_ObjectOrder.end())
      m_ObjectOrder.push_back(pObject->m_id);

   CDisplayModelObserver *pobsrvr;
   CObserverList<CDisplayModelObserver>::ReadLock lock(viewObservers);
   FOR_EACH(pobsrvr, viewObservers) {
      pobsrvr->onViewChanged(NULL, this);
   }

}

void CDisplayView::setSubscription(const std::string& id, bool active)
{
   if (active) {
      m_SubscribedObjects[id] = true;
      typeof(m_ObjectOrder.begin()) itorder;
      for (itorder = m_ObjectOrder.begin(); itorder != m_ObjectOrder.end(); itorder++) {
         if (*itorder == id) break;
      }
      if (itorder == m_ObjectOrder.end())
         m_ObjectOrder.push_back(id);
   }
   else {
      std::map<std::string, bool>::iterator waiting = m_SubscribedObjects.find(id);
      if (waiting != m_SubscribedObjects.end())
         m_SubscribedObjects.erase(waiting);
   }
}

void CDisplayView::removeObject(const std::string& id)
{
   TObjectMap::iterator existing = m_Objects.find(id);
   if (existing != m_Objects.end()) {
      DMESSAGE(m_id << ": Removing  object: " << id);
      m_Objects.erase(existing);

      CDisplayModelObserver *pobsrvr;
      CObserverList<CDisplayModelObserver>::ReadLock lock(viewObservers);
      FOR_EACH(pobsrvr, viewObservers) {
         pobsrvr->onViewChanged(NULL, this);
      }
   }

   std::map<std::string, bool>::iterator waiting = m_SubscribedObjects.find(id);
   if (waiting != m_SubscribedObjects.end())
      m_SubscribedObjects.erase(waiting);

   // XXX keep the object id in m_ObjectOrder (for now). What is better? To keep or to remove?
   // If the order is removed, then if the object is readded, it will be ordered differently.
}

void CDisplayView::removeAllObjects()
{
   m_Objects.clear();
   m_SubscribedObjects.clear();

   CDisplayModelObserver *pobsrvr;
   CObserverList<CDisplayModelObserver>::ReadLock lock(viewObservers);
   FOR_EACH(pobsrvr, viewObservers) {
      pobsrvr->onViewChanged(NULL, this);
   }
}

void CDisplayView::replaceObject(const std::string& id, CDisplayObject *pNew)
{
   DTRACE("CDisplayObject::replaceObject " << id);
   bool canAdd = false;
   TObjectMap::iterator existing = m_Objects.find(id);
   if (existing != m_Objects.end()) {
      canAdd = true;
      m_Objects.erase(existing);
   }
   if (!pNew) {
      if (canAdd) {
         // Special case: the object is not removed, it is replaced with NULL.
         // NULLs are not kept in TObjectMap, but the view still has to know
         // about the object, so we register it's id in subscriptions.
         m_SubscribedObjects[id] = true;
      }
      return;
   }

   if (!canAdd) {
      std::map<std::string, bool>::iterator waiting = m_SubscribedObjects.find(pNew->m_id);
      if (waiting != m_SubscribedObjects.end()) {
         canAdd = true;
      }
   }

   if (canAdd) {
      DMESSAGE(m_id << ": replace --> add: " << pNew->m_id);
      addObject(pNew);
   }
   else {
      DMESSAGE(m_id << ": *** NO OBJECT TO REPLACE: " << pNew->m_id);
   }
}

void CDisplayView::refreshObject(const std::string& id)
{
   TObjectMap::iterator existing = m_Objects.find(id);
   if (existing == m_Objects.end()) return;

   CDisplayModelObserver *pobsrvr;
   CObserverList<CDisplayModelObserver>::ReadLock lock(viewObservers);
   FOR_EACH(pobsrvr, viewObservers) {
      pobsrvr->onViewChanged(NULL, this);
   }
}

void CDisplayView::onUiDataChanged(CGuiElement *pElement, const std::string& newValue)
{
   DTRACE("CDisplayView::onUiDataChanged");
   CDisplayModelObserver *pobsrvr;
   CObserverList<CDisplayModelObserver>::ReadLock lock(viewObservers);
   FOR_EACH(pobsrvr, viewObservers) {
      pobsrvr->onUiDataChanged(NULL, this, pElement, newValue);
   }
}

void CDisplayView::onOwnerDataChanged(CGuiElement *pElement, const std::string& newValue)
{
   DTRACE("CDisplayView::onOwnerDataChanged");
   CDisplayModelObserver *pobsrvr;
   CObserverList<CDisplayModelObserver>::ReadLock lock(viewObservers);
   FOR_EACH(pobsrvr, viewObservers) {
      pobsrvr->onUiDataChanged(NULL, NULL, pElement, newValue);
   }
}

void CDisplayView::getObjects(CPtrVector<CDisplayObject>& objects, bool bOrdered)
{
   CDisplayObject *pObject;
   if (bOrdered) {
      typeof(m_ObjectOrder.begin()) itorder;
      typeof(m_Objects.begin()) itobj;
      for (itorder = m_ObjectOrder.begin(); itorder != m_ObjectOrder.end(); itorder++) {
         itobj = m_Objects.find(*itorder);
         if (itobj == m_Objects.end()) continue;
         pObject = itobj->second;
         if (!pObject) continue;
         objects.push_back(pObject);
      }
   }
   else {
      FOR_EACH_V(pObject, m_Objects) {
         if (pObject) objects.push_back(pObject);
      }
   }
}

// The CDisplayObject should not draw itself, instead it should provide another
// object (Renderer) that will draw it. function: getRenderer(context).
// Implementation: all objects of the same class share the same (static)
// renderer for each type of context (2D, 3D, text, etc.).
// XXX - a single static renderer is not thread-safe!
// TODO: the context may provide additional display options.
// TODO: drawing order may need to be defined if the objects are not displayed side by side
void CDisplayView::draw2D(QPainter &painter)
{
   CDisplayObject *pObject;
   CRenderer *pRender;
   //FOR_EACH_V(pObject, m_Objects) {
   typeof(m_ObjectOrder.begin()) itorder;
   typeof(m_Objects.begin()) itobj;
   for (itorder = m_ObjectOrder.begin(); itorder != m_ObjectOrder.end(); itorder++) {
      itobj = m_Objects.find(*itorder);
      if (itobj == m_Objects.end()) continue;
      pObject = itobj->second;
      if (!pObject) continue;
      pRender = pObject->getRenderer(Context2D);
      if (pRender) {
         painter.save();
         if (m_Trafos.count(pObject->m_id)) {
            std::vector<double>& trmatrix = m_Trafos[pObject->m_id];
            QTransform trans;
            trans.setMatrix(
                 trmatrix[0], trmatrix[1], trmatrix[2],
                 trmatrix[3], trmatrix[4], trmatrix[5],
                 trmatrix[6], trmatrix[7], trmatrix[8]);
            painter.setWorldTransform(trans, true);
         }
         pRender->draw(pObject, &painter);
         painter.restore();
      }
   }
}

// TODO: drawScene should replace draw2D!
void CDisplayView::drawScene(QGraphicsScene &scene)
{
   DTRACE("CDisplayView::drawScene " << m_ObjectOrder.size());
   CDisplayObject *pObject;
   CRenderer *pRender;
   //FOR_EACH_V(pObject, m_Objects) {
   typeof(m_ObjectOrder.begin()) itorder;
   typeof(m_Objects.begin()) itobj;
   for (itorder = m_ObjectOrder.begin(); itorder != m_ObjectOrder.end(); itorder++) {
      DMESSAGE(*itorder);
      itobj = m_Objects.find(*itorder);
      if (itobj == m_Objects.end()) continue;
      pObject = itobj->second;
      if (!pObject) continue;
      pRender = pObject->getRenderer(ContextGraphics);
      if (pRender) {
         QGraphicsItemGroup* pGroup = scene.createItemGroup(QList<QGraphicsItem*>());
         if (m_Trafos.count(pObject->m_id)) {
            std::vector<double>& trmatrix = m_Trafos[pObject->m_id];
            QTransform trans;
            trans.setMatrix(
                 trmatrix[0], trmatrix[1], trmatrix[2],
                 trmatrix[3], trmatrix[4], trmatrix[5],
                 trmatrix[6], trmatrix[7], trmatrix[8]);
            pGroup->setTransform(trans);
         }
         pRender->draw(pObject, pGroup);
      }
   }
}

// TODO: dawing multiple objects in 3D
//    - all objects in a single scene (viewport = window)
//    - multiple scenes side by side (different viewports); how would zoom work here?
void CDisplayView::drawGL()
{
   CDisplayObject *pObject;
   CRenderer *pRender;
   //FOR_EACH_V(pObject, m_Objects) {
   typeof(m_ObjectOrder.begin()) itorder;
   typeof(m_Objects.begin()) itobj;
   for (itorder = m_ObjectOrder.begin(); itorder != m_ObjectOrder.end(); itorder++) {
      itobj = m_Objects.find(*itorder);
      if (itobj == m_Objects.end()) continue;
      pObject = itobj->second;
      if (!pObject) continue;
      pRender = pObject->getRenderer(ContextGL);
      if (pRender) {
         pRender->draw(pObject, NULL);
      }
   }
}

void CDisplayView::drawHtml(QStringList &head, QStringList &body)
{
   CDisplayObject *pObject;
   CRenderer *pRender;
   //FOR_EACH_V(pObject, m_Objects) {
   typeof(m_ObjectOrder.begin()) itorder;
   typeof(m_Objects.begin()) itobj;
   for (itorder = m_ObjectOrder.begin(); itorder != m_ObjectOrder.end(); itorder++) {
      itobj = m_Objects.find(*itorder);
      if (itobj == m_Objects.end()) continue;
      pObject = itobj->second;
      if (!pObject) continue;
      pRender = pObject->getRenderer(ContextHtml);
      if (pRender) {
         pRender->draw("head", pObject, &head);
         pRender->draw("body", pObject, &body);
      }
   }
}

int CDisplayView::getHtmlChunks(CPtrVector<CHtmlChunk>& forms, int typeMask)
{
   int count = 0;
   CDisplayObject *pObject;
   // FOR_EACH_V(pObject, objects) {
   typeof(m_ObjectOrder.begin()) itorder;
   typeof(m_Objects.begin()) itobj;
   for (itorder = m_ObjectOrder.begin(); itorder != m_ObjectOrder.end(); itorder++) {
     itobj = m_Objects.find(*itorder);
     if (itobj == m_Objects.end()) continue;
     pObject = itobj->second;
     if (!pObject) continue;
     count += pObject->getHtmlChunks(forms, typeMask);
   }
   return count;
}

}} // namespace
