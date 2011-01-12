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

#ifdef DEBUG_TRACE
#undef DEBUG_TRACE
#endif
#include "convenience.hpp"

using namespace std;

namespace cogx { namespace display {

CDisplayModel::CDisplayModel()
{
}

CDisplayModel::~CDisplayModel()
{
   CDisplayView* pview;
   FOR_EACH_V(pview, m_Views) if (pview) delete pview;
   m_Views.clear();

   CDisplayObject* pobj;
   FOR_EACH_V(pobj, m_Objects) if (pobj) delete pobj;
   m_Objects.clear();

   CGuiElement* pgel;
   FOR_EACH(pgel, m_GuiElements) if (pgel) delete pgel;
   m_GuiElements.clear();
}

void CDisplayModel::createView(const std::string& id, ERenderContext context,
      const std::vector<std::string>& objects)
{
   CDisplayModelObserver *pobsrvr;
   TViewMap::iterator itview = m_Views.find(id);
   CDisplayView *pview = (itview == m_Views.end()) ? NULL : itview->second;
   
   if (! pview) {
      DMESSAGE("Creating new view: " << id << ": context " << context);
      pview = new cogx::display::CDisplayView();
      pview->m_id = id;
   }
   else {
      DMESSAGE("Replacing view: " << id << ": context " << context);
      CObserverList<CDisplayModelObserver>::ReadLock lock(modelObservers);
      m_Views.erase(itview);
      FOR_EACH(pobsrvr, modelObservers) {
         if (pobsrvr) pobsrvr->onViewRemoved(this, pview->m_id);
      }
      pview->removeAllObjects();
   }

   pview->m_bDefaultView = false;
   pview->m_preferredContext = context;

   std::vector<std::string>::const_iterator it;
   for (it = objects.begin(); it != objects.end(); it++) {
      CDisplayObject *pObj = getObject(*it);
      if (pObj) pview->addObject(pObj);
      else pview->setSubscription(*it, true);
   }

   CObserverList<CDisplayModelObserver>::ReadLock lock(modelObservers);
   m_Views[pview->m_id] = pview;
   FOR_EACH(pobsrvr, modelObservers) {
      if (pobsrvr) pobsrvr->onViewAdded(this, pview);
   }
}

void CDisplayModel::enableDefaultView(const std::string& objectId, bool enable)
{
   if (enable) {
      std::map<std::string, bool>::iterator it = m_DisabledDefaultViews.find(objectId);
      if (it != m_DisabledDefaultViews.end()) {
         m_DisabledDefaultViews.erase(it);
      }
      return;
   }

   m_DisabledDefaultViews[objectId] = true;

   // enable=false => Remove an existing default view
   TViewMap::iterator itview = m_Views.find(objectId);
   CDisplayView *pview = (itview == m_Views.end()) ? NULL : itview->second;
   if (pview && pview->m_bDefaultView) {
      DMESSAGE("Removing default view: " << pview->m_id);
      CDisplayModelObserver *pobsrvr;
      CObserverList<CDisplayModelObserver>::ReadLock lock(modelObservers);
      m_Views.erase(itview);
      FOR_EACH(pobsrvr, modelObservers) {
         if (pobsrvr) pobsrvr->onViewRemoved(this, pview->m_id);
      }
      delete pview;
   }
}

CDisplayView* CDisplayModel::getView(const string& id)
{
   CDisplayView *pview;
   TViewMap::iterator itview = m_Views.find(id);
   pview = (itview == m_Views.end()) ? NULL : itview->second;
   return pview;
}

bool CDisplayModel::isValidView(CDisplayView *pView)
{
   TViewMap::iterator it;
   for (it = m_Views.begin(); it != m_Views.end(); it++) {
      if (it->second == pView) return true;
   }
   return false;
}

CPtrVector<CDisplayView> CDisplayModel::findViewsWithObject(const std::string &id)
{
   CPtrVector<CDisplayView> views;
   CDisplayView *pview;
   FOR_EACH_V(pview, m_Views) {
      if (pview && pview->hasObject(id))
         views.push_back(pview);
   }
   return views;
}

CPtrVector<CDisplayView> CDisplayModel::findViewsWaitingFor(const std::string &objectId)
{
   CPtrVector<CDisplayView> views;
   CDisplayView *pview;
   FOR_EACH_V(pview, m_Views) {
      if (!pview) continue;
      if (pview->hasObject(objectId)) continue;
      if (pview->waitsForObject(objectId))
         views.push_back(pview);
   }
   return views;
}

CDisplayObject* CDisplayModel::getObject(const std::string &id)
{
   TObjectMap::iterator itobj = m_Objects.find(id);
   CDisplayObject* pfound = (itobj == m_Objects.end()) ? NULL : itobj->second;
   return pfound;
}

CRasterImage* CDisplayModel::getImage(const std::string &id)
{
   TObjectMap::iterator itobj = m_Objects.find(id);
   CDisplayObject* pfound = (itobj == m_Objects.end()) ? NULL : itobj->second;
   if (pfound && pfound->isBitmap())
      return (CRasterImage*) pfound;
   return NULL;
}

void CDisplayModel::setObject(CDisplayObject *pObject)
{
   if (pObject == NULL) return;
   DTRACE("CDisplayModel::setObject");

   CDisplayView *pview;
   CDisplayObject *pfound;

   TObjectMap::iterator itobj = m_Objects.find(pObject->m_id);
   pfound = (itobj == m_Objects.end()) ? NULL : itobj->second;
   if (pfound && pfound != pObject) {
      // we have another object with the same id => replace
      m_Objects.erase(itobj);
      m_Objects[pObject->m_id] = pObject;
      FOR_EACH_V(pview, m_Views) {
         pview->replaceObject(pfound->m_id, pObject);
      }
      delete pfound;
   }
   else if (!pfound) {
      m_Objects[pObject->m_id] = pObject;
   }

   CPtrVector<CDisplayView> views = findViewsWaitingFor(pObject->m_id);
   FOR_EACH(pview, views) {
      if (pview) pview->addObject(pObject);
   }

   views = findViewsWithObject(pObject->m_id);
   if (views.size() < 1) {
      // Check if there is a default view for the object
      typeof(m_Views.begin()) it = m_Views.find(pObject->m_id);
      if (it != m_Views.end()) {
         pview = it->second;
         // XXX: Set preferred context based on object type
         pview->m_preferredContext = pObject->getPreferredContext();
         pview->addObject(pObject);
         views.push_back(pview);
      }
   }

   CDisplayModelObserver *pobsrvr;
   if (views.size() < 1) {
      // XXX Create a default view for each object (this may create too many views)
      std::map<std::string, bool>::iterator it = m_DisabledDefaultViews.find(pObject->m_id);
      if (it == m_DisabledDefaultViews.end()) {
         DMESSAGE("Creating default view for: " << pObject->m_id);
         pview = new cogx::display::CDisplayView();
         pview->m_bDefaultView = true;
         pview->m_preferredContext = pObject->getPreferredContext();

         pview->m_id = pObject->m_id;
         pview->addObject(pObject);
         views.push_back(pview);

         CObserverList<CDisplayModelObserver>::ReadLock lock(modelObservers);
         m_Views[pview->m_id] = pview;
         FOR_EACH(pobsrvr, modelObservers) {
            if (pobsrvr) pobsrvr->onViewAdded(this, pview);
         }
      }
   }
   else {
      // Notify interested observers that the views containing the object have changed.
      DMESSAGE("Object " << pObject->m_id << " found in " << views.size() << "views");
      // XXX this was already done by CDisplayView::replaceObject etc. 
      FOR_EACH(pview, views) {
         CObserverList<CDisplayModelObserver>::ReadLock lock(modelObservers);
         FOR_EACH(pobsrvr, modelObservers) {
            if (pobsrvr) pobsrvr->onViewChanged(this, pview);
         }
      }
   }
}

void CDisplayModel::refreshObject(const std::string &id)
{
   //DTRACE("CDisplayModel::refreshObject");
   TObjectMap::iterator itobj = m_Objects.find(id);
   CDisplayObject* pfound = (itobj == m_Objects.end()) ? NULL : itobj->second;
   if (pfound) {
      CPtrVector<CDisplayView> views = findViewsWithObject(id);
      CDisplayView *pview;
      FOR_EACH(pview, views) {
         pview->refreshObject(id);
      }
   }
}

void CDisplayModel::removeObject(const std::string &id)
{
   DTRACE("CDisplayModel::removeObject");
   CDisplayObject *pfound;
   TObjectMap::iterator itobj = m_Objects.find(id);
   pfound = (itobj == m_Objects.end()) ? NULL : itobj->second;
   if (pfound != NULL) {
      m_Objects.erase(m_Objects.find(pfound->m_id));

      CPtrVector<CDisplayView> views = findViewsWithObject(id);
      CDisplayView *pview;
      FOR_EACH(pview, views) {
         pview->removeObject(id);
      }

      delete pfound;
   }
}

void CDisplayModel::removePart(const std::string &id, const std::string& partId)
{
   DTRACE("CDisplayModel::removePart");
   CDisplayObject *pfound;
   TObjectMap::iterator itobj = m_Objects.find(id);
   pfound = (itobj == m_Objects.end()) ? NULL : itobj->second;
   if (pfound != NULL) {
      // TODO: removing a part should be a 3 stage process
      //    1. remove part from object parts
      //    2. notify observers
      //    3. delete the part <- this one is now in 1 and could cause a segfault.
      // Also: only view observers are notified, but model observers may have to
      // be notified, too.
      pfound->removePart(partId);

      CPtrVector<CDisplayView> views = findViewsWithObject(id);
      CDisplayView *pview;
      FOR_EACH(pview, views) {
         pview->refreshObject(id);
      }
   }
}

bool CDisplayModel::addGuiElement(CGuiElement* pGuiElement)
{
   if (! pGuiElement) return false;

   CGuiElement *pgel;
   FOR_EACH(pgel, m_GuiElements) {
      if (pgel && pgel->isSameElement(pGuiElement)) return false;
   }
   m_GuiElements.push_back(pGuiElement);

   // Make sure there is a view with pgel->m_viewId
   CDisplayView *pview;
   bool found = false;
   typeof(m_Views.begin()) it;
   for (it = m_Views.begin(); it != m_Views.end(); it++) {
      pview = it->second;
      if (!pview) continue;
      if (pview->m_id == pGuiElement->m_viewId) {
         found = true;
         break;
      }
   }
   if (! found) {
     pview = new cogx::display::CDisplayView();
     pview->m_id = pGuiElement->m_viewId;
     m_Views[pview->m_id] = pview;

     CDisplayModelObserver *pobsrvr;
     CObserverList<CDisplayModelObserver>::ReadLock lock(modelObservers);
     FOR_EACH(pobsrvr, modelObservers) {
        if (pobsrvr) pobsrvr->onViewAdded(this, pview);
     }
   }
   return true;
}

int CDisplayModel::getGuiElements(const std::string &viewId, CPtrVector<CGuiElement>& elements)
{
   CGuiElement *pgel;
   int count = 0;
   FOR_EACH(pgel, m_GuiElements) {
      if (pgel && pgel->m_viewId == viewId) {
         elements.push_back(pgel);
         count++;
      }
   }
   return count;
}

CDisplayObject::CDisplayObject()
{
   m_timestamp = 0;
}

CDisplayObject::~CDisplayObject()
{
}

bool CDisplayObject::isBitmap()
{
   return false;
}

void CDisplayObject::removePart(const std::string& partId)
{
}

void CDisplayObject::setTransform2D(const std::string& partId, const std::vector<double>& transform)
{
   DTRACE("CDisplayObject::setTransform2D");
}

CRenderer* CDisplayObject::getRenderer(ERenderContext context)
{
   return NULL;
}

void CDisplayObject::setPose3D(const std::string& partId, const std::vector<double>& positioXYZ,
      const std::vector<double>& rotationQaternionXYZW)
{
}

ERenderContext CDisplayObject::getPreferredContext()
{
   return ContextGraphics;
}

int CDisplayObject::getHtmlChunks(CPtrVector<CHtmlChunk>& forms, int typeMask)
{
   return 0;
}

}} // namespace
