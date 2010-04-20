/*
 * Author: Marko Mahnič
 * Created: 2010-03-11
 */
#include "Model.hpp"

using namespace std;

namespace cogx { namespace display {


CDisplayModel::CDisplayModel()
{
   //// XXX Testing: the one and only view with one and only image
   //cogx::display::CDisplayView* pView = new cogx::display::CDisplayView();
   //pView->m_id = "video.viewer";
   //m_Views[pView->m_id] = pView;

   //cogx::display::CRasterImage* pImage = new cogx::display::CRasterImage();
   //pImage->m_id = "video.viewer";
   //pImage->m_pImage = new QImage(200, 200, QImage::Format_RGB32);
   //QPainter painter(pImage->m_pImage);
   //painter.setPen(QColor("red"));
   //painter.drawEllipse(100, 88, 50, 70);
   //pView->newObject(pImage);
   //setObject(pImage);
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

CDisplayView* CDisplayModel::getView(const string& id)
{
   CDisplayView *pview;
   TViewMap::iterator itview = m_Views.find(id);
   pview = (itview == m_Views.end()) ? NULL : itview->second;
   return pview;
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
   if (pfound && pfound->isRasterImage())
      return (CRasterImage*) pfound;
   return NULL;
}

void CDisplayModel::setObject(CDisplayObject *pObject)
{
   if (pObject == NULL) return;

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

   // Notify interested observers that the views containing the object have changed.
   CDisplayModelObserver *pobsrvr;
   CPtrVector<CDisplayView> views = findViewsWithObject(pObject->m_id);

   // XXX Create a default view for each object (this may create too many views)
   if (views.size() < 1) {
      pview = new cogx::display::CDisplayView();
      pview->m_id = pObject->m_id;
      pview->addObject(pObject);
      m_Views[pview->m_id] = pview;
      views.push_back(pview);
      FOR_EACH(pobsrvr, modelObservers) {
         pobsrvr->onViewAdded(this, pview);
      }
   }
   else {
      // XXX this was already done by CDisplayView::replaceObject etc. 
      FOR_EACH(pview, views) {
         FOR_EACH(pobsrvr, modelObservers) {
            pobsrvr->onViewChanged(this, pview);
         }
      }
   }
}

void CDisplayModel::refreshObject(const std::string &id)
{
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

CDisplayObject* CDisplayModel::removeObject(const std::string &id)
{
   CDisplayObject *pfound;
   TObjectMap::iterator itobj = m_Objects.find(id);
   pfound = (itobj == m_Objects.end()) ? NULL : itobj->second;
   if (pfound != NULL) {
      m_Objects.erase(m_Objects.find(pfound->m_id));
      return pfound;
   }
   return NULL;
}

bool CDisplayModel::addGuiElement(CGuiElement* pGuiElement)
{
   if (! pGuiElement) return false;

   CGuiElement *pgel;
   FOR_EACH(pgel, m_GuiElements) {
      if (pgel && pgel->isSameElement(pGuiElement)) return false;
   }
   m_GuiElements.push_back(pGuiElement);
   return true;
}

CPtrVector<CGuiElement> CDisplayModel::getGuiElements(const std::string &viewId)
{
   CPtrVector<CGuiElement> elements;
   CGuiElement *pgel;
   FOR_EACH(pgel, m_GuiElements) {
      if (pgel && pgel->m_viewId == viewId) elements.push_back(pgel);
   }
   return elements;
}

//bool CDisplayModel::Merge(CDisplayObject *pObject)
//{
//   return false;
//}

CDisplayObject::CDisplayObject()
{
   m_isBitmap = false;
}

CDisplayObject::~CDisplayObject()
{
}

CDisplayView::CDisplayView()
{
}

CDisplayView::~CDisplayView()
{
   m_Objects.clear(); // do not delete, owned by the model
}

bool CDisplayView::hasObject(const std::string &id)
{
   return m_Objects[id] ? true : false;
}

void CDisplayView::addObject(CDisplayObject *pObject)
{
   if (! pObject) return;
   // CDisplayObject *pExisting = m_Objects.find(pObject->m_id);
   // if (pExisting) return; // TODO: This is an error! Should use ReplaceObject
   TObjectMap::iterator existing = m_Objects.find(pObject->m_id);
   if (existing != m_Objects.end()) return;

   m_Objects[pObject->m_id] = pObject;
   CDisplayModelObserver *pobsrvr;
   FOR_EACH(pobsrvr, viewObservers) {
      pobsrvr->onViewChanged(NULL, this);
   }
}

void CDisplayView::removeObject(const std::string& id)
{
   //CDisplayObject *pExisting = m_Objects.find(pObject->m_id);
   //if (! pExisting) return;
   TObjectMap::iterator existing = m_Objects.find(id);
   if (existing == m_Objects.end()) return;

   m_Objects.erase(existing);

   CDisplayModelObserver *pobsrvr;
   FOR_EACH(pobsrvr, viewObservers) {
      pobsrvr->onViewChanged(NULL, this);
   }
}

void CDisplayView::replaceObject(const std::string& id, CDisplayObject *pNew)
{
   //CDisplayObject *pExisting = m_Objects.find(pRemove->m_id);
   //if (! pExisting) return;
   TObjectMap::iterator existing = m_Objects.find(id);
   if (existing == m_Objects.end()) return;

   m_Objects.erase(existing);
   if (pNew)
      m_Objects[pNew->m_id] = pNew;

   CDisplayModelObserver *pobsrvr;
   FOR_EACH(pobsrvr, viewObservers) {
      pobsrvr->onViewChanged(NULL, this);
   }
}

void CDisplayView::refreshObject(const std::string& id)
{
   //CDisplayObject *pExisting = m_Objects.find(pObject->m_id);
   //if (! pExisting) return;
   TObjectMap::iterator existing = m_Objects.find(id);
   if (existing == m_Objects.end()) return;

   CDisplayModelObserver *pobsrvr;
   FOR_EACH(pobsrvr, viewObservers) {
      pobsrvr->onViewChanged(NULL, this);
   }
}

// The CDisplayObject should not draw itself, instead it should provide another
// object (Renderer) that will draw it. function: getRenderer(context).
// Implementation: all objects of the same class share the same (static)
// renderer for each type of context (2D, 3D, text, etc.).
// XXX - a single static renderer is not thread-safe!
// TODO: the context may provide additional display options.
void CDisplayView::draw2D(QPainter &painter)
{
   // painter.drawText(0, 40, QString("View ID: ") + QString(m_id.c_str()));
   CDisplayObject *pObject;
   CRenderer *pRender;
   FOR_EACH_V(pObject, m_Objects) {
      if (!pObject) continue;
      pRender = pObject->getRenderer(Context2D);
      if (pRender) {
         pRender->draw(pObject, &painter);
      }
   }
}

}} // namespace
