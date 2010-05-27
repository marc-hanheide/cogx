/*
 * Author: Marko Mahnič
 * Created: 2010-03-11
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

   //// XXX Testing composed views
   //cogx::display::CDisplayView* pview = new cogx::display::CDisplayView();
   //pview->m_id = "Composed View";
   //m_Views[pview->m_id] = pview;
   //pview->m_SubscribedObjects["video.viewer"] = true;
   //pview->m_SubscribedObjects["Visualization.test.SVG"] = true;
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

   // Notify interested observers that the views containing the object have changed.
   CDisplayModelObserver *pobsrvr;
   views = findViewsWithObject(pObject->m_id);

   // XXX Create a default view for each object (this may create too many views)
   if (views.size() < 1) {
      DMESSAGE("Creating new view for: " << pObject->m_id);
      pview = new cogx::display::CDisplayView();
      // XXX: Set preferred context based on object type
      if (pObject->is3D()) pview->m_preferredContext = ContextGL;

      pview->m_id = pObject->m_id;
      pview->addObject(pObject);
      m_Views[pview->m_id] = pview;
      views.push_back(pview);
      FOR_EACH(pobsrvr, modelObservers) {
         if (pobsrvr) pobsrvr->onViewAdded(this, pview);
      }
   }
   else {
      DMESSAGE("Object " << pObject->m_id << " found in " << views.size() << "views");
      // XXX this was already done by CDisplayView::replaceObject etc. 
      FOR_EACH(pview, views) {
         FOR_EACH(pobsrvr, modelObservers) {
            if (pobsrvr) pobsrvr->onViewChanged(this, pview);
         }
      }
   }
}

void CDisplayModel::refreshObject(const std::string &id)
{
   DTRACE("CDisplayModel::refreshObject");
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
      delete pfound;
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
}

CDisplayObject::~CDisplayObject()
{
}

bool CDisplayObject::isBitmap()
{
   return false;
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

bool CDisplayObject::is3D()
{
   return false;
}

CDisplayView::CDisplayView()
{
   m_preferredContext = Context2D;
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
   // CDisplayObject *pExisting = m_Objects.find(pObject->m_id);
   // if (pExisting) return; // TODO: This is an error! Should use ReplaceObject
   TObjectMap::iterator existing = m_Objects.find(pObject->m_id);
   if (existing != m_Objects.end()) return;
   DMESSAGE(m_id << ": Adding object: " << pObject->m_id);

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
   if (existing == m_Objects.end() || existing->second == NULL) return;
   DMESSAGE(m_id << ": Removing  object: " << id);

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
   if (existing == m_Objects.end() || existing->second == NULL) return;
   DMESSAGE(m_id << ": Replacing object: " << id);

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
   DMESSAGE(m_id << ": Refresh object: " << id);

   CDisplayModelObserver *pobsrvr;
   FOR_EACH(pobsrvr, viewObservers) {
      pobsrvr->onViewChanged(NULL, this);
   }
}

void CDisplayView::onUiDataChanged(CGuiElement *pElement, const std::string& newValue)
{
   CDisplayModelObserver *pobsrvr;
   FOR_EACH(pobsrvr, viewObservers) {
      pobsrvr->onUiDataChanged(NULL, this, pElement, newValue);
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
   CDisplayObject *pObject;
   CRenderer *pRender;
   FOR_EACH_V(pObject, m_Objects) {
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

void CDisplayView::drawGL()
{
   CDisplayObject *pObject;
   CRenderer *pRender;
   FOR_EACH_V(pObject, m_Objects) {
      if (!pObject) continue;
      pRender = pObject->getRenderer(ContextGL);
      if (pRender) {
         pRender->draw(pObject, NULL);
      }
   }
}

}} // namespace
