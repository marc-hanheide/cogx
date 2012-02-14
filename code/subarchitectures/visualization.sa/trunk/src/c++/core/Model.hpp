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
#ifndef __TEMP_MODEL_HPP__
#define __TEMP_MODEL_HPP__

#include "ptrvector.hpp"
#include "observer.hpp"
#include "GuiElements.hpp"

#include <QPainter>
#include <QGraphicsScene>
#include <QStringList>
#include <string>
#include <map>
#include <memory>

namespace cogx { namespace display {

class CDisplayObjectPart;
class CDisplayObject;
class CDisplayView;
class CDisplayModel;
class CDisplayCamera;
class CRasterImage;
class CRenderer;
class CGarbage;

// CHtmlChunk is a CDisplayObjectPart; defined in "HtmlElements.hpp"
// Used in some functions instead of CDisplayObjectPart for convenience.
class CHtmlChunk;

typedef enum { Context2D=1, ContextGL=2, ContextHtml=3, ContextGraphics=4 } ERenderContext;

typedef std::map<std::string, CDisplayObject*> TObjectMap;
typedef TObjectMap::iterator TObjectMapIterator;
typedef std::map<std::string, CDisplayView*> TViewMap;
typedef TViewMap::iterator TViewMapIterator;

class CDisplayModelObserver
{
public:
   virtual void onViewChanged(CDisplayModel *pModel, CDisplayView *pView) {}
   virtual void onViewAdded(CDisplayModel *pModel, CDisplayView *pView) {}
   virtual void onViewRemoved(CDisplayModel *pModel, const std::string& id) {}
   virtual void onDialogAdded(CDisplayModel *pModel, CGuiDialog *pDialog) {}
   virtual void onModel_UiDataChanged(CDisplayModel *pModel, CDisplayView *pSourceView,
         CGuiElement *pElement, const std::string& newValue) {}
};

class COwnerDataProxy
{
public:
   // Issue a request to retreive the value for a control from the (remote) owner.
   // see: <url:GuiElements.hpp#tn=syncControlState>
   // see: <url:CDisplayServer.cpp#tn=CDisplayServerI::addDataChange>
   // see: <url:CDisplayServer.cpp#tn=CDisplayServerI::run>
   virtual void getControlStateAsync(cogx::display::CGuiElement *pElement) = 0;

   // Issue a request to retreive the values of a form from the (remote) owner.
   virtual void getFormStateAsync(CHtmlChunk* pForm) = 0;

   // This function should go in sth. like CPersistentStorage interface
   // but since ATM we will only be using it in one place (one provider,
   // one consumer) it is added to this interface.
   // It retreives the name of the persistent storage that contains data
   // for the HTML forms etc. ATM it is a filename, configured by a 
   // CDisplayServer config parameter.
   // Used by QCastMainFrame to access the persistent storage.
   virtual std::string getPersistentStorageName() = 0;
};

// A temporary fix for crashes when objects are deleted during redraw.
class CGarbage
{
   CPtrVector<CDisplayObject> m_objects;
   CPtrVector<CDisplayObjectPart> m_parts;
public:
   ~CGarbage()
   {
      m_objects.delete_all();
      m_parts.delete_all();
   }
   void add(CPtrVector<CDisplayObject>& objects)
   {
      CDisplayObject* pObject;
      FOR_EACH(pObject, objects) {
         if (pObject) m_objects.push_back(pObject);
      }
   }
   void add(CDisplayObject* pObject)
   {
      if (pObject) m_objects.push_back(pObject);
   }
   void add(CPtrVector<CDisplayObjectPart>& parts)
   {
      CDisplayObjectPart* pPart;
      FOR_EACH(pPart, parts) {
         if (pPart) m_parts.push_back(pPart);
      }
   }
   void add(CDisplayObjectPart* pPart)
   {
      if (pPart) m_parts.push_back(pPart);
   }
};

// Holder for all data that can be displayed.
class CDisplayModel
{
private:
   CGarbage m_garbage;
   TObjectMap m_Objects;
   CPtrVector<CGuiElement> m_GuiElements;
   CPtrVector<CGuiDialog> m_GuiDialogs;
   std::map<std::string, bool> m_DisabledDefaultViews;
   // TODO: locking for m_Objects, m_Views, m_GuiElements

public: // XXX Qt needs to know about the views.
   TViewMap m_Views;

public:
   CDisplayModel();
   virtual ~CDisplayModel();
   void setObject(CDisplayObject *pObject);
   void refreshObject(const std::string &id, bool bNotifyChanged=false);
   void removeObject(const std::string &id);
   void removePart(const std::string &id, const std::string& partId);
   CDisplayObject* getObject(const std::string &id);
   CRasterImage* getImage(const std::string &id);
   CDisplayView* getView(const std::string &id);
   CGuiDialog* getDialog(const std::string &id);
   bool isValidView(CDisplayView *pView);
   void createView(const std::string& id, ERenderContext context, const std::vector<std::string>& objects);
   void removeView(const std::string& id);
   void removeAllViews();
   void removeAllObjects();

   // Default views are created for objects that don't exist in any other
   // views. They are enabled by default. This function can be used to remove a
   // default view that was (or would be) created for an object.
   void enableDefaultView(const std::string& objectId, bool enable=true);

public:
   bool addGuiElement(CGuiElement* pGuiElement);
   int getGuiElements(const std::string &viewId, CPtrVector<CGuiElement>& elements);
   bool addGuiDialog(CGuiDialog* pGuiDialog);
   int getDialogs(CPtrVector<CGuiDialog>& dialogs);

public:
   CObserverList<CDisplayModelObserver> modelObservers;

private:
   CPtrVector<CDisplayView> findViewsWithObject(const std::string &id);
   CPtrVector<CDisplayView> findViewsWaitingFor(const std::string &objectId);
};

class CDisplayObjectPart
{
public:
   std::string m_id;

public:
   virtual ~CDisplayObjectPart() {}
};

// The base class for an object to be displayed.
class CDisplayObject
{
public:
   class ReadLock
   {
      CDisplayObject* pOwner;
   public:
      ReadLock(CDisplayObject& owner) { pOwner = &owner; pOwner->_objectMutex.readLock(); }
      ~ReadLock() { pOwner->_objectMutex.unlock(); }
   };
   class WriteLock
   {
      CDisplayObject* pOwner;
   public:
      WriteLock(CDisplayObject& owner) { pOwner = &owner; pOwner->_objectMutex.writeLock(); }
      ~WriteLock() { pOwner->_objectMutex.unlock(); }
   };

private:
   IceUtil::RWRecMutex _objectMutex;
   friend class CDisplayObject::ReadLock;
   friend class CDisplayObject::WriteLock;

public:
   std::string m_id;
   double m_timestamp;
   CDisplayObject();
   virtual ~CDisplayObject();
   virtual bool isBitmap();
   virtual ERenderContext getPreferredContext();

   virtual CRenderer* getRenderer(ERenderContext context);
   virtual void setTransform2D(const std::string& partId, const std::vector<double>& transform);
   virtual void setPose3D(const std::string& partId, const std::vector<double>& positioXYZ,
         const std::vector<double>& rotationQaternionXYZW);

   virtual int getHtmlChunks(CPtrVector<CHtmlChunk>& forms, int typeMask);
   virtual void getParts(CPtrVector<CDisplayObjectPart>& parts, bool bOrdered=false);
   virtual int getCameras(CPtrVector<CDisplayCamera>& cameras)
   {
      return 0;
   }

   // Returns true if the part existed and was successfully removed.
   virtual bool removePart(const std::string& partId, CPtrVector<CDisplayObjectPart>& parts) = 0;
};

// The state can't be stored with CDisplayObject because an object may be
// displayed in multiple views.
// Note: CViewedObjectState is a nested map.
class CViewedObjectState
{
public:
   bool m_bVisible;
   std::map<std::string, CViewedObjectState> m_childState;
   CViewedObjectState() {
      m_bVisible = true;
   }
};

// An abstract class that renders an object into a context.
// Each object returns an instance of a renderer to render it in
// the specified context (if the context type is supported).
//
// A renderer should not have any member variables so that a single
// static instance can be used for all drawing -- the renderer can
// be safely shared between multiple threads (although it will allways
// be executed in the GUI thread).
class CRenderer
{
public:
   // Draws the object into the specified context. The function casts
   // the pointers to the desired types draws the object.
   virtual void draw(CDisplayView* pView, CDisplayObject *pObject, void *pContext) = 0;

   // Some contexts require extra drawing info (eg. htlm: head & body rendered separately)
   virtual void draw(CDisplayView* pView, const std::string& info, CDisplayObject *pObject, void *pContext) {}
};

// Objects may define positions from which they are best visible.
class CDisplayCamera
{
public:
   std::string name;
   double xEye, yEye, zEye;    // Camera position
   double xView, yView, zView; // Viewing direction
   double xUp, yUp, zUp;       // Up vector
   double viewAngle;           // Camera 'zoom', in degrees

public:
   CDisplayCamera()
   {
      xEye = yEye = zEye = 0;
      xView = yView = 0; zView = 1;
      xUp = zUp = 0; yUp = 1;
      viewAngle = 45;
   }
   void setEye(double x, double y, double z)
   {
      xEye = x;
      yEye = y;
      zEye = z;
   }
   void setViewVector(double x, double y, double z)
   {
      xView = x;
      yView = y;
      zView = z;
   }
   void setUpVector(double x, double y, double z)
   {
      xUp = x;
      yUp = y;
      zUp = z;
   }
};

// TODO rename to sth. like CGlContextProxy
class CGlTextWriter
{
public:
   virtual void renderText(double x, double y, double z, const std::string& text, double size) = 0;
};

// A view defines a set of objects to be displayed side by side.
// It also defines the layout of the objects.
class CDisplayView: public CGuiElementObserver
{
   CDisplayModel *m_pModel;
   TObjectMap m_Objects;
   std::vector<std::string> m_ObjectOrder;

   // Each object may have its own transformation in the view. XXX NOT YET
   std::map<std::string, std::vector<double> > m_Trafos;

   // The view may know about objects even before they exist. When objects
   // are created, they are added to the subscribed views.
   std::map<std::string, bool> m_SubscribedObjects;

   // Display properties of objects and parts.
   // We keep state information even after an object/part is removed from
   // the view just in case if an object with the same id is recreated.
   CViewedObjectState m_ObjectState;

public:
   std::string m_id;
   bool m_bDefaultView;
   ERenderContext m_preferredContext;
   CDisplayView(CDisplayModel *pModel);
   virtual ~CDisplayView();
   void addObject(CDisplayObject *pObject);
   void setSubscription(const std::string& id, bool active=true);
   void replaceObject(const std::string& id, CDisplayObject *pNew);
   void refreshObject(const std::string& id);
   void removeObject(const std::string& id);
   void removeAllObjects();
   bool hasObject(const std::string &id);
   bool waitsForObject(const std::string &id);
   void getObjects(CPtrVector<CDisplayObject>& objects, bool bOrdered=false);
   CViewedObjectState* getObjectState(const std::string& id);

   virtual void drawGL(CGlTextWriter* pTextWriter=0);
   // XXX: Qt objects shouldn't be here ...
   virtual void draw2D(QPainter &painter);
   virtual void drawScene(QGraphicsScene &scene);
   virtual void drawHtml(QStringList &head, QStringList &body);

   // TODO: should getHtmlChunks observe CViewedObjectState.m_bVisible?
   virtual int getHtmlChunks(CPtrVector<CHtmlChunk>& forms, int typeMask);
   
   int getCameras(CPtrVector<CDisplayCamera>& cameras);

public:
   // CGuiElementObserver
   void onGuiElement_CtrlDataChanged(CGuiElement *pElement, const std::string& newValue);
   void onGuiElement_OwnerDataChanged(CGuiElement *pElement, const std::string& newValue);

public:
   CObserverList<CDisplayModelObserver> viewObservers;
};

}} // namespace
#endif
