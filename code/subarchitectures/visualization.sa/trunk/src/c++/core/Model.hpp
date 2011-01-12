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
#include "HtmlElements.hpp"

#include <QPainter>
#include <QGraphicsScene>
#include <QStringList>
#include <string>
#include <map>
#include <memory>

namespace cogx { namespace display {

class CDisplayObject;
class CDisplayView;
class CDisplayModel;
class CRasterImage;
class CRenderer;

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
   virtual void onUiDataChanged(CDisplayModel *pModel, CDisplayView *pSourceView,
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

// Holder for all data that can be displayed.
class CDisplayModel
{
private:
   TObjectMap m_Objects;
   CPtrVector<CGuiElement> m_GuiElements;
   std::map<std::string, bool> m_DisabledDefaultViews;
   // TODO: locking for m_Objects, m_Views, m_GuiElements

public: // XXX Qt needs to know about the views.
   TViewMap m_Views;

public:
   CDisplayModel();
   virtual ~CDisplayModel();
   void setObject(CDisplayObject *pObject);
   void refreshObject(const std::string &id);
   void removeObject(const std::string &id);
   void removePart(const std::string &id, const std::string& partId);
   CDisplayObject* getObject(const std::string &id);
   CRasterImage* getImage(const std::string &id);
   CDisplayView* getView(const std::string &id);
   bool isValidView(CDisplayView *pView);
   void createView(const std::string& id, ERenderContext context, const std::vector<std::string>& objects);

   // Default views are created for objects that don't exist in any other
   // views. They are enabled by default. This function can be used to remove a
   // default view that was (or would be) created for an object.
   void enableDefaultView(const std::string& objectId, bool enable=true);

public:
   bool addGuiElement(CGuiElement* pGuiElement);
   int getGuiElements(const std::string &viewId, CPtrVector<CGuiElement>& elements);

public:
   CObserverList<CDisplayModelObserver> modelObservers;

private:
   CPtrVector<CDisplayView> findViewsWithObject(const std::string &id);
   CPtrVector<CDisplayView> findViewsWaitingFor(const std::string &objectId);
};

// The base class for an object to be displayed.
class CDisplayObject
{
public:
   IceUtil::RWRecMutex _objectMutex;

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
   virtual void removePart(const std::string& partId);
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
   virtual void draw(CDisplayObject *pObject, void *pContext) = 0;

   // Some contexts require extra drawing info (eg. htlm: head & body rendered separately)
   virtual void draw(const std::string& info, CDisplayObject *pObject, void *pContext) {}
};


// A view defines a set of objects to be displayed side by side.
// It also defines the layout of the objects.
class CDisplayView: public CGuiElementObserver
{
   TObjectMap m_Objects;
   std::vector<std::string> m_ObjectOrder;

   // Each object may have its own transformation in the view. XXX NOT YET
   std::map<std::string, std::vector<double> > m_Trafos;

   // The view may know about objects even before they exists. When objects
   // are created, they are added to the subscribed views.
   std::map<std::string, bool> m_SubscribedObjects;

public:
   std::string m_id;
   bool m_bDefaultView;
   ERenderContext m_preferredContext;
   CDisplayView();
   virtual ~CDisplayView();
   void addObject(CDisplayObject *pObject);
   void setSubscription(const std::string& id, bool active=true);
   void replaceObject(const std::string& id, CDisplayObject *pNew);
   void refreshObject(const std::string& id);
   void removeObject(const std::string& id);
   void removeAllObjects();
   bool hasObject(const std::string &id);
   bool waitsForObject(const std::string &id);

   virtual void drawGL();
   // XXX: Qt objects shouldn't be here ...
   virtual void draw2D(QPainter &painter);
   virtual void drawScene(QGraphicsScene &scene);
   virtual void drawHtml(QStringList &head, QStringList &body);
   virtual int getHtmlChunks(CPtrVector<CHtmlChunk>& forms, int typeMask);

public:
   // CGuiElementObserver
   void onUiDataChanged(CGuiElement *pElement, const std::string& newValue);
   void onOwnerDataChanged(CGuiElement *pElement, const std::string& newValue);

public:
   CObserverList<CDisplayModelObserver> viewObservers;
};

}} // namespace
#endif
