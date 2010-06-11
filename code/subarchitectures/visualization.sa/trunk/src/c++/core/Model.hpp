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

#include <QPainter>
#include <QStringList>
#include <string>
#include <map>
#include <memory>

#include "ptrvector.hpp"
#include "observer.hpp"
#include "GuiElements.hpp"

namespace cogx { namespace display {

class CDisplayObject;
class CDisplayView;
class CDisplayModel;
class CRasterImage;
class CRenderer;

typedef enum { Context2D=1, ContextGL=2, ContextHtml=3 } ERenderContext;

typedef std::map<std::string, CDisplayObject*> TObjectMap;
typedef std::map<std::string, CDisplayView*> TViewMap;

class CDisplayModelObserver
{
public:
   virtual void onViewChanged(CDisplayModel *pModel, CDisplayView *pView) {}
   virtual void onViewAdded(CDisplayModel *pModel, CDisplayView *pView) {}
   virtual void onViewRemoved(CDisplayModel *pModel, const std::string& id) {}
   virtual void onUiDataChanged(CDisplayModel *pModel, CDisplayView *pSourceView,
         CGuiElement *pElement, const std::string& newValue) {}
};

// Holder for all data that can be displayed.
class CDisplayModel
{
private:
   TObjectMap m_Objects;
   CPtrVector<CGuiElement> m_GuiElements;
   // TODO: locking for m_Objects, m_Views, m_GuiElements

public: // XXX Qt needs to know about the views.
   TViewMap m_Views;

public:
   CDisplayModel();
   virtual ~CDisplayModel();
   void setObject(CDisplayObject *pObject);
   void refreshObject(const std::string &id);
   void removeObject(const std::string &id);
   CDisplayObject* getObject(const std::string &id);
   CRasterImage* getImage(const std::string &id);
   CDisplayView* getView(const std::string &id);
   bool isValidView(CDisplayView *pView);

public:
   bool addGuiElement(CGuiElement* pGuiElement);
   CPtrVector<CGuiElement> getGuiElements(const std::string &viewId);

//public:
   CObserver<CDisplayModelObserver> modelObservers;

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

   //// Some objects can be merged with a new version instead of being replaced (eg. bitmaps)
   //// Returns true if the objects were successfully merged.
   //// When false is returned, the old object should be replaced with the new one.
   //virtual bool Merge(CDisplayObject *pObject);
};


class CRenderer
{
public:
   // Draws the object into the specified context. The function casts
   // the pointers to the desired types draws the object.
   virtual void draw(CDisplayObject *pObject, void *pContext) = 0;
};


// A view defines a set of objects to be displayed side by side.
// It also defines the layout of the objects.
class CDisplayView: public CGuiElementObserver
{
   TObjectMap m_Objects;
public: // XXX remove
   std::map<std::string, std::vector<double> > m_Trafos;
   std::map<std::string, bool> m_SubscribedObjects;

public:
   std::string m_id;
   ERenderContext m_preferredContext;
   CDisplayView();
   virtual ~CDisplayView();
   void addObject(CDisplayObject *pObject);
   void replaceObject(const std::string& id, CDisplayObject *pNew);
   void removeObject(const std::string& id);
   void refreshObject(const std::string& id);
   bool hasObject(const std::string &id);
   bool waitsForObject(const std::string &id);

   virtual void drawGL();
   // XXX: Qt objects shouldn't be here ...
   virtual void draw2D(QPainter &painter);
   virtual void drawHtml(QStringList &list);

public:
   // CGuiElementObserver
   void onUiDataChanged(CGuiElement *pElement, const std::string& newValue);
   void onOwnerDataChanged(CGuiElement *pElement, const std::string& newValue);

public:
   CObserver<CDisplayModelObserver> viewObservers;
};

}} // namespace
#endif
