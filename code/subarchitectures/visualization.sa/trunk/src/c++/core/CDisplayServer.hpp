/*
 * Author: Marko Mahnič
 * Created: 2010-03-08
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
#ifndef __DISPLAYSERVER_CDISPLAYSERVER_H__
#define __DISPLAYSERVER_CDISPLAYSERVER_H__
#include "DisplayServer.hpp" // generated from ice
#include "qtui/QCastMainFrame.hpp"
#include "Model.hpp"
#include "GuiElements.hpp"
#include "HtmlElements.hpp"

#include <cast/architecture/ManagedComponent.hpp>
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>

#include <string>
#include <map>
#include <set>
#include <stdexcept>

namespace cogx { namespace display {

class CDisplayServerI;
typedef IceUtil::Handle<CDisplayServerI> CDisplayServerIPtr;

// CDisplayServer is the component that will be created when CAST starts.
// The ICE server interface (CDisplayServerI) will be created in start().
class CDisplayServer:
   public cast::ManagedComponent,
   public CGuiElementObserver,
   public CHtmlFormObserver,
   public COwnerDataProxy
{
private:
   CDisplayModel m_Model;
   QCastMainFrame *pMainFrame;
   CDisplayServerIPtr hIceDisplayServer;

public:
   CDisplayServer();
   ~CDisplayServer();

   // CASTComponent methods
   virtual void configure(const std::map<std::string,std::string> & _config)
         throw(std::runtime_error);
   virtual void start();
   virtual void runComponent();

   // TODO: CDisplayServer Methods
   // These methods will be called by the ICE server to perform the real work.
   // The methods have the same prototype as the methods created by slice2cpp,
   // but without the last parameter (IceContext).
   // Example:
   //    virtual int AddOne(int value) { return value+1; }

   // TODO: (maybe) create a different image implementation for client/server comm.
   void setRawImage(const std::string& id, int width, int height, int channels,
         const std::vector<unsigned char>& data);
   void setCompressedImage(const std::string& id, const std::vector<unsigned char>& data,
         const std::string &format);
   void setObject(const std::string& id, const std::string& partId, const std::string& xmlData);
   void setTomGineObject(const std::string& id, const std::string& partId, 
         const std::vector<unsigned char>& data);
   void setLuaGlObject(const std::string& id, const std::string& partId, const std::string& script);
   void setHtml(const std::string& id, const std::string& partId, const std::string& htmlData);
   void setHtmlHead(const std::string& id, const std::string& partId, const std::string& htmlData);
   void setActiveHtml(const Ice::Identity& ident, const std::string& id, const std::string& partId,
         const std::string& htmlData);
   void setHtmlForm(const Ice::Identity& ident, const std::string& id, const std::string& partId,
         const std::string& htmlData);
   void setHtmlFormData(const std::string& id, const std::string& partId,
         const std::map<std::string, std::string>& fields);
   void setObjectTransform2D(const std::string& id, const std::string& partId,
         const std::vector<double>& transform);
   void setObjectPose3D(const std::string& id, const std::string& partId,
         const Math::Vector3& position, const Visualization::Quaternion& rotation);
   void addCheckBox(const Ice::Identity& ident, const std::string& viewId,
         const std::string& ctrlId, const std::string& label);
   void addButton(const Ice::Identity& ident, const std::string& viewId,
         const std::string& ctrlId, const std::string& label);

private:
   void startIceServer();

   // CGuiElementObserver
   void onUiDataChanged(CGuiElement *pElement, const std::string& newValue); /*override*/

   // CHtmlFormObserver
   void onFormSubmitted(CHtmlChunk *pForm, const TFormValues& newValues); /*override*/
   void onHtmlClick(CHtmlChunk *pChunk, const std::string& ctrlId); /*override*/

   // CControlDataProxy
   void getControlStateAsync(cogx::display::CGuiElement *pElement); /*override*/
   void getFormStateAsync(CHtmlChunk* pForm); /*override*/
   std::string getPersistentStorageName(); /*override*/
};

// The implementation of DisplayInterface. This class implements
// an ICE server that will respond to remote requests.
class CDisplayServerI: public Visualization::DisplayInterface
{
private:
    CDisplayServer *m_pDisplayServer;

public:
   CDisplayServerI(CDisplayServer *pDisplayServer);

   // These methods will be called by remote clients. They should just
   // pass the parameters to the worker (m_pDisplayServer) and return
   // its results.
   virtual void setObject(const std::string& id, const std::string& partId,
         const std::string& xmlData, const Ice::Current&)
   {
      m_pDisplayServer->setObject(id, partId, xmlData);
   }

   virtual void setImage(const std::string& id, const Video::Image& image, const Ice::Current&) {
      m_pDisplayServer->setRawImage(id, image.width, image.height, 3, image.data);
   }

   virtual void setRawImage(const std::string& id, int width, int height, int channels,
         const std::vector<unsigned char>& data, const Ice::Current&)
   {
      m_pDisplayServer->setRawImage(id, width, height, channels, data);
   }

   virtual void setCompressedImage(const std::string& id,
         const std::vector<unsigned char>& data, const std::string &format, const Ice::Current&)
   {
      m_pDisplayServer->setCompressedImage(id, data, format);
   }

   virtual void setTomGineObject(const std::string& id, const std::string& partId, 
         const std::vector<unsigned char>& data, const Ice::Current&)
   {
      m_pDisplayServer->setTomGineObject(id, partId, data);
   }

   virtual void setLuaGlObject(const std::string& id, const std::string& partId,
         const std::string& script, const Ice::Current&)
   {
      m_pDisplayServer->setLuaGlObject(id, partId, script);
   }

   virtual void setHtml(const std::string& id, const std::string& partId,
         const std::string& htmlData, const Ice::Current&)
   {
      m_pDisplayServer->setHtml(id, partId, htmlData);
   }

   virtual void setHtmlHead(const std::string& id, const std::string& partId,
         const std::string& htmlData, const Ice::Current&)
   {
      m_pDisplayServer->setHtmlHead(id, partId, htmlData);
   }

   virtual void setActiveHtml(const Ice::Identity& ident, const std::string& id, const std::string& partId,
         const std::string& htmlData, const Ice::Current&)
   {
      m_pDisplayServer->setActiveHtml(ident, id, partId, htmlData);
   }

   virtual void setHtmlForm(const Ice::Identity& ident, const std::string& id, const std::string& partId,
         const std::string& htmlData, const Ice::Current&)
   {
      m_pDisplayServer->setHtmlForm(ident, id, partId, htmlData);
   }

   virtual void setObjectTransform2D(const std::string& id, const std::string& partId,
         const std::vector<double>& matrix33, const Ice::Current&)
   {
      //std::vector<double> tr;
      //tr.push_back(transform.m00);
      //tr.push_back(transform.m01);
      //tr.push_back(transform.m02);
      //tr.push_back(transform.m10);
      //tr.push_back(transform.m11);
      //tr.push_back(transform.m12);
      //tr.push_back(transform.m20);
      //tr.push_back(transform.m21);
      //tr.push_back(transform.m22);
      m_pDisplayServer->setObjectTransform2D(id, partId, matrix33);
   }

   virtual void setObjectPose3D(const std::string& id, const std::string& partId,
         const Math::Vector3& position, const Visualization::Quaternion& rotation, const Ice::Current&)
   {
      m_pDisplayServer->setObjectPose3D(id, partId, position, rotation);
   }

   virtual void addCheckBox(const Ice::Identity& ident, const std::string& viewId,
         const std::string& ctrlId, const std::string& label, const Ice::Current&)
   {
      m_pDisplayServer->addCheckBox(ident, viewId, ctrlId, label);
   }

   virtual void addButton(const Ice::Identity& ident, const std::string& viewId,
         const std::string& ctrlId, const std::string& label, const Ice::Current&)
   {
      m_pDisplayServer->addButton(ident, viewId, ctrlId, label);
   }

   virtual void addToolButton(const Ice::Identity& ident, const std::string& viewId,
         const std::string& ctrlId, const Visualization::ActionInfo& info,
         const Ice::Current&)
   {
      // TODO m_pDisplayServer->addToolButton(ident, viewId, ctrlId, info);
   }

   virtual void enableMouseEvents(const Ice::Identity& ident, const std::string& viewId,
         bool enable, const Ice::Current&)
   {
      // TODO m_pDisplayServer->enableMouseEvents(ident, viewId, ctrlId, enable);
   }

   //-----------------------------------------------------------------
   // Implement event callbacks; the sender is a separate thread
   // See Ice demo: demo/Ice/bidir
   //-----------------------------------------------------------------
public:
   class CQueuedOperation
   {
   public:
      Ice::Identity m_clientId;
      CQueuedOperation(const Ice::Identity& clientId) 
      {
         m_clientId = clientId;
      }
      virtual void execute(Visualization::EventReceiverPrx& pClient) = 0;
   };

private:
   class CCallbackSenderThread : public IceUtil::Thread
   {
      const CDisplayServerIPtr _callbackSender;

   public:
      CCallbackSenderThread(const CDisplayServerIPtr& callbackSender)
         : _callbackSender(callbackSender)
      {}

      virtual void run() {
         _callbackSender->run();
      }
   };

   bool m_stopEventServer;
   IceUtil::Monitor<IceUtil::Mutex> m_EventMonitor;
   // TODO: client subscribes to events from selected views/dialogs
   std::set<Visualization::EventReceiverPrx> m_EventClients;
   CPtrVector<CQueuedOperation> m_OperationQueue;
   IceUtil::ThreadPtr m_pEventSenderThread;

   void run();

public:
   void startEventServer();
   void destroyEventServer(); 
   virtual void addClient(const Ice::Identity& ident, const Ice::Current& current); 
   void addOperation(CQueuedOperation* pOperation);
};

} } // namespace

#endif // include once
// vim:sw=3:ts=8:et:
