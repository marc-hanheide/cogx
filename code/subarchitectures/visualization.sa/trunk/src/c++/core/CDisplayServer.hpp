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
   public CGuiDialogObserver,
   public CHtmlFormObserver,
   public COwnerDataProxy
{
private:
   CDisplayModel m_Model;
   CDisplayServerIPtr hIceDisplayServer;
   
   // Standalone Display Server TCP/IP host name.
   // TODO: support the form hostname:port for standalone display server
   //
   // The display server can run in a standalone process on a remote machine.
   // A client can connect to the server if it knows the IP/name of the
   // remote machine. To configure a client the option --standalone-display-host
   // was intially used, but this may be too hard to manage.
   //
   // An alternative is to run the display server with a similar option. Then a
   // client that connects to this server first checks if it should connect to
   // a remote server instead (getStandaloneHost() returns a nonempty string;
   // configured with the server option --redirect-to-host).  The client should
   // then break the current connection and establish a new one to the remote
   // host.
   //
   // The order of connection attempts on the client:
   //   1) if set: connect to --standalone-display-host or die
   //   2) if set: connect to --displayserver or die
   //   2.1) if getStandaloneHost(): disconnect & connect to getStandaloneHost() or die
   //   2.2) else: keep this connection
   // * die: the client still runs, but is not connected (a dead connection)
   //
   // The default value is empty, which means: use this server.
   std::string m_standaloneHost;

protected:
   // CASTComponent methods
   virtual void configure(const std::map<std::string,std::string> & _config)
         throw(std::runtime_error);
   virtual void start();
   virtual void runComponent();

   bool isUsingRemoteHost() {
      return m_standaloneHost.size() > 0;
   }

public:
   CDisplayServer();
   ~CDisplayServer();

   /// nah: extra method to allow standalone server access to functionality.
   virtual void run(); 

   // TODO: CDisplayServer Methods
   // These methods will be called by the ICE server to perform the real work.
   // The methods have the same prototype as the methods created by slice2cpp,
   // but without the last parameter (IceContext).
   // Example:
   //    virtual int AddOne(int value) { return value+1; }

   void getStandaloneHost(std::string& hostname) {
      hostname = m_standaloneHost;
   }

   void resetServer(int secret);

   void createView(const std::string& id, Visualization::ViewType type, const std::vector<std::string>& objects);
   void enableDefaultView(const std::string& objectId, bool enable);

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
         double x, double y, double z, const Visualization::Quaternion& rotation);

   void removeObject(const std::string& id);
   void removePart(const std::string& id, const std::string& partId);

   void addCheckBox(const Ice::Identity& ident, const std::string& viewId,
         const std::string& ctrlId, const std::string& label);
   void addButton(const Ice::Identity& ident, const std::string& viewId,
         const std::string& ctrlId, const std::string& label);
   void addDialog(const Ice::Identity& ident, const std::string& dialogId,
         const std::string& designCode, const std::string& scriptCode,
         const std::string& constructorName);
   void addAction(const Ice::Identity& ident, const std::string& viewId,
         const Visualization::ActionInfo& action);
   void execInDialog(const std::string& dialogId, const std::string& scriptCode);

private:
   void startIceServer();

   // CGuiElementObserver
   void onGuiElement_CtrlDataChanged(CGuiElement *pElement, const std::string& newValue); /*override*/

   // CGuiDialogObserver
   void onGuiDialog_setValue(CGuiDialog *pDialog, const std::string& name, const std::string& value);
   void onGuiDialog_call(CGuiDialog *pDialog, const std::string& name, const std::string& value);

   // CHtmlFormObserver
   void onFormSubmitted(CHtmlChunk *pForm, const TFormValues& newValues); /*override*/
   void onHtmlClick(CHtmlChunk *pChunk, const std::string& ctrlId); /*override*/
   void onHtmlSendValue(CHtmlChunk *pChunk, const std::string& ctrlId, const std::string& value);

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

   virtual void resetServer(int secret, const Ice::Current&)
   {
      m_pDisplayServer->resetServer(secret);
      m_EventClients.clear();
   }

   // These methods will be called by remote clients. They should just
   // pass the parameters to the worker (m_pDisplayServer) and return
   // its results.
   virtual void createView(const std::string& id, Visualization::ViewType type,
         const std::vector<std::string>& objects, const Ice::Current&)
   {
      m_pDisplayServer->createView(id, type, objects);
   }

   virtual void enableDefaultView(const std::string& objectId, bool enable, const Ice::Current&)
   {
      m_pDisplayServer->enableDefaultView(objectId, enable);
   }

   virtual void setObject(const std::string& id, const std::string& partId,
         const std::string& xmlData, const Ice::Current&)
   {
      m_pDisplayServer->setObject(id, partId, xmlData);
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

   virtual void setHtmlFormData(const std::string& id, const std::string& partId,
         const std::map<std::string, std::string>& fields, const Ice::Current&)
   {
      m_pDisplayServer->setHtmlFormData(id, partId, fields);
   }

   virtual void setObjectTransform2D(const std::string& id, const std::string& partId,
         const std::vector<double>& matrix33, const Ice::Current&)
   {
      m_pDisplayServer->setObjectTransform2D(id, partId, matrix33);
   }

   virtual void setObjectPose3D(const std::string& id, const std::string& partId,
         double x, double y, double z, const Visualization::Quaternion& rotation, const Ice::Current&)
   {
      m_pDisplayServer->setObjectPose3D(id, partId, x, y, z, rotation);
   }

   virtual void removeObject(const std::string& id, const Ice::Current&)
   {
      m_pDisplayServer->removeObject(id);
   }

   virtual void removePart(const std::string& id, const std::string& partId, const Ice::Current&)
   {
      m_pDisplayServer->removePart(id, partId);
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

   virtual void addAction(const Ice::Identity& ident, const std::string& viewId,
         const Visualization::ActionInfo& action, const Ice::Current&)
   {
      m_pDisplayServer->addAction(ident, viewId, action);
   }

   virtual void enableMouseEvents(const Ice::Identity& ident, const std::string& viewId,
         bool enable, const Ice::Current&)
   {
      // TODO m_pDisplayServer->enableMouseEvents(ident, viewId, ctrlId, enable);
   }

   virtual void addDialog(const Ice::Identity& ident, const std::string& dialogId,
         const std::string& designCode, const std::string& scriptCode,
         const std::string& constructorName, const Ice::Current&)
   {
      m_pDisplayServer->addDialog(ident, dialogId, designCode, scriptCode, constructorName);
   }

   virtual void execInDialog(const std::string& dialogId, const std::string& script, const Ice::Current&)
   {
      m_pDisplayServer->execInDialog(dialogId, script);
   }

   virtual void getStandaloneHost(std::string& hostname, const Ice::Current&)
   {
      m_pDisplayServer->getStandaloneHost(hostname);
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
  virtual void addClient(const Ice::Identity& ident, const std::string & host, Ice::Int port, const Ice::Current& current); 
   void addOperation(CQueuedOperation* pOperation);
};

} } // namespace

#endif // include once
// vim:sw=3:ts=8:et:
