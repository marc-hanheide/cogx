/*
 * Author: Marko Mahnič
 * Created: 2010-03-08
 */
#ifndef __DISPLAYSERVER_CDISPLAYSERVER_H__
#define __DISPLAYSERVER_CDISPLAYSERVER_H__
#include <string>
#include <map>
#include <set>
#include <stdexcept>
#include <cast/architecture/ManagedComponent.hpp>
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>

#include <DisplayServer.hpp> // generated from ice
#include "qtui/QCastMainFrame.hpp"
#include "Model.hpp"

namespace cogx { namespace display {

class CDisplayServerI;
typedef IceUtil::Handle<CDisplayServerI> CDisplayServerIPtr;

// CDisplayServer is the component that will be created when CAST starts.
// The ICE server interface (CDisplayServerI) will be created in start().
class CDisplayServer:
   public cast::ManagedComponent,
   public CGuiElementObserver
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
   void setObjectTransform(const std::string& id, const std::string& partId,
         const std::vector<double>& transform);
   void addCheckBox(const Ice::Identity& ident, const std::string& viewId,
         const std::string& ctrlId, const std::string& label);
   void addButton(const Ice::Identity& ident, const std::string& viewId,
         const std::string& ctrlId, const std::string& label);

private:
   void startIceServer();

   // CGuiElementObserver
   void onUiDataChanged(CGuiElement *pElement, const std::string& newValue); /*override*/
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
   virtual void setObjectTransform(const std::string& id, const std::string& partId,
         const cogx::Math::Matrix33& transform, const Ice::Current&)
   {
      std::vector<double> tr;
      tr.push_back(transform.m00);
      tr.push_back(transform.m01);
      tr.push_back(transform.m02);
      tr.push_back(transform.m10);
      tr.push_back(transform.m11);
      tr.push_back(transform.m12);
      tr.push_back(transform.m20);
      tr.push_back(transform.m21);
      tr.push_back(transform.m22);
      m_pDisplayServer->setObjectTransform(id, partId, tr);
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

   //-----------------------------------------------------------------
   // Implement event callbacks; the sender is a separate thread
   // See Ice demo: demo/Ice/bidir
   //-----------------------------------------------------------------
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
   CPtrVector<CGuiElementValue> m_EventQueue;
   IceUtil::ThreadPtr m_pEventSenderThread;

   void run();

public:
   void startEventServer();
   void destroyEventServer(); 
   virtual void addClient(const Ice::Identity& ident, const Ice::Current& current); 
   void addDataChange(CGuiElementValue *pChange);
};

} } // namespace

#endif // include once
// vim:sw=3:ts=8:et:
