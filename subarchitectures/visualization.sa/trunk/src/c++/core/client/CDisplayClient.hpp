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
#ifndef __DISPLAYCLIENT_CDISPLAYCLIENT_H__
#define __DISPLAYCLIENT_CDISPLAYCLIENT_H__

#include <string>
#include <map>
#include <stdexcept>
#include <cast/core/CASTComponent.hpp>
#include <DisplayServer.hpp> // generated from ice
#ifdef HAVE_COGX_VIDEO
#include <Video.hpp> // generated from ice
#endif

// convenient classes for clients
#include "CFormValues.hpp"
#include "CMilliTimer.hpp"

// Optional use of opencv on the client
#ifdef FEAT_VISUALIZATION_OPENCV
# include <highgui.h> // OpenCV - transport an IplImage
# define _IplImagePtr IplImage*
# define _CvMatPtr    CvMat*
#else
# define _IplImagePtr void*
# define _CvMatPtr    void*
#endif

namespace cogx { namespace display {

class CDisplayClient
{
   // The timer is restarted in connectIceClient. If you clear it manually, also clear m_nextSendTime;
   CMilliTimer m_timer;
   std::map<std::string, long long> m_nextSendTime;
   void setRawImageInternal(const std::string& id, int width, int height, int channels,
         const std::vector<unsigned char>& data);
   void setCompressedImageInternal(const std::string& id, const std::vector<unsigned char>& data,
         const std::string &format="");
protected:

   /// if this is not "" then try to connect to a display server outside of CAST on this host
   /// See also: CDisplayServer::m_standaloneHost 
   std::string m_standaloneHost;

   std::string m_serverName;
   Visualization::DisplayInterfacePrx m_pServer;
   cast::CASTComponent* m_pOwner;

   // If gt 0 it will send images to the server at most every xMs millisecons (per object/part)
   int m_imageSendLocalMs;
   int m_imageSendRemoteMs;
   int m_imageSendMs; // depends on display host; either equal to LocalMs or RemoteMs

   std::string getComponentId() {
      if (! m_pOwner) return "";
      return m_pOwner->getComponentID();
   }
   Ice::Identity getEventClientId() {
      Ice::Identity id;
      id.name = getComponentId();
      id.category = "Visualization.EventReceiver";
      return id;
   }
   void debug(const std::string& message) {
      if (m_pOwner) m_pOwner->debug(message);
   }
   void log(const std::string& message) {
      if (m_pOwner) m_pOwner->log(message);
   }
   void println(const std::string& message) {
      if (m_pOwner) m_pOwner->println(message);
   }

   void connectToStandaloneHost(cast::CASTComponent& owner);

public:
   CDisplayClient();
   virtual ~CDisplayClient();
   void configureDisplayClient(const std::map<std::string,std::string> & _config);

   /// Connect to the client to a display server.
   /// For the connection parameters and procedure see: CDisplayServer::m_standaloneHost.
   void connectIceClient(cast::CASTComponent& owner);

   // -----------------------------------------------------------------
   // CDisplayClient Methods
   // These methods will call the remote server. They should mostly just
   // pass the parameters to the server (m_pServer) and return its results.
   // -----------------------------------------------------------------
public:
   void createView(const std::string& id, Visualization::ViewType type, const std::vector<std::string>& objects);
   void enableDefaultView(const std::string& objectId, bool enable=true);

   // Set image from raw data.
   void setImage(const std::string& id, int width, int height, int channels,
     const std::vector<unsigned char>& data);
#ifdef HAVE_COGX_VIDEO
   void setImage(const std::string& id, const Video::Image& image); 
#endif

   // Set image from compressed/formatted data.
   // Formats: (supported by Qt) bmp,gif,jpeg,jpg,png,pbm,pgm,ppm,tiff,xbm,xpm
   // If no format is specified, Qt will try to detect it from data headers.
   void setImage(const std::string& id, const std::vector<unsigned char>& data, const std::string &format=""); 

   void setObject(const std::string& id, const std::string& partId, const std::string& xmlData); 
   void setLuaGlObject(const std::string& id, const std::string& partId, const std::string& script);
   void setHtml(const std::string& id, const std::string& partId, const std::string& htmlData);
   void setHtmlHead(const std::string& id, const std::string& partId, const std::string& htmlData);
   void setActiveHtml(const std::string& id, const std::string& partId, const std::string& htmlData);
   void setHtmlForm(const std::string& id, const std::string& partId, const std::string& htmlData);
   void setHtmlFormData(const std::string& id, const std::string& partId,
         const std::map<std::string, std::string>& fields);
   void setObjectTransform2D(const std::string& id, const std::string& partId,
         const std::vector<double>& transform);
#ifdef HAVE_COGX_MATH
   void setObjectTransform2D(const std::string& id, const std::string& partId,
         const cogx::Math::Matrix33& transform);
   void setObjectPose3D(const std::string& id, const std::string& partId,
         const cogx::Math::Vector3& position, const Visualization::Quaternion& rotation);
#endif

   void removeObject(const std::string& id);
   void removePart(const std::string& id, const std::string& partId);

   // Events from GUI are only available in CActiveDisplayClient.
   // component-id, view-id, control-id, ...
   void addCheckBox(const std::string& viewId, const std::string& ctrlId, const std::string& label);
   void addButton(const std::string& viewId, const std::string& ctrlId, const std::string& label);
   void addDialog(const std::string& dialogId, const std::string& designCode, const std::string& scriptCode,
         const std::string& constructorName);
   void addAction(const std::string& viewId, const Visualization::ActionInfo& action);

   void execInDialog(const std::string& dialogId, const std::string& script);
   void setImage(const std::string& id, const _IplImagePtr pImage); 
   void setObjectTransform2D(const std::string& id, const std::string& partId, _CvMatPtr pTransform); 

private:
   // -----------------------------------------------------------------
   // An Active Display Client can receive events from a Display Server.
   // -----------------------------------------------------------------
   class CEventReceiverI: public Visualization::EventReceiver
   {
   private:
      CDisplayClient *m_pClient;
   public:
      CEventReceiverI(CDisplayClient *pClient) {
         m_pClient = pClient;
      }
      void handleEvent(const Visualization::TEvent &event, const Ice::Current&) {
         if (m_pClient)
            m_pClient->handleEvent(event);
      }
      std::string getControlState(const std::string& ctrlId, const Ice::Current&) {
         if (m_pClient)
            return m_pClient->getControlState(ctrlId);
         return "";
      }
      void handleForm(const std::string& id, const std::string& partId,
            const std::map<std::string, std::string>& fields, const Ice::Current&)
      {
         if (m_pClient)
            m_pClient->handleForm(id, partId, fields);
      }
      bool getFormData(const std::string& id, const std::string& partId,
            std::map<std::string, std::string>& fields, const Ice::Current&)
      {
         if (m_pClient)
            return m_pClient->getFormData(id, partId, fields);
         return false;
      }
      void onDialogValueChanged(const std::string& dialogId, const std::string& name,
            const std::string& value, const Ice::Current&)
      {
         if (m_pClient)
            m_pClient->onDialogValueChanged(dialogId, name, value);
      }
      void handleDialogCommand(const std::string& dialogId, const std::string& name,
            const std::string& value, const Ice::Current&)
      {
         if (m_pClient)
            m_pClient->handleDialogCommand(dialogId, name, value);
      }
   };

   // To make a Display Client active, the methods to handle events
   // must be implemented in a CDisplayClient subclass and
   // installEventReceiver() must be called.
private:
   Visualization::EventReceiverPtr m_pEventReceiverIceSrv;

public:
   void installEventReceiver() throw(std::runtime_error); 

public:
   virtual void handleEvent(const Visualization::TEvent &event);
   virtual std::string getControlState(const std::string& ctrlId);
   virtual void handleForm(const std::string& id, const std::string& partId,
         const std::map<std::string, std::string>& fields);
   virtual bool getFormData(const std::string& id, const std::string& partId,
         std::map<std::string, std::string>& fields);
   virtual void onDialogValueChanged(const std::string& dialogId, const std::string& name,
         const std::string& value);
   virtual void handleDialogCommand(const std::string& dialogId, const std::string& command,
         const std::string& params);
};



#if 0
// -----------------------------------------------------------------
// An Active Display Client can receive events from a Display Server.
//
// Visualization::EventReceiver interface
// The client needs to subscribe to events and then it will receive
// them through handleEvent. A component that adds GUI elements to
// the server is automatically subscribed to the events from the
// added elements.
// -----------------------------------------------------------------
// template<class T>
class CActiveDisplayClient: public CDisplayClient
{
private:
   class CEventReceiverI: public Visualization::EventReceiver
   {
   private:
      CActiveDisplayClient/*<T>*/ *m_pClient;
   public:
      CEventReceiverI(CActiveDisplayClient/*<T>*/ *pClient) {
         m_pClient = pClient;
      }
      void handleEvent(const Visualization::TEvent &event, const Ice::Current&) { /*override*/
         if (m_pClient) m_pClient->handleEvent(event);
      }
      std::string getControlState(const std::string& ctrlId, const Ice::Current&) { /*override*/
         if (m_pClient) return m_pClient->getControlState(ctrlId);
         return "";
      }
      void handleForm(const std::string& formId, const std::map<std::string, std::string>& fields,
            const Ice::Current&)
      {
         if (m_pClient) m_pClient->handleForm(formId, fields);
      }
      void getFormData(const std::string& formId, std::map<std::string, std::string>& fields,
            const Ice::Current&)
      {
         if (m_pClient) m_pClient->getFormData(formId, fields);
      }
   };

public:
   //typedef void (T::*TEventCallbackFn)(const Visualization::TEvent& event);
   //typedef std::string (T::*TStateQueryFn)(const std::string& ctrlId);
   CActiveDisplayClient() {
      m_pEventReceiverIceSrv = NULL;
      //m_pReceiver = NULL;
      //m_pEventCallback = NULL;
      //m_pStateCallback = NULL;
   }
   ~CActiveDisplayClient() {
      m_pEventReceiverIceSrv = NULL;
   }

private:
   Visualization::EventReceiverPtr m_pEventReceiverIceSrv;
   //T* m_pReceiver; // XXX: m_pOwner could also be used
   //TEventCallbackFn m_pEventCallback;
   //TStateQueryFn m_pStateCallback;

public:
   void installEventReceiver() throw(std::runtime_error) {
      if (! m_pOwner)
         throw std::runtime_error(cast::exceptionMessage(__HERE__,
                  "CDisplayClient: connectIceClient() must be called before installEventReciever()."));

      if (! m_pServer) {
         log("CActiveDisplayClient: server not connected.");
         return;
      }

      debug("CDisplayClient installing an EventReceiver.");
      if (m_pEventReceiverIceSrv.get()) {
         //throw std::runtime_error(cast::exceptionMessage(__HERE__,
         //         "CDisplayClient already has an EventReceiver."));
         log("CActiveDisplayClient already has an EventReceiver.");
         return;
      }

      Ice::Identity id = getEventClientId();
      debug(id.name + id.category);
      m_pEventReceiverIceSrv = new CEventReceiverI(this);
      m_pOwner->registerIceServer<Visualization::EventReceiver>(id.name, id.category, m_pEventReceiverIceSrv);
      m_pServer->addClient(id);
      debug("CDisplayClient EventReceiver installed.");
   }

   //void setEventCallback(T* pReceiver, TEventCallbackFn callback) {
   //   setReceiver(pReceiver);
   //   m_pEventCallback = callback;
   //}

   //void setStateQueryCallback(T* pReceiver, TStateQueryFn callback) {
   //   setReceiver(pReceiver);
   //   m_pStateCallback = callback;
   //}

protected:
   virtual void handleEvent(const Visualization::TEvent &event) {
      //debug(event.data + " (received)");
      //if (m_pReceiver != NULL && m_pEventCallback != NULL) {
      //   debug("ActiveDisplayClient: Passing event to the Receiver component.");
      //   ((m_pReceiver)->*(m_pEventCallback))(event);
      //}
   }

   virtual std::string getControlState(const std::string& ctrlId) { /*override*/
      //debug(ctrlId + " queried");
      //if (m_pReceiver != NULL && m_pStateCallback != NULL) {
      //   debug("ActiveDisplayClient: Querying Receiver component state.");
      //   return ((m_pReceiver)->*(m_pStateCallback))(ctrlId);
      //}
      return "";
   }

   virtual void handleForm(const std::string& formId, const std::map<std::string, std::string>& fields)
   {
      //debug(formId + " (handleForm)");
   }

   virtual void getFormData(const std::string& formId, std::map<std::string, std::string>& fields)
   {
      //debug(formId + " (getFormData)");
   }

//private:
//   void setReceiver(T* pReceiver) {
//      if (pReceiver == NULL && m_pReceiver == NULL) {
//         throw std::runtime_error(cast::exceptionMessage(__HERE__,
//                  "CActiveDisplayClient: A receiver must be set for the callback."));
//      }
//      if (m_pReceiver != NULL && pReceiver != m_pReceiver) {
//         throw std::runtime_error(cast::exceptionMessage(__HERE__,
//                  "CActiveDisplayClient: All callbacks must have the same receiver."));
//      }
//      if(m_pReceiver == NULL) {
//         m_pReceiver = pReceiver;
//         if (m_pReceiver != m_pOwner) {
//            debug("ActiveDisplayClient: WARNING: Event reciever is not the owner of the display client.");
//         }
//      }
//   }
};
#endif

}} // namespace
#endif // include once
// vim:sw=3:ts=8:et
