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

#ifdef FEAT_VISUALIZATION_OPENCV
# include <highgui.h> // OpenCV - transport an IplImage
#endif

#include <DisplayServer.hpp> // generated from ice

namespace cogx { namespace display {

class CDisplayClient
{
protected:
   std::string m_serverName;
   Visualization::DisplayInterfacePrx m_pServer;
   cast::CASTComponent* m_pOwner;

   std::string getComponentId() {
      if (! m_pOwner) return "";
      return m_pOwner->getComponentID();
   }
   Ice::Identity getEventClientId() {
      Ice::Identity id;
      id.name = getComponentId();
      id.category = "Visualization_EventReceiver";
      return id;
   }
   void debug(const std::string& message) {
      if (m_pOwner) m_pOwner->debug(message);
   }
   void log(const std::string& message) {
      if (m_pOwner) m_pOwner->log(message);
   }

public:
   CDisplayClient();
   virtual ~CDisplayClient();
   void configureDisplayClient(const std::map<std::string,std::string> & _config);
   void connectIceClient(cast::CASTComponent& owner) throw(std::runtime_error);

   // -----------------------------------------------------------------
   // CDisplayClient Methods
   // These methods will call the remote server. They should mostly just
   // pass the parameters to the server (m_pServer) and return its results.
   // -----------------------------------------------------------------
public:
   // Set image from raw data.
   void setImage(const std::string& id, int width, int height, int channels,
     const std::vector<unsigned char>& data);
   void setImage(const std::string& id, const Video::Image& image); 

   // Set image from compressed/formatted data.
   // Formats: (supported by Qt) bmp,gif,jpeg,jpg,png,pbm,pgm,ppm,tiff,xbm,xpm
   // If no format is specified, Qt will try to detect it from data headers.
   void setImage(const std::string& id, const std::vector<unsigned char>& data, const std::string &format=""); 

   void setObject(const std::string& id, const std::string& partId, const std::string& xmlData); 
   void setLuaGlObject(const std::string& id, const std::string& partId, const std::string& script);
   void setHtml(const std::string& id, const std::string& partId, const std::string& htmlData);
   void setObjectTransform2D(const std::string& id, const std::string& partId,
         const cogx::Math::Matrix33& transform);
   void setObjectTransform2D(const std::string& id, const std::string& partId,
         const std::vector<double>& transform);
   void setObjectPose3D(const std::string& id, const std::string& partId,
         const cogx::Math::Vector3& position, const Visualization::Quaternion& rotation);

   // Events from GUI are only available in CActiveDisplayClient.
   // component-id, view-id, control-id, ...
   void addCheckBox(const std::string& viewId, const std::string& ctrlId, const std::string& label);
   void addButton(const std::string& viewId, const std::string& ctrlId, const std::string& label);

#ifdef FEAT_VISUALIZATION_OPENCV
   void setImage(const std::string& id, const IplImage* pImage); 
   void setObjectTransform2D(const std::string& id, const std::string& partId, CvMat* pTransform); 
#endif
};


#if 1
// -----------------------------------------------------------------
// An Active Display Client can receive events from a Display Server.
//
// Visualization::EventReceiver interface
// The client needs to subscribe to events and then it will receive
// them through handleEvent. A component that adds GUI elements to
// the server is automatically subscribed to the events from the
// added elements.
// -----------------------------------------------------------------
template<class T>
class CActiveDisplayClient: public CDisplayClient
{
private:
   class CEventReceiverI: public Visualization::EventReceiver
   {
   private:
      CActiveDisplayClient<T> *m_pClient;
   public:
      CEventReceiverI(CActiveDisplayClient<T> *pClient) {
         m_pClient = pClient;
      }
      void handleEvent(const Visualization::TEvent &event, const Ice::Current&) { /*override*/
         if (m_pClient) m_pClient->handleEvent(event);
      }
      std::string getControlState(const std::string& ctrlId, const Ice::Current&) { /*override*/
         if (m_pClient) return m_pClient->getControlState(ctrlId);
         return "";
      }
   };

public:
   typedef void (T::*TEventCallbackFn)(const Visualization::TEvent& event);
   typedef std::string (T::*TStateQueryFn)(const std::string& ctrlId);
   CActiveDisplayClient() {
      m_pEventReceiverIceSrv = NULL;
      m_pReceiver = NULL;
      m_pEventCallback = NULL;
      m_pStateCallback = NULL;
   }
   ~CActiveDisplayClient() {
      m_pEventReceiverIceSrv = NULL;
   }

private:
   Visualization::EventReceiverPtr m_pEventReceiverIceSrv;
   T* m_pReceiver; // XXX: m_pOwner could also be used
   TEventCallbackFn m_pEventCallback;
   TStateQueryFn m_pStateCallback;

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

   void setEventCallback(T* pReceiver, TEventCallbackFn callback) {
      setReceiver(pReceiver);
      m_pEventCallback = callback;
   }

   void setStateQueryCallback(T* pReceiver, TStateQueryFn callback) {
      setReceiver(pReceiver);
      m_pStateCallback = callback;
   }

protected:
   virtual void handleEvent(const Visualization::TEvent &event) {
      debug(event.data + " (received)");
      if (m_pReceiver != NULL && m_pEventCallback != NULL) {
         debug("ActiveDisplayClient: Passing event to the Receiver component.");
         ((m_pReceiver)->*(m_pEventCallback))(event);
      }
   }

   virtual std::string getControlState(const std::string& ctrlId) { /*override*/
      debug(ctrlId + " queried");
      if (m_pReceiver != NULL && m_pStateCallback != NULL) {
         debug("ActiveDisplayClient: Querying Receiver component state.");
         return ((m_pReceiver)->*(m_pStateCallback))(ctrlId);
      }
      return "";
   }

private:
   void setReceiver(T* pReceiver) {
      if (pReceiver == NULL && m_pReceiver == NULL) {
         throw std::runtime_error(cast::exceptionMessage(__HERE__,
                  "CActiveDisplayClient: A receiver must be set for the callback."));
      }
      if (m_pReceiver != NULL && pReceiver != m_pReceiver) {
         throw std::runtime_error(cast::exceptionMessage(__HERE__,
                  "CActiveDisplayClient: All callbacks must have the same receiver."));
      }
      if(m_pReceiver == NULL) {
         m_pReceiver = pReceiver;
         if (m_pReceiver != m_pOwner) {
            debug("ActiveDisplayClient: WARNING: Event reciever is not the owner of the display client.");
         }
      }
   }
#endif
};

} } // namespace
#endif // include once
