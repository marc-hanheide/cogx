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
#include "CDisplayClient.hpp"

#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <cast/core/CASTUtils.hpp>
#include <cast/core/CASTComponent.hpp>

#include <sstream>

using namespace std;
using namespace cast;

namespace cogx { namespace display {


CDisplayClient::CDisplayClient()
{
   m_standaloneHost = "";
   m_serverName = "display.srv";
   m_pServer = nullptr;
   m_pOwner = nullptr;
   m_imageSendLocalMs = 0;
   m_imageSendRemoteMs = 0;
   m_imageSendMs = 0;
}

CDisplayClient::~CDisplayClient()
{
}

void CDisplayClient::configureDisplayClient(const map<string,string> & _config)
{
   map<string,string>::const_iterator it;

   if((it = _config.find("--displayserver")) != _config.end()) {
      m_serverName = it->second;
   }

   if((it = _config.find("--standalone-display-host")) != _config.end()) {
      string s = it->second;
      // trim
      s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
      if (s == "/no") s = "";
      m_standaloneHost = s;      
   }

   if((it = _config.find("--image-send-ms")) != _config.end()) {
      string s = it->second;
      istringstream ss(s);
      ss >> m_imageSendLocalMs;
      ss >> m_imageSendRemoteMs;
      if (m_imageSendLocalMs < 0) m_imageSendLocalMs = 0;
      if (m_imageSendRemoteMs < m_imageSendLocalMs) m_imageSendRemoteMs = m_imageSendLocalMs;
      
      // std::cout << "*** local " << m_imageSendLocalMs << " *** remote " << m_imageSendRemoteMs << std::endl;
   }
}

void CDisplayClient::connectToStandaloneHost(CASTComponent &owner)
{
   owner.log("CDisplayClient connecting to standalone server.");
   try {

      Ice::ObjectPrx prx = owner.getIceServer(Visualization::V11NSTANDALONENAME,
            owner.toServantCategory<Visualization::DisplayInterface>(),
            m_standaloneHost, Visualization::V11NSTANDALONEPORT);

      m_pServer = Visualization::DisplayInterfacePrx::checkedCast(prx);
      owner.println("CDisplayClient connected to standalone server on '%s'.",
            m_standaloneHost.c_str());
   }
   catch (...) {
      owner.println(" *** CDisplayClient could not connect standalone server on '%s'.",
            m_standaloneHost.c_str());
   }
}

void CDisplayClient::connectIceClient(CASTComponent& owner)
{
   m_pOwner = &owner;
   m_timer.restart();

   owner.debug("CDisplayClient connecting to CDisplayServer.");
   if (m_pServer) {
      //throw runtime_error(exceptionMessage(__HERE__,
      //      "CDisplayClient already connected to server."));
      owner.log("CDisplayClient already connected to a display server.");
   }

   //look for a standalone host if this is set
   if(!m_standaloneHost.empty()) {
      connectToStandaloneHost(owner);
   }
   else {
      if (m_serverName.empty()) {
         //throw runtime_error(exceptionMessage(__HERE__,
         //      "DisplayServer server id not set. Use --displayserver"));
         owner.println(" *** DisplayServer server id not set. Use --displayserver.");
      }

      try {
         m_pServer = owner.getIceServer<Visualization::DisplayInterface>(m_serverName);
      }
      catch (...) {
         owner.println(" *** CDisplayClient could not connect to '%s'.", m_serverName.c_str());
         m_pServer = nullptr;
      }

      if (m_pServer.get() != nullptr) {
         try {
            m_pServer = owner.getIceServer<Visualization::DisplayInterface>(m_serverName);
            m_pServer->getStandaloneHost(m_standaloneHost);
            if(m_standaloneHost.empty())
               owner.debug("CDisplayClient Connected.");
            else {
               m_pServer = nullptr;
               owner.debug("CDisplayClient Redirecting connection to standalone display server.");
               connectToStandaloneHost(owner);
            }
         }
         catch (...) {
            owner.println("CDisplayClient: Unknown error during getStandaloneHost/redirection.");
         }
      }
   }
   owner.debug("connectIceClient finished in %lldms.", m_timer.elapsed());

   // TODO: set m_imageSendMs from values of the host of the client and the host of the server
   // XXX default to local
   m_imageSendMs = m_imageSendLocalMs;
}

void CDisplayClient::installEventReceiver() throw(std::runtime_error)
{
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
   const string& myHost(m_pOwner->getComponentManager()->getComponentDescription(m_pOwner->getComponentID()).hostName);      
   m_pServer->addClient(id, myHost, cast::cdl::CPPSERVERPORT);
   debug("CDisplayClient EventReceiver installed.");
}

void CDisplayClient::handleEvent(const Visualization::TEvent &event)
{
   debug("handleEvent NOT IMPLEMENTED");
}

std::string CDisplayClient::getControlState(const std::string& ctrlId)
{
   debug("getControlState NOT IMPLEMENTED");
   return "";
}

void CDisplayClient::handleForm(const std::string& id, const std::string& partId,
      const std::map<std::string, std::string>& fields)
{
   debug("handleForm NOT IMPLEMENTED");
}

bool CDisplayClient::getFormData(const std::string& id, const std::string& partId,
      std::map<std::string, std::string>& fields)
{
   debug("getFormData NOT IMPLEMENTED");
   return false;
}

void CDisplayClient::onDialogValueChanged(const std::string& dialogId, const std::string& name,
      const std::string& value)
{
   if (m_pOwner)
      m_pOwner->debug("onDialogValueChanged NOT IMPLEMENTED. %s[%s]='%s'",
            dialogId.c_str(), name.c_str(), value.c_str());
}

void CDisplayClient::handleDialogCommand(const std::string& dialogId, const std::string& name,
      const std::string& value)
{
   if (m_pOwner)
      m_pOwner->debug("handleDialogCommand NOT IMPLEMENTED. %s#%s(%s)",
            dialogId.c_str(), name.c_str(), value.c_str());
}

void CDisplayClient::createView(const std::string& id, Visualization::ViewType type,
      const std::vector<std::string>& objects)
{
   if (! m_pServer) return;
   m_pServer->createView(id, type, objects);
}

void CDisplayClient::enableDefaultView(const std::string& objectid, bool enable)
{
   if (! m_pServer) return;
   m_pServer->enableDefaultView(objectid, enable);
}

void CDisplayClient::setRawImageInternal(const std::string& id, int width, int height, int channels,
      const std::vector<unsigned char>& data)
{
   if (! m_pServer) return;
   long long nexttm = 0;
   if (m_imageSendMs > 0) {
      auto it = m_nextSendTime.find(id);
      if (it != m_nextSendTime.end()) nexttm = it->second;
      long long now = m_timer.elapsed();
      if (now < nexttm) return; // XXX: should cache the image and send it at nexttm; it's too much work for now.
      if (now - nexttm < m_imageSendMs) nexttm = nexttm + m_imageSendMs;
      else nexttm = now + m_imageSendMs;
      m_nextSendTime[id] = nexttm;
   }
   m_pServer->setRawImage(id, width, height, channels, data);
}

void CDisplayClient::setCompressedImageInternal(const std::string& id,
      const std::vector<unsigned char>& data, const std::string &format)
{
   if (! m_pServer) return;
   long long nexttm = 0;
   if (m_imageSendMs > 0) {
      auto it = m_nextSendTime.find(id);
      if (it != m_nextSendTime.end()) nexttm = it->second;
      long long now = m_timer.elapsed();
      if (now < nexttm) return; // XXX: should cache the image and send it at nexttm; it's too much work for now.
      if (now - nexttm < m_imageSendMs) nexttm = nexttm + m_imageSendMs;
      else nexttm = now + m_imageSendMs;
      m_nextSendTime[id] = nexttm;
   }
   m_pServer->setCompressedImage(id, data, format);
}

#ifdef HAVE_COGX_VIDEO
void CDisplayClient::setImage(const std::string& id, const Video::Image& image)
{
   if (! m_pServer) return;
   setRawImageInternal(id, image.width, image.height, 3, image.data);
}
#endif

void CDisplayClient::setImage(const std::string& id, const std::vector<unsigned char>& data,
      const std::string &format)
{
   if (! m_pServer) return;
   setCompressedImageInternal(id, data, format);
}

void CDisplayClient::setImage(const std::string& id, int width, int height, int channels,
      const std::vector<unsigned char>& data)
{
   if (! m_pServer) return;
   if (width * height * channels != data.size()) {
      m_pOwner->println(" *** CDisplayClient: raw image size doesn't match the size of the data.");
      return;
   }
   if (channels != 1 && channels != 3) {
      m_pOwner->println(" *** CDisplayClient: only 1 and 3 channel raw images are supported.");
   }
   setRawImageInternal(id, width, height, channels, data);
}

#ifndef FEAT_VISUALIZATION_OPENCV
#error "FEAT_VISUALIZATION_OPENCV must be defined during v11n build."
#endif
void CDisplayClient::setImage(const std::string& id, const IplImage* pImage) 
{
   if (! m_pServer) return;
   if (! pImage) return;
   int nbytes = pImage->width * pImage->height * pImage->nChannels;
   if (nbytes < 1) return;
   int step = pImage->widthStep;
   unsigned char* pIplData = (unsigned char*) pImage->imageData;

   std::vector<unsigned char> data;
   data.resize(nbytes);
   unsigned char* pvdata = (unsigned char*) &(data[0]);

   int rowlen = pImage->width * pImage->nChannels;
   for(int i=0; i < pImage->height; i++) {
      unsigned char* piplrow = pIplData + i * step;
      memcpy(pvdata, piplrow, rowlen);
      pvdata += rowlen;
      //Alternative:
      //for(int j=0; j < pImage->width; j++) {
      //   for(k=0; k < pImage->nChannels; k++) {
      //      *pvdata++ = *pIplData++
      //   }
      //}
   }

   setRawImageInternal(id, pImage->width, pImage->height, pImage->nChannels, data);
   data.resize(0);
}

void CDisplayClient::setObject(const std::string& id, const std::string& partId, const std::string& xmlData)
{
   if (! m_pServer) return;
   m_pServer->setObject(id, partId, xmlData);
}

void CDisplayClient::setLuaGlObject(const std::string& id, const std::string& partId, const std::string& script)
{
   if (! m_pServer) return;
   m_pServer->setLuaGlObject(id, partId, script);
}

void CDisplayClient::setHtml(const std::string& id, const std::string& partId, const std::string& htmlData)
{
   if (! m_pServer) return;
   m_pServer->setHtml(id, partId, htmlData);
}

void CDisplayClient::setHtmlHead(const std::string& id, const std::string& partId, const std::string& htmlData)
{
   if (! m_pServer) return;
   m_pServer->setHtmlHead(id, partId, htmlData);
}

void CDisplayClient::setActiveHtml(const std::string& id, const std::string& partId, const std::string& htmlData)
{
   if (! m_pServer) return;
   Ice::Identity iceid = getEventClientId();
   m_pServer->setActiveHtml(iceid, id, partId, htmlData);
}

void CDisplayClient::setHtmlForm(const std::string& id, const std::string& partId, const std::string& htmlData)
{
   if (! m_pServer) return;
   Ice::Identity iceid = getEventClientId();
   m_pServer->setHtmlForm(iceid, id, partId, htmlData);
}

void CDisplayClient::setHtmlFormData(const std::string& id, const std::string& partId,
      const std::map<std::string, std::string>& fields)
{
   if (! m_pServer) return;
   m_pServer->setHtmlFormData(id, partId, fields);
}

void CDisplayClient::setObjectTransform2D(const std::string& id, const std::string& partId,
      const std::vector<double>& transform)
{
   if (! m_pServer) return;
   m_pServer->setObjectTransform2D(id, partId, transform);
}

#ifdef HAVE_COGX_MATH
void CDisplayClient::setObjectTransform2D(const std::string& id, const std::string& partId,
      const cogx::Math::Matrix33& transform)
{
   if (! m_pServer) return;
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
   m_pServer->setObjectTransform2D(id, partId, tr);
}
#endif

#ifndef FEAT_VISUALIZATION_OPENCV
#error "FEAT_VISUALIZATION_OPENCV must be defined during v11n build."
#endif
void CDisplayClient::setObjectTransform2D(const std::string& id, const std::string& partId, CvMat* pTransform)
{
   if (! m_pServer) return;
   if (!pTransform) return;
   if (pTransform->rows < 2 || pTransform->cols < 2) return;

   std::vector<double> tr;

   for (int ir = 0; ir < 2; ir++) {
      for (int j = 0; j < 2; j++) tr.push_back(cvmGet(pTransform, ir, j));
      if (pTransform->cols < 3) tr.push_back(0);
      else tr.push_back(cvmGet(pTransform, ir, 2));
   }
   if (pTransform->rows < 3) {
      tr.push_back(0);
      tr.push_back(0);
      tr.push_back(1);
   }
   else {
      for (int j = 0; j < 2; j++) tr.push_back(cvmGet(pTransform, 2, j));
      if (pTransform->cols < 3) tr.push_back(1);
      else tr.push_back(cvmGet(pTransform, 2, 2));
   }

   //cogx::Math::Matrix33 mat;
   //mat.m00 = cvmGet(pTransform, 0, 0);
   //mat.m01 = cvmGet(pTransform, 0, 1);
   //mat.m10 = cvmGet(pTransform, 1, 0);
   //mat.m11 = cvmGet(pTransform, 1, 1);
   //if (pTransform->cols > 2) {
   //   mat.m02 = 0; //cvmGet(pTransform, 0, 2);
   //   mat.m12 = 0; //cvmGet(pTransform, 1, 2);
   //}
   //else {
   //   mat.m02 = 0;
   //   mat.m12 = 0;
   //}
   //if (pTransform->rows > 2) {
   //   mat.m20 = cvmGet(pTransform, 2, 0);
   //   mat.m21 = cvmGet(pTransform, 2, 1);
   //   if (pTransform->cols > 2) {
   //      mat.m22 = cvmGet(pTransform, 2, 2);
   //   }
   //   else {
   //      mat.m22 = 1;
   //   }
   //}
   //else {
   //   mat.m20 = 0;
   //   mat.m21 = 0;
   //   mat.m22 = 1;
   //}
   m_pServer->setObjectTransform2D(id, partId, tr);
}

#ifdef HAVE_COGX_MATH
void CDisplayClient::setObjectPose3D(const std::string& id, const std::string& partId,
      const cogx::Math::Vector3& position, const Visualization::Quaternion& rotation)
{
   if (! m_pServer) return;
   m_pServer->setObjectPose3D(id, partId, position.x, position.y, position.z, rotation);
}
#endif

void CDisplayClient::removeObject(const std::string& id)
{
   if (! m_pServer) return;
   m_pServer->removeObject(id);
}

void CDisplayClient::removePart(const std::string& id, const std::string& partId)
{
   if (! m_pServer) return;
   m_pServer->removePart(id, partId);
}

void CDisplayClient::addCheckBox(const std::string& viewId, const std::string& ctrlId, const std::string& label)
{
   if (! m_pServer) return;
   Ice::Identity id = getEventClientId();
   m_pServer->addCheckBox(id, viewId, ctrlId, label);
}

void CDisplayClient::addButton(const std::string& viewId, const std::string& ctrlId, const std::string& label)
{
   if (! m_pServer) return;
   Ice::Identity id = getEventClientId();
   m_pServer->addButton(id, viewId, ctrlId, label);
}

void CDisplayClient::addDialog(const std::string& dialogId, const std::string& uiCode, const std::string& jsCode,
      const std::string& ctorName)
{
   if (! m_pServer) return;
   Ice::Identity id = getEventClientId();
   m_pServer->addDialog(id, dialogId, uiCode, jsCode, ctorName);
}

void CDisplayClient::execInDialog(const std::string& dialogId, const std::string& script)
{
   if (! m_pServer) return;
   m_pServer->execInDialog(dialogId, script);
}

void CDisplayClient::addAction(const std::string& viewId, const Visualization::ActionInfo& action)
{
   if (! m_pServer) return;
   Ice::Identity id = getEventClientId();
   m_pServer->addAction(id, viewId, action);
}


#if 0
   void CActiveDisplayClient::installEventReceiver()
throw(std::runtime_error)
{
   if (! m_pOwner || ! m_pServer.get())
      throw runtime_error(exceptionMessage(__HERE__,
               "CDisplayClient: connectIceClient() must be called before installEventReciever()."));

   m_pOwner->debug("CDisplayClient installing an EventReceiver.");
   if (m_pEventReceiverIceSrv.get())
      throw runtime_error(exceptionMessage(__HERE__,
               "CDisplayClient already has an EventReceiver."));

   Ice::Identity id;
   id.name = m_pOwner->getComponentID();
   id.category = "Visualization_EventReceiver";
   m_pEventReceiverIceSrv = new CEventReceiverI(this);
   m_pOwner->registerIceServer<Visualization::EventReceiver>(id.name, id.category, m_pEventReceiverIceSrv);
   const string & myHost(getComponentManager()->getComponentDescription(getComponentID()).hostName);
   m_pServer->addClient(id, myHost, cast::cdl::CPPSERVERPORT);
   m_pOwner->debug("CDisplayClient EventReceiver installed.");
}

void CActiveDisplayClient::setEventCallback(cast::CASTComponent* pReceiver, TEventCallbackFn callback)
{
   m_pReceiver = pReceiver;
   m_pEventCallback = callback;
}

void CActiveDisplayClient::handleEvent(const Visualization::TEvent& event)
{
   m_pOwner->debug(event.data + " (received)");
   if (m_pOwner != NULL && m_pEventCallback != nullptr) {
      m_pOwner->debug("Receiver: Passing event to owner");
      ((m_pOwner)->*(m_pEventCallback))(event);
   }
}
#endif

}} // namespace
// vim:sw=3:ts=8:et

