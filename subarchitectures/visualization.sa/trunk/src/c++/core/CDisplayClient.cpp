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

#include <sstream>
#include <sstream>
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <cast/core/CASTUtils.hpp>

using namespace std;
using namespace cast;

namespace cogx { namespace display {

CDisplayClient::CDisplayClient()
{
   m_serverName = "display.srv";
   m_pServer = NULL;
   m_pOwner = NULL;
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
}

void CDisplayClient::connectIceClient(CASTComponent& owner)
      throw(runtime_error)
{
   m_pOwner = &owner;

   owner.debug("CDisplayClient connecting to CDisplayServer.");
   if (m_pServer) {
      //throw runtime_error(exceptionMessage(__HERE__,
      //      "CDisplayClient already connected to server."));
     owner.log("CDisplayClient already connected to server.");
   }

   if (m_serverName.empty()) {
      //throw runtime_error(exceptionMessage(__HERE__,
      //      "DisplayServer server id not set. Use --display-server-id."));
      owner.println(" *** DisplayServer server id not set. Use --displayserver.");
   }

   try {
     m_pServer = owner.getIceServer<Visualization::DisplayInterface>(m_serverName);
     owner.debug("CDisplayClient Connected.");
   }
   catch (...) {
     owner.debug(" *** CDisplayClient could not connect to '%s'.", m_serverName.c_str());
   }
}

void CDisplayClient::setImage(const std::string& id, const Video::Image& image)
{
   if (m_pServer == NULL) return;
   m_pServer->setImage(id, image);
}

void CDisplayClient::setImage(const std::string& id, const std::vector<unsigned char>& data,
    const std::string &format)
{
   if (m_pServer == NULL) return;
   m_pServer->setCompressedImage(id, data, format);
}

#ifndef FEAT_VISUALIZATION_OPENCV
#error "FEAT_VISUALIZATION_OPENCV must be defined during v11n build."
#endif
void CDisplayClient::setImage(const std::string& id, const IplImage* pImage) 
{
   if (m_pServer == NULL) return;
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

   m_pServer->setRawImage(id, pImage->width, pImage->height, pImage->nChannels, data);
}

void CDisplayClient::setObject(const std::string& id, const std::string& partId, const std::string& xmlData)
{
   if (m_pServer == NULL) return;
   m_pServer->setObject(id, partId, xmlData);
}

void CDisplayClient::setLuaGlObject(const std::string& id, const std::string& partId, const std::string& script)
{
   if (m_pServer == NULL) return;
   m_pServer->setLuaGlObject(id, partId, script);
}

void CDisplayClient::setObjectTransform2D(const std::string& id, const std::string& partId,
       const std::vector<double>& transform)
{
   if (m_pServer == NULL) return;
   m_pServer->setObjectTransform2D(id, partId, transform);
}

void CDisplayClient::setObjectTransform2D(const std::string& id, const std::string& partId,
      const cogx::Math::Matrix33& transform)
{
   if (m_pServer == NULL) return;
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

#ifndef FEAT_VISUALIZATION_OPENCV
#error "FEAT_VISUALIZATION_OPENCV must be defined during v11n build."
#endif
void CDisplayClient::setObjectTransform2D(const std::string& id, const std::string& partId, CvMat* pTransform)
{
   if (m_pServer == NULL) return;
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

void CDisplayClient::setObjectPose3D(const std::string& id, const std::string& partId,
         const cogx::Math::Vector3& position, const Visualization::Quaternion& rotation)
{
   if (m_pServer == NULL) return;
   m_pServer->setObjectPose3D(id, partId, position, rotation);
}

void CDisplayClient::addCheckBox(const std::string& viewId, const std::string& ctrlId, const std::string& label)
{
   if (m_pServer == NULL) return;
   Ice::Identity id = getEventClientId();
   m_pServer->addCheckBox(id, viewId, ctrlId, label);
}

void CDisplayClient::addButton(const std::string& viewId, const std::string& ctrlId, const std::string& label)
{
   if (m_pServer == NULL) return;
   Ice::Identity id = getEventClientId();
   m_pServer->addButton(id, viewId, ctrlId, label);
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
   m_pServer->addClient(id);
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
   if (m_pOwner != NULL && m_pEventCallback != NULL) {
      m_pOwner->debug("Receiver: Passing event to owner");
      ((m_pOwner)->*(m_pEventCallback))(event);
   }
}
#endif

} } // namespace

