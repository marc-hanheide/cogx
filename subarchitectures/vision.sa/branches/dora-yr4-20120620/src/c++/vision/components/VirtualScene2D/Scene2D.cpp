/*
 * Author: Marko Mahnič
 * Created: 2011-01-12
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
#include "Scene2D.h"

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <sstream>
#include <iomanip>
#include <iterator>
#include <algorithm>
#include <cv.h>

#if defined(HAS_LIBPLOT)
#include <CSvgPlotter.hpp> // libplot-dev, wrapper
#endif

extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cogx::vision::CScene2D();
  }
}

using namespace VisionData;
using namespace cast;

namespace cogx { namespace vision {

void CScene2D::CDisplayClient::handleEvent(const Visualization::TEvent &event)
{
}

std::string CScene2D::CDisplayClient::getControlState(const std::string& ctrlId)
{
   return "";
}

void CScene2D::CDisplayClient::handleForm(const std::string& id,
      const std::string& partId, const std::map<std::string, std::string>& fields)
{
}

bool CScene2D::CDisplayClient::getFormData(const std::string& id,
      const std::string& partId, std::map<std::string, std::string>& fields)
{
   return false;
}

CScene2D::CScene2D()
{
}

void CScene2D::configure(const std::map<std::string,std::string> & _config)
{
   m_display.configureDisplayClient(_config);
   std::map<std::string,std::string>::const_iterator it;

   m_objectList = "video.viewer";
   if ((it = _config.find("--v11n-objects")) != _config.end()) {
      m_objectList = it->second;
   }

   // TODO: ADD PARAMETER. The size of the video image so that we can draw in the right scale
   m_outputWidth = 640;
   m_outputHeight = 480;
}

#define OBJ_VISUAL_OBJECTS "scene2d.VisualObjects"

void CScene2D::start()
{
   addChangeFilter(createGlobalTypeFilter<VisualObject>(cdl::ADD),
         new MemberFunctionChangeReceiver<CScene2D>(this, &CScene2D::onAdd_VisualObject));
   addChangeFilter(createGlobalTypeFilter<VisualObject>(cdl::OVERWRITE),
         new MemberFunctionChangeReceiver<CScene2D>(this, &CScene2D::onChange_VisualObject));
   addChangeFilter(createGlobalTypeFilter<VisualObject>(cdl::DELETE),
         new MemberFunctionChangeReceiver<CScene2D>(this, &CScene2D::onDelete_VisualObject));

   //addChangeFilter(createGlobalTypeFilter<SOI>(cdl::ADD),
   //      new MemberFunctionChangeReceiver<CScene2D>(this, &CScene2D::onAdd_SOI));
   //addChangeFilter(createGlobalTypeFilter<SOI>(cdl::OVERWRITE),
   //      new MemberFunctionChangeReceiver<CScene2D>(this, &CScene2D::onChange_SOI));
   //addChangeFilter(createGlobalTypeFilter<SOI>(cdl::DELETE),
   //      new MemberFunctionChangeReceiver<CScene2D>(this, &CScene2D::onDelete_SOI));

   m_display.connectIceClient(*this);
   m_display.installEventReceiver();

   std::vector<std::string> objects;
   std::istringstream iss(m_objectList);
   std::string idobj;

   while (iss.good() && !iss.eof()) {
      iss >> idobj;
      if (idobj != "")
         objects.push_back(idobj);
   }

   objects.push_back(OBJ_VISUAL_OBJECTS);
   m_display.createView("VirtualScene2D", Visualization::VtGraphics, objects);
}

IplImage* wrapMask(const VisionData::SegmentMask &img)
{
   // HACK: Data shared between IplImage and vector
   IplImage* pImg = cvCreateImageHeader(cvSize(img.width, img.height), IPL_DEPTH_8U, 1);
   if (! pImg)
      return pImg;
   pImg->imageData = (char*) &(img.data[0]);
   pImg->imageDataOrigin = pImg->imageData;
   pImg->widthStep = img.width;
   pImg->imageSize = pImg->widthStep * img.height;
   return pImg;
}

void releaseWrappedImage(IplImage** pImagePtr)
{
   if (! pImagePtr) return;
   if (*pImagePtr) {
      // HACK: Data shared between IplImage and vector; the data will be released with Video::Image
      (*pImagePtr)->imageData = NULL;
      (*pImagePtr)->imageDataOrigin = NULL;
      cvReleaseImage(pImagePtr);
   }
}

#if 1
#define YY(y) -(y)
#else
#define YY(y) y
#endif
#if defined(HAS_LIBPLOT)
void drawContours(Plotter& p, double x0, double y0, vector<vector<cv::Point> >&contours)
{
   vector<vector<cv::Point> >::iterator itpart;
   vector<cv::Point>::iterator itpoint;

   for (itpart = contours.begin(); itpart != contours.end(); itpart++) {
      itpoint = itpart->begin();
      p.fmove(itpoint->x + x0, YY(itpoint->y + y0));
      itpoint++;
      while (itpoint != itpart->end()) {
         p.fcont(itpoint->x + x0, YY(itpoint->y + y0));
         itpoint++;
      }
      itpoint = itpart->begin();
      if (itpoint != itpart->end()) {
         p.fcont(itpoint->x + x0, YY(itpoint->y + y0));
      }
      p.endsubpath();
   }
   p.endpath();
}
#endif

void CScene2D::drawVisualObject(const std::string& id, const VisualObjectPtr& pVisObj,
      const ProtoObjectPtr& pProtoObj)
{
#if defined(HAS_LIBPLOT)
   struct _local_ {
      static std::string getBestLabel(std::vector<std::string>& labels, std::vector<double>& values)
      {
         if (labels.size() < 1) return "";
         if (values.size() < 1) return "";
         std::vector<double>::iterator it = std::max_element(values.begin(), values.end());
         int i = std::distance(values.begin(), it);
         if (i > labels.size()) return "";
         ostringstream ss;
         ss << std::fixed << std::setprecision(2) << values[i] << " " << labels[i];
         return ss.str();
      }
   };
   std::string ident = _local_::getBestLabel(pVisObj->identLabels, pVisObj->identDistrib);
   std::string color = _local_::getBestLabel(pVisObj->colorLabels, pVisObj->colorDistrib);
   std::string shape = _local_::getBestLabel(pVisObj->shapeLabels, pVisObj->shapeDistrib);

   vector<vector<cv::Point> > contours;
   vector<cv::Vec4i> hierarchy;
   IplImage* pImg = 0;
   try {
      pImg = wrapMask(pProtoObj->mask);
   }
   catch (...) {
      println("wrapMask FAILED");
      sleepComponent(100);
      throw;
   }
   if (pImg) {
      cv::Mat imgMat(pImg); // XXX copy=true crashes; so we modify the original data.

      // XXX: mask created in SOIFilter: 1-object, 2-background.
      // findContours requires 0 for background, everything else is the object.
      for (int i=0; i< pProtoObj->mask.height; i++) {
         unsigned char *prow = imgMat.ptr(i);
         unsigned char *pend = prow + pProtoObj->mask.width;
         while(prow < pend) {
            if (*prow != 1) *prow = 0;
            prow++;
         }
      }
      try {
         cv::findContours( imgMat, contours, hierarchy,
               CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
      }
      catch (...) {
         println("cv::findContours FAILED");
         sleepComponent(100);
         throw;
      }
      releaseWrappedImage(&pImg);
   }

   double W = pProtoObj->sourceImageSize.x;
   double H = pProtoObj->sourceImageSize.y;
   double x0 = pProtoObj->imageOrigin.x;
   double y0 = pProtoObj->imageOrigin.y;
   double w0 = pProtoObj->mask.width;
   double h0 = pProtoObj->mask.height;
   double ps = W / m_outputWidth;
   std::ostringstream ss;
   cogx::display::CSvgStringPlotter p(ss);
   p.openpl();
   p.fscale(W / m_outputWidth, H / m_outputHeight);
   p.flinewidth(ps * 1.0);

   p.pencolorname("red");
   if (pImg)
      drawContours(p, x0, y0, contours);

   double true_size = p.fontsize(ps * 12);
   double dy = 0;

   p.fontname("sans-serif");
   p.pencolorname("yellow");
   p.fframedtext(x0, YY(y0+h0/2+dy), id);
   dy += true_size;
   if (ident != "") {
      p.fframedtext(x0, YY(y0+h0/2+dy), ident);
      dy += true_size;
   }
   if (color != "") {
      p.fframedtext(x0, YY(y0+h0/2+dy), color);
      dy += true_size;
   }
   if (shape != "") {
      p.fframedtext(x0, YY(y0+h0/2+dy), shape);
      dy += true_size;
   }
   p.endpath();

   p.closepl();
   std::string svg = p.getScreenSvg();

   m_display.setObject(OBJ_VISUAL_OBJECTS, id, svg);
#endif
}

// TODO: We need left-camera parameters to project the SOI
//void CScene2D::drawSoi(const std::string& id, const SOIPtr& pSoi)
//{
//#if defined(HAS_LIBPLOT)
//   std::ostringstream ss;
//   cogx::display::CSvgStringPlotter p(ss);
//   p.openpl();
//   p.fscale(W / m_outputWidth, H / m_outputHeight);
//   p.flinewidth(ps * 1.0);

//   p.pencolorname("red");
//   drawContours(p, x0, y0, contours);

//   double true_size = p.fontsize(ps * 12);
//   double dy = 0;

//   p.fontname("sans-serif");
//   p.pencolorname("yellow");
//   p.fframedtext(x0, YY(y0+h0/2+dy), id);
//   dy += true_size;
//   if (ident != "") {
//      p.fframedtext(x0, YY(y0+h0/2+dy), ident);
//      dy += true_size;
//   }
//   if (color != "") {
//      p.fframedtext(x0, YY(y0+h0/2+dy), color);
//      dy += true_size;
//   }
//   if (shape != "") {
//      p.fframedtext(x0, YY(y0+h0/2+dy), shape);
//      dy += true_size;
//   }
//   p.endpath();

//   p.closepl();
//   std::string svg = p.getScreenSvg();

//   m_display.setObject(OBJ_VISUAL_OBJECTS, id, svg);
//#endif
//}

void CScene2D::onChange_VisualObject(const cdl::WorkingMemoryChange & _wmc)
{
   cdl::WorkingMemoryAddress addr = _wmc.address;

   // VisualObject provides label values
   VisualObjectPtr pVisObj;
   try {
      pVisObj = getMemoryEntry<VisualObject>(addr);
   }
   catch(DoesNotExistOnWMException){
      //log("CScene2D: VisualObject %s deleted while working...", descAddr(addr).c_str());
      return;
   };
   if (! pVisObj.get()) return;

   // ProtoObject provides position and outline
   ProtoObjectPtr pProtoObj;
   try {
      cdl::WorkingMemoryPointerPtr pomp = pVisObj->protoObject;
      if (pomp->type == cast::typeName<ProtoObject>()) {
         pProtoObj = getMemoryEntry<ProtoObject>(pomp->address);
      }
   }
   catch(DoesNotExistOnWMException){
      //log("CScene2D: ProtoObject %s deleted while working...", descAddr(addr).c_str());
      return; // we don't know where on the image to show the labels
   };
   if (! pProtoObj.get()) return;

   try {
      drawVisualObject(_wmc.address.id, pVisObj, pProtoObj);
   }
   catch(const std::exception &e) {
      println("drawVisualObject FAILED with: %s", e.what());
      sleepComponent(100);
   }
   catch (...) {
      println("drawVisualObject FAILED");
      sleepComponent(100);
   }

}

void CScene2D::onAdd_VisualObject(const cdl::WorkingMemoryChange & _wmc)
{
   onChange_VisualObject(_wmc);
}

void CScene2D::onDelete_VisualObject(const cdl::WorkingMemoryChange & _wmc)
{
   m_display.removePart(OBJ_VISUAL_OBJECTS, _wmc.address.id);
}


//void CScene2D::onChange_SOI(const cdl::WorkingMemoryChange & _wmc)
//{
//   cdl::WorkingMemoryAddress addr = _wmc.address;

//   // VisualObject provides label values
//   SOIPtr pSoi;
//   try {
//      pSoi = getMemoryEntry<VisualObject>(addr);
//   }
//   catch(DoesNotExistOnWMException){
//      //log("CScene2D: VisualObject %s deleted while working...", descAddr(addr).c_str());
//      return;
//   };
//   if (! pSoi) return;

//   drawSoi(_wmc.address.id, pSoi);
//}

//void CScene2D::onAdd_SOI(const cdl::WorkingMemoryChange & _wmc)
//{
//   onChange_SOI(_wmc);
//}

//void CScene2D::onDelete_SOI(const cdl::WorkingMemoryChange & _wmc)
//{
//   m_display.removePart(OBJ_VISUAL_OBJECTS, _wmc.address.id);
//}


void CScene2D::destroy()
{
}

void CScene2D::runComponent()
{
   while (isRunning()) {
      sleepComponent(500);
   }
}

}} // namespace
// vim:sw=3:ts=8:et
