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
#include "belief_utils.hpp"

#include <Math.hpp>
#include <CameraParameters.h> // projectPoint
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <beliefs.hpp>
#include <beliefs_cogx.hpp>
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
namespace beliefcore = de::dfki::lt::tr::beliefs::slice;
namespace beliefcogx = eu::cogx::beliefs::slice;

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
   m_objectList = "video.viewer";
   m_videoServerName = "";
   m_camid = -1;
}

void CScene2D::configure(const std::map<std::string,std::string> & _config)
{
   m_display.configureDisplayClient(_config);
   std::map<std::string,std::string>::const_iterator it;

   if ((it = _config.find("--v11n-objects")) != _config.end()) {
      m_objectList = it->second;
   }

   if ((it = _config.find("--videoname")) != _config.end()) {
      m_videoServerName = it->second;
   }

   if ((it = _config.find("--camid")) != _config.end()) {
      istringstream is(it->second);
      is >> m_camid;
   }

   if ( (m_videoServerName != "") != (m_camid >= 0) ) {
      error("Both --videoname and --camid have to be set to enable 3D->2D transformations.");
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

   Video::CameraMotionStatePtr mpState;
   addChangeFilter(createGlobalTypeFilter<Video::CameraMotionState>(cdl::ADD),
         new MemberFunctionChangeReceiver<CScene2D>(this,
            &CScene2D::onAdd_CameraMotionState));
   addChangeFilter(createGlobalTypeFilter<Video::CameraMotionState>(cdl::OVERWRITE),
         new MemberFunctionChangeReceiver<CScene2D>(this,
            &CScene2D::onChange_CameraMotionState));

   addChangeFilter(createGlobalTypeFilter<beliefcogx::MergedBelief>(cdl::ADD),
         new MemberFunctionChangeReceiver<CScene2D>(this,
            &CScene2D::onAdd_MergedBelief));
   addChangeFilter(createGlobalTypeFilter<beliefcogx::MergedBelief>(cdl::OVERWRITE),
         new MemberFunctionChangeReceiver<CScene2D>(this,
            &CScene2D::onChange_MergedBelief));

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

   if (m_camid >= 0 && m_videoServerName != "") {
      println("Connecting to camera (id=%d) on '%s'", m_camid, m_videoServerName.c_str());
      m_pVideoServer = getIceServer<Video::VideoInterface>(m_videoServerName);
      sleepComponent(300);
      if (m_pVideoServer.get()) {
         int w, h;
         m_pVideoServer->getImageSize(w, h);
         m_outputWidth = w;
         m_outputHeight = h;
      }
   }
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
class CContours
{
   vector<vector<cv::Point> > mContours;
   vector<cv::Vec4i> mHierarchy;
public:
   void drawContours(Plotter& p, double x0, double y0)
   {
      vector<vector<cv::Point> >::iterator itpart;
      vector<cv::Point>::iterator itpoint;

      for (itpart = mContours.begin(); itpart != mContours.end(); itpart++) {
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

   void findContours(VisionData::SegmentMask& bwMask)
   {
      IplImage* pImg = 0;
      try {
         pImg = wrapMask(bwMask);
      }
      catch (...) {
         //println("wrapMask FAILED");
         //sleepComponent(100);
         throw;
      }
      if (pImg) {
         cv::Mat imgMat(pImg); // XXX copy=true crashes; so we modify the original data.

         // XXX: mask created in SOIFilter: 1-object, 2-background.
         // findContours requires 0 for background, everything else is the object.
         for (int i=0; i< bwMask.height; i++) {
            unsigned char *prow = imgMat.ptr(i);
            unsigned char *pend = prow + bwMask.width;
            while(prow < pend) {
               if (*prow != 1) *prow = 0;
               prow++;
            }
         }
         try {
            cv::findContours( imgMat, mContours, mHierarchy,
                  CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
         }
         catch (...) {
            //println("cv::findContours FAILED");
            //sleepComponent(100);
            releaseWrappedImage(&pImg);
            throw;
         }
         releaseWrappedImage(&pImg);
      }
   }
};
#endif

void CScene2D::drawVisualObject(const cast::cdl::WorkingMemoryAddress& objaddr,
      const VisualObjectPtr& pVisualObject)
#if 1
{
   if (! m_pVideoServer.get()) {
      debug("VideoServer is not set. Visual Object will not be drawn.");
      return;
   }

   Video::CameraParameters camPars;
   if (!m_pVideoServer->getCameraParameters(m_camid, camPars)) {
      error("Could not obtain CameraParameters (id=%d) from '%s'",
            m_camid, m_videoServerName.c_str());
      return;
   }

   VisualObjectPtr pVisObj;
   if (pVisualObject.get()) {
      pVisObj = pVisualObject;
   }
   else {
      if (mObjects.find(objaddr) != mObjects.end()) {
         pVisObj = mObjects[objaddr];
      }
   }
   if (!pVisObj.get()) {
      return;
   }

#if 0
   // ProtoObject provides segmentation outline
   ProtoObjectPtr pProtoObj;
   try {
      cdl::WorkingMemoryPointerPtr pomp = pVisObj->protoObject;
      if (pomp->type == cast::typeName<ProtoObject>()) {
         pProtoObj = getMemoryEntry<ProtoObject>(pomp->address);
      }
   }
   catch(DoesNotExistOnWMException){
   };
#endif

   cogx::Math::Vector2 vop = projectPoint(camPars, pVisObj->pose.pos);

   auto getBestLabel = [](std::vector<std::string>& labels, std::vector<double>& values) -> std::string
   {
      if (labels.size() < 1) return "";
      if (values.size() < 1) return "";
      std::vector<double>::iterator it = std::max_element(values.begin(), values.end());
      int i = std::distance(values.begin(), it);
      if (i > labels.size()) return "";
      ostringstream ss;
      ss << std::fixed << std::setprecision(2) << values[i] << " " << labels[i];
      return ss.str();
   };
   std::string ident = getBestLabel(pVisObj->identLabels, pVisObj->identDistrib);
   std::string color = getBestLabel(pVisObj->colorLabels, pVisObj->colorDistrib);
   std::string shape = getBestLabel(pVisObj->shapeLabels, pVisObj->shapeDistrib);

   //cast::cdl::WorkingMemoryAddress objaddr;
   //objaddr.subarchitecture = "vision.sa";
   //objaddr.id = id;
   if (mObjectBeliefsByVo.find(objaddr) != mObjectBeliefsByVo.end()) {
     CBeliefContentPtr pbel = mObjectBeliefsByVo[objaddr]; 
     auto fmtfloat = [](double x) -> std::string {
        ostringstream ss;
        ss << std::fixed << std::setprecision(2) << x;
        return ss.str();
     };
     std::string v;
     if (pbel->colorName != "") {
        v = fmtfloat(pbel->colorProb) + " " + pbel->colorName;
        if (v != color) {
           color = v + " (" + color + ")";
        }
     }
     if (pbel->shapeName != "") {
        v = fmtfloat(pbel->shapeProb) + " " + pbel->shapeName;
        if (v != shape) {
           shape = v + " (" + shape + ")";
        }
     }
     if (pbel->identName != "") {
        v = fmtfloat(pbel->identProb) + " " + pbel->identName;
        if (v != ident) {
           ident = v + " (" + ident + ")";
        }
     }
   }

   //CContours ctrs;
   // if (pProtoObj.get())
   //ctrs.findContours(pProtoObj->mask);

   //double w0 = pProtoObj->mask.width;
   //double h0 = pProtoObj->mask.height;
   double h0 = 0;
   double x0 = vop.x; // - w0/2;
   double y0 = vop.y; // - h0/2;
   double ps = 1.0;
   std::ostringstream ss;
   cogx::display::CSvgStringPlotter p(ss);

   p.openpl();
   p.fscale(1.0, 1.0);
   p.flinewidth(ps * 1.0);

   double true_size = p.fontsize(ps * 12);
   double dy = 0;

   // if (pProtoObj.get()) {
   //p.pencolorname("red");
   //ctrs.drawContours(p, x0, y0);
   // }

   p.fontname("sans-serif");
   p.pencolorname("yellow");
   p.fframedtext(x0, YY(y0+h0/2+dy), objaddr.id);
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

   m_display.setObject(OBJ_VISUAL_OBJECTS, objaddr.id, svg);
}
#else
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
#endif

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
   const cdl::WorkingMemoryAddress& addr = _wmc.address;

   // VisualObject provides label values
   VisualObjectPtr pVisObj;
   try {
      pVisObj = getMemoryEntry<VisualObject>(addr);
      if (pVisObj.get()) {
         mObjects[addr] = pVisObj;
      }
   }
   catch(DoesNotExistOnWMException){
      m_display.removePart(OBJ_VISUAL_OBJECTS, addr.id);
      mObjects.erase(addr);
      //log("CScene2D: VisualObject %s deleted while working...", descAddr(addr).c_str());
      return;
   };
   if (! pVisObj.get()) return;

   if (pVisObj->presence == VisionData::VopREMOVED) {
      //println(" **** Remove '%s'", addr.id.c_str());
      m_display.removePart(OBJ_VISUAL_OBJECTS, addr.id);
      mObjects.erase(addr);
      return;
   }

   try {
      drawVisualObject(_wmc.address, pVisObj);
   }
   catch(const std::exception &e) {
      println("drawVisualObject FAILED with: %s", e.what());
      sleepComponent(100);
   }
   catch (...) {
      println("drawVisualObject FAILED");
      sleepComponent(100);
   }

   //queueRedraw(addr);

#if 0
   if (pVisObj->presence != VisionData::VopVISIBLE) {
      m_display.removePart(OBJ_VISUAL_OBJECTS, _wmc.address.id);
      return;
   }

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
#endif

}

void CScene2D::onAdd_VisualObject(const cdl::WorkingMemoryChange & _wmc)
{
   onChange_VisualObject(_wmc);
}

void CScene2D::onDelete_VisualObject(const cdl::WorkingMemoryChange & _wmc)
{
   m_display.removePart(OBJ_VISUAL_OBJECTS, _wmc.address.id);
   mObjects.erase(_wmc.address);
}

void CScene2D::onAdd_CameraMotionState(const cdl::WorkingMemoryChange & _wmc)
{
   onChange_CameraMotionState(_wmc);
}

void CScene2D::onChange_CameraMotionState(const cdl::WorkingMemoryChange & _wmc)
{
   Video::CameraMotionStatePtr pState;
   try {
      pState = getMemoryEntry<Video::CameraMotionState>(_wmc.address);
   }
   catch(DoesNotExistOnWMException){
      return;
   }

   if (pState->bMoving) {
      // Hide all
      for (auto ivo : mObjects) {
         m_display.removePart(OBJ_VISUAL_OBJECTS, ivo.first.id);
      }
   }
   else {
      // Show all
      for (auto ivo : mObjects) {
         if (ivo.second.get()) {
            if (ivo.second->presence == VisionData::VopVISIBLE)
               drawVisualObject(ivo.first, ivo.second);
            else
               m_display.removePart(OBJ_VISUAL_OBJECTS, ivo.first.id);
         }
      }
   }
}

void CScene2D::onAdd_MergedBelief(const cdl::WorkingMemoryChange & _wmc)
{
   onChange_MergedBelief(_wmc);
}

void CScene2D::onChange_MergedBelief(const cdl::WorkingMemoryChange & _wmc)
{
   beliefcogx::MergedBeliefPtr pbel;
   try {
      pbel = getMemoryEntry<beliefcogx::MergedBelief>(_wmc.address);
      //std::string html = pbel->id + " = " + pbel->type;
      //if (pbel->content.get()) {
      //   html = html + " = " + pbel->content->ice_id();
      //}
      //m_display.setHtml("BELIEF-TESTING", pbel->id, html + "<br>");
      if (pbel->type != "visualobject") {
        return;
      }
   }
   catch(DoesNotExistOnWMException){
      log("Belief deleted before it could be read.");
   }
   if (!pbel.get() || !pbel->content.get()) {
      return;
   }

   auto pmrgCont = dynamic_cast<beliefcore::distribs::CondIndependentDistribs*>(pbel->content.get());
   if (! pmrgCont) {
      return;
   }
   //m_display.setHtml("BELIEF-TESTING", pbel->id + "a", "YO-cast<br>");

   CBeliefContentPtr pvoProps;
   if (mObjectBeliefsByB.find(_wmc.address) == mObjectBeliefsByB.end()) {
      pvoProps.reset(new CBeliefContent());
      pvoProps->beliefAddr = _wmc.address;
      mObjectBeliefsByB[_wmc.address] = pvoProps;
   }
   else {
      pvoProps = mObjectBeliefsByB[_wmc.address];
   }

   //char ch = 'A';
   // Visit all MergedBelief porperties from content
   for (auto itdist : pmrgCont->distribs) { 
      //m_display.setHtml("BELIEF-TESTING", pbel->id + "b" + ++ch,
      //      "&nbsp;-" + itdist.first + " = " + itdist.second->ice_id() + "<br>");

      // process only BasicProbDistribution properties
      auto ppropdist = dynamic_cast<beliefcore::distribs::BasicProbDistribution*>(itdist.second.get());
      if (!ppropdist) {
         continue;
      }
      // key == itdist.first
      //m_display.setHtml("BELIEF-TESTING", pbel->id + "b" + ch + "b",
      //      "&nbsp;&nbsp;-> key= " + ppropdist->key + "<br>");

      // ppropdist->values for color, shape, type: DistributionValues -> FormulaValues
      // Look only for FormulaValues
      auto ppropvlaues = dynamic_cast<beliefcore::distribs::FormulaValues*>(ppropdist->values.get());
      if (!ppropvlaues) {
         continue;
      }
      //m_display.setHtml("BELIEF-TESTING", pbel->id + "b" + ch + "c",
      //      "&nbsp;&nbsp;-> values-ice_id = " + ppropdist->values->ice_id() + "<br>");

      // Process each formula/float pair (FormulaProbPair) from FormulaValues
      for (auto itpair : ppropvlaues->values) {
         // Labels are in ElementaryFormula
         auto pname = dynamic_cast<beliefcore::logicalcontent::ElementaryFormula*>(itpair.val.get());
         if (pname) {
            if (itdist.first == "color") pvoProps->colorName = pname->prop;
            else if (itdist.first == "shape") pvoProps->shapeName = pname->prop;
            else if (itdist.first == "objecttype") pvoProps->identName = pname->prop;
         }

         // Probabilities for labels are in prob, while the 'label' is a FloatFormula (?!)
         auto pval = dynamic_cast<beliefcore::logicalcontent::FloatFormula*>(itpair.val.get());
         if (pval) {
            if (itdist.first == "color-prob") pvoProps->colorProb = pval->val; // itpair.prob;
            else if (itdist.first == "shape-prob") pvoProps->shapeProb = pval->val; // itpair.prob;
            else if (itdist.first == "objecttype-prob") pvoProps->identProb = pval->val; // itpair.prob;
         }
      }
      //ostringstream ss;
      //ss << "&nbsp;&nbsp;&npsp; * ";
      //ss << " co-" << pvoProps->colorName << "=" << pvoProps->colorProb;
      //ss << " sh-" << pvoProps->shapeName << "=" << pvoProps->shapeProb;
      //ss << " id-" << pvoProps->identName << "=" << pvoProps->identProb;
      //ss << "<br>";
      //m_display.setHtml("BELIEF-TESTING", pbel->id + "b" + ch + "d", ss.str());
   }

   // To get to the VO id:
   //   this belief (A)
   //   hist -> ancestors -> verified belief id (B)
   //   verified belief -> ancestors -> grounded belief id (C)
   //   grounded belief -> ancestors -> *visual-object id* (D)
   //

   // Discover the address of the VisualObject if it was not discovered, yet
   if (pvoProps->visualObjectAddr.id == "") {
      //println("*** recurseAncestorsForType");
      auto pwmpVo = cast::beliefs::recurseAncestorsForType(*this, pbel, cast::typeName<VisionData::VisualObject>());
      if (pwmpVo.get()) {
         pvoProps->visualObjectAddr = pwmpVo->address;
         mObjectBeliefsByVo[pvoProps->visualObjectAddr] = pvoProps;
      }
   }

   if (pvoProps->visualObjectAddr.id != "") {
      drawVisualObject(pvoProps->visualObjectAddr, nullptr);
   }
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
