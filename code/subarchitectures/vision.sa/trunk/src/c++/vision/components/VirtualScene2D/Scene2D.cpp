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

#include <VisionData.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <plotter.h> // libplot-dev
#include <sstream>

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

   m_display.connectIceClient(*this);
   m_display.installEventReceiver();

   std::vector<std::string> objects;
   // TODO: ADD PARAMETER. "video.viewer" is not ok, it's the name of a component -> changes!
   // --video-object; alternative: views are defined in DisplayServer
   objects.push_back("video.viewer");
   objects.push_back(OBJ_VISUAL_OBJECTS);
   m_display.createView("VirtualScene2D", Visualization::VtGraphics, objects);
}

void CScene2D::onAdd_VisualObject(const cdl::WorkingMemoryChange & _wmc)
{
   cdl::WorkingMemoryAddress addr = _wmc.address;

   // VisualObject provides label values
   VisualObjectPtr pVisObj;
   try {
      pVisObj = getMemoryEntry<VisualObject>(addr);
      addr.id = pVisObj->protoObjectID;
   }
   catch(DoesNotExistOnWMException){
      //log("CScene2D: VisualObject %s deleted while working...", descAddr(addr).c_str());
      return;
   };

   // ProtoObject provides position and outline
   ProtoObjectPtr pProtoObj;
   try {
      pProtoObj = getMemoryEntry<ProtoObject>(addr);
   }
   catch(DoesNotExistOnWMException){
      //log("CScene2D: ProtoObject %s deleted while working...", descAddr(addr).c_str());
      return; // we don't know where on the image to show the labels
   };

   std::ostringstream svg;
   SVGPlotter p(std::cin, svg, std::cerr);
   p.line(110, 110, 230, 230);

   m_display.setObject(OBJ_VISUAL_OBJECTS, _wmc.address.id, svg.str());
}

void CScene2D::onChange_VisualObject(const cdl::WorkingMemoryChange & _wmc)
{
}

void CScene2D::onDelete_VisualObject(const cdl::WorkingMemoryChange & _wmc)
{
   m_display.removePart(OBJ_VISUAL_OBJECTS, _wmc.address.id);
}


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
