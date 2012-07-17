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
#ifndef _SCENE2D_S9BHETJS_
#define _SCENE2D_S9BHETJS_

#include <cast/architecture/ManagedComponent.hpp>
#include <CDisplayClient.hpp>
#include <VisionData.hpp>
#include <Video.hpp>

namespace cogx { namespace vision {

class CScene2D:
   public cast::ManagedComponent
{
   struct CBeliefContent
   {
      cast::cdl::WorkingMemoryAddress beliefAddr;
      cast::cdl::WorkingMemoryAddress visualObjectAddr;

      std::string colorName;
      double colorProb;
      std::string shapeName;
      double shapeProb;
      std::string identName;
      double identProb;
      CBeliefContent() {
         colorProb = -1;
         shapeProb = -1;
         identProb = -1;
      }
   };
   typedef std::shared_ptr<CBeliefContent> CBeliefContentPtr;
   std::map<cast::cdl::WorkingMemoryAddress, VisionData::VisualObjectPtr> mObjects;
   std::map<cast::cdl::WorkingMemoryAddress, CBeliefContentPtr> mObjectBeliefsByB;
   std::map<cast::cdl::WorkingMemoryAddress, CBeliefContentPtr> mObjectBeliefsByVo;

private:
   class CDisplayClient: public cogx::display::CDisplayClient
   {
      CScene2D* pScn;
   public:
      CDisplayClient() { pScn = NULL; }
      void setClientData(CScene2D* pScene) { pScn = pScene; }
      void handleEvent(const Visualization::TEvent &event); /*override*/
      std::string getControlState(const std::string& ctrlId); /*override*/
      void handleForm(const std::string& id, const std::string& partId,
            const std::map<std::string, std::string>& fields); /*override*/
      bool getFormData(const std::string& id, const std::string& partId,
            std::map<std::string, std::string>& fields); /*override*/
   };
   CDisplayClient m_display;

   double m_outputWidth;
   double m_outputHeight;
   std::string m_objectList;  // param: list of v11n objects to merge into the scene
   std::string m_labelColor;  // param: label color for visual objects

   std::string m_videoServerName; // param: video server for transformations (3D->2D)
   int m_camid;                   // param: camera id for transformations (3D->2D)
   Video::VideoInterfacePrx m_pVideoServer;

private:
#if 0 // maybe someday the objects will be drawn in runComponent()
   bool mbHideAll;
   struct CRedrawOp
   {
      enum ERedrawOp { Redraw, ShowAll, HideAll };
      ERedrawOp mRedrawOp;
      cast::cdl::WorkingMemoryAddress mAddr;
      CRedrawOp(ERedrawOp op) { mRedrawOp = op; }
      CRedrawOp(ERedrawOp op, cast::cdl::WorkingMemoryAddress& addr) {
         mRedrawOp = op;
         mAddr = addr;
      }
   };
   void queueRedraw(cast::cdl::WorkingMemoryAddress& addr);
   void queueShowAll();
   void queueHideAll();
#endif

protected:
   // ManagedComponent overrides
   virtual void configure(const std::map<std::string,std::string> & _config);
   virtual void start();
   virtual void destroy();
   virtual void runComponent();

   void onAdd_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc);
   void onChange_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc);
   void onDelete_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc);
   void onAdd_CameraMotionState(const cast::cdl::WorkingMemoryChange & _wmc);
   void onChange_CameraMotionState(const cast::cdl::WorkingMemoryChange & _wmc);
   void onAdd_MergedBelief(const cast::cdl::WorkingMemoryChange & _wmc);
   void onChange_MergedBelief(const cast::cdl::WorkingMemoryChange & _wmc);

   void drawVisualObject(const cast::cdl::WorkingMemoryAddress& objaddr, const VisionData::VisualObjectPtr& pVisObj);
         // , const VisionData::ProtoObjectPtr& pProtoObj);

public:
   CScene2D();
};

}} // namespace
#endif /* end of include guard: _SCENE2D_S9BHETJS_ */
// vim:sw=3:ts=8:et
