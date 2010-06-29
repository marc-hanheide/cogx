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
#include "CDisplayServer.hpp"
extern "C"
{
   cast::CASTComponentPtr newComponent()
   {
      return new cogx::display::CDisplayServer();
   }
}

#include "qtui/QCastApplication.hpp"
#include <QImage>

#include "object/CSvgImage.hpp"
#include "object/CRasterImage.hpp"
#ifdef V11N_OBJECT_TOMGINE_MODEL
#include "object/CTomGineModel.hpp"
#endif
#ifdef V11N_OBJECT_LUA_GL
#include "object/CLuaGlScript.hpp"
#endif
#ifdef V11N_OBJECT_HTML
#include "object/CHtmlObject.hpp"
#endif

#ifdef DEBUG_TRACE
#undef DEBUG_TRACE
#endif
#include "convenience.hpp"

//#include "fake/tomgine_test.cpp"

using namespace std;
using namespace cast;

namespace cogx { namespace display {

// -----------------------------------------------------------------
// CDisplayServer
// -----------------------------------------------------------------

CDisplayServer::CDisplayServer()
{
   pMainFrame = NULL;
   hIceDisplayServer = NULL;
}

CDisplayServer::~CDisplayServer()
{
   pMainFrame = NULL; // don't delete
}

void CDisplayServer::startIceServer()
{
   //Visualization::DisplayInterfacePtr hIceDisplayServer = new CDisplayServerI(this);
   hIceDisplayServer = new CDisplayServerI(this);
   registerIceServer<Visualization::DisplayInterface, CDisplayServerI>(hIceDisplayServer);
}

void CDisplayServer::configure(const map<string,string> & _config)
      throw(runtime_error)
{
   debug("CDisplayServer Server: configuring");

#ifdef V11N_OBJECT_HTML
   debug("v11n: Subsystem 'HTML' enabled.");
   setHtml("@info.DisplayServer", "001", "CogX Display Server<br>");
   setHtml("@info.DisplayServer", "002", "Version 0.5<br><br>");
   setHtml("@info.DisplayServer", "010", "Subsystem 'HTML' enabled.<br>");
#ifdef V11N_OBJECT_HTML_PLUGINS
   setHtml("@info.DisplayServer", "011", "Subsystem 'HTML Plugins' enabled.<br>");
#endif
   /* XXX: This was probably fixed (customguipanel didn't remove m_pView from observers)
   setHtmlHead("@info.DisplayServer.bugs", "css001",
         "<style> .em { color: red; } </style>");
   setHtml("@info.DisplayServer.bugs", "900",
         "<br><span class='em'>WARNING</span>: Closing windows may crash the CAST system, "
         "especially if they display objects that change frequently.<br>");
   setHtml("@info.DisplayServer.bugs", "901",
         "<br><span class='em'>WARNING</span>: 'Restore Window Layout' may crash the CAST system. "
         "It should be safe to use it at the beginning of a run.<br>");
   */
#ifdef V11N_OBJECT_LUA_GL
   setHtml("@info.DisplayServer", "020", "Subsystem 'LuaGlScript' enabled.<br>");
#endif
#ifdef V11N_OBJECT_TOMGINE_MODEL
   setHtml("@info.DisplayServer", "030", "Subsystem 'TomGine Model' enabled.<br>");
#endif
#endif

#ifdef V11N_OBJECT_LUA_GL
   debug("v11n: Subsystem 'LuaGlScript' enabled.");
#endif
#ifdef V11N_OBJECT_TOMGINE_MODEL
   debug("v11n: Subsystem 'TomGine Model' enabled.");
#endif
   CASTComponent::configure(_config);

   // TODO: Parse more parameters here
   // map<string,string>::const_iterator it;
   // if((it = _config.find("--setting")) != _config.end()) {
   //    istringstream istr(it->second);
   //    istr >> m_Setting;
   // }

   // The servers have to be started before all other components
   // so we start them in configure() instead of in start()
   debug("CDisplayServer Server: starting");
   startIceServer();
}

void CDisplayServer::start()
{
}

void CDisplayServer::runComponent()
{
   debug("CDisplayServer Server: running");

   debug("Starting EventServer");
   hIceDisplayServer.get()->startEventServer();
   debug("EventServer started");

   int argc=0;
   char **argv = NULL;
   QCastApplication app(argc, argv, this);
   QCastMainFrame* pMainFrame = new QCastMainFrame();

   debug("Passing the model to MainWindow");
   pMainFrame->setModel(&m_Model);
   pMainFrame->setControlDataProxy(this);
   pMainFrame->show();
   pMainFrame = NULL; // Owned by QApplication
   app.exec();

   debug("CDisplayServer Server: GUI closed.");

   while(isRunning()) {
      sleepComponent(1000);
   }

   debug("Stopping EventServer");
   hIceDisplayServer.get()->destroyEventServer();
   debug("EventServer stopped");

   debug("CDisplayServer Server: Done.");
}

void CDisplayServer::setRawImage(const std::string& id, int width, int height,
      int channels, const std::vector<unsigned char>& data)
{
   // DTRACE("CDisplayServer::setRawImage");
   CRasterImage *pImage = NULL;
   CRasterImage *pExisting = m_Model.getImage(id);

   bool sameformat = pExisting && pExisting->m_pImage;
   if (sameformat) {
      QImage *pqimg = pExisting->m_pImage;
      if (width != pqimg->width() || height != pqimg->height()) sameformat = false;
      if (pqimg->format() != QImage::Format_RGB32) sameformat = false;

      // No GS format in Qt, only indexed, which is more work (palette)
      // So we use RGB32 for all.
      //if (channels == 3 && pqimg->format() != QImage::Format_RGB32) sameformat = false;
      //if (channels == 1 && pqimg->format() != QImage::Format_RGB32) sameformat = false;
   }

   if (sameformat) {
      pImage = pExisting;
   }
   else {
      pImage = new CRasterImage();
      pImage->m_id = id;
      pImage->m_pImage = new QImage(width, height, QImage::Format_RGB32);
   }

   const unsigned char* pbits_ = pImage->m_pImage->bits();
   QRgb* ppix = (QRgb*) pbits_;
   unsigned char* pvimg = (unsigned char*) &(data[0]);

   long npix = width * height;
   if (channels == 1) {
      for (int i = 0; i < npix; i++) {
         *ppix = qRgb(*pvimg, *pvimg, *pvimg);
         pvimg++;
         ppix++;
      }
   }
   else if (channels == 3) {
      bool bgr = true;
      if (bgr) {
         int r, g, b;
         for (int i = 0; i < npix; i++) {
            b = *pvimg; g = *(++pvimg); r = *(++pvimg);
            *ppix = qRgb(r, g, b);
            pvimg++;
            ppix++;
         }
      }
      else {
         for (int i = 0; i < npix; i++) {
            *ppix = qRgb(*pvimg, *(++pvimg), *(++pvimg));
            pvimg++;
            ppix++;
         }
      }
   }
   else {
      QPainter painter(pImage->m_pImage);
      painter.fillRect(0, 0, width, height, QColor(255, 0, 0));
      // XXX crashes: painter.drawText(1, 20, QString("Invalid image format."));
   }

   if (pImage != pExisting) m_Model.setObject(pImage);
   else m_Model.refreshObject(id);

   if (pMainFrame) {
      // TODO: pMainFrame->notifyObjectAdded(pImage);
      //    Is this ok? some objects may be associated with views, but not all!
      //    The application/component should register it's basic views!
      //    registerObjectView("object_id");
   }
}

void CDisplayServer::setCompressedImage(const std::string& id, const std::vector<unsigned char>& data,
      const std::string &format)
{
   // DTRACE("CDisplayServer::setCompressedImage");
   CRasterImage *pExisting = m_Model.getImage(id);
   CRasterImage *pImage = NULL;

   pImage = new CRasterImage();
   pImage->m_id = id;
   pImage->m_pImage = new QImage();
   bool ok = pImage->m_pImage->loadFromData(&data[0], data.size(), format.c_str());

   if (ok) m_Model.setObject(pImage);
   else if(pExisting) m_Model.removeObject(id);
}

void CDisplayServer::setObject(const std::string& id, const std::string& partId, const std::string& xmlData)
{
   DTRACE("CDisplayServer::setObject");
   //{ // FAKE
   // static bool done = false;
   // if (!done) {
   //    DTRACE("tomgine_test_createModel");
   //    tomgine_test_createModel(&m_Model);
   //    lua_test_createModel(&m_Model);
   // }
   // done = true;
   //}

   CSvgImage *pImage = NULL;
   CDisplayObject *pExisting = m_Model.getObject(id);
   if (pExisting) {
      pImage = dynamic_cast<CSvgImage*>(pExisting);
      if (! pImage) {
         // The retreived model is of a different type, we must replace it
         m_Model.removeObject(id);
         DMESSAGE("Replacing an exisiting object of different type.");
      }
   }

   if (pImage) {
      pImage->setPart(partId, xmlData);
      m_Model.refreshObject(id);
   }
   else {
      pImage = new CSvgImage();
      pImage->m_id = id;
      pImage->setPart(partId, xmlData);
      m_Model.setObject(pImage);
   }
}

void CDisplayServer::setTomGineObject(const std::string& id, const std::string& partId, 
      const std::vector<unsigned char>& data)
{
#ifdef V11N_OBJECT_TOMGINE_MODEL
   DTRACE("CDisplayServer::setTomGineObject");

   CTomGineModel *pModel = NULL;
   CDisplayObject *pExisting = m_Model.getObject(id);
   if (pExisting) {
      pModel = dynamic_cast<CTomGineModel*>(pExisting);
      if (! pModel) {
         // The retreived model is of a different type, we must replace it
         m_Model.removeObject(id);
         DMESSAGE("Replacing an exisiting object of different type.");
      }
   }

   if (pModel) {
      if (data.size() < 1) pModel->removePart(partId);
      else pModel->deserialize(partId, data);
      m_Model.refreshObject(id);
   }
   else {
      pModel = new CTomGineModel();
      pModel->m_id = id;
      pModel->deserialize(partId, data);
      m_Model.setObject(pModel);
   }
#endif
}

void CDisplayServer::setLuaGlObject(const std::string& id, const std::string& partId, const std::string& script)
{
#ifdef V11N_OBJECT_LUA_GL
   DTRACE("CDisplayServer::setLuaGlObject");

   CLuaGlScript *pModel = NULL;
   CDisplayObject *pExisting = m_Model.getObject(id);
   if (pExisting) {
      pModel = dynamic_cast<CLuaGlScript*>(pExisting);
      if (! pModel) {
         // The retreived model is of a different type, we must replace it
         m_Model.removeObject(id);
         DMESSAGE("Replacing an exisiting object of different type.");
      }
   }

   if (pModel) {
      if (script.size() < 1) pModel->removePart(partId);
      else pModel->loadScript(partId, script);
      m_Model.refreshObject(id);
   }
   else {
      pModel = new CLuaGlScript();
      pModel->m_id = id;
      pModel->loadScript(partId, script);
      m_Model.setObject(pModel);
   }
#endif
}

void CDisplayServer::setHtml(const std::string& id, const std::string& partId, const std::string& htmlData)
{
#ifdef V11N_OBJECT_HTML
   DTRACE("CDisplayServer::setHtml");

   CHtmlObject *pModel = NULL;
   CDisplayObject *pExisting = m_Model.getObject(id);
   if (pExisting) {
      pModel = dynamic_cast<CHtmlObject*>(pExisting);
      if (! pModel) {
         // The retreived model is of a different type, we must replace it
         m_Model.removeObject(id);
         DMESSAGE("Replacing an exisiting object of different type.");
      }
   }

   if (pModel) {
      if (htmlData.size() < 1) pModel->removePart(partId);
      else pModel->setHtml(partId, htmlData);
      m_Model.refreshObject(id);
   }
   else {
      pModel = new CHtmlObject();
      pModel->m_id = id;
      pModel->setHtml(partId, htmlData);
      m_Model.setObject(pModel);
   }
#endif
}

void CDisplayServer::setHtmlHead(const std::string& id, const std::string& partId, const std::string& htmlData)
{
#ifdef V11N_OBJECT_HTML
   DTRACE("CDisplayServer::setHtmlHead");

   CHtmlObject *pModel = NULL;
   CDisplayObject *pExisting = m_Model.getObject(id);
   if (pExisting) {
      pModel = dynamic_cast<CHtmlObject*>(pExisting);
      if (! pModel) {
         // The retreived model is of a different type, we must replace it
         m_Model.removeObject(id);
         DMESSAGE("Replacing an exisiting object of different type.");
      }
   }

   if (pModel) {
      if (htmlData.size() < 1) pModel->removePart(partId);
      else pModel->setHead(partId, htmlData);
      m_Model.refreshObject(id);
   }
   else {
      pModel = new CHtmlObject();
      pModel->m_id = id;
      pModel->setHead(partId, htmlData);
      m_Model.setObject(pModel);
   }
#endif
}

void CDisplayServer::setHtmlForm(const Ice::Identity& ident, const std::string& id,
      const std::string& partId, const std::string& htmlData)
{
#ifdef V11N_OBJECT_HTML
   DTRACE("CDisplayServer::setHtmlForm");

   CHtmlObject *pModel = NULL;
   CDisplayObject *pExisting = m_Model.getObject(id);
   if (pExisting) {
      pModel = dynamic_cast<CHtmlObject*>(pExisting);
      if (! pModel) {
         // The retreived model is of a different type, we must replace it
         m_Model.removeObject(id);
         DMESSAGE("Replacing an exisiting object of different type.");
      }
   }

   if (pModel) {
      if (htmlData.size() < 1) pModel->removePart(partId);
      else pModel->setForm(ident, partId, htmlData);
      //m_Model.registerHtmlForm(ident, id, partId);
      m_Model.refreshObject(id);
   }
   else {
      pModel = new CHtmlObject();
      pModel->m_id = id;
      pModel->setForm(ident, partId, htmlData);
      //m_Model.registerHtmlForm(ident, id, partId);
      m_Model.setObject(pModel);
   }
#endif
}

void CDisplayServer::setObjectTransform2D(const std::string& id, const std::string& partId,
      const std::vector<double>& transform)
{
   // DTRACE("CDisplayServer::setTransform2D");
   CDisplayObject *pExisting = m_Model.getObject(id);
   if (!pExisting) return;

   //{ // XXX FAKE, Testing
   //  tomgine_test_updateModel(this);
   //  //CDisplayView* pview = m_Model.getView("Composed View");
   //  //if (pview) pview->m_Trafos["video.viewer"] = transform;
   //}

   // TODO check size if (transform.size() != 9) transform.resize(9, 0.0);
   pExisting->setTransform2D(partId, transform);
   m_Model.refreshObject(id);
}

void CDisplayServer::setObjectPose3D(const std::string& id, const std::string& partId,
      const Math::Vector3& position, const Visualization::Quaternion& rotation)
{
   CDisplayObject *pExisting = m_Model.getObject(id);
   if (!pExisting) return;

   std::vector<double> pos, rot;
   pos.reserve(3); rot.reserve(4);
   pos.push_back(position.x);
   pos.push_back(position.y);
   pos.push_back(position.z);
   rot.push_back(rotation.x);
   rot.push_back(rotation.y);
   rot.push_back(rotation.z);
   rot.push_back(rotation.w);
   pExisting->setPose3D(partId, pos, rot);
   m_Model.refreshObject(id);
}

void CDisplayServer::addCheckBox(const Ice::Identity& ident, const std::string& viewId,
      const std::string& ctrlId, const std::string& label)
{
   CGuiElement *pgel = new CGuiElement();
   pgel->m_type = CGuiElement::wtCheckBox;
   pgel->m_viewId = viewId;
   pgel->m_id = ctrlId;
   pgel->m_label = label;
   pgel->m_dataOwner = ident;
   pgel->Observers += this;
   if (m_Model.addGuiElement(pgel))
      debug(std::string("CheckBox added: ") + ctrlId);
   else {
      if (pgel) delete pgel;
      debug(std::string("Gui element already exists: ") + ctrlId);
   }
}

void CDisplayServer::addButton(const Ice::Identity& ident, const std::string& viewId,
      const std::string& ctrlId, const std::string& label)
{
   CGuiElement *pgel = new CGuiElement();
   pgel->m_type = CGuiElement::wtButton;
   pgel->m_viewId = viewId;
   pgel->m_id = ctrlId;
   pgel->m_label = label;
   pgel->m_dataOwner = ident;
   pgel->Observers += this;
   if (m_Model.addGuiElement(pgel))
      debug(std::string("Button added: ") + ctrlId);
   else {
      if (pgel) delete pgel;
      debug(std::string("Gui element already exists: ") + ctrlId);
   }
}

// Add change notification to queue, processed in <url:#tn=CDisplayServerI::run>
void CDisplayServer::onUiDataChanged(CGuiElement *pElement, const std::string& newValue)
{
   debug(std::string("Time: ") + sfloat (fclocks()));
   debug(std::string("New value ") + pElement->m_id + "=" + newValue);
   DTRACE("CDisplayServer::onUiDataChanged");

   if (! hIceDisplayServer.get()) return;
   hIceDisplayServer->addDataChange(new CGuiElementValue(pElement, newValue));
}

// Add request for value to queue, processed in <url:#tn=CDisplayServerI::run>
void CDisplayServer::getControlStateAsync(cogx::display::CGuiElement *pElement)
{
   DTRACE("CDisplayServer::getControlStateAsync");
   if (! hIceDisplayServer.get()) return;
   hIceDisplayServer->addDataChange(new CGuiElementValue(pElement, "", CGuiElementValue::get));
}

// -----------------------------------------------------------------
// CDisplayServerI
// -----------------------------------------------------------------

// TODO: m_pEventSenderThread should probably be created during start() so that we
// can restart the CAST component.
CDisplayServerI::CDisplayServerI(CDisplayServer *pDisplayServer)
{
   m_pDisplayServer = pDisplayServer;
   m_stopEventServer = false;
   m_pEventSenderThread = NULL;
}

void CDisplayServerI::startEventServer()
{
   m_pDisplayServer->debug("CDisplayServerI->run()");
   try {
      if (m_pEventSenderThread.get() != NULL) {
        m_pDisplayServer->log("WARING m_pEventSenderThread ALREADY RUNNING");
        return;
      }
      m_stopEventServer = false;
      m_pEventSenderThread = new CCallbackSenderThread(this);
      m_pEventSenderThread->start(); // e.start() ==> e.run() ==> this.run()
      m_pDisplayServer->debug("CDisplayServerI RUNNING");
   }
   catch(const Ice::Exception& ex) {
      cout << "CDisplayServerI FAILURE `" << ex << endl;
   }
}

// This is the body of the thread that sends events to subscribed clients.
// starting:
//    cast.runComponent() ==> hIceDisplayServer.start() ==> m_pEventSenderThread.start()
//    ==> m_pEventSenderThread.run() ==> this.run()
// stopping:
//    cast.runComponent() ==> hIceDisplayServer.destroy() ==> m_pEventSenderThread.join()
void CDisplayServerI::run()
{
   m_pDisplayServer->debug("CDisplayServerI->run() entered");
   // Called by CCallbackSenderThread
   while (true) {
      std::set<Visualization::EventReceiverPrx> clients;
      CPtrVector<CGuiElementValue> changes;

      {
         // SYNC: Lock the monitor
         IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
         // SYNC: Unlock the monitor and wait for notify() or timeout
         m_EventMonitor.timedWait(IceUtil::Time::seconds(2));
         // SYNC: Continue with a locked monitor

         if(m_stopEventServer) {
            break;
         }

         clients = m_EventClients;
         changes = m_EventQueue;
         m_EventQueue.clear();
         // SYNC: unlock the monitor
      }

      Visualization::TEvent event;

      if(!clients.empty() && !changes.empty()) {
         DTRACE("EventServer Woke up. Sending events.");
         // check the queues and send messages
         CGuiElementValue *pChange;
         set<Visualization::EventReceiverPrx>::iterator p;
         for(p = clients.begin(); p != clients.end(); p++) {
            try {
               FOR_EACH(pChange, changes) {
                  if (!pChange || !pChange->pElement) continue;
                  if (!p->get() || pChange->pElement->m_dataOwner != (*p)->ice_getIdentity())
                     continue;

                  if (pChange->mode == CGuiElementValue::get) {
                     // Get data from the ui elemente owner
                     std::string val = (*p)->getControlState(pChange->pElement->m_id);
                     if (val.size() > 0)
                        pChange->pElement->syncControlState(val);
                  }
                  else if (pChange->mode == CGuiElementValue::set) {
                     // Send new data to the ui elemente owner
                     switch (pChange->pElement->m_type) {
                        case CGuiElement::wtCheckBox:
                           event.type = Visualization::evCheckBoxChange; break;
                        case CGuiElement::wtButton:
                           event.type = Visualization::evButtonClick; break;
                        case CGuiElement::wtDropList:
                           event.type = Visualization::evDropListChange; break;
                     }

                     event.sourceId = pChange->pElement->m_id;
                     event.data = pChange->value;

                     (*p)->handleEvent(event);
                  }
               }
            }
            catch(const Ice::Exception& ex) {
               DMESSAGE("handleEvent crashed with Ice::Exception: " << ex);
               //cerr << "removing client `" << _communicator->identityToString((*p)->ice_getIdentity())
               //   << "':\n" << ex << endl;

               IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
               m_EventClients.erase(*p);
            }
            catch(...) {
               DMESSAGE("handleEvent crashed for unknonw reasons");
            }
         }
         FOR_EACH(pChange, changes) {
            if(pChange) delete pChange;
         }
         changes.clear();
      }
   }
} 

void CDisplayServerI::destroyEventServer()
{
   m_pDisplayServer->debug("CDisplayServerI->destroyEventServer() entered");
   IceUtil::ThreadPtr eventSenderThread;

   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);

      // cout << "destroying callback sender" << endl;
      m_stopEventServer = true;

      m_EventMonitor.notify();

      eventSenderThread = m_pEventSenderThread;
      m_pEventSenderThread = NULL; // Resolve cyclic dependency.
   }

   if (eventSenderThread.get() != NULL) {
      m_pDisplayServer->debug("Waiting for m_pEventSenderThread to finish.");
      eventSenderThread->getThreadControl().join();
      m_pDisplayServer->debug("m_pEventSenderThread joined (finished).");
   }
}

void CDisplayServerI::addClient(const Ice::Identity& ident, const Ice::Current& current)
{
   IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
   cout << "v11n: adding client `" << ident.name << ":" << ident.category << "'"<< endl;

   // HACK: The EventReceiver has the same id as the main component, but a different
   // category; it runs on the same machine as the component.
   cdl::ComponentDescription desc = m_pDisplayServer->getComponentManager()->
      getComponentDescription(ident.name);

   const std::string & host(desc.hostName);
   unsigned int port = cast::languageToPort(desc.language);

   Ice::ObjectPrx prx = m_pDisplayServer->getIceServer(ident.name, ident.category, host, port);
   // END OF HACK

   Visualization::EventReceiverPrx client = Visualization::EventReceiverPrx::uncheckedCast(prx);
   m_EventClients.insert(client);

   // from the demo (doesn't work, returns NULL even on the same machine):
   // Visualization::EventReceiverPrx client =
   //      Visualization::EventReceiverPrx::uncheckedCast(current.con->createProxy(ident));
   // -------------
   cout << "v11n: added `" << ident.name << ":" << ident.category << "'"<< endl;
}

// put the event into a queue and wake up the event (callback) server
void CDisplayServerI::addDataChange(CGuiElementValue *pChange)
{
   if (!pChange || !pChange->pElement) return;

   IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
   m_EventQueue.push_back(pChange);
   m_EventMonitor.notify();
}

}} // namespace
// vim:set fileencoding=utf-8 sw=3 ts=8 et:vim

