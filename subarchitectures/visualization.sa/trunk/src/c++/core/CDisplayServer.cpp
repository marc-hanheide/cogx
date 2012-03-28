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
// #undef DEBUG_TRACE
#endif
#include "convenience.hpp"

//#include "fake/tomgine_test.cpp"

using namespace std;
using namespace cast;

namespace cogx { namespace display {

long SECRET_TOKEN = 2349807;

// -----------------------------------------------------------------
// CDisplayServer
// -----------------------------------------------------------------

CDisplayServer::CDisplayServer()
{
   hIceDisplayServer = NULL;
   m_standaloneHost = "";
   m_sGuiObserverName = "CDisplayServer";
}

CDisplayServer::~CDisplayServer()
{
   printf("Destroying CDisplayServer\n");
}

void CDisplayServer::startIceServer()
{
   hIceDisplayServer = new CDisplayServerI(this);
   registerIceServer<Visualization::DisplayInterface, CDisplayServerI>(hIceDisplayServer);
}

void CDisplayServer::configure(const map<string,string> & _config)
      throw(runtime_error)
{
   debug("CDisplayServer Server: configuring");

   map<string,string>::const_iterator it;
   if((it = _config.find("--redirect-to-host")) != _config.end()) {
      string s = it->second;
      // trim
      s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
      if (s == "/no") s = "";
      m_standaloneHost = s;      
   }

   if (isUsingRemoteHost()) {
      println("Clients will use a remote display server that should be running on: '%s'", m_standaloneHost.c_str());
   }
   else {
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
   }

   // The servers have to be started before all other components
   // so we start them in configure() instead of in start()
   debug("CDisplayServer Server: starting");
   startIceServer();

   // send command to clear server objects, before other components start.
   if (isUsingRemoteHost()) {
      log("Resetting the standalone server");
      try {

         Ice::ObjectPrx prx = getIceServer(Visualization::V11NSTANDALONENAME,
               toServantCategory<Visualization::DisplayInterface>(),
               m_standaloneHost, Visualization::V11NSTANDALONEPORT);

         Visualization::DisplayInterfacePrx pserver;
         pserver = Visualization::DisplayInterfacePrx::checkedCast(prx);
         pserver->resetServer(SECRET_TOKEN);
      }
      catch (...) {
         println(" *** CDisplayServer could not connect standalone server on '%s'.",
               m_standaloneHost.c_str());
      }
   }
}

void CDisplayServer::start()
{
}

void CDisplayServer::run()
{
   if (! isUsingRemoteHost()) {
      debug("Starting EventServer");
      hIceDisplayServer.get()->startEventServer();
      debug("EventServer started");

      int argc=0;
      char **argv = nullptr;
      QCastApplication app(argc, argv, this);
      QCastMainFrame* pMainFrame = new QCastMainFrame();

      debug("Passing the model to MainWindow");
      pMainFrame->setModel(&m_Model);
      pMainFrame->setControlDataProxy(this);
      pMainFrame->show();
      pMainFrame->loadStartupLayout();
      pMainFrame = nullptr; // Owned by QApplication
      app.exec();

      debug("CDisplayServer Server: GUI closed.");

      debug("Stopping EventServer");
      hIceDisplayServer.get()->destroyEventServer();
      debug("EventServer stopped");
   }

   while(isRunning()) {
      sleepComponent(1000);
   }
}
    
void CDisplayServer::runComponent()
{
   debug("CDisplayServer Server: Running");
   run();
   debug("CDisplayServer Server: Done.");
}

void CDisplayServer::resetServer(int secret)
{
   if (SECRET_TOKEN != secret)
      return;

   //m_Model.removeAllViews();
   m_Model.removeAllObjects();
   // TODO: should also clear all GUI elements and dialogs
}

void CDisplayServer::createView(const std::string& id, Visualization::ViewType type,
      const std::vector<std::string>& objects)
{
   ERenderContext ctx;
   switch (type) {
      case Visualization::VtGraphics:
         ctx = ContextGraphics;
         break;
      case Visualization::VtOpenGl:
         ctx = ContextGL;
         break;
      case Visualization::VtHtml:
         ctx = ContextHtml;
         break;
      default:
         ctx = ContextGraphics;
   }
   m_Model.createView(id, ctx, objects);
}

void CDisplayServer::enableDefaultView(const std::string& objectId, bool enable)
{
   m_Model.enableDefaultView(objectId, enable);
}

void CDisplayServer::setRawImage(const std::string& id, int width, int height,
      int channels, const std::vector<unsigned char>& data)
{
   // DTRACE("CDisplayServer::setRawImage");
   CRasterImage *pImage = nullptr;
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
         ++pvimg;
         ++ppix;
      }
   }
   else if (channels == 3) {
      bool bgr = true; // reverse channels by default; TODO: parameter in setRawImage
      int r, g, b;
      if (bgr) {
         for (int i = 0; i < npix; i++) {
            b = *pvimg; g = *(++pvimg); r = *(++pvimg);
            *ppix = qRgb(r, g, b);
            ++pvimg;
            ++ppix;
         }
      }
      else {
         for (int i = 0; i < npix; i++) {
            r = *pvimg; g = *(++pvimg); b = *(++pvimg);
            *ppix = qRgb(r, g, b);
            ++pvimg;
            ++ppix;
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
}

void CDisplayServer::setCompressedImage(const std::string& id, const std::vector<unsigned char>& data,
      const std::string &format)
{
   // DTRACE("CDisplayServer::setCompressedImage");
   CRasterImage *pExisting = m_Model.getImage(id);
   CRasterImage *pImage = nullptr;

   pImage = new CRasterImage();
   pImage->m_id = id;
   pImage->m_pImage = new QImage();
   bool ok = pImage->m_pImage->loadFromData(&data[0], data.size(), format.c_str());

   if (ok) m_Model.setObject(pImage);
   else if(pExisting) m_Model.removeObject(id);
}

void CDisplayServer::setObject(const std::string& id, const std::string& partId, const std::string& xmlData)
{
   //DTRACE("CDisplayServer::setObject");
   //{ // FAKE
   // static bool done = false;
   // if (!done) {
   //    DTRACE("tomgine_test_createModel");
   //    tomgine_test_createModel(&m_Model);
   //    lua_test_createModel(&m_Model);
   // }
   // done = true;
   //}

   CSvgImage *pImage = nullptr;
   CDisplayObject *pExisting = m_Model.getObject(id);
   if (pExisting) {
      pImage = dynamic_cast<CSvgImage*>(pExisting);
      if (! pImage) {
         // The retreived model is of a different type, we must replace it
         m_Model.removeObject(id);
         DMESSAGE("setObject: Replacing an exisiting object of different type.");
      }
   }

   if (pImage) {
      bool bNotifyNew = pImage->setPart(partId, xmlData);
      m_Model.refreshObject(id, bNotifyNew);
   }
   else {
      pImage = new CSvgImage();
      pImage->m_id = id;
      pImage->setPart(partId, xmlData);
      m_Model.setObject(pImage);
   }
}

void CDisplayServer::removeObject(const std::string& id)
{
   m_Model.removeObject(id);
}

void CDisplayServer::removePart(const std::string& id, const std::string& partId)
{
   m_Model.removePart(id, partId);
}

void CDisplayServer::setTomGineObject(const std::string& id, const std::string& partId, 
      const std::vector<unsigned char>& data)
{
#ifdef V11N_OBJECT_TOMGINE_MODEL
   DTRACE("CDisplayServer::setTomGineObject");

   CTomGineModel *pModel = nullptr;
   CDisplayObject *pExisting = m_Model.getObject(id);
   if (pExisting) {
      pModel = dynamic_cast<CTomGineModel*>(pExisting);
      if (! pModel) {
         // The retreived model is of a different type, we must replace it
         m_Model.removeObject(id);
         DMESSAGE("setTomGineObject: Replacing an exisiting object of different type.");
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
   //DTRACE("CDisplayServer::setLuaGlObject");

   CLuaGlScript *pModel = nullptr;
   CDisplayObject *pExisting = m_Model.getObject(id);
   if (pExisting) {
      pModel = dynamic_cast<CLuaGlScript*>(pExisting);
      if (! pModel) {
         // The retreived model is of a different type, we must replace it
         m_Model.removeObject(id);
         DMESSAGE("setLuaGlObject: Replacing an exisiting object of different type.");
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
   // DTRACE("CDisplayServer::setHtml");

   CHtmlObject *pObject = nullptr;
   CDisplayObject *pExisting = m_Model.getObject(id);
   if (pExisting) {
      pObject = dynamic_cast<CHtmlObject*>(pExisting);
      if (! pObject) {
         // The retreived model is of a different type, we must replace it
         m_Model.removeObject(id);
         DMESSAGE("setHtml: Replacing an exisiting object of different type.");
      }
   }

   if (pObject) {
      pObject->setHtml(partId, htmlData);
      m_Model.refreshObject(id);
   }
   else {
      pObject = new CHtmlObject();
      pObject->m_id = id;
      pObject->setHtml(partId, htmlData);
      m_Model.setObject(pObject);
   }
#endif
}

void CDisplayServer::setHtmlHead(const std::string& id, const std::string& partId, const std::string& htmlData)
{
#ifdef V11N_OBJECT_HTML
   DTRACE("CDisplayServer::setHtmlHead");

   CHtmlObject *pObject = nullptr;
   CDisplayObject *pExisting = m_Model.getObject(id);
   if (pExisting) {
      pObject = dynamic_cast<CHtmlObject*>(pExisting);
      if (! pObject) {
         // The retreived model is of a different type, we must replace it
         m_Model.removeObject(id);
         DMESSAGE("setHtmlHead: Replacing an exisiting object of different type.");
      }
   }

   if (pObject) {
      pObject->setHead(partId, htmlData);
      m_Model.refreshObject(id);
   }
   else {
      pObject = new CHtmlObject();
      pObject->m_id = id;
      pObject->setHead(partId, htmlData);
      m_Model.setObject(pObject);
   }
#endif
}

void CDisplayServer::setHtmlForm(const Ice::Identity& ident, const std::string& id,
      const std::string& partId, const std::string& htmlData)
{
#ifdef V11N_OBJECT_HTML
   DTRACE("CDisplayServer::setHtmlForm");
   
   CHtmlObject *pObject = nullptr;
   CDisplayObject *pExisting = m_Model.getObject(id);
   if (pExisting) {
      pObject = dynamic_cast<CHtmlObject*>(pExisting);
      if (! pObject) {
         // The retreived model is of a different type, we must replace it
         m_Model.removeObject(id);
         DMESSAGE("setHtmlForm: Replacing an exisiting object of different type.");
      }
   }

   if (pObject) {
      CHtmlChunk* pForm = nullptr;
      pForm = pObject->setForm(ident, partId, htmlData);
      if (pForm) pForm->Observers.addObserver(this);
      m_Model.refreshObject(id);
   }
   else {
      pObject = new CHtmlObject();
      pObject->m_id = id;
      CHtmlChunk* pForm = pObject->setForm(ident, partId, htmlData);
      if (pForm) pForm->Observers.addObserver(this);
      m_Model.setObject(pObject);
   }
#endif
}

void CDisplayServer::setActiveHtml(const Ice::Identity& ident, const std::string& id, const std::string& partId,
      const std::string& htmlData)
{
#ifdef V11N_OBJECT_HTML
   DTRACE("CDisplayServer::setActiveHtml");

   CHtmlObject *pObject = nullptr;
   CDisplayObject *pExisting = m_Model.getObject(id);
   if (pExisting) {
      pObject = dynamic_cast<CHtmlObject*>(pExisting);
      if (! pObject) {
         // The retreived model is of a different type, we must replace it
         m_Model.removeObject(id);
         DMESSAGE("setActiveHtml: Replacing an exisiting object of different type.");
      }
   }

   if (pObject) {
      //if (htmlData.size() < 1) pObject->removePart(partId); --> use RemovePart for this
      CHtmlChunk* pChunk = pObject->setActiveHtml(ident, partId, htmlData);
      if (pChunk) pChunk->Observers.addObserver(this);
      m_Model.refreshObject(id);
   }
   else {
      pObject = new CHtmlObject();
      pObject->m_id = id;
      CHtmlChunk* pChunk = pObject->setActiveHtml(ident, partId, htmlData);
      if (pChunk) pChunk->Observers.addObserver(this);
      m_Model.setObject(pObject);
   }
#endif
}

void CDisplayServer::setHtmlFormData(const std::string& id, const std::string& partId,
      const std::map<std::string, std::string>& fields)
{
#ifdef V11N_OBJECT_HTML
   DTRACE("CDisplayServer::setHtmlFormData");

   CDisplayObject *pExisting = m_Model.getObject(id);
   if (!pExisting) return;

   CHtmlObject *pModel = dynamic_cast<CHtmlObject*>(pExisting);
   if (! pModel) return;

   CHtmlChunk* pForm = pModel->getPart(partId);
   if (! pForm) return;
   if (pForm->type() != CHtmlChunk::form) return;

   pForm->syncFormData(fields, /*notify=*/true);
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
      double x, double y, double z, const Visualization::Quaternion& rotation)
{
   CDisplayObject *pExisting = m_Model.getObject(id);
   if (!pExisting) return;

   std::vector<double> pos, rot;
   pos.reserve(3); rot.reserve(4);
   pos.push_back(x);
   pos.push_back(y);
   pos.push_back(z);
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

void CDisplayServer::addDialog(const Ice::Identity& ident, const std::string& dialogId,
      const std::string& designCode, const std::string& scriptCode,
      const std::string& constructorName)
{
   DTRACE("CDisplayServer::addDialog");
   CGuiDialog* pDialog = new CGuiDialog();
   pDialog->m_id = dialogId;
   pDialog->m_designCode = designCode;
   pDialog->m_scriptCode = scriptCode;
   pDialog->m_ctorName = constructorName;

   if (!m_Model.addGuiDialog(pDialog)) {
      delete pDialog;
      pDialog = nullptr;
   }

   if (pDialog) {
      pDialog->Observers.addObserver(this);
      bool bFound = false;
      for (int i = 0; !bFound && i < pDialog->m_dataOwners.size(); i++) {
         if (pDialog->m_dataOwners[i] == ident)
            bFound = true;
      }

      // TODO: push_back only if ident not in m_dataOwner AND all the other parameters have the same value
      // as the primary dialog!
      if (! bFound)
         pDialog->m_dataOwners.push_back(ident);
   }
}

void CDisplayServer::execInDialog(const std::string& dialogId, const std::string& scriptCode)
{
   DTRACE("CDisplayServer::execInDialog: " << scriptCode);
   CGuiDialog* pDialog = m_Model.getDialog(dialogId);
   if (!pDialog) return;
   pDialog->execute(scriptCode);
}

void CDisplayServer::addAction(const Ice::Identity& ident, const std::string& viewId,
      const Visualization::ActionInfo& action)
{
   DTRACE("CDisplayServer::addAction");
   CGuiElement *pgel = new CGuiElement();
   pgel->m_type = CGuiElement::wtAction;
   pgel->m_viewId = viewId;
   pgel->m_id = action.id;
   pgel->m_label = action.label;
   pgel->m_iconLabel = (action.iconLabel.length() > 0) ? action.iconLabel : action.label;
   pgel->m_tooltip = (action.tooltip.length() > 0) ? action.tooltip : action.label;
   pgel->m_iconSvg = action.iconSvg;
   pgel->m_bCheckable = action.checkable;
   pgel->m_dataOwner = ident;
   pgel->Observers += this;
   if (m_Model.addGuiElement(pgel))
      debug(std::string("Action added: ") + action.id);
   else {
      if (pgel) delete pgel;
      debug(std::string("Gui element already exists: ") + action.id);
   }
}

class CUiDataChangeOperation: public CDisplayServerI::CQueuedOperation
{
public:
   CGuiElement* pElement;
   std::string value;
   CUiDataChangeOperation(const Ice::Identity& clientId)
      : CDisplayServerI::CQueuedOperation(clientId)
   {
      pElement = nullptr;
   }

   void execute(Visualization::EventReceiverPrx& pClient)
   {
      if (! pElement) return;
      Visualization::TEvent event;
      switch (pElement->m_type) {
         case CGuiElement::wtCheckBox:
            event.type = Visualization::evCheckBoxChange; break;
         case CGuiElement::wtButton:
            event.type = Visualization::evButtonClick; break;
         case CGuiElement::wtDropList:
            event.type = Visualization::evDropListChange; break;
         case CGuiElement::wtAction:
            if (pElement->m_bCheckable)
               event.type = Visualization::evCheckBoxChange;
            else
               event.type = Visualization::evButtonClick;
            break;
         default:
            DMESSAGE("CUiDataChangeOperation: unknown CGuiElement type");
            return;
      }

      event.objectId = pElement->m_viewId;
      event.sourceId = pElement->m_id;
      event.data = value;

      pClient->handleEvent(event);
   }

};

// Add change notification to queue, processed in <url:#tn=CDisplayServerI::run>
void CDisplayServer::onGuiElement_CtrlDataChanged(CGuiElement *pElement, const std::string& newValue)
{
   debug(std::string("Time: ") + sfloat (fclocks()));
   debug(std::string("New value ") + pElement->m_id + "=" + newValue);
   DTRACE("CDisplayServer::onGuiElement_CtrlDataChanged");

   if (! hIceDisplayServer.get()) return;
   if (! pElement) return;

   CUiDataChangeOperation* pOp = new CUiDataChangeOperation(pElement->m_dataOwner);
   pOp->pElement = pElement;
   pOp->value = newValue;
   hIceDisplayServer->addOperation(pOp);
}

void CDisplayServer::onGuiDialog_setHtmlChunk(CGuiDialog* pDialog, const std::string& object,
      const std::string& part, const std::string& value)
{
   setHtml(object, part, value);
}

class CDialogValueChangeOperation: public CDisplayServerI::CQueuedOperation
{
public:
   CGuiDialog* pDialog;
   std::string name;
   std::string value;
   CDialogValueChangeOperation(const Ice::Identity& clientId)
      : CDisplayServerI::CQueuedOperation(clientId)
   {
      pDialog = nullptr;
   }

   void execute(Visualization::EventReceiverPrx& pClient)
   {
      if (! pDialog) return;
      pClient->onDialogValueChanged(pDialog->m_id, name, value);
   }
};

void CDisplayServer::onGuiDialog_setValue(CGuiDialog *pDialog, const std::string& name, const std::string& value)
{
   DTRACE("CDisplayServer::onGuiDialog_setValue");
   if (! pDialog)
      return;

   auto it = pDialog->m_dataOwners.begin();
   for (; it != pDialog->m_dataOwners.end(); ++it) {
      CDialogValueChangeOperation* pOp = new CDialogValueChangeOperation(*it);
      pOp->pDialog = pDialog;
      pOp->name = name;
      pOp->value = value;
      hIceDisplayServer->addOperation(pOp);
      DMESSAGE("setValue " << name << "=" << value);
   }
}

class CDialogCallOperation: public CDisplayServerI::CQueuedOperation
{
public:
   CGuiDialog* pDialog;
   std::string name;
   std::string value;
   CDialogCallOperation(const Ice::Identity& clientId)
      : CDisplayServerI::CQueuedOperation(clientId)
   {
      pDialog = nullptr;
   }

   void execute(Visualization::EventReceiverPrx& pClient)
   {
      if (! pDialog) return;
      pClient->handleDialogCommand(pDialog->m_id, name, value);
   }
};

void CDisplayServer::onGuiDialog_call(CGuiDialog *pDialog, const std::string& name, const std::string& value)
{
   DTRACE("CDisplayServer::onGuiDialog_call");
   if (! pDialog)
      return;

   auto it = pDialog->m_dataOwners.begin();
   if (it != pDialog->m_dataOwners.end()) {
      CDialogCallOperation* pOp = new CDialogCallOperation(*it);
      pOp->pDialog = pDialog;
      pOp->name = name;
      pOp->value = value;
      hIceDisplayServer->addOperation(pOp);
      DMESSAGE("call " << name << "(" << value << ")");
   }
}

class CUiGetControlStateOperation: public CDisplayServerI::CQueuedOperation
{
public:
   CGuiElement* pElement;
   CUiGetControlStateOperation(const Ice::Identity& clientId)
      : CDisplayServerI::CQueuedOperation(clientId)
   {
      pElement = nullptr;
   }

   void execute(Visualization::EventReceiverPrx& pClient)
   {
      DTRACE("CUiGetControlStateOperation::execute");
      if (! pElement) return;
      // Get data from the ui element owner
      std::string val = pClient->getControlState(pElement->m_id);
      DMESSAGE("got value '" << val << "' for " << pElement->m_id);
      if (val.size() > 0) {
         pElement->syncControlState(val);
      }
   }
};

// Add request for value to queue, processed in <url:#tn=CDisplayServerI::run>
void CDisplayServer::getControlStateAsync(CGuiElement *pElement)
{
   DTRACE("CDisplayServer::getControlStateAsync");
   if (! hIceDisplayServer.get()) return;
   if (! pElement) return;

   CUiGetControlStateOperation* pOp = new CUiGetControlStateOperation(pElement->m_dataOwner);
   pOp->pElement = pElement;
   hIceDisplayServer->addOperation(pOp);
}

class CHtmlFormSubmitOperation: public CDisplayServerI::CQueuedOperation
{
public:
   TFormValues values;
   std::string objectId;
   std::string partId;
   CHtmlFormSubmitOperation(const Ice::Identity& clientId)
      : CDisplayServerI::CQueuedOperation(clientId)
   {
   }

   void execute(Visualization::EventReceiverPrx& pClient)
   {
      pClient->handleForm(objectId, partId, values);
   }
};

void CDisplayServer::onFormSubmitted(CHtmlChunk* pForm, const TFormValues& newValues)
{
   DTRACE("CDisplayServer::onFormSubmitted");
   if (! hIceDisplayServer.get()) return;
   if (! pForm) return;

   CHtmlFormSubmitOperation* pOp = new CHtmlFormSubmitOperation(pForm->m_dataOwner);
   pOp->objectId = pForm->id();
   pOp->partId = pForm->partId();
   pOp->values = newValues;
   hIceDisplayServer->addOperation(pOp);
}

class CHtmlGetFormDataOperation: public CDisplayServerI::CQueuedOperation
{
   CDisplayServer* m_pServer;
public:
   std::string objectId;
   std::string partId;
   CHtmlGetFormDataOperation(const Ice::Identity& clientId, CDisplayServer* pServer)
      : CDisplayServerI::CQueuedOperation(clientId)
   {
      m_pServer = pServer;
   }

   void execute(Visualization::EventReceiverPrx& pClient)
   {
      if (! m_pServer) return;
      TFormValues values;
      if (pClient->getFormData(objectId, partId, values)) {
         m_pServer->setHtmlFormData(objectId, partId, values);
      }
   }
};

void CDisplayServer::getFormStateAsync(CHtmlChunk* pForm)
{
   DTRACE("CDisplayServer::getFormStateAsync");
   if (! hIceDisplayServer.get()) return;

   CHtmlGetFormDataOperation* pOp = new CHtmlGetFormDataOperation(pForm->m_dataOwner, this);
   pOp->objectId = pForm->id();
   pOp->partId = pForm->partId();
   hIceDisplayServer->addOperation(pOp);
}


class CHtmlEventOperation: public CDisplayServerI::CQueuedOperation
{
public:
   Visualization::TEvent event;
   CHtmlEventOperation(const Ice::Identity& clientId)
      : CDisplayServerI::CQueuedOperation(clientId)
   {
   }

   void execute(Visualization::EventReceiverPrx& pClient)
   {
      pClient->handleEvent(event);
   }
};

void CDisplayServer::onHtmlClick(CHtmlChunk *pChunk, const std::string& ctrlId)
{
   DTRACE("CDisplayServer::onHtmlClick");
   if (! hIceDisplayServer.get()) return;
   if (! pChunk) return;
   if (ctrlId.size() < 1) return;
   CHtmlEventOperation* pOp = new CHtmlEventOperation(pChunk->m_dataOwner);
   pOp->event.type = Visualization::evHtmlOnClick;
   pOp->event.objectId = pChunk->id();
   pOp->event.partId = pChunk->partId();
   pOp->event.sourceId = ctrlId;

   hIceDisplayServer->addOperation(pOp);
}

void CDisplayServer::onHtmlSendValue(CHtmlChunk *pChunk, const std::string& ctrlId,
      const std::string& value)
{
   DTRACE("CDisplayServer::onHtmlSendValue");
   if (! hIceDisplayServer.get()) return;
   if (! pChunk) return;
   if (ctrlId.size() < 1) return;
   CHtmlEventOperation* pOp = new CHtmlEventOperation(pChunk->m_dataOwner);
   pOp->event.type = Visualization::evHtmlOnClick;
   pOp->event.objectId = pChunk->id();
   pOp->event.partId = pChunk->partId();
   pOp->event.sourceId = ctrlId;
   pOp->event.data = value;

   hIceDisplayServer->addOperation(pOp);
}

std::string CDisplayServer::getPersistentStorageName()
{
   // TODO: add config parameter for DisplayServerData.ini
   return "DisplayServerData.ini";
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
   m_pEventSenderThread = nullptr;
}

void CDisplayServerI::startEventServer()
{
   m_pDisplayServer->debug("CDisplayServerI->startEventServer()");
   try {
      if (m_pEventSenderThread.get() != nullptr) {
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
      CPtrVector<CQueuedOperation> operations;

      {
         // SYNC: Lock the monitor
         IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
         // SYNC: Unlock the monitor and wait for notify() or timeout
         if (m_OperationQueue.size() < 1)
            m_EventMonitor.timedWait(IceUtil::Time::seconds(2));
         // SYNC: Continue with a locked monitor

         if(m_stopEventServer) {
            break;
         }

         clients = m_EventClients;
         operations = m_OperationQueue;
         m_OperationQueue.clear();
         // SYNC: unlock the monitor
      }

      Visualization::TEvent event;

      if(!clients.empty() && !operations.empty()) {
         DTRACE("Sending operations.");
         CQueuedOperation *pOp;

         // XXX: we are assuming that a pElement won't be deleted while in queue
         // If at some point we start deleting pElements, this code should be reviewed,
         // and the content of CGuiElementValue changed.
         set<Visualization::EventReceiverPrx>::iterator p;
         for(p = clients.begin(); p != clients.end(); p++) {
            Visualization::EventReceiverPrx pRcvr = *p;
            Ice::Identity idReceiver = pRcvr->ice_getIdentity();
            FOR_EACH(pOp, operations) {
               if (!pOp) continue;
               if (!p->get() || pOp->m_clientId != idReceiver) continue;

               try {
                  DMESSAGE("Executing");
                  pOp->execute(pRcvr);
               }
               catch(const Ice::Exception& ex) {
                  DMESSAGE(" *** execute crashed with Ice::Exception: " << ex);
                  //IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
                  //m_EventClients.erase(*p);
                  DMESSAGE(" *** idReceiver: " << idReceiver.name << "/" << idReceiver.category);
                  break;
               }
               catch(...) {
                  DMESSAGE(" *** execute crashed for unknonw reasons");
                  break;
               }
            }
         }
         FOR_EACH(pOp, operations) {
            if(pOp) delete pOp;
         }
         operations.clear();
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
      m_pEventSenderThread = nullptr; // Resolve cyclic dependency.
   }

   if (eventSenderThread.get() != nullptr) {
      m_pDisplayServer->debug("Waiting for m_pEventSenderThread to finish.");
      eventSenderThread->getThreadControl().join();
      m_pDisplayServer->debug("m_pEventSenderThread joined (finished).");
   }
}

void CDisplayServerI::addClient(const Ice::Identity& ident, const std::string& host, Ice::Int port, const Ice::Current& current)
{
   IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
   cout << "v11n: adding client `" << ident.name << ":" << ident.category
      << "'@" << host << ":" << port << endl;
   Ice::ObjectPrx prx = m_pDisplayServer->getIceServer(ident.name, ident.category, host, port);

   Visualization::EventReceiverPrx client = Visualization::EventReceiverPrx::uncheckedCast(prx);
   m_EventClients.insert(client);

   // from the demo (doesn't work, returns NULL even on the same machine):
   // Visualization::EventReceiverPrx client =
   //      Visualization::EventReceiverPrx::uncheckedCast(current.con->createProxy(ident));
   // -------------
   cout << "v11n: added `" << ident.name << ":" << ident.category << "'"<< endl;
}

void CDisplayServerI::addOperation(CQueuedOperation* pOperation)
{
   DTRACE("CDisplayServerI::addOperation");
   if (!pOperation) return;

   IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
   m_OperationQueue.push_back(pOperation);
   m_EventMonitor.notify();
}

}} // namespace
// vim:fileencoding=utf-8:sw=3:ts=8:et

