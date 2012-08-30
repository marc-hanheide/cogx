// Based on code from: http://www.ogre3d.org/tikiwiki/QtOgre

#include "QCastViewOgre.hpp"
#include <QToolButton>
#include <QMenu>
#include <QAction>
#include <exception>
#include "../convenience.hpp"

#if 0
class COgreInit
{
private:
   static int count;
   static Ogre::Root *mOgreRoot;
   static void initRoot(const std::string& plugins_file, const std::string& logfile);
   static void configure();
   static void setupResources(const std::string& resources_cfg_file);
public:
   COgreInit()
   {
      count++;
   }

   ~COgreInit()
   {
      count--;
      if (mOgreRoot && !count) {
         delete mOgreRoot;
         mOgreRoot = 0;
      }
   }

   Ogre::Root* getRoot()
   {
      if (!mOgreRoot) {
         //initOgre("../bin/plugins.cfg", "../bin/ogre.cfg", "../bin/ogre.log");
         try {
            initRoot("ogre_plugins.cfg", "ds-ogre.log");
            setupResources("ogre_resources.cfg");
            configure();
         }
         catch (std::exception& e) {
            printf(" **** %s\n", e.what());
         }
      }
      return mOgreRoot;
   }
};

int COgreInit::count = 0;
Ogre::Root* COgreInit::mOgreRoot = nullptr;
COgreInit OgreInit;

void COgreInit::initRoot(const std::string& plugins_file, const std::string& ogre_log)
{
   DTRACE("COgreInit::initRoot");
   // create the main ogre object
   mOgreRoot = new Ogre::Root(plugins_file, "", ogre_log);

}

void COgreInit::setupResources(const std::string& resources_cfg_file)
{
   DTRACE("COgreInit::setupResources");

   // Load resource paths from config file
   Ogre::ConfigFile cf;
   cf.load(resources_cfg_file);

   // Go through all sections & settings in the file
   Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

   Ogre::String secName, typeName, archName;
   while (seci.hasMoreElements()) {
      secName = seci.peekNextKey();
      Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
      Ogre::ConfigFile::SettingsMultiMap::iterator i;
      for (i = settings->begin(); i != settings->end(); ++i) {
         typeName = i->first;
         archName = i->second;
         Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
               archName, typeName, secName);
      }
   }
}


void COgreInit::configure()
{
   DTRACE("COgreInit::configure");
   // setup a renderer
#if 0 // Ogre 1.6
   Ogre::RenderSystemList *renderers = mOgreRoot->getAvailableRenderers();
   assert(!renderers->empty()); // we need at least one renderer to do anything useful

   Ogre::RenderSystem *renderSystem;
   renderSystem = chooseRenderer(renderers);

   assert(renderSystem); // user might pass back a null renderer, which would be bad!
#else // ogre 1.7.3
   Ogre::RenderSystemList::const_iterator renderers = mOgreRoot->getAvailableRenderers().begin();
   while(renderers != mOgreRoot->getAvailableRenderers().end()) {
      Ogre::String rName = (*renderers)->getName();
      printf(" **** %s\n", rName.c_str());
      if (rName == "OpenGL Rendering Subsystem")
         break;
      renderers++;
   }
   Ogre::RenderSystem *renderSystem = *renderers;
   DMESSAGE("renderSystem ok");
#endif

   mOgreRoot->setRenderSystem(renderSystem);
#if 0
   //QString dimensions = QString("%1x%2")
   //   .arg(this->width())
   //   .arg(this->height());
   QString dimensions = "800x600";

   renderSystem->setConfigOption("Video Mode", dimensions.toStdString());

   // initialize without creating window
   mOgreRoot->getRenderSystem()->setConfigOption("Full Screen", "No");
   mOgreRoot->saveConfig();
#endif
   mOgreRoot->initialise(false, "CAST Viewer OGRE Window"); // don't create a window

#if 0
   // setup resources
   // Only add the minimally required resource locations to load up the Ogre head mesh
   //Ogre::ResourceGroupManager& rgm = Ogre::ResourceGroupManager::getSingleton();
   //std::string r1 = "output/Media";
   //rgm.addResourceLocation(r1 + "/materials/programs", "FileSystem", "General");
   //rgm.addResourceLocation(r1 + "/materials/scripts", "FileSystem", "General");
   //rgm.addResourceLocation(r1 + "/materials/textures", "FileSystem", "General");
   //rgm.addResourceLocation(r1 + "/models", "FileSystem", "General");   

   DMESSAGE("configured");
#endif
}

//int QCastViewOgre::winId = 0;
//int QCastViewOgre::camId = 0;
#endif

QCastViewOgre::QCastViewOgre(QWidget *parent):
   QGLWidget(parent)
   //mOgreWindow(nullptr)
{
   DTRACE("QCastViewOgre::QCastViewOgre");
   pView = nullptr;
   COgreBase::mFnamePlugins = "ogre_plugins.cfg";
   COgreBase::mFnameResources = "ogre_resources.cfg";
   COgreBase::mFnameLog = "ds-ogre.log";
   //mOgreWindow = nullptr;
   //mCamera = nullptr;
   //mViewport = nullptr;
   //mSceneMgr = nullptr;
   //mOgreRoot = OgreInit.getRoot();

}

QCastViewOgre::~QCastViewOgre()
{
   DTRACE("QCastViewOgre::~QCastViewOgre");
   if (pView != nullptr) {
      pView->viewObservers.removeObserver(this);
   }
   pView = nullptr;

#if 0
   //mOgreRoot->shutdown();
   //delete mOgreRoot;
   if (mOgreWindow) {
      mOgreWindow->removeAllViewports();
      // delete mViewport; // XXX - this crashed; does removeAllViewports destroy the viewports?
      mViewport = nullptr;
      mOgreRoot->detachRenderTarget(mOgreWindow);
      delete mOgreWindow;
      mOgreWindow = nullptr;
   }
   if (mSceneMgr) {
      if (mCamera) {
         mSceneMgr->destroyCamera(mCamera);
         mCamera = nullptr;
      }
      if (mOgreRoot) {
         mOgreRoot->destroySceneManager(mSceneMgr);
         mOgreRoot = nullptr;
      }
      mSceneMgr = nullptr;
   }
#endif
   destroy();
}
 
void QCastViewOgre::initializeGL()
{
   DTRACE("QCastViewOgre::initializeGL");
   //setAttribute(Qt::WA_PaintOnScreen, true);
   //setAttribute(Qt::WA_NoBackground);
   checkOgreWindow(true);

#if 0
   // Get the parameters of the window QT created
   Ogre::String winHandle;
#ifdef WIN32
   winHandle = Ogre::StringConverter::toString((unsigned long)(this->parentWidget()->winId()));
#elif MACOS
   winHandle = Ogre::StringConverter::toString(winId());
#else
   // Unix code
   QX11Info info = x11Info();
   QString id = QString("%1:%2:%3")
      .arg((unsigned long)info.display())
      .arg((unsigned int)info.screen())
      .arg((unsigned long)this->parentWidget()->winId());
   winHandle = id.toStdString();
#endif

   DMESSAGE("Creating window");
   Ogre::NameValuePairList params;
   winId++;
   QString winname = QString("QCastViewOgre::RenderWin::%1").arg(winId);
#ifndef MACOS
   // code for Windows and Linux
   params["parentWindowHandle"] = winHandle;
   mOgreWindow = mOgreRoot->createRenderWindow(winname.toStdString(),
         this->width(), this->height(),
         false, &params);

   mOgreWindow->setActive(true);
   WId ogreWinId = 0x0;
   mOgreWindow->getCustomAttribute("WINDOW", &ogreWinId);
   assert(ogreWinId);
   this->create(ogreWinId);
#else
   // code for Mac
   params["externalWindowHandle"] = winHandle;
   params["macAPI"] = "cocoa";
   params["macAPICocoaUseNSView"] = "true";
   mOgreWindow = mOgreRoot->createRenderWindow(winname.toStdString(),
         width(), height(), false, &params);
   mOgreWindow->setActive(true);
   makeCurrent();
#endif
   DMESSAGE("Initilaising resources");
   // this has to be done after the ogre window is created
   Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

   // Ogre Initialization
   DMESSAGE("Creating scene_manager");
   Ogre::SceneType scene_manager_type = Ogre::ST_GENERIC; // Ogre::ST_EXTERIOR_CLOSE;
   mSceneMgr = mOgreRoot->createSceneManager(scene_manager_type);
   mSceneMgr->setAmbientLight(Ogre::ColourValue(1,1,1));
   // Create a light
   Ogre::Light* l = mSceneMgr->createLight("MainLight");
   l->setPosition(20,80,50);

   DMESSAGE("Creating scene");
   Ogre::Entity* something = mSceneMgr->createEntity("Head", "ogrehead.mesh");
   Ogre::SceneNode* headNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
   headNode->attachObject(something);

   DMESSAGE("Creating camera");
   camId++;
   QString camname = QString("QCastViewOgre::Camera::%1").arg(camId);
   mCamera = mSceneMgr->createCamera(camname.toStdString());
   mCamera->setPosition(Ogre::Vector3(0,0,80));
   // Look back along -Z
   mCamera->lookAt(Ogre::Vector3(0,0,-300));
   mCamera->setNearClipDistance(5);
   //mCamera->setPosition(Ogre::Vector3(0,1,0));
   //mCamera->lookAt(Ogre::Vector3(0,0,0));
   //mCamera->setNearClipDistance(1.0);
   //mCamera->setAutoAspectRatio(true); // note: camera must be used with only one viewport

   DMESSAGE("Creating Viewport");
   mViewport = mOgreWindow->addViewport(mCamera);
   mViewport->setBackgroundColour(Ogre::ColourValue(0.8,0.8,1));
   mCamera->setAspectRatio(
        Ogre::Real(mViewport->getActualWidth()) / Ogre::Real(mViewport->getActualHeight()));

   //mOgreWindow->addListener(this);
#endif
}

class CGlWidgetTextWriter: public cogx::display::CGlTextWriter
{
   QGLWidget* pWriter;
public:
   CGlWidgetTextWriter(QGLWidget* pWidget)
   {
      pWriter = pWidget;
   }
   void renderText(double x, double y, double z, const std::string& text, double size)
   {
      pWriter->renderText(x, y, z, QString::fromStdString(text));
   }
};

void QCastViewOgre::paintGL()
{
   DTRACE("QCastViewOgre::paintGL");
   checkOgreWindow(true);
   assert(mOgreWindow);
   getOgreRoot()->renderOneFrame();

   //if (pView) {
   //   glMatrixMode(GL_MODELVIEW);
   //   glLoadIdentity();

   //   //la::Vector3 &e = m_camera.eye, &v = m_camera.view, &u = m_camera.up;
   //   //gluLookAt(e.x, e.y, e.z, e.x + v.x, e.y + v.y, e.z + v.z, u.x, u.y, u.z);

   //   //glRotatef(xRot, 1.0, 0.0, 0.0);
   //   //glRotatef(yRot, 0.0, 1.0, 0.0);
   //   //glRotatef(zRot, 0.0, 0.0, 1.0);

   //   CGlWidgetTextWriter writer(this);
   //   pView->drawGL(&writer);
   //}
}

void QCastViewOgre::postViewportUpdate (const Ogre::RenderTargetViewportEvent &evt)
{
   DTRACE("QCastViewOgre::postViewportUpdate");
   if (pView) {
      CGlWidgetTextWriter writer(this);
      pView->drawGL(&writer);
   }
}

void QCastViewOgre::postRenderTargetUpdate (const Ogre::RenderTargetEvent &evt)
{
   DTRACE("QCastViewOgre::postRenderTargetUpdate");
   if (pView) {
      CGlWidgetTextWriter writer(this);
      pView->drawGL(&writer);
   }
}

void QCastViewOgre::resizeGL(int width, int height)
{
   checkOgreWindow(true);
   assert(mOgreWindow);
   mOgreWindow->windowMovedOrResized();

#if 0
   // LuaGl settings
   float aspect = 1.0*width/height;
   float zoomLevel = 0;
   const int ZOOM_RANGE = 5; // -N .. N

   glViewport(0, 0, width, height);
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   if (1) {
      // const float PI = M_PI; // 3.14159265; const float PI2 = PI / 2;
      const float x[3] = {0, 0.5, 1};
      const float zoomangles[3] = {170, 50, 5}; // at x[]
      const float A = 1.0 / ((x[0] - x[1]) * (x[0] - x[2]));
      const float B = 1.0 / ((x[1] - x[0]) * (x[1] - x[2]));
      const float C = 1.0 / ((x[2] - x[0]) * (x[2] - x[1]));
      float zl = (zoomLevel + ZOOM_RANGE) / (2 * ZOOM_RANGE); // 0 .. 1
      float ang =
         zoomangles[0] * (A * (zl-x[1]) * (zl-x[2])) +
         zoomangles[1] * (B * (zl-x[0]) * (zl-x[2])) +
         zoomangles[2] * (C * (zl-x[0]) * (zl-x[1]));

      if (ang < 5) ang = 5;
      if (ang > 170) ang = 170;
      gluPerspective(ang, aspect, 0.001, 1000.0);

      // glFrustum (-1.0*zoomFactor, 1.0*zoomFactor, -1.0*zoomFactor, 1.0*zoomFactor, 1.5, 20.0);
   }
   //else {
   //   glOrtho(-0.5, +0.5, -0.5, +0.5, 4.0, 15.0);
   //}
   glMatrixMode(GL_MODELVIEW);
#endif
}

#if 0
Ogre::RenderSystem* QCastViewOgre::chooseRenderer(Ogre::RenderSystemList *renderers)
{
   // It would probably be wise to do something more friendly 
   // that just use the first available renderer
   return *renderers->begin();
}
#endif

void QCastViewOgre::setView(cogx::display::CDisplayView* pDisplayView)
{
   if (pView != nullptr) {
      pView->viewObservers.removeObserver(this);
   }
   pView = pDisplayView;
   if (pView != nullptr) {
      pView->viewObservers.addObserver(this);
   }

   // Look from the first camera
   CPtrVector<cogx::display::CDisplayCamera> cameras;
   pView->getCameras(cameras);
   if (cameras.size() > 0) selectCamera(cameras.front());
   update();
}

void QCastViewOgre::selectCamera(cogx::display::CDisplayCamera* pCamera)
{
   //xRot = 0;
   //yRot = 0;
   //zRot = 0;
   if (! pCamera) return;
   // TODO: change mCamera
   //m_camera.eye.set(pCamera->xEye, pCamera->yEye, pCamera->zEye);
   //m_camera.view.set(pCamera->xView, pCamera->yView, pCamera->zView);
   //m_camera.up.set(pCamera->xUp, pCamera->yUp, pCamera->zUp);
   //m_camera.normalize();
   //std::cout << " *** Camera set to " << pCamera->name << std::endl;
}

void QCastViewOgre::getViewPosition(std::vector<double>& matrix)
{
   matrix.clear();
   //matrix.push_back(m_camera.eye.x);
   //matrix.push_back(m_camera.eye.y);
   //matrix.push_back(m_camera.eye.z);
   //matrix.push_back(m_camera.view.x);
   //matrix.push_back(m_camera.view.y);
   //matrix.push_back(m_camera.view.z);
   //matrix.push_back(m_camera.up.x);
   //matrix.push_back(m_camera.up.y);
   //matrix.push_back(m_camera.up.z);
   // XXX: Extra scene rotation
   //matrix.push_back(xRot);
   //matrix.push_back(yRot);
   //matrix.push_back(zRot);
}

void QCastViewOgre::setViewPosition(const std::vector<double>& matrix)
{
   if (matrix.size() == 12) {
      //m_camera.eye.set(matrix[0], matrix[1], matrix[2]);
      //m_camera.view.set(matrix[3], matrix[4], matrix[5]);
      //m_camera.up.set(matrix[6], matrix[7], matrix[8]);
      //// XXX: Extra scene rotation
      //xRot = matrix[9];
      //yRot = matrix[10];
      //zRot = matrix[11];
      update();
   }
}

void QCastViewOgre::onViewChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView)
{
   if (pView == this->pView) update();
}

void QCastViewOgre::onCameraItemChanged(int index)
{
   CPtrVector<cogx::display::CDisplayCamera> cameras;
   pView->getCameras(cameras);
   if (index >= (int) cameras.size()) return;
   selectCamera(cameras[index]);
}

void QCastViewOgre::onCameraChangeAction()
{
   QAction *pAct = qobject_cast< QAction* >(QObject::sender());
   if (!pAct) return;
   QVariant v = pAct->data();
   void* pc = v.value<void*>();
   cogx::display::CDisplayCamera* pCamera = static_cast<cogx::display::CDisplayCamera*>(pc);
   //std::cout << pAct << " " << pAct->text().toStdString() << " " << pc << " " << pCamera << std::endl;

   if (pCamera) {
      // TODO: should check if it is valid -- is it in getCameras()?
      selectCamera(pCamera);
   }

   // Replace the action for the button.
   // Parent hierarchy: action -> menu -> button
   QMenu *pMenu = qobject_cast< QMenu* >(pAct->parent());
   if (pMenu) {
      QToolButton* pBut = qobject_cast< QToolButton* >(pMenu->parent());
      if (pBut) {
         QAction *pOldAct = pBut->defaultAction();
         if (pOldAct && pOldAct->parent() == pAct->parent()) {
            //std::cout << "Changing default action" << std::endl;
            pBut->setDefaultAction(pAct);
         }
      }
   }
}

void QCastViewOgre::onActConfigureCameras()
{
   // TODO: display dialog with one list for each camera
   // TODO: save camera selections to the registry; reload on (first) activation
}

void QCastViewOgre::getToolbars(CPtrVector<QToolBar>& toolbars)
{
   if (! pView) return;
   CPtrVector<cogx::display::CDisplayCamera> cameras;
   pView->getCameras(cameras);
   if (cameras.size() < 1) return;
   // pBar->parent will be reset in QViewContainer
   QToolBar *pBar = new QToolBar(QString::fromStdString(pView->m_id), this);
   if (pBar) {
      unsigned int nc = cameras.size();
      if (nc > 3) nc = 3;
      cogx::display::CDisplayCamera* pCamera;
      for (unsigned int i= 0; i < nc; i++) {
         QToolButton *pBut = new QToolButton(pBar);
         pCamera = cameras[i];
         QString text = QString::fromStdString(pCamera->name);
         QAction* pAct = new QAction(QIcon(":/toolButton/camera-photo.png"), text, pBut);
         pAct->setToolTip("Select Camera: " + text);
         pAct->setData(qVariantFromValue((void*)pCamera));
         pBut->setDefaultAction(pAct);
         pBar->addWidget(pBut);
         pBar->connect(pAct, SIGNAL(triggered()), this, SLOT(onCameraChangeAction()));

         // With more than 3 cameras things become complicated...
         if (i == 2 && cameras.size() > 2) {
            QAction *pPopAct;
            QMenu *pMenu = new QMenu(pBut); // parent MUST be button, see onCameraChangeAction
            pBut->setMenu(pMenu);
            pBut->setPopupMode(QToolButton::MenuButtonPopup);

            for (unsigned int j = 0; j < cameras.size(); j++) {
               if (i == j) {
                  pMenu->addAction(pAct);
                  pAct->setParent(pMenu);   // parent MUST be menu, see onCameraChangeAction
               }
               else {
                  pCamera = cameras[j];
                  text = QString::fromStdString(pCamera->name);
                  pPopAct = pMenu->addAction(QIcon(":/toolButton/camera-photo.png"), text);
                  pPopAct->setData(qVariantFromValue((void*)pCamera));
                  pPopAct->setToolTip("Select Camera: " + text);
                  pBar->connect(pPopAct, SIGNAL(triggered()), this, SLOT(onCameraChangeAction()));
               }
            }

            if (0) {
               text = "TODO: Configure camera buttons...";
               pPopAct = pMenu->addAction(QIcon(":/toolButton/camera-photo.png"), text);
               pBar->connect(pPopAct, SIGNAL(triggered()), this, SLOT(onActConfigureCameras()));
            }
         }
      }
      toolbars.push_back(pBar);
   }
}

