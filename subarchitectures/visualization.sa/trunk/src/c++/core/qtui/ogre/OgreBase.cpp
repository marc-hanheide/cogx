
#include "OgreBase.hpp"
#if !defined(Q_WS_WIN)
#include <QX11Info>
#endif

std::string COgreBase::mFnamePlugins;
std::string COgreBase::mFnameConfig;
std::string COgreBase::mFnameLog;
std::string COgreBase::mFnameResources;
int COgreBase::id = 0;
Ogre::Root* COgreBase::mOgreRoot = nullptr;

class COgreDestroyer
{
public:
   ~COgreDestroyer()
   {
      if (COgreBase::mOgreRoot) {
         delete COgreBase::mOgreRoot;
         COgreBase::mOgreRoot = nullptr;
      }
   }
} destroyer;


COgreBase::COgreBase()
{
   id++;
   mOgreWindow = nullptr;
   mOgreCamera = nullptr;
   mOgreViewport = nullptr;
   mOgreSceneMgr = nullptr;
   mOgreMainLight = nullptr;
}

COgreBase::~COgreBase()
{
   //if(mOgreViewport){
   //   delete mOgreViewport;
   //   mOgreViewport = nullptr;
   //}

   // XXX mOgreMainLight ?

   if(mOgreSceneMgr){
      delete mOgreSceneMgr;
      mOgreSceneMgr = nullptr;
   }
   if(mOgreWindow){
      delete mOgreWindow;
      mOgreWindow = nullptr;
   }
}

void COgreBase::initOgre()
{
   mOgreRoot = new Ogre::Root(mFnamePlugins, mFnameConfig, mFnameLog);

   if (mFnameConfig == "") {
      Ogre::RenderSystemList::const_iterator renderers = mOgreRoot->getAvailableRenderers().begin();
      while(renderers != mOgreRoot->getAvailableRenderers().end()) {
         Ogre::String rName = (*renderers)->getName();
         //printf(" **** %s\n", rName.c_str());
         if (rName == "OpenGL Rendering Subsystem")
            break;
         renderers++;
      }
      Ogre::RenderSystem *renderSystem = *renderers;
      mOgreRoot->setRenderSystem(renderSystem);
   }

   setupResources();
   mOgreRoot->restoreConfig();
   mOgreRoot->initialise(false);
}

void COgreBase::setupResources()
{
   //mLogListener = new myLogListener();
   //LogManager::getSingleton().getDefaultLog()->addListener(mLogListener);

   if (mFnameResources == "") {
      return;
   }

   Ogre::ConfigFile cf;
   cf.load(mFnameResources);

   Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();
   Ogre::String secName, typeName, archName;
   while(seci.hasMoreElements()) {
      secName = seci.peekNextKey();
      Ogre::ConfigFile::SettingsMultiMap* settings = seci.getNext();
      Ogre::ConfigFile::SettingsMultiMap::iterator i;
      for(i=settings->begin(); i!=settings->end(); ++i) {
         typeName = i->first;
         archName = i->second;
         Ogre::ResourceGroupManager::getSingleton().addResourceLocation(archName, typeName, secName);
      }
   }
}

void COgreBase::initDisplay()
{
   getOgreRoot();
   createView();

   assert(mOgreWindow && mOgreCamera && mOgreViewport);
   setupView(mOgreWindow, mOgreCamera, mOgreViewport);

   Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

   assert(mOgreSceneMgr);
   setupScene(mOgreSceneMgr);
}

void COgreBase::createView()
{
   if (mOgreWindow) {
      return;
   }

   QWidget* pWidget = getOgreWidget();
   Ogre::NameValuePairList params;

#if !defined(Q_WS_WIN)
   QWidget *q_parent = dynamic_cast <QWidget *> (pWidget->parent());
   QX11Info xInfo = pWidget->x11Info();

   params["parentWindowHandle"] = Ogre::StringConverter::toString ((unsigned long)xInfo.display()) +
      ":" + Ogre::StringConverter::toString ((unsigned int)xInfo.screen()) +
      ":" + Ogre::StringConverter::toString ((unsigned long)q_parent->winId());

#else
   params["externalWindowHandle"] = Ogre::StringConverter::toString((size_t)(HWND)pWidget->winId());
#endif

   QString xid = QString("OgreWindow::%1").arg(id);
   mOgreWindow = mOgreRoot->createRenderWindow(xid.toStdString(),
         pWidget->width(), pWidget->height(), false /*full screen*/, &params);

   mOgreSceneMgr = mOgreRoot->createSceneManager(Ogre::ST_GENERIC);

   xid = QString("OgreCamera::%1").arg(id);
   mOgreCamera = mOgreSceneMgr->createCamera(xid.toStdString());
   mOgreCamera->setPosition(Ogre::Vector3(0,100,0));
   mOgreCamera->lookAt(Ogre::Vector3(0,100,100));
   mOgreCamera->setNearClipDistance(1);

   Ogre::Viewport* pvp;
   mOgreViewport = mOgreWindow->addViewport(mOgreCamera);
   mOgreViewport->setBackgroundColour(Ogre::ColourValue(0.58, 0.65, 0.76, 1));
   pvp = mOgreViewport;

   mOgreCamera->setAspectRatio(Ogre::Real(pvp->getActualWidth()) / Ogre::Real(pvp->getActualHeight()));

   mOgreSceneMgr->setAmbientLight(Ogre::ColourValue(1, 1, 1));
   xid = QString("OgreMainLight::%1").arg(id);
   mOgreMainLight = mOgreSceneMgr->createLight(xid.toStdString());
   mOgreMainLight->setPosition(0, 100, -500);

#if !defined(Q_WS_WIN) // Linux
   WId window_id;
   mOgreWindow->getCustomAttribute("GLXWINDOW", &window_id);

   recreateOgreWidget(pWidget, window_id);
   // Take over the ogre created window.
   //QWidget::create(window_id);
   //pWidget->create(window_id);

   mOgreWindow->reposition(pWidget->x(), pWidget->y());
#endif
}

void COgreBase::setupView(Ogre::RenderWindow *pWindow,
      Ogre::Camera *pCamera, Ogre::Viewport *pViewport)
{
}

void COgreBase::setupScene(Ogre::SceneManager *pScnMgr)
{
   // TODO: move elsewhere, these are global settings!
   Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
   Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(Ogre::TFO_ANISOTROPIC);
   Ogre::MaterialManager::getSingleton().setDefaultAnisotropy(2);
}

