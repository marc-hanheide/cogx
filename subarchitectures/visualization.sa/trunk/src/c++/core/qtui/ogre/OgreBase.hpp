#ifndef _OGRE_OGREBASE_HPP_4F44E353_
#define _OGRE_OGREBASE_HPP_4F44E353_

#include <OGRE/Ogre.h>
#include <QWidget>
#include <string>

class COgreBase
{
private:
   friend class COgreDestroyer;
   static int id;
   static Ogre::Root *mOgreRoot;

protected:
   Ogre::RenderWindow *mOgreWindow;
   Ogre::Camera *mOgreCamera;
   Ogre::Viewport *mOgreViewport;
   Ogre::SceneManager *mOgreSceneMgr;
   Ogre::Light *mOgreMainLight;

public:
   static std::string mFnamePlugins;
   static std::string mFnameConfig;
   static std::string mFnameLog;
   static std::string mFnameResources;

private:
   void initOgre();
   void initDisplay();

protected:
   virtual void setupResources();
   virtual void createView();
   virtual void setupView(Ogre::RenderWindow *pWindow, Ogre::Camera *pCamera, Ogre::Viewport *pViewport);
   virtual void setupScene(Ogre::SceneManager *pScnMgr);
   virtual QWidget* getOgreWidget() = 0;
   virtual void recreateOgreWidget(QWidget* pWidget, WId window_id) = 0;

public:
   COgreBase();
   virtual ~COgreBase();
   Ogre::Root* getOgreRoot();
   bool checkOgreWindow(bool create=true);
};

inline
Ogre::Root* COgreBase::getOgreRoot()
{
   if (!mOgreRoot) {
      initOgre();
   }
   return mOgreRoot;
}

inline
bool COgreBase::checkOgreWindow(bool create)
{
   if (!mOgreWindow && create) {
      initDisplay();
   }
   return mOgreWindow != nullptr;
}

#endif /* _OGRE_OGREBASE_HPP_4F44E353_ */
