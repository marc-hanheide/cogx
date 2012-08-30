#ifndef _QTUI_QCASTVIEWOGRE_HPP_4F422119_
#define _QTUI_QCASTVIEWOGRE_HPP_4F422119_

#include "QCastViewBase.hpp"
#include "ogre/OgreBase.hpp"
#include <OGRE/Ogre.h>
#include <QGLWidget>
#include <QX11Info>
 
class QCastViewOgre :
   public QGLWidget, public QCastViewBase,
   public COgreBase,
   public Ogre::RenderTargetListener
{
   Q_OBJECT;
//private:
//   static int winId;
//   static int camId;
protected:
   cogx::display::CDisplayView* pView;
   QPoint m_lastPos;

//protected:
//   Ogre::Root *mOgreRoot;
//   Ogre::RenderWindow *mOgreWindow;
//   Ogre::Camera *mCamera;
//   Ogre::Viewport *mViewport;
//   Ogre::SceneManager *mSceneMgr;

public:
   QCastViewOgre(QWidget *parent=nullptr);
   virtual ~QCastViewOgre();

protected:
   virtual void initializeGL();
   virtual void paintGL();
   virtual void resizeGL(int width, int height);

protected:
//   void initOgre(const std::string& plugins_file, const std::string& ogre_cfg_file, const std::string& log_file);
//   virtual Ogre::RenderSystem* chooseRenderer(Ogre::RenderSystemList*);
   QWidget* getOgreWidget() /*override*/
   {
      return this;
   }
   void recreateOgreWidget(QWidget* pWidget, WId window_id)
   {
      if (pWidget == this) {
         QGLWidget::create(window_id);
      }
   }

public:
   // QCastViewBase
   void setView(cogx::display::CDisplayView* pDisplayView); /*override*/
   cogx::display::CDisplayView* getView() { return pView; } /*override*/
   operator QWidget&() { return *this; } /*override*/
   void getViewPosition(std::vector<double>& matrix); /*override*/
   void setViewPosition(const std::vector<double>& matrix); /*override*/
   void getToolbars(CPtrVector<QToolBar>& toolbars); /*override*/
   // CDisplayModelObserver
   void onViewChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView); /*override*/

//public slots:
//   void setCameraEye(const linalgebra::Vector3 &e);

private:
   // Ogre::RenderTargetListener
   virtual void postViewportUpdate (const Ogre::RenderTargetViewportEvent &evt); /*override*/ 
   virtual void postRenderTargetUpdate (const Ogre::RenderTargetEvent &evt); /*override*/

private slots:
   void onCameraItemChanged(int index);
   void onCameraChangeAction();
   void onActConfigureCameras();

protected:
   void selectCamera(cogx::display::CDisplayCamera* pCamera);
};
 
#endif /* _QTUI_QCASTVIEWOGRE_HPP_4F422119_ */
