/**
 * @author Marko Mahnič
 * @created September 2010
 */
#ifndef _VIDEOGRABBER_GRABBER_H_4F547509_
#define _VIDEOGRABBER_GRABBER_H_4F547509_

#include "VideoGrabber.h"
#ifdef FEAT_VIDEOGRABBER_POINTCLOUD
#include "PcGrabber.h"
#endif
#include "CoreStructs.h"
#include "ticker.h"

#include <cast/architecture/ManagedComponent.hpp>
#include <castutils/Timers.hpp>

#include <string>
#include <vector>
#include <queue>

namespace cogxgrabber
{

class CGrabber: public cast::ManagedComponent
{
private:
  /**
   * wether we are currently receiving images from the server
   */
  bool m_bReceiving;

  std::vector<CVideoGrabClient*> m_video;
#ifdef FEAT_VIDEOGRABBER_POINTCLOUD
  std::vector<CPcGrabClient*> m_pointcloud;
#endif
  CRecordingInfo m_RecordingInfo;
  bool m_fakeRecording;
  long m_frameGrabMs;
  std::string m_deviceNames;

#ifdef FEAT_VISUALIZATION
  // HACK: The image data in IplImage will point into char data of m_DisplayBuffer.
  // This way an IplImage can be prapared and sent to the display server without
  // copying the data to a temporary vector or Video::Image.
  IplImage* m_pDisplayCanvas; // Image for the DisplayServer
  std::vector<unsigned char> m_DisplayBuffer; // For transfering data to the server;
  castutils::CMilliTimer m_displayTimer;
  void prepareCanvas(int width, int height);
  void releaseCanvas();
  void sendCachedImages();

  class CVvDisplayClient: public cogx::display::CDisplayClient
  {
    CGrabber* pViewer;
  public:
    cogx::display::CFormValues m_frmSettings;
    // Access form variables
    std::string getModelName();
    std::string getDirectory();
    void setDirectory(const std::string& name);
    bool getCreateDirectory();
    std::string getDeviceNames();
    void setDeviceNames(const std::string& name);
    std::string getImageFilenamePatt();
    void setImageFilenamePatt(const std::string& pattern);
    long getCounterDigits();
    void setCounterDigits(long nDigits);
    long getCounterValue();
    void setCounterValue(long nValue);
    long getRecordTimeLimit();
    void setRecordTimeLimit(long nValue);
    float getGrabbingSpeed();
    void setGrabbingSpeed(float fValue);

  public:
    CVvDisplayClient() { pViewer = NULL; }
    void setClientData(CGrabber* pVideoGrabber) { pViewer = pVideoGrabber; }
    void showCurrentSettings();

    void createForms();

    // Send current form data to the DisplayServer
    // (used when the counter must be updated)
    void updateDisplay();

    void handleEvent(const Visualization::TEvent &event); /*override*/
    std::string getControlState(const std::string& ctrlId); /*override*/
    void handleForm(const std::string& id, const std::string& partId,
        const std::map<std::string, std::string>& fields);
    bool getFormData(const std::string& id, const std::string& partId,
        std::map<std::string, std::string>& fields);

  };
  CVvDisplayClient m_display;
#endif

  IceUtil::Handle<IceUtil::Timer> m_pTimer;
  class CGrabQueThread: public IceUtil::Thread, public CTickSyncedTask
  {
    CGrabber *m_pGrabber;

  public:
    struct CFramePack 
    {
      CRecordingInfo frameInfo;
      std::vector<CGrabbedItemPtr> images;
    };

  private:
    std::vector<CFramePack> m_items;
    IceUtil::Monitor<IceUtil::Mutex> m_itemsLock;

  public:
    CGrabQueThread(CGrabber *pGrabber);

    void getItems(std::vector<CFramePack>& items, unsigned int maxItems = 0);
    virtual void grab();
    virtual void run();
  };
  IceUtil::ThreadPtr m_pQueue;
  IceUtil::Handle<CTickerTask> m_pQueueTick;

  class CExtraSaveThread: public IceUtil::Thread
  {
    CGrabber *m_pGrabber;
  private:
    std::queue<CExtraSaverPtr> m_savers;
    IceUtil::Monitor<IceUtil::Mutex> m_saversLock;
    CExtraSaverPtr next()
    {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_saversLock);
      //CExtraSaverPtr saver = m_savers.back();
      //m_savers.pop_back();
      CExtraSaverPtr saver = m_savers.front();
      m_savers.pop();
      return saver;
    }
  public:
    CExtraSaveThread(CGrabber *pGrabber);
    void addSaver(CExtraSaverPtr saver)
    {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_saversLock);
      m_savers.push(saver);
    }
    virtual void run();
    bool isIdle()
    {
      return m_savers.empty();
    }
    size_t itemCount()
    {
      return m_savers.size();
    }
  };
  IceUtil::ThreadPtr m_pExtraSaver;

  class CDrawingThread: public IceUtil::Thread, public CTickSyncedTask
  {
    CGrabber *m_pGrabber;

  public:
    CDrawingThread(CGrabber *pGrabber);
    virtual void run();
  };
  IceUtil::ThreadPtr m_pDrawer;
  IceUtil::Handle<CTickerTask> m_pDrawTick;

protected:
  virtual void configure(const std::map<std::string,std::string> & _config);
  virtual void start();
  virtual void destroy();
  virtual void runComponent();

public:
  CGrabber();
  virtual ~CGrabber();

  /**
   * The callback function for images pushed by the image server.
   * To be overwritten by derived classes.
   */
  virtual void receiveImages(const std::string& serverName, const std::vector<Video::Image>& images);

  std::vector<std::string> getDeviceNames();
  //void saveImages(const std::vector<Video::Image>& images);
  //void saveQueuedImages(const std::vector<Video::CCachedImagePtr>& images, CRecordingInfo& frameInfo);
  void saveQueuedImages(const std::vector<CGrabbedItemPtr>& images, CRecordingInfo& frameInfo);
  void getClients(std::vector<CDataSource*>& clients)
  {
    clients.clear();
    for(auto video : m_video) clients.push_back(video);
#ifdef FEAT_VIDEOGRABBER_POINTCLOUD
    for(auto pc : m_pointcloud) clients.push_back(pc);
#endif
  }
  void fillRecordingInfo(CRecordingInfo &info);
  void startGrabbing(const std::string& command);
  void stopGrabbing();
  void checkStopGrabbing();
  bool isGrabbing()
  {
    return m_RecordingInfo.recording;
  }
  bool isSaving()
  {
    if (! m_pExtraSaver.get()) return false;
    return dynamic_cast<CExtraSaveThread*>(m_pExtraSaver.get())->itemCount() > 0;
  }
  size_t saveQueueSize()
  {
    if (! m_pExtraSaver.get()) return 0;
    return dynamic_cast<CExtraSaveThread*>(m_pExtraSaver.get())->itemCount();
  }
};

} // namespace
#endif
/* vim: set sw=2 ts=8 et: */
