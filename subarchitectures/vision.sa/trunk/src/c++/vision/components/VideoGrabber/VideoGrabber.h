/**
 * @author Marko Mahnič
 * @date September 2010
 *
 */

#ifndef VIDEOGRABBER_V7JAPUYG
#define VIDEOGRABBER_V7JAPUYG

#include "ticker.h"

#include <cast/architecture/ManagedComponent.hpp>
#include <Video.hpp>
#include <VideoClient2.h>
#include <ImageCache.h>

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#include <castutils/Timers.hpp>
#endif
#ifdef FEAT_VIDEOGRABBER_POINTCLOUD
#include <PointCloudClient.h>
#endif
#include <castutils/CastLoggerMixin.hpp>

namespace cogx
{

struct CRecordingInfo
{
   bool recording;
   std::string directory;
   long directoryStatus; // 0: don't; 1: check; 2: exists; -1: error
   std::string filenamePatt;
   std::vector<std::string> deviceNames;
   long counterDigits;
   long counterStart;
   long counterEnd;
   long counter;
   IceUtil::Time tmStart;
   IceUtil::Time tmEnd;
   CRecordingInfo() {
      recording = false;
      counterStart = 0;
      counterEnd = 0;
      counter = 0;
#if 1 // XXX testing
      directory = "/tmp/path/to/Model";
      filenamePatt = "image-%c-%d.png";
      deviceNames.push_back("L");
      deviceNames.push_back("R");
      counterDigits = 3;
      tmStart = IceUtil::Time::seconds(0);
      tmEnd = tmStart;
#endif
   }
};

// Some data is saved in a temporary file to leave more time and memory for grabbing.
// The extra saver opens this file and transforms the data to the final form;
class CExtraSaver
{
public:
   std::string tmpFilename;
   std::string finalFilename;
   virtual void save() = 0;
};
typedef std::shared_ptr<CExtraSaver> CExtraSaverPtr;

class CGrabbedItem
{
public:
   std::string mName;

public:
   std::string makeFilename(const CRecordingInfo& recinfo, int deviceId, const std::string& ext=".png");
   std::string makeTempFilename(const CRecordingInfo& recinfo, int deviceId, const std::string& ext=".tmp");
   // # of devices from which the contents was grabbed; usually 1
   virtual int numDevices()
   {
      return 1;
   }
   virtual ~CGrabbedItem()
   {
   }
   virtual CExtraSaverPtr save(const CRecordingInfo& recinfo, int deviceId) = 0;
};

class CPreview
{
   static Video::CIplImageCache gCache;
   int mWidth;
   int mHeight;
   std::string mName;
public:
   std::string deviceName;
   std::string deviceInfo;

public:
   CPreview(const std::string& name, int width, int height)
   {
      mName = name;
      mWidth = width;
      mHeight = height;
   }
   IplImage* getImage()
   {
      return gCache.getImage(mName, mWidth, mHeight, 8, 3);
   }
};

typedef std::shared_ptr<CGrabbedItem> CGrabbedItemPtr;

class CDataSource
{
   static int count;
protected:
   int mId;

public:
   std::string mName;
   CDataSource()
   {
      mId = count;
      ++count;
   }
   virtual ~CDataSource()
   {
      --count;
   }
   virtual void grab(std::vector<CGrabbedItemPtr>& items) = 0;
   virtual void getPreviews(std::vector<CPreview>& items, int width, int height, bool isGrabbing) = 0;
#ifdef FEAT_VISUALIZATION
   virtual void configExtraV11n(cogx::display::CDisplayClient& display)
   {
   }
   virtual void displayExtra(cogx::display::CDisplayClient& display)
   {
   }
#endif
};

class CVideoGrabClient: public Video::CVideoClient2, public CDataSource,
   public castutils::CCastLoggerMixin
{
public:
   CVideoGrabClient(cast::CASTComponent* pComponent);
   virtual void grab(std::vector<CGrabbedItemPtr>& items) /*override*/;
   virtual void getPreviews(std::vector<CPreview>& previews,
         int width, int height, bool isGrabbing) /*override*/;
};

class CGrabbedCachedImage: public CGrabbedItem
{
private:
   friend class CVideoGrabClient;
   Video::CCachedImagePtr mpImage;
public:
   CGrabbedCachedImage(Video::CCachedImagePtr& pimage)
   {
      mName = "Cached Image";
      mpImage = pimage;
   }
   virtual CExtraSaverPtr save(const CRecordingInfo& recinfo, int deviceId) /*override*/;
};

class CGrabbedImage: public CGrabbedItem
{
public:
   Video::Image mImage;
   CGrabbedImage()
   {
      mName = "Image";
   }
   virtual CExtraSaverPtr save(const CRecordingInfo& recinfo, int deviceId) /*override*/;
};

#ifdef FEAT_VIDEOGRABBER_POINTCLOUD
class CPcGrabClient: public cast::PointCloudClient, public CDataSource
{
   CGrabbedItemPtr mLastPoints;
   CGrabbedItemPtr mLastDepth;
   CGrabbedItemPtr mLastRectImage;
public:
   bool mbGrabPoints;
   bool mbGrabDepth;
   bool mbGrabRectImage;
   CPcGrabClient();
   void configurePcComm(const std::map<std::string,std::string> & _config)
   {
      configureServerCommunication(_config);
   }
   void startPcComm(cast::CASTComponent &owner)
   {
      startPCCServerCommunication(owner);
   };
   virtual void grab(std::vector<CGrabbedItemPtr>& items) /*override*/;
   virtual void getPreviews(std::vector<CPreview>& previews,
         int width, int height, bool isGrabbing) /*override*/;
#ifdef FEAT_VISUALIZATION
   virtual void configExtraV11n(cogx::display::CDisplayClient& display) /*override*/;
   virtual void displayExtra(cogx::display::CDisplayClient& display) /*override*/;
#endif
};

class CGrabbedPcPoints: public CGrabbedItem
{
private:
   friend class CPcGrabClient;
   std::vector<PointCloud::SurfacePoint> mPoints;
public:
   CGrabbedPcPoints()
   {
      mName = "Point Cloud Points";
   }
   virtual CExtraSaverPtr save(const CRecordingInfo& recinfo, int deviceId) /*override*/;
};
#endif

class CVideoGrabber: public cast::ManagedComponent
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
      CVideoGrabber* pViewer;
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

   public:
      CVvDisplayClient() { pViewer = NULL; }
      void setClientData(CVideoGrabber* pVideoGrabber) { pViewer = pVideoGrabber; }
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
      CVideoGrabber *m_pGrabber;

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
      CGrabQueThread(CVideoGrabber *pGrabber);

      void getItems(std::vector<CFramePack>& items, unsigned int maxItems = 0);
      virtual void grab();
      virtual void run();
   };
   IceUtil::ThreadPtr m_pQueue;
   IceUtil::Handle<CTickerTask> m_pQueueTick;

   class CExtraSaveThread: public IceUtil::Thread
   {
      CVideoGrabber *m_pGrabber;
   private:
      std::vector<CExtraSaverPtr> m_savers;
      IceUtil::Monitor<IceUtil::Mutex> m_saversLock;
   public:
      CExtraSaveThread(CVideoGrabber *pGrabber);
      void addSaver(CExtraSaverPtr saver)
      {
         IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_saversLock);
         m_savers.push_back(saver);
      }
      virtual void run();
   };
   IceUtil::ThreadPtr m_pSaver;

   class CDrawingThread: public IceUtil::Thread, public CTickSyncedTask
   {
      CVideoGrabber *m_pGrabber;

   public:
      CDrawingThread(CVideoGrabber *pGrabber);
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
   CVideoGrabber();
   virtual ~CVideoGrabber();

   /**
    * The callback function for images pushed by the image server.
    * To be overwritten by derived classes.
    */
   virtual void receiveImages(const std::string& serverName, const std::vector<Video::Image>& images);

   std::vector<std::string> getDeviceNames();
   void saveImages(const std::vector<Video::Image>& images);
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
};

} // namespace
#endif
/* vim: set sw=3 ts=8 et: */

