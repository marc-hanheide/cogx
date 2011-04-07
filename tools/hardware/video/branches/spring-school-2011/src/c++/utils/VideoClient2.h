/**
 * @author Marko Mahnič
 * @date November 2010
 */

#ifndef VIDEOCLIENT_2_V7JAPUYG
#define VIDEOCLIENT_2_V7JAPUYG

#include <cast/core/CASTComponent.hpp>
#include <Video.hpp>
#include <stdexcept>

namespace Video
{

class CVideoClient2;

// An instance of a CImageReceiver will install an ICE server to receive images
// from a video server.
//
// CAST server discovery is done through CAST which discovers servers by the
// cast-id of the component that creates the server. This means that only one
// image receiver can be created per component.
//
// A component library can create only one active component at runtime, so
// a CImageReceiver can only be known under one address (component-id).
// Thus it makes sense to implement CImageReceiver as as singleton.
//
// The receiver will dispatch the received images to registered instances of
// CVideoClient2. There can be at most one CVideoClient2 instance for each
// VideoServer used in a CAST configuration.
//
// A CImageReceiver is not meant to be used directly. Create instances of
// CVideoClient2 instead.
class CImageReceiver
{
private:
  class VideoClientI : virtual public Video::VideoClientInterface
  {
  private:
    CImageReceiver *m_pReceiver;

  public:
    VideoClientI(CImageReceiver *pReceiver)
    {
       m_pReceiver = pReceiver;
    }
    void receiveImages(const Video::ImageSeq &images, const Ice::Current&)
    {
       // This could happen if an old VideoServer implementation is being used.
       throw std::runtime_error("obsolete: receiveImages() was called without serverName.");
    }
    void receiveImages2(const std::string& serverName, const Video::ImageSeq &images, const Ice::Current&)
    {
       m_pReceiver->receiveImages(serverName, images);
    }
  };

private:
   typedef std::map<std::string, CVideoClient2*> VideoClientMapT;
   static CImageReceiver* g_pReceiver;
   VideoClientMapT m_clientMap;
   Ice::Identity m_id;
   CImageReceiver()
   {
      // make the constructor private to implement a singleton.
   }

   // Receive images from various video servers and dispatch to appropriate clients.
   void receiveImages(const std::string& serverName, const std::vector<Video::Image>& images);

   void checkId(cast::CASTComponent* pOwner)
   {
      if (m_id.name.length() > 0) return;
      m_id.name = pOwner->getComponentID();
      m_id.category = "Video.VideoClientInterface";
   }

public:
   static CImageReceiver& getInstance()
   {
      if (g_pReceiver == 0) g_pReceiver = new CImageReceiver();
      return *g_pReceiver;
   }

public:
   void addClient(CVideoClient2* pClient);
   void removeClient(CVideoClient2* pClient);
};

class CReceiverMethodBase
{
public:
   virtual void receiveImages(const std::string& serverName, const std::vector<Video::Image>& images) = 0;
};

template<class T>
class CReceiverMethod: public CReceiverMethodBase
{
public:
   typedef void(T::*ReceiverMethodT)(const std::string& serverName, const std::vector<Video::Image>& images);

private:
   T* m_pInstance;
   ReceiverMethodT m_pMethod;

public:
   CReceiverMethod(T* pInstance, ReceiverMethodT pMethod)
   {
      assert(pInstance != NULL && pMethod != NULL);
      m_pInstance = pInstance;
      m_pMethod = pMethod;
   }
   void receiveImages(const std::string& serverName, const std::vector<Video::Image>& images)
   {
      ((*m_pInstance).*(m_pMethod))(serverName, images);
   }
};

// CCachedImagePtr is a smart pointer to a cached Video::Image.
//
// The smart pointer uses a reference count to lock a cached image.
//
// It is different from an ordinary smart pointer because it doesn't release
// the underlying Video::Image when the reference count drops to 0 (ie the
// image is unlocked). Count 0 only informs the cache manager (in this case
// a CVideoClient2, @see cacheImages) that the image can be reused.
//
// When an image is not locked it can be reused for new images when they arrive
// so that reallocations are avoided.
class CCachedImagePtr
{
public:
  class CStorage
  {
  private:
    friend class CCachedImagePtr;
    Video::Image* m_pImage;
    int m_refCount;
    IceUtil::Monitor<IceUtil::Mutex> m_refMonitor;
    void addref()
    {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_refMonitor);
      m_refCount++;
    }
    void release()
    {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_refMonitor);
      m_refCount--;
    }
  public:
    CStorage()
    {
      m_pImage = 0;
      m_refCount = 0;
    }
    ~CStorage()
    {
      if (m_pImage) delete m_pImage;
    }
    bool isLocked() const
    {
      return m_refCount > 0;
    }
    void setImage(const Video::Image& image)
    {
      if (m_pImage == 0) m_pImage = new Video::Image();
      *m_pImage = image; // XXX: hopefully this doesn't reallocate image data when the size is the same
    }
  };

private:
  CStorage* m_pStore;

public:
  CCachedImagePtr()
  {
    m_pStore = 0;
  }
  CCachedImagePtr(const CCachedImagePtr& aImage)
  {
    m_pStore = aImage.m_pStore;
    if (m_pStore) m_pStore->addref();
  }
  CCachedImagePtr(CStorage* pStore)
  {
    m_pStore = pStore;
    if (pStore) pStore->addref();
  }
  ~CCachedImagePtr()
  {
    if (m_pStore) m_pStore->release();
  }
  CCachedImagePtr& operator=(const CCachedImagePtr& aImage)
  {
    if (m_pStore)
    {
      if (aImage.m_pStore == m_pStore) return *this;
      m_pStore->release();
    }
    m_pStore = aImage.m_pStore;
    if (m_pStore) m_pStore->addref();
    return *this;
  }

  Video::Image* operator->() const
  {
    return m_pStore->m_pImage;
  }

  Video::Image& operator*() const
  {
    return *m_pStore->m_pImage;
  }
};

/**
 * An instance of CVideoClient2 will receive images from one VideoServer.
 *
 * Usage:
 *   - create a member variable CVideoClient2 m_video;
 *   - configure the client in start():
 *     m_video.setServer(this, m_videoServerName, m_camIds);
 *     m_video.setReceiver(new CReceiverMethod<CVideoGrabber>(this, &CVideoGrabber::receiveImages));
 *   - connect and start receiving
 *     m_video.connect();
 *     m_video.setReceiving(true);
 *
 * If a component connects to only one VideoServer, the old approach can be used:
 *   - the receiver is derived from CVideoClient2
 *     class CVideoViewer: public CVideoClient2
 *   - override one of the methods receiveImages()
 *   - setReceiver() doesn't have to be used
 */
class CVideoClient2
{
private:
   friend class CImageReceiver;
   cast::CASTComponent* m_pOwner;

   /**
    * Which camera to get images from
    */
   std::vector<int> m_camIds;

   /**
    * The size of images that we want to receive from the server.
    * 0 means native size.
    */
   int m_width;
   int m_height;

   /**
    * component ID of the video server to connect to
    */
   std::string m_videoServerName;

   /**
    * our ICE proxy to the video server
    */
   Video::VideoInterfacePrx m_pVideoServer;

   /**
    * wether we are currently receiving images from the server
    */
   bool m_bReceiving;

   /**
    * The client can keep the last received images in cache.
    * The size of m_cache should be the same as the size of m_camIds.
    */
   bool m_bCaching;
   std::vector<CCachedImagePtr::CStorage*> m_cache;
   std::vector<CCachedImagePtr::CStorage*> m_locked;
   IceUtil::Monitor<IceUtil::Mutex> m_cacheMonitor;

   /**
    * Replaces m_cache with a set of received images.
    * If an image in m_cache is locked, it is moved to m_locked and an unlocked
    * one is moved from m_locked to m_cache. If an unlocked imaga doesn't exist
    * in m_locked, a new one is created.
    *
    * TODO: cleanup m_locked when it grows too big (erase unlocked images)
    */
   void cacheImages(const std::vector<Video::Image>& images);

   /**
    * The callback function for images pushed by the image server.
    *
    * The version with the serverName parameter calls a receiver method if it
    * was installed with setReceiver. Otherwise it will call receiveImages
    * without the serverName parameter (for backward compatibility with the old
    * implementation of the VideoServer where the receiver function was meant
    * to be overridden).
    */
   CReceiverMethodBase* m_pReceiver;
   virtual void receiveImages(const std::vector<Video::Image>& images);
   virtual void receiveImages(const std::string& serverName, const std::vector<Video::Image>& images);

public:
   // Note: m_cacheMonitor will prevent copying instaces of this class, which is ok.
   CVideoClient2();
   ~CVideoClient2();

   void setServer(cast::CASTComponent* pOwner, const std::string& serverName, const std::vector<int>& cameraIds);

   /**
    * Set the class method that will receive images from the server.
    * Example:
    *   m_Client.setReceiver(new CReceiverMethod<CVideoGrabber>(this, &CVideoGrabber::receiveImages));
    */
   template<class T>
   void setReceiver(CReceiverMethod<T>* pReceiver)
   {
      if (m_pReceiver) delete m_pReceiver;
      m_pReceiver = pReceiver;
   }
   
   // Connect to the server.
   // Creates a receiver server if one doesn't exist.
   void connect();
 
   // Destroys a receiver server when the last video client disconnects.
   void disconnect();
 
   // turn on/off receiving images pushed by the video server
   void setReceiving(bool bReceiving = true);
   bool isReceiving();

   // Set the desired image size. The size will be applied on next setReceiving().
   void setImageSize(int width, int height);

   void setCaching(bool bCaching = true);
   bool isCaching();
   void getCachedImages(std::vector<CCachedImagePtr>& images);
};

} // namespace

#endif
