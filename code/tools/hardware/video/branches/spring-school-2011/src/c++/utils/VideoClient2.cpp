/**
 * @author Marko Mahnič
 * @date November 2010
 */

#include "VideoClient2.h"

namespace Video {

CImageReceiver* CImageReceiver::g_pReceiver = 0;

// Receive images from various video servers and dispatch to appropriate clients.
void CImageReceiver::receiveImages(const std::string& serverName, const std::vector<Video::Image>& images)
{
   VideoClientMapT::iterator it = m_clientMap.find(serverName);
   if (it == m_clientMap.end()) return;
   it->second->receiveImages(serverName, images);
}

void CImageReceiver::addClient(CVideoClient2* pClient)
{
   VideoClientMapT::iterator it = m_clientMap.find(pClient->m_videoServerName);
   if (it != m_clientMap.end()) return;
   if (m_clientMap.size() == 0) {
      checkId(pClient->m_pOwner);
      Video::VideoClientInterfacePtr servant = new VideoClientI(this);
      pClient->m_pOwner->registerIceServer<Video::VideoClientInterface>(m_id.name, m_id.category, servant);
   }
   m_clientMap[pClient->m_videoServerName] = pClient;
}

void CImageReceiver::removeClient(CVideoClient2* pClient)
{
   if (m_clientMap.size() == 0) return;
   VideoClientMapT::iterator it = m_clientMap.find(pClient->m_videoServerName);
   if (it == m_clientMap.end()) return;

   m_clientMap.erase(it);
   if (m_clientMap.size() == 0) {
      pClient->m_pOwner->getObjectAdapter()->remove(m_id);
   }
}

CVideoClient2::CVideoClient2()
{
   m_pOwner = 0;
   m_pVideoServer = 0;
   m_width = 0;
   m_height = 0;
   m_bReceiving = false;
   m_pReceiver = 0;
   m_bCaching = false;
}

CVideoClient2::~CVideoClient2()
{
   if (m_pReceiver) {
      delete m_pReceiver;
      m_pReceiver = 0;
   }
}

void CVideoClient2::setServer(cast::CASTComponent* pOwner,
      const std::string& serverName, const std::vector<int>& cameraIds)
{
   if (m_pVideoServer != 0) return; // TODO: throw exception
   m_pOwner = pOwner;
   m_videoServerName = serverName;
   m_camIds = cameraIds;
}

void CVideoClient2::receiveImages(const std::vector<Video::Image>& images)
{
}

void CVideoClient2::receiveImages(const std::string& serverName, const std::vector<Video::Image>& images)
{
   if (m_bCaching) cacheImages(images);

   if (m_pReceiver) m_pReceiver->receiveImages(serverName, images);
   else receiveImages(images);
}

void CVideoClient2::connect() 
{
   if (m_pVideoServer != 0) return; // TODO: throw exception

   // get connection to the video server
   m_pVideoServer = m_pOwner->getIceServer<Video::VideoInterface>(m_videoServerName);

   CImageReceiver::getInstance().addClient(this);
}

void CVideoClient2::disconnect()
{
   if (m_pVideoServer == 0) return; // TODO: throw exception

   if (m_bReceiving) setReceiving(false);
   m_pVideoServer = 0;

   CImageReceiver::getInstance().removeClient(this);
}

void CVideoClient2::setReceiving(bool bReceiving)
{
   if (m_pVideoServer == 0) return; // TODO: throw exception

   if (bReceiving) {
      m_pVideoServer->startReceiveImages(m_pOwner->getComponentID().c_str(), m_camIds, m_width, m_height);
      m_bReceiving = true;
   }
   else {
      m_pVideoServer->stopReceiveImages(m_pOwner->getComponentID().c_str());
      m_bReceiving = false;
   }
}

bool CVideoClient2::isReceiving()
{
   return m_bReceiving;
}

void CVideoClient2::setImageSize(int width, int height)
{
   m_width = width;
   m_height = height;
}

void CVideoClient2::setCaching(bool bCaching)
{
   m_bCaching = bCaching;
   if (! m_bCaching) {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_cacheMonitor);
      m_cache.clear();
   }
}

bool CVideoClient2::isCaching()
{
   return m_bCaching;
}

void CVideoClient2::cacheImages(const std::vector<Video::Image>& images)
{
   struct _local_ {
      std::vector<CCachedImagePtr::CStorage*>* m_pLocked;
      CCachedImagePtr::CStorage* getFreeStore()
      {
         std::vector<CCachedImagePtr::CStorage*>::iterator it;
         for(it = m_pLocked->begin(); it != m_pLocked->end(); it++) {
            if (! (*it)->isLocked()) break;
         }
         if (it == m_pLocked->end()) {
            return new CCachedImagePtr::CStorage();
         }

         CCachedImagePtr::CStorage* ps = *it;
         m_pLocked->erase(it);
         return ps;
      }
      void addLockedStore(CCachedImagePtr::CStorage* pStore)
      {
         m_pLocked->push_back(pStore);
         // std::cout << "Images in locked store: " << m_pLocked->size() << std::endl;
      }
      _local_(std::vector<CCachedImagePtr::CStorage*>& locked)
      {
         m_pLocked = &locked;
      }
   } local(m_locked);

   IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_cacheMonitor);

   if (images.size() > m_cache.size())
      m_cache.resize(images.size(), 0);

   for(unsigned int i = 0; i < images.size(); i++) {
      if (m_cache[i] != 0 && m_cache[i]->isLocked()) {
         local.addLockedStore(m_cache[i]);
         m_cache[i] = 0;
      }
      if (m_cache[i] == 0)
         m_cache[i] = local.getFreeStore();
      m_cache[i]->setImage(images[i]);
   }
}

void CVideoClient2::getCachedImages(std::vector<CCachedImagePtr>& images)
{
   IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_cacheMonitor);
   images.clear();
   for(unsigned int i = 0; i < m_cache.size(); i++) {
      images.push_back(CCachedImagePtr(m_cache[i]));
   }
}

} // namespace
