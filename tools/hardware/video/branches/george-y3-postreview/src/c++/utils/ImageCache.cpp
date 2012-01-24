/*
 * Author: Marko Mahnič
 * Created: 2010-11-16
 *
 */

#include "ImageCache.h"

using namespace std;

namespace Video {

CIplImageCache::~CIplImageCache()
{
  map<string,IplImage*>::iterator it;
  for(it = m_cache.begin(); it != m_cache.end(); it++) {
    IplImage* pImg = it->second;
    cvReleaseImage(&pImg);
  }
  m_cache.clear();
}

IplImage* CIplImageCache::getImage(const std::string& id, int width, int height, int depth, int channels)
{
  map<string,IplImage*>::iterator it = m_cache.find(id);
  if (it != m_cache.end()) {
    IplImage* pImg = it->second;
    if (pImg->width == width && pImg->height == height
        && pImg->nChannels == channels && pImg->depth == depth)
    {
      return pImg;
    }
    // Image has changed => delete and create a new one
    cvReleaseImage(&pImg);
  }
  m_cache[id] = cvCreateImage(cvSize(width, height), depth, channels);
  return m_cache[id];
}

void CIplImageCache::releaseImage(const std::string& id)
{
  map<string,IplImage*>::iterator it = m_cache.find(id);
  if (it == m_cache.end()) return;
  IplImage* pImg = it->second;
  m_cache.erase(it);
  cvReleaseImage(&pImg);
}

} // namespace
