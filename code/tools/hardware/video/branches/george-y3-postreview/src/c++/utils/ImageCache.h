/*
 * Author: Marko Mahnič
 * Created: 2010-11-16
 *
 */

#ifndef VIDEO_IMAGECACHE_H
#define VIDEO_IMAGECACHE_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <map>
#include <string>

namespace Video {

/**
 * An instance of CIplImageCache stores reusable IplImages.
 *
 * Images are accesed by id and are reallocated only when
 * the required format changes.
 *
 * The class is used by various VideoServers for temporary frames
 * during frame conversion to substantially reduce reallocations.
 */
class CIplImageCache
{
  std::map<std::string,IplImage*> m_cache;
public:
  ~CIplImageCache();

  /**
   * An instance of CImageCache stores reusable IplImages.
   *
   * Images are accesed by id. The function getImage will verify if
   * an image with the given id exists and create one if it doesn't.
   * For an existing image it will verify that the image still has
   * the required format. If it doesn't, the cached image will be
   * reallocated with the correct format.
   */
  IplImage* getImage(const std::string& id, int width, int height, int depth, int channels);
  void releaseImage(const std::string& id);
};

} // namespace
#endif /* end of include guard: VIDEO_IMAGECACHE_H */

