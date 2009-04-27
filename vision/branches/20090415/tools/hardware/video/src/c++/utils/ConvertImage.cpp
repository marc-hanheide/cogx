/**
 * @author Michael Zillich
 * @date February 2009
 */

#include <cassert>
#include <CASTUtils.hpp>
#include "ConvertImage.h"

namespace Video
{

using namespace std;
using namespace cast;

IplImage* convertImageToIpl(const Video::Image & image)
{
  IplImage *iplImage =
    cvCreateImage(cvSize(image.width,image.height), IPL_DEPTH_8U, 3);
  assert(iplImage != 0);
  int x, y, c;
  // note: this neat triple loop might be somewhat slower than a memcpy, but
  // makes sure images are copied correctly irrespective of memory layout and
  // line padding.
  for(y = 0; y < image.height; y++)
    for(x = 0; x < image.width; x++)
      for(c = 0; c < 3; c++)
        iplImage->imageData[y*iplImage->widthStep + 3*x + c] =
          image.data[3*(y*image.width + x) + c];
  return iplImage;
}

IplImage* convertImageToIplGray(const Video::Image & image)
{
  // colour channel scaling factors, taken from OpenCV cvcolor.cpp
  const float cscGr_32f = 0.299;
  const float cscGg_32f = 0.587;
  const float cscGb_32f = 0.114;

  IplImage *grayImage =
    cvCreateImage(cvSize(image.width,image.height), IPL_DEPTH_8U, 1);
  assert(grayImage != 0);
  int x, y;
  for(y = 0; y < image.height; y++)
    for(x = 0; x < image.width; x++)
    {
      int i = 3*(y*image.width + x);
      float rf = (float)image.data[i];
      float gf = (float)image.data[i+1];
      float bf = (float)image.data[i+2];
      grayImage->imageData[y*grayImage->widthStep + x] = (unsigned char)
        (cscGr_32f*rf + cscGg_32f*gf + cscGb_32f*bf);
    }
  return grayImage;
}

void convertImageFromIpl(const IplImage *iplImg, Video::Image &img)
  throw(runtime_error)
{
  assert(iplImg != 0);
  if(iplImg->nChannels != 3)
    throw runtime_error(exceptionMessage(__HERE__,
      "can only handle 3 channel colour images, have %d channels",
      iplImg->nChannels));

  img.width = iplImg->width;
  img.height = iplImg->height;
  img.data.resize(iplImg->width*iplImg->height*iplImg->nChannels);
  // note: this neat triple loop might be somewhat slower than a memcpy, but
  // makes sure images are copied correctly irrespective of memory layout and
  // line padding.
  if(iplImg->depth == (int)IPL_DEPTH_8U || iplImg->depth == (int)IPL_DEPTH_8S)
  {
    int x, y, c;
    for(y = 0; y < iplImg->height; y++)
      for(x = 0; x < iplImg->width; x++)
        for(c = 0; c < 3; c++)
          img.data[3*(y*img.width + x) + c] =
            iplImg->imageData[y*iplImg->widthStep + 3*x + c];
  }
  else
    throw runtime_error(exceptionMessage(__HERE__,
      "can only handle 8 bit colour values"));
}

}

