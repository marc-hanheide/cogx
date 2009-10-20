/**
 * @author Michael Zillich
 * @date February 2009
 */

#include <cassert>
#include <cast/core/CASTUtils.hpp>
#include "ConvertImage.h"

// define this to use memcpy() instead of looping over image lines and pixels
// memcpy() is a LOT faster, but will screw up the images if the memory layout
// (byte per pixel, bytes per line) is not exactly the same.
//#define FAST_DIRTY_CONVERSION

namespace Video
{

using namespace std;
using namespace cast;

void convertImageToIpl(const Video::Image & img, IplImage ** iplImg)
{
  if(*iplImg != 0)
    if(img.width != (*iplImg)->width || img.height != (*iplImg)->height)
      cvReleaseImage(iplImg);
  if(*iplImg == 0)
    *iplImg = cvCreateImage(cvSize(img.width,img.height), IPL_DEPTH_8U, 3);
  assert(*iplImg != 0);
#ifndef FAST_DIRTY_CONVERSION
  int x, y, c;
  // note: this neat triple loop might be somewhat slower than a memcpy, but
  // makes sure images are copied correctly irrespective of memory layout and
  // line padding.
  for(y = 0; y < img.height; y++)
    for(x = 0; x < img.width; x++)
      for(c = 0; c < 3; c++)
        (*iplImg)->imageData[y*(*iplImg)->widthStep + 3*x + c] =
          img.data[3*(y*img.width + x) + c];
#else
  memcpy((*iplImg)->imageData, &img.data[0],
      (*iplImg)->height*(*iplImg)->widthStep);
#endif
}

IplImage* convertImageToIpl(const Video::Image & img)
{
  IplImage *iplImg = 0;
  convertImageToIpl(img, &iplImg);
  return iplImg;
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
#ifndef FAST_DIRTY_CONVERSION
    int x, y, c;
    for(y = 0; y < iplImg->height; y++)
      for(x = 0; x < iplImg->width; x++)
        for(c = 0; c < 3; c++)
          img.data[3*(y*img.width + x) + c] =
            iplImg->imageData[y*iplImg->widthStep + 3*x + c];
#else
    memcpy(&img.data[0], iplImg->imageData, iplImg->height*iplImg->widthStep);
#endif
  }
  else
    throw runtime_error(exceptionMessage(__HERE__,
      "can only handle 8 bit colour values"));
}

IplImage* convertImageToIplGray(const Video::Image & img)
{
  // colour channel scaling factors, taken from OpenCV cvcolor.cpp
  const float cscGr_32f = 0.299;
  const float cscGg_32f = 0.587;
  const float cscGb_32f = 0.114;

  IplImage *grayImg =
    cvCreateImage(cvSize(img.width,img.height), IPL_DEPTH_8U, 1);
  assert(grayImg != 0);
  int x, y;
  for(y = 0; y < img.height; y++)
    for(x = 0; x < img.width; x++)
    {
      int i = 3*(y*img.width + x);
      float rf = (float)img.data[i];
      float gf = (float)img.data[i+1];
      float bf = (float)img.data[i+2];
      grayImg->imageData[y*grayImg->widthStep + x] = (unsigned char)
        (cscGr_32f*rf + cscGg_32f*gf + cscGb_32f*bf);
    }
  return grayImg;
}

void SwapRedBlueChannel(Video::Image & img)
{
  unsigned char t;
  size_t i, j;
  for(i = 0, j = 2; j < img.data.size(); i += 3, j += 3)
  {
    t = img.data[i];
    img.data[i] = img.data[j];
    img.data[j] = t;
  }
}

}

