/**
 * @author Michael Zillich
 * @date February 2009
 */

#include <cassert>
#include <cast/core/CASTUtils.hpp>
#include "AccessImage.h"
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

bool convertBytesToIpl(const std::vector<unsigned char>& data, int width, int height, int nchn, IplImage ** iplImg)
{
  if(*iplImg != 0)
    if(width != (*iplImg)->width || height != (*iplImg)->height || nchn != (*iplImg)->nChannels != nchn)
      cvReleaseImage(iplImg);

  if (nchn != 1 && nchn != 3) return false;
  if (width * height * nchn > data.size()) return false;

  if(*iplImg == 0)
    *iplImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, nchn);
  assert(*iplImg != 0);

#ifndef FAST_DIRTY_CONVERSION
  int x, y, c;
  // note: this neat triple loop might be somewhat slower than a memcpy, but
  // makes sure images are copied correctly irrespective of memory layout and
  // line padding.
  for(y = 0; y < height; y++)
    for(x = 0; x < width; x++)
      for(c = 0; c < nchn; c++)
        (*iplImg)->imageData[y*(*iplImg)->widthStep + nchn*x + c] =
          data[nchn*(y*width + x) + c];
#else
  memcpy((*iplImg)->imageData, &data[0],
      (*iplImg)->height*(*iplImg)->widthStep);
#endif

  return true;
}

IplImage* convertBytesToIpl(const std::vector<unsigned char>& data, int width, int height, int nchn)
{
  IplImage *iplImg = 0;
  convertBytesToIpl(data, width, height, nchn, &iplImg);
  return iplImg;
}

void convertImageFromIpl(const IplImage *iplImg, Video::Image &img)
  throw(runtime_error)
{
  assert(iplImg != 0);

  img.width = iplImg->width;
  img.height = iplImg->height;
  // Video::Image's always have 3 channels
  img.data.resize(img.width*img.height*3);

  if(iplImg->nChannels == 3 &&
     (iplImg->depth == (int)IPL_DEPTH_8U || iplImg->depth == (int)IPL_DEPTH_8S))
  {
#ifdef FAST_DIRTY_CONVERSION
    if(iplImg->widthStep == iplImg->width*3)
    {
      assert(iplImg->imageSize == img.data.size());
      memcpy(&img.data[0], iplImg->imageData, iplImg->height*iplImg->widthStep);
    }
    else
#endif
    {
      // note: this neat triple loop is a lot slower than a memcpy, but
      // works irrespective of the image memory layouts (e.g. line padding)
      AccessRgbImage accImg(iplImg);
      int x, y;
      for(y = 0; y < iplImg->height; y++)
        for(x = 0; x < iplImg->width; x++)
        {
          // note: AccessRgbPixel has order b, g, r
          const AccessRgbPixel &pix = accImg[y][x];
          // Video::Image's always have 3 channels
          int i = 3*(y*img.width + x);
          img.data[i] = pix.b;
          img.data[i+1] = pix.g;
          img.data[i+2] = pix.r;
        }
    }
  }
  else if(iplImg->nChannels == 1 && iplImg->depth == IPL_DEPTH_32F)
  {
    AccessBwImageFloat accImg(iplImg);
    int x, y;
    for(y = 0; y < iplImg->height; y++)
      for(x = 0; x < iplImg->width; x++)
      {
        // Video::Image's always have 3 channels
        int i = 3*(y*img.width + x);
        float f = accImg[y][x];
        img.data[i] = (unsigned char)f;
        img.data[i+1] = (unsigned char)f;
        img.data[i+2] = (unsigned char)f;
      }
  }
  else
    throw runtime_error(exceptionMessage(__HERE__,
      "can not handle %d channel %d bit images", iplImg->nChannels, iplImg->depth));
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

void PruneImageArea(IplImage *iplImg_src, IplImage & iplImg_dst, int width, int height, int offsetX, int offsetY)	/// TODO aufräumen!
{
// printf("nChannels = %u\n", iplImg_src->nChannels);
// printf("iplImage->width/height: %u/%u\n", iplImg_src->width, iplImg_src->height);
// printf("width/height: %u/%u\n", width, height);

	unsigned imgP = 0;
	for(unsigned y=offsetY; y<offsetY+height; y++)
	{
		for(unsigned x=offsetX; x<offsetX+width; x++)
		{
			for(unsigned c=0; c<iplImg_src->nChannels; c++)
			{
// if (first) printf("x-y: %u-%u\n", offsetX, offsetY);
// if (first) printf("widthStep: %u\n", iplImg_src->widthStep);
// if (first) printf("imgP: %u from %u\n", imgP, 3*x + y*iplImg_src->widthStep + c);
// first = false;
				iplImg_dst.imageData[imgP] = iplImg_src->imageData[3*x + y*iplImg_src->widthStep + c];
				imgP++;
// printf("%u \n", imgP);
			}
		}
	}

	printf("PruneImageArea ready!!!\n");
	printf("  w/h: %u/%u\n", iplImg_dst.width, iplImg_dst.height);
/*	
	int x, y, c;
	for(y = 0; y < iplImg->height; y++)
		for(x = 0; x < iplImg->width; x++)
			for(c = 0; c < 3; c++)
			{
				assert(3*(y*img.width + x) + c < img.data.size());
				img.data[3*(y*img.width + x) + c] = iplImg->imageData[y*iplImg->widthStep + 3*x + c];
			}*/

// ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0]=111; // B
// ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1]=112; // G
// ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2]=113; // R


// 	CvRect rect = cvRect(offsetX, offsetY, width, height);
// 	cvSetImageROI(iplImg_src, rect);
//   cvCopyImage(iplImg_src, iplImg_dst);
}
}





