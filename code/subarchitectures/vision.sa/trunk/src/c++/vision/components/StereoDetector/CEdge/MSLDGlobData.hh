/**
 * $Id$
 */

#ifndef P_MSLD_GLOB_DATA_HH
#define P_MSLD_GLOB_DATA_HH

#include "Vector2.hh"
#include "PNamespace.hh"
#include <vector>


namespace P
{

/**
 * Global data and parameter for MSLD computation
 */
class MSLDGlobData
{

public:

  Vector2 point[2];               //clipped line

  char *tmp;
  char *patch;
  short *dx, *dy;

  IplImage *hTmp;
  IplImage *hPatch;
  IplImage *hDx, *hDy;

  int xmin, xmax, ymin, ymax;     // image border

  Vector2 m;
  double len;
  double phi;
  int maxLen;
  int heightPSR, widthPSR;

                                            // according to Wang 2008 ...
  static const float F_SIGMA = .5;                // gaussian feature weight
  static const float FEATURE_THR = .4;            // threshold to bound nonlinear illumination effects
  float MIN_LINE_LENGTH; 

  static const int NUM_PSR = 9;
  static const int SIZE_PSR = 5;

  MSLDGlobData()
   : tmp(0), patch(0), dx(0), dy(0), hTmp(0), hPatch(0), hDx(0), hDy(0)
  {
    heightPSR = NUM_PSR*SIZE_PSR;
    MIN_LINE_LENGTH = 15.;
  };
  ~MSLDGlobData()
  {
    ReleaseMem();
    ReleaseLine();
  };

  inline void AllocMem(IplImage *img, Array<Line*> &lines);
  inline void AllocMem(IplImage *img, const vector<vector<cv::Vec2d> > &lines);
  inline void ReleaseMem();
  inline bool SetLine(Line* l);
  inline bool SetLine(const cv::Vec2d &p1, const cv::Vec2d &p2);
  inline void ReleaseLine();
};



/***************************** INLINE METHODES ********************************/

inline bool MSLDGlobData::SetLine(Line* l)
{
  ReleaseLine();

  point[0] = l->point[0];
  point[1] = l->point[1];

  if (ClipLine(xmin, ymin, xmax, ymax, &point[0].x, &point[0].y, &point[1].x, &point[1].y))
  {
    if (Distance(point[0], point[1]) >= MIN_LINE_LENGTH)
    {
      m = MidPoint(point[0],point[1]);
      len = Distance(point[0],point[1]);
      phi = PolarAngle(l->dir);
      int widthPSR = ((int)len/2)*2 + 1;

      if (widthPSR > maxLen)
        return false; 

      // +2 for dx, dy convolution border
      hTmp = cvCreateImageHeader(cvSize(widthPSR+2,heightPSR+2), IPL_DEPTH_8U, 1 );
      hPatch = cvCreateImageHeader(cvSize(widthPSR+2,heightPSR+2), IPL_DEPTH_8U, 1 );
      hDx = cvCreateImageHeader(cvSize(widthPSR+2,heightPSR+2), IPL_DEPTH_16S, 1 );
      hDy = cvCreateImageHeader(cvSize(widthPSR+2,heightPSR+2), IPL_DEPTH_16S, 1 );

      hTmp->imageData = (char*)tmp;
      hPatch->imageData = (char*)patch;
      hDx->imageData = (char*)dx;
      hDy->imageData = (char*)dy;

      return true;
    }
  }

  return false;
}

inline bool MSLDGlobData::SetLine(const cv::Vec2d &p1, const cv::Vec2d &p2)
{
  ReleaseLine();
  
  point[0] = Vector2(p1[0],p1[1]);
  point[1] = Vector2(p2[0],p2[1]);

  if (ClipLine(xmin, ymin, xmax, ymax, &point[0].x, &point[0].y, &point[1].x, &point[1].y))
  {
    if (Distance(point[0], point[1]) >= MIN_LINE_LENGTH)
    {
      m = MidPoint(point[0],point[1]);
      len = Distance(point[0],point[1]);
      Vector2 dir = Normalise(point[1]-point[0]);
      phi = PolarAngle( dir );
      int widthPSR = ((int)len/2)*2 + 1;

      if (widthPSR > maxLen)
        return false; 

      // +2 for dx, dy convolution border
      hTmp = cvCreateImageHeader(cvSize(widthPSR+2,heightPSR+2), IPL_DEPTH_8U, 1 );
      hPatch = cvCreateImageHeader(cvSize(widthPSR+2,heightPSR+2), IPL_DEPTH_8U, 1 );
      hDx = cvCreateImageHeader(cvSize(widthPSR+2,heightPSR+2), IPL_DEPTH_16S, 1 );
      hDy = cvCreateImageHeader(cvSize(widthPSR+2,heightPSR+2), IPL_DEPTH_16S, 1 );

      hTmp->imageData = (char*)tmp;
      hPatch->imageData = (char*)patch;
      hDx->imageData = (char*)dx;
      hDy->imageData = (char*)dy;

      return true;
    }
  }

  return false;
}

inline void MSLDGlobData::ReleaseLine()
{
  if (hTmp!=0) {cvReleaseImageHeader(&hTmp); hTmp=0;} 
  if (hPatch!=0) {cvReleaseImageHeader(&hPatch); hPatch=0;} 
  if (hDx!=0) {cvReleaseImageHeader(&hDx); hDx=0;} 
  if (hDy!=0) {cvReleaseImageHeader(&hDy); hDy=0;} 
}

inline void MSLDGlobData::AllocMem(IplImage *img, Array<Line*> &lines)
{
  xmin = (double)(SIZE_PSR*NUM_PSR/2.) + 3;
  xmax = img->width - xmin;
  ymin = (double)(SIZE_PSR*NUM_PSR/2.) + 3;
  ymax = img->height - ymin;

  maxLen=0;
  for (unsigned i=0; i<lines.Size(); i++)
    if (lines[i]->len > maxLen)
      maxLen = lines[i]->len;

  maxLen = maxLen+SIZE_PSR+2;

  ReleaseMem();

  tmp = new char[(maxLen+2)*(heightPSR+2)];
  patch = new char[(maxLen+2)*(heightPSR+2)];
  dx = new short[(maxLen+2)*(heightPSR+2)];
  dy = new short[(maxLen+2)*(heightPSR+2)];
}

inline void MSLDGlobData::AllocMem(IplImage *img, const vector<vector<cv::Vec2d> > &lines)
{
  xmin = (double)(SIZE_PSR*NUM_PSR/2.) + 3;
  xmax = img->width - xmin;
  ymin = (double)(SIZE_PSR*NUM_PSR/2.) + 3;
  ymax = img->height - ymin;

  maxLen=0;
  double len;
  for (int i=0; i<lines.size(); i++)
  {
    const vector<cv::Vec2d> &line = lines[i];
    for (int j=1; j<line.size(); j++)
    {
      len = sqrt(Sqr(line[j-1][0]-line[j][0])+Sqr(line[j-1][1]-line[j][1]));
      if (len > maxLen)
        maxLen = len;
    }
  }

  maxLen = maxLen+SIZE_PSR+2;

  ReleaseMem();

  tmp = new char[(maxLen+2)*(heightPSR+2)];
  patch = new char[(maxLen+2)*(heightPSR+2)];
  dx = new short[(maxLen+2)*(heightPSR+2)];
  dy = new short[(maxLen+2)*(heightPSR+2)];
}

inline void MSLDGlobData::ReleaseMem()
{
  if (tmp!=0) { delete[] tmp; tmp=0;}
  if (patch!=0) { delete[] patch; patch=0;}
  if (dx!=0) { delete[] dx; dx=0;}
  if (dy!=0) { delete[] dy; dy=0;}
    
}



}

#endif



