/**
 * $Id$
 * Johann Prankl, 20100327
 */

#ifndef P_CREATE_MSLD_HH
#define P_CREATE_MSLD_HH

#include "Array.hh"
#include "Line.hh"
#include "Vector2.hh"
#include "Math.hh"
#include "MSLDGlobData.hh"

namespace P 
{

class CreateMSLD
{
private:

  float *gauss;                   // gaussian weight

  MSLDGlobData data;

  bool GetPSR(IplImage *img, IplImage *patch, IplImage *tmp, Vector2 &m, double len, double phi);
  void WeightPSRGauss(float *gauss, IplImage *img);
  void SampleMSLD(IplImage *hDx, IplImage *hDy, float *vec);
  void GetMean(IplImage *grad, int x1, int y1, int x2, int y2, 
               float &p, float &n, unsigned &nump, unsigned &numn);
  void GetSigma(IplImage *grad, int x1, int y1, int x2, int y2, float mp, float mn, 
               unsigned nump, unsigned numn, float &sp, float &sn);
  void GetImageAffine(IplImage *img, float xg, float yg, 
               float Axx, float Ayx , float Axy, float Ayy, 
               IplImage *patch); 
  void NormalizeAndCutMSLD(float *vec);
  void Rotate180(IplImage *src, IplImage *dst);
  void SwapLineDir(Line *l);

  inline float Interpolate(float x, float y, IplImage *img);

public:

  CreateMSLD();
  ~CreateMSLD();

  void Operate(IplImage *img, Array<Line*> &lines, float minLineLength=15.);
  void Operate(const cv::Mat &image, const vector<vector<cv::Vec2d> > &lines, vector<vector<float> > &descriptors, double minLineLength=15.);

  void SaveImage(const char *file, IplImage *img, float scale=FLT_MAX);
};



/****************************** INLINE METHODES ****************************/

 /**
 * Interpolate
 * Given a point (x,y) in an image, computes the bilinear interpolated
 * edge image (short = IPL_DEPTH_16S) value of the point.
 */
inline float CreateMSLD::Interpolate(float x, float y, IplImage *img)
{
  int xt = (int) x;  // coordinates of top-left corner 
  int yt = (int) y;
  float ax = x - xt;
  float ay = y - yt;

  assert (xt >= 0 && yt >= 0 && xt <= ((int)img->width) - 2 && yt <= ((int)img->height) - 2);
  return ( (1-ax) * (1-ay) * (float)GetPx8UC1(img, xt, yt) +
           ax   * (1-ay) * (float)GetPx8UC1(img, xt+1, yt) +
           (1-ax) *   ay   * (float)GetPx8UC1(img, xt, yt+1) +
           ax   *   ay   * (float)GetPx8UC1(img, xt+1, yt+1) );
;
}


 
}

#endif

