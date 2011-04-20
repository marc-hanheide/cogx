/**
 * $Id$
 * Johann Prankl, 2010-03-30
 */

#include <float.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "CreateMSLD.hh"



namespace P 
{


CreateMSLD::CreateMSLD()
{
  gauss = new float[data.heightPSR];

  double sigma = data.F_SIGMA*(double)data.heightPSR;

  for (int i=data.heightPSR/2, j=data.heightPSR/2, z=0; i>=0; i--,j++, z++)
  {
    gauss[i] = exp(-.5*Sqr(z/sigma));
    gauss[j] = gauss[i];
  }
}

CreateMSLD::~CreateMSLD()
{
  delete[] gauss;
}

/**************************** PRIVATE **************************************/

/**
 * Get affine transformed gradient path
 *
 * Given two images and the window center in both images,
 * aligns the images with the window
 * between the two overlaid images using the affine mapping.
 *       A =  [ Axx Axy]
 *            [ Ayx Ayy]
 */
void CreateMSLD::GetImageAffine(IplImage *img, float xg, float yg,  
             float Axx, float Ayx , float Axy, float Ayy,
             IplImage *patch)
{
  register int hw = patch->width/2, hh = patch->height/2;
  register int i, j;
  float mi, mj;

  for (j = -hh ; j <= hh ; j++)
    for (i = -hw ; i <= hw ; i++)  
    {
      mi = Axx * i + Axy * j;
      mj = Ayx * i + Ayy * j;
      SetPx8UC1(patch, i+hw, j+hh, (uchar)Interpolate(xg+mi, yg+mj, img));
    }
}

/**
 * Rotate 180°
 */
void CreateMSLD::Rotate180(IplImage *src, IplImage *dst)
{
  for (int v=0, j=dst->height-1; v<dst->height; v++, j--)
  {
    for (int u=0, i=dst->width-1; u<dst->width; u++, i--)
    {
      SetPx8UC1(dst,i,j,GetPx8UC1(src,u,v));
    }
  }
}

/**
 * Copy pixel support region (PSR) to a rotation invariant image and weight gaussian
 */
bool CreateMSLD::GetPSR(IplImage *img, IplImage *patch, IplImage *tmp, Vector2 &m, double len, double phi)
{
  int sum1=0, sum2=0;
  float Axx, Ayx, Axy, Ayy;
  register int hh;

  Axx = cos(phi);  Ayx = sin(phi);
  Axy = -sin(phi); Ayy = cos(phi);

  GetImageAffine(img, m.x, m.y, Axx, Ayx , Axy, Ayy, tmp);  

  //flip image if wrong gradient direction
  hh = tmp->height/2 - 1;
  for (int u=0; u<tmp->width; u++)
    sum1 += ((uchar*)(tmp->imageData + tmp->widthStep*hh))[u];

  hh = tmp->height/2 + 1;
  for (int u=0; u<tmp->width; u++)
    sum2 += ((uchar*)(tmp->imageData + tmp->widthStep*hh))[u];

  if (sum2 > sum1)
  {
    Rotate180(tmp, patch);
    return true;
  }
  else
  {
    cvCopy(tmp,patch);
    return false;
  }
}

/**
 * use a precomputed gauss table to weight the PSR
 */
void CreateMSLD::WeightPSRGauss(float *gauss, IplImage *img)
{
  short *d;

  for (int v=0; v<img->height; v++)
  {
    d = (short*)(img->imageData + img->widthStep*v);
    for (int u=0; u<img->width; u++)
      d[u] = (short)(((float)d[u]) * gauss[v]); 
  }
}

/**
 * get mean of the positiv and negative orientation values 
 */
void CreateMSLD::GetMean(IplImage *grad, int x1, int y1, int x2, int y2, 
                         float &p, float &n, unsigned &nump, unsigned &numn)
{
  short d;
  p = n = 0.;
  nump = numn = 0;

  for (int v=y1; v<y2; v++)
  {
    for (int u=x1; u<x2; u++)
    {
      d = GetPx16SC1(grad,u,v);

      if (d >= 0.)
      {
        p += d;
        nump++;
      }
      else
      {
        n += d;
        numn++;
      }
    }
  }

  if (nump>eps) p /= (float)nump;
  if (numn>eps) n /= (float)numn;
}

/**
 * get sigma of positive and negative values
 */
void CreateMSLD::GetSigma(IplImage *grad, int x1, int y1, int x2, int y2, float mp, float mn, 
                          unsigned nump, unsigned numn, float &sp, float &sn)
{
  short d;
  sp = sn = 0.;

  for (int v=y1; v<y2; v++)
  {
    for (int u=x1; u<x2; u++)
    {
      d = GetPx16SC1(grad,u,v);

      if (d >= 0.)
        sp += Sqr(d - mp);
      else
        sn += Sqr(d - mn);
    }
  }

  if ((nump-1)>eps) sp = sqrt(sp / (float)(nump-1));
  if ((numn-1)>eps) sn = sqrt(sn / (float)(numn-1));
}

/**
 * Compute the line descriptor
 * We do not use a squared subregion (Wang 2009), 
 * but we compute the mean/stdev over the whole line length
 * 
 * Descriptor: 
 *   M = NUM_PSR;
 *
 *   mxp_1 myp_1 mxn_1 myn_1 sxp_1 syp_1 sxn_1 syn_1 ...
 *   ... mxp_M myp_M mxn_M myn_M sxp_M syp_M sxn_M syn_M
 *
 */
void CreateMSLD::SampleMSLD(IplImage *hDx, IplImage *hDy, float *vec)
{
  int x1=1, x2=hDx->width-1, y1, y2;
  unsigned num[4];

  for (int v=0; v<data.NUM_PSR; v++, vec+=8)
  {
    y1 = v*data.SIZE_PSR + 1;
    y2 = (v+1)*data.SIZE_PSR + 1;

    GetMean(hDx, x1,y1, x2,y2, vec[0], vec[2], num[0], num[2]);
    GetMean(hDy, x1,y1, x2,y2, vec[1], vec[3], num[1], num[3]);
    
    GetSigma(hDx, x1,y1, x2,y2, vec[0], vec[2], num[0], num[2], vec[4], vec[6]);
    GetSigma(hDy, x1,y1, x2,y2, vec[1], vec[3], num[1], num[3], vec[5], vec[7]);
  }
}

/**
 * NormalizeAndCutMSLD
 * Normalization and threshold descriptor
 */
void CreateMSLD::NormalizeAndCutMSLD(float *vec)
{
  float *d;
  float norms=0., normm=0.;

  d = vec;
  for (int i=0; i<data.NUM_PSR; i++)
  {
    for (unsigned j=0; j<4; j++, d++)
      normm += Sqr(*d);
      
    for (unsigned j=0; j<4; j++, d++)
      norms += Sqr(*d);
  }
  
  if (normm > eps) normm = 1./sqrt(normm);
  if (norms > eps) norms = 1./sqrt(norms);

  d = vec;
  for (int i=0; i<data.NUM_PSR; i++)
  {
    if (normm > eps)
    {
      for (unsigned j=0; j<4; j++, d++)
      {
        *d = fabs(*d) * normm;
        if (*d > data.FEATURE_THR)
          *d = data.FEATURE_THR;
      }
    }
    else
    { 
      d+=4;
    }
    if (norms > eps)
    {
      for (unsigned j=0; j<4; j++, d++)
      {
        *d = fabs(*d) * norms;
        if (*d > data.FEATURE_THR)
          *d = data.FEATURE_THR;
      }
    }
    else
    {
      d+=4;
    }
  }
}

/**
 * Swap line orientation
 */
void CreateMSLD::SwapLineDir(Line *l)
{
  Vector2 p;
  
  p = l->point[0];
  l->point[0] = l->point[1];
  l->point[1] = p;

  l->CalculateParameters();
}


/**************************** PUBLIC ***************************************/

/**
 * Compute MSDL line descriptor
 */
void CreateMSLD::Operate(IplImage *img, Array<Line*> &lines, float minLineLength)
{
  if(!IsImage8UC1(img))
    throw Except(__HERE__,"Grey scale image should be IPL_DEPTH_8U, 1 channel!");

  //alloc global memory
  data.AllocMem(img,lines);
  data.MIN_LINE_LENGTH = minLineLength;

  for (unsigned i=0; i<lines.Size(); i++)
  {
    if (data.SetLine(lines[i]))
    {
      bool swap = GetPSR(img, data.hPatch, data.hTmp, data.m, data.len, data.phi);

      cvSobel( data.hPatch, data.hDx, 1, 0, 3);
      cvSobel( data.hPatch, data.hDy, 0, 1, 3);

      WeightPSRGauss(gauss, data.hDx);
      WeightPSRGauss(gauss, data.hDy);

      lines[i]->AllocVec(data.NUM_PSR*8);

      SampleMSLD(data.hDx, data.hDy, lines[i]->vec); 
        
      NormalizeAndCutMSLD(lines[i]->vec);

      if (swap) SwapLineDir(lines[i]);
      lines[i]->CalculateParameters2();
    }
    else
      lines[i]->DeleteVec();
  }
}

/**
 * Compute MSDL line descriptors
 * @param image 1 channel gray scale image (8U)
 * @param lines input [start, end; start, end;...]
 * @param descriptors according descriptors for each line with size is zero if failed
 * @param min. line length to compute descirptor
 */
void CreateMSLD::Operate(const cv::Mat &image, const vector<vector<cv::Vec2d> > &lines, vector<vector<float> > &descriptors, double minLineLength)
{
  IplImage img = image;

  if(!IsImage8UC1(&img))
    throw Except(__HERE__,"Grey scale image should be IPL_DEPTH_8U, 1 channel!");

  //alloc global memory
  data.AllocMem(&img,lines);
  data.MIN_LINE_LENGTH = minLineLength;

  descriptors.resize(lines.size());
  for (unsigned i=0; i<lines.size(); i++)
  {
    if (lines[i].size()==2)
    {
      if (data.SetLine(lines[i][0],lines[i][1]))
      {
        bool swap = GetPSR(&img, data.hPatch, data.hTmp, data.m, data.len, data.phi);

        cvSobel( data.hPatch, data.hDx, 1, 0, 3);
        cvSobel( data.hPatch, data.hDy, 0, 1, 3);

        WeightPSRGauss(gauss, data.hDx);
        WeightPSRGauss(gauss, data.hDy);

        descriptors[i].resize(data.NUM_PSR*8);

        SampleMSLD(data.hDx, data.hDy, &descriptors[i][0]); 
          
        NormalizeAndCutMSLD(&descriptors[i][0]);
      }
    }
    else throw Except(__HERE__,"Invalide line!");
  }
}








/********************************** DEBUG *******************************
 * SaveImage
 */
void CreateMSLD::SaveImage(const char *file, IplImage *img, float scale)
{
  IplImage *img8U = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, img->nChannels );
  double min,max;
  cvMinMaxLoc( img, &min, &max);
  if (scale==FLT_MAX)
    cvConvertScale( img, img8U, 255./(max-min), -min*(255./(max-min)));
  else
    cvConvertScale( img, img8U, 255./scale, 0);
  cvConvertScale( img, img8U, 1., 0);
  cvSaveImage(file,img8U);
  cvReleaseImage(&img8U);
}



}

