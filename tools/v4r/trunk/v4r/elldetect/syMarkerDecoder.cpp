//
// (C) 2010, Gerhard Obernberger <gerhard@obernberger.at>
//
//

#include "syDebug.hpp"

#include "syMarkerDecoder.hpp"
#include "syStringConversion.hpp"

#include "ImgUtils.h"
#include "syExcept.hpp"



NAMESPACE_CLASS_BEGIN( RTE )

////////////////////////////////////////////////////////////////////////////////
CzMarkerDecoder::CzMarkerDecoder()
{
   m_Mark = new CzMark();
   if (m_Mark == 0)
      throw CzExcept(__HERE__, "Could not create instance of CzMark! Out of memory?");
   m_dDestinationRadius = 12.0;//px
   m_dQuotientOuterInner = 3.0;// OuterRadius/InnerRadius
   m_iRingWidth = 4;//px
   m_dMinContrast = 0.5;
}

////////////////////////////////////////////////////////////////////////////////
CzMarkerDecoder::~CzMarkerDecoder()
{
   delete m_Mark;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize with decoding options
void CzMarkerDecoder::Init(bool bEllipseWhite,
                           int iBitCode,
                           double dQuotientOuterInner,
                           double dDestInnerRadius,
                           int iRingWidth,
                           double dMinContrast,
                           bool bWriteDebugImages,
                           CzN::CzString sDebugDirectory)
{
   m_Mark->SetBitCode(iBitCode);
   if (bEllipseWhite) {
      m_Mark->SetType(FM_TYPE_WB);
   } else {
      m_Mark->SetType(FM_TYPE_BW);
   }
   m_dQuotientOuterInner = dQuotientOuterInner;
   m_dDestinationRadius = dDestInnerRadius;
   m_iRingWidth = iRingWidth;
   m_dMinContrast = dMinContrast;
   
   m_bDebugImages = bWriteDebugImages;
   m_sDebugDirectory = sDebugDirectory;
}

////////////////////////////////////////////////////////////////////////////////
// Decodes the mark around the given ellipse from the original image
int CzMarkerDecoder::DecodeEllipse(const IplImage *pImage, 
                                   CzEllipse *pEllipse,
                                   int iIdx)
{
   IplImage *img = TransformEllipse(pImage, pEllipse);
   if (img == NULL) {
      return 0;
   }
   // decode marker
   m_Mark->SetImage(img);
   m_Mark->FindMark(img->width / 2., img->height / 2., 
                    m_dDestinationRadius * m_dQuotientOuterInner, 
                    m_iRingWidth,
                    // innerer radius for contrast check: half of the radius of the inner circle
                    m_dDestinationRadius / 2.,
                    // middle radius for contrast check: half of the radius of the outermost circle of the expected mark
                    m_dDestinationRadius * m_dQuotientOuterInner / 2.0, 
                    m_dMinContrast);
   int mark_id = m_Mark->IdentifyMark();
   m_Mark->SetImage(NULL);

   // write debugging image
   if (m_bDebugImages) {
      WriteDebugImage(img, iIdx, mark_id);
   }

/*
   cout<<" EllipseID: "<<mark_id<<endl;
   char filename[300];
   sprintf(filename, "MarkerImage.PNG");
   cvSaveImage(filename, img);
   cvWaitKey(0);
*/

   cvReleaseImage(&img);
   return mark_id;
}

////////////////////////////////////////////////////////////////////////////////
// Transforms the given ellipse area of the originial image to a square (ellipse -> circle)
IplImage *CzMarkerDecoder::TransformEllipse(const IplImage *pImage, 
                                            CzEllipse *pEllipse)
{
   if (pEllipse->dA < 5) {
      return NULL;
   }

   double phi = -pEllipse->dPhi;
   double fact = m_dQuotientOuterInner * 1.02;//3.04;

   //compute src size where the whole ring code fits
   IplImage *src  = cvCreateImage(cvSize((int)(pEllipse->dA*fact*2),
                                         (int)(pEllipse->dB*fact*2)), 
                                  pImage->depth, pImage->nChannels);
   IplImage *dst  = cvCreateImage(cvSize((int)(pEllipse->dA*fact*2),
                                         (int)(pEllipse->dB*fact*2)), 
                                  pImage->depth, pImage->nChannels);
   IplImage *dst2 = cvCreateImage(cvSize((int)(m_dDestinationRadius*fact*2),
                                         (int)(m_dDestinationRadius*fact*2)), 
                                  pImage->depth, pImage->nChannels);
   if ((src == 0) || (dst == 0) || (dst2 == 0))
      throw CzExcept(__HERE__, "Could not create instances of IplImages! Out of memory?");

   cvGetRectSubPix(pImage, src, cvPoint2D32f(pEllipse->dX, pEllipse->dY));

   double a[] = { cos(phi), sin(phi), pEllipse->dX,
                 -sin(phi), cos(phi), pEllipse->dY };

   double ell_a = pEllipse->dA / m_dDestinationRadius;
   double ell_b = pEllipse->dB / m_dDestinationRadius; 
   double scale[] = { 1./ell_a,        0, 0,
                             0, 1./ell_b, 0,
                             0,        0, 1 };

   CvMat Ma     = cvMat(2, 3, CV_64FC1, a);
   CvMat MScale = cvMat(3, 3, CV_64FC1, scale);

   cvGetQuadrangleSubPix(pImage, dst, &Ma);
   cvWarpPerspective(dst, dst2, &MScale, CV_INTER_CUBIC);

   cvReleaseImage(&src);
   cvReleaseImage(&dst);
   return dst2;
}


////////////////////////////////////////////////////////////////////////////////
// write debugging image with ellipses and marker IDs
void CzMarkerDecoder::WriteDebugImage(const IplImage *pImage, 
                                      int iIdx,
                                      int iMarkID)
{

   IplImage *img = cvCreateImage(cvGetSize(pImage), IPL_DEPTH_8U, 3);
   if (img == 0)
      throw CzExcept(__HERE__, "Could not create instance of IplImage! Out of memory?");
   cvConvertImage(pImage, img);
   
   CzStringConversion s(m_sDebugDirectory);
   char filename[300];
   sprintf(filename, "%soutput-%04d-%03d-1.jpg", s.GetStr(), iIdx, iMarkID);
   cvSaveImage(filename, img);

   int radius = cvRound(m_dDestinationRadius * m_dQuotientOuterInner);
   cvCircle(img, cvPoint((int)(img->width/2.),
                         (int)(img->height/2.)), 
                 radius, 
                 CV_RGB(255,0,0), 1, CV_AA, 0);
   cvCircle(img, cvPoint((int)(img->width/2.),
                         (int)(img->height/2.)), 
                 radius - m_iRingWidth, 
                 CV_RGB(255,0,0), 1, CV_AA, 0);

   sprintf(filename, "%soutput-%04d-%03d-2.jpg", s.GetStr(), iIdx, iMarkID);
   cvSaveImage(filename, img);
   
   cvReleaseImage(&img);
}

NAMESPACE_CLASS_END()
