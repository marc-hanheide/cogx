//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#include "syDebug.hpp"

#include "syEllipseHypothesis.hpp"

#include "ImgUtils.h"
#include "syArray.hpp"
#include "syVector2.hpp"
#include "syEllipse.hpp"
#include "syExcept.hpp"
#include "syEdge.hpp"



NAMESPACE_CLASS_BEGIN( RTE )

////////////////////////////////////////////////////////////////////////////////
CzEllipseHypothesis::CzEllipseHypothesis()
{
   m_SobelDx = NULL;
   m_SobelDy = NULL;
   m_Canny = NULL;
}

////////////////////////////////////////////////////////////////////////////////
CzEllipseHypothesis::~CzEllipseHypothesis()
{
   Clear();
}

////////////////////////////////////////////////////////////////////////////////
// Find ellipse hypotheses on given pImage
int CzEllipseHypothesis::FindHypotheses(const IplImage *pImage,
                                        int iMinEllipseDiameter,
                                        int iMaxEllipseDiameter,
                                        double dMaxQuotient,
                                        double dRadiusInOut,
                                        int iMinContrast,
                                        double dMaxContrastDev,
                                        double dMinSupport,
                                        double dMaxHypFiterror,
                                        int iSobelApertureSize,
                                        int iCannyLow,
                                        int iCannyHigh,
                                        int iLinkingGrowCount,
                                        bool bSmooth,
                                        bool bEllipseWhite)
{
   struct timespec start, end;

   Clear();

   IplImage *dx = NULL;
   IplImage *dy = NULL;
   IplImage *edges = NULL;

   getRealTime(&start);

   // calculate sobel and canny images
   GetSobelCanny(pImage, 
                 bSmooth,
                 iSobelApertureSize,
                 iCannyLow,
                 iCannyHigh,
                 dx, dy, edges);

   getRealTime(&end);

//   cout<<"Time Sobel&Canny [s]: "<<timespec_diff(&end, &start)<<endl;

   getRealTime(&start);
   int id_linking;
   LogStart(id_linking);
   // link canny edges
   LinkEdges(pImage, dx, dy, edges,
             iMinEllipseDiameter,
             iMaxEllipseDiameter,
             dMaxQuotient,
             dRadiusInOut,
             iMinContrast,
             dMaxContrastDev,
             dMinSupport,
             dMaxHypFiterror,
             bEllipseWhite,
             iLinkingGrowCount);
   LogEnd(id_linking, "time linking [s]: ");
	getRealTime(&end);
//		cout<<"Time LinkEdge&CheckContrast [s]: "<<timespec_diff(&end, &start)<<endl;

	return m_Hypotheses.Size();
}

////////////////////////////////////////////////////////////////////////////////
// Creates sobel and canny images, if not yet created
void CzEllipseHypothesis::GetSobelCanny(const IplImage *pImage, 
                                        bool bSmooth,
                                        int iSobelApertureSize,
                                        int iCannyLow,
                                        int iCannyHigh,
                                        IplImage *&pOutSobelDx, 
                                        IplImage *&pOutSobelDy, 
                                        IplImage *&pOutCanny)
{
   if ((m_SobelDx == NULL) || 
       (m_SobelDy == NULL) ||
       (m_Canny == NULL)) {

      ClearSobelCanny();
      m_Canny = cvCreateImage( cvGetSize(pImage), pImage->depth, pImage->nChannels );
      if (m_Canny == NULL)
         throw CzExcept(__HERE__, "could not create IplImage! Out of memory?");
         
      cvZero(m_Canny);
      if (bSmooth) {
  	      cvSmooth( pImage, m_Canny, CV_BLUR, 3, 3, 0, 0 );
      } else {
		   cvCopy( pImage, m_Canny );
      }
      CzEdge *ce = new CzEdge();
      if (ce == NULL) 
         throw CzExcept(__HERE__, "could not create edge detection class instance CzEdge! Out of memory?");
         
      m_SobelDx = cvCreateImage( cvGetSize(pImage), IPL_DEPTH_16S, 1 );
      m_SobelDy = cvCreateImage( cvGetSize(pImage), IPL_DEPTH_16S, 1 );   
      if ((m_SobelDx == NULL) || (m_SobelDy == NULL)) 
         throw CzExcept(__HERE__, "could not create IplImage! Out of memory?");
         
      ce->Set( iSobelApertureSize );
      int id_sobel, id_canny;
      LogStart(id_sobel);
      ce->Sobel( m_Canny, m_SobelDx, m_SobelDy );
      LogEnd(id_sobel, "time sobel [s]: ");
      LogStart(id_canny);
      ce->Canny( m_SobelDx, m_SobelDy, m_Canny, iCannyLow, iCannyHigh );
      LogEnd(id_sobel, "time canny [s]: ");
      delete ce;
   }

   // return the created images
   pOutSobelDx = m_SobelDx;
   pOutSobelDy = m_SobelDy;
   pOutCanny = m_Canny;
}

////////////////////////////////////////////////////////////////////////////////
// Check contrast of given ellipse
void CzEllipseHypothesis::LinkEdges(const IplImage *pImage,
                                    IplImage *pSobelDx,
                                    IplImage *pSobelDy,
                                    IplImage *pCanny,
                                    int iMinEllipseDiameter,
                                    int iMaxEllipseDiameter,
                                    double dMaxQuotient,
                                    double dRadiusInOut,
                                    int iMinContrast,
                                    double dMaxContrastDev,
                                    double dMinSupport,
                                    double dMaxHypFiterror,
                                    bool bEllipseWhite,
                                    int iLinkingGrowCount)
{
		
   CzArray<CzEdgel> points(10000);
   CzArray<bool> inlIdx(10000);
   CzHypothesis *ell;
   double ex,ey,ea,eb,phi;

   CzEdge *ce = new CzEdge();
   if (ce == 0)
      throw CzExcept(__HERE__, "Could not create edge detection class instance CzEdge! Out of memory?");
  	
   CzArray<CzSegment *> segments(10000);
   CzArray<CzSegment *> segments_mem(10000);
   segments.Clear();
   segments_mem.Clear();

   int link_grow_count = iLinkingGrowCount;
   if (link_grow_count < 0) {
      // auto calculate linking grow count
      link_grow_count = (int)(iMaxEllipseDiameter * M_PI * (1 - dMinSupport) / 2.);
      Log("linking grow count calculated [px]: %d", link_grow_count);
   }
   ce->LinkEdge( pCanny, segments, segments_mem, pSobelDx, pSobelDy, link_grow_count );

   // calculating ellipse hypotheses out of the segments
   for (int s=0; s < (int)segments.Size(); s++) {
      points.Clear();
      points = segments[s]->edgels;
	
      if (points.Size() > 6) {
         CzHypothesis::FitEllipse(points,ex,ey,ea,eb,phi);
         ell = new CzHypothesis(ex,ey,ea,eb,phi); 
         if (ell == 0)
            throw CzExcept(__HERE__, "Could not create instance of CzHypothesis! Out of memory?");

         if ((2*ea < iMaxEllipseDiameter) && 
             (2*eb > iMinEllipseDiameter) && 
             (ea/eb < dMaxQuotient)) {
            // check contrast of ellipse hypothesis
            bool del_ellipse = !CheckContrast(pImage, pSobelDx, pSobelDy, 
                                              ell, points, 
                                              dRadiusInOut, iMinContrast, dMaxContrastDev,
                                              bEllipseWhite);

            // calculate ellipse support and fiterror
            inlIdx.Clear();
            ell->EllipseSupport(points, .5, inlIdx);

            // check ellipse support and fiterror
            if ((ell->dSupport > dMinSupport) && 
                (ell->dFitError < dMaxHypFiterror) && 
                (!del_ellipse)) {
               // copy ellipse points (edgels)
               // they are needed for exact ellipse calculation
               for (int i=0; i<(int)points.Size(); i++) {
                  ell->m_pEdgels.PushBack(CzEdgel(points[i]));
               }
               m_Hypotheses.PushBack(ell);
            } else {
               delete ell;
            }
         } else {
            if(ell) delete ell;         
         }                  
      }
   }
   points.Clear();
   inlIdx.Clear();
   for (int s=0; s < (int)segments_mem.Size(); s++) {
      delete segments_mem[s];
   }
   segments_mem.Clear();
   segments.Clear();
   delete ce;
}

////////////////////////////////////////////////////////////////////////////////
// Check contrast of given ellipse
bool CzEllipseHypothesis::CheckContrast(const IplImage *pImage,
                                        IplImage *pSobelDx, 
                                        IplImage *pSobelDy, 
                                        CzHypothesis *pEllipse, 
                                        CzArray<CzEdgel> &pEllPoints,
                                        double dRadiusInOut,
                                        int iMinContrast,
                                        double dMaxContrastDev,
                                        bool bEllipseWhite)
{
   double ex=pEllipse->dX,
          ey=pEllipse->dY,
          ea=pEllipse->dA,
          eb=pEllipse->dB,
          phi=pEllipse->dPhi;

   unsigned int i=0;

   //Checks for contrast and ellipse color!
   float contrastSigma=0;
   float posSum=0, negSum=0;
   unsigned int pointsSkipped=0;
   if (dRadiusInOut < 2) 
      dRadiusInOut = 2.0;
      
   for (i=0; i < pEllPoints.Size(); i++) {
      float norm = sqrt(pow((float)GetPixelI16(pSobelDx, (int)pEllPoints[i].p.x, (int)pEllPoints[i].p.y),2) + 
                        pow((float)GetPixelI16(pSobelDy, (int)pEllPoints[i].p.x, (int)pEllPoints[i].p.y),2));      
      float dxx = (float)cvRound(dRadiusInOut*GetPixelI16(pSobelDx, (int)pEllPoints[i].p.x, (int)pEllPoints[i].p.y)/norm);
      float dyy = (float)cvRound(dRadiusInOut*GetPixelI16(pSobelDy, (int)pEllPoints[i].p.x, (int)pEllPoints[i].p.y)/norm);

      if (InRange(pImage, (int)pEllPoints[i].p.x, (int)pEllPoints[i].p.y, dxx, dyy) ) {
         if (pEllipse->InsideEllipse(ea, eb, ex, ey, phi, pEllPoints[i].p.x + dxx, 
                                                          pEllPoints[i].p.y + dyy)) {   
            posSum += (float)(GetPixel(pImage, (int)(pEllPoints[i].p.x + dxx), 
                                               (int)(pEllPoints[i].p.y + dyy)));
            negSum += (float)(GetPixel(pImage, (int)(pEllPoints[i].p.x - dxx), 
                                               (int)(pEllPoints[i].p.y - dyy)));
         } else {
            negSum += (float)(GetPixel(pImage, (int)(pEllPoints[i].p.x + dxx), 
                                               (int)(pEllPoints[i].p.y + dyy)));
            posSum += (float)(GetPixel(pImage, (int)(pEllPoints[i].p.x - dxx), 
                                               (int)(pEllPoints[i].p.y - dyy)));
         }
      } else {
         pointsSkipped++;
      }
   }

   int totalPointss=pEllPoints.Size()-pointsSkipped;
   int contrast;
   if (bEllipseWhite) {
      contrast = (int)((posSum / totalPointss) - (negSum / totalPointss));
   } else {
      contrast = (int)((negSum / totalPointss) - (posSum / totalPointss));
   }

   if (contrast < iMinContrast)
      return false;

   if (dMaxContrastDev > 0) {
      contrastSigma = 0;
      for (i=0; i < pEllPoints.Size(); i++) {
         // compute contrast standard deviation
         // no need to check for inside/outside ellipse
         float norm = sqrt(pow((float)GetPixelI16(pSobelDx, (int)pEllPoints[i].p.x, (int)pEllPoints[i].p.y),2) + 
                           pow((float)GetPixelI16(pSobelDy, (int)pEllPoints[i].p.x, (int)pEllPoints[i].p.y),2));      
         float dxx = (float)cvRound(dRadiusInOut*GetPixelI16(pSobelDx, (int)pEllPoints[i].p.x, (int)pEllPoints[i].p.y)/norm);
         float dyy = (float)cvRound(dRadiusInOut*GetPixelI16(pSobelDy, (int)pEllPoints[i].p.x, (int)pEllPoints[i].p.y)/norm);

         if (InRange(pImage, (int)pEllPoints[i].p.x, (int)pEllPoints[i].p.y, dxx, dyy) ) {
            contrastSigma += pow( (float)(GetPixel(pImage, (int)(pEllPoints[i].p.x + dxx), 
                                                           (int)(pEllPoints[i].p.y + dyy)))
                                  - (int)(GetPixel(pImage, (int)(pEllPoints[i].p.x - dxx), 
                                                           (int)(pEllPoints[i].p.y - dyy))) - contrast, 2);
         }
      }
      contrastSigma = sqrt(contrastSigma/totalPointss);
      if (contrastSigma > dMaxContrastDev) {
         //cerr << "Contrast sigma:" << contrastSigma;                                              
         return false;
      }
   }
   return true;
}

////////////////////////////////////////////////////////////////////////////////
// Set ellipse hypotheses for further processing
void CzEllipseHypothesis::SetHypotheses(CzArray<CzHypothesis*> &pHypotheses)
{
   Clear();
   for (int i=0; i<(int)pHypotheses.Size(); i++) {
      m_Hypotheses.PushBack(pHypotheses[i]);
   }
}

////////////////////////////////////////////////////////////////////////////////
// Clear results array of ellipse hypothesis and sobel/canny images
void CzEllipseHypothesis::Clear()
{
   for (int i=0; i<(int)m_Hypotheses.Size(); i++) {
      delete m_Hypotheses[i];
   }
   m_Hypotheses.Clear();
   // release created images
   ClearSobelCanny();
}

////////////////////////////////////////////////////////////////////////////////
// Release sobel and canny result images
void CzEllipseHypothesis::ClearSobelCanny()
{
   if (m_SobelDx != NULL) {
      cvReleaseImage(&m_SobelDx);
   }
   if (m_SobelDy != NULL) {
      cvReleaseImage(&m_SobelDy);
   }
   if (m_Canny != NULL) {
      cvReleaseImage(&m_Canny);
   }
   m_SobelDx = NULL;
   m_SobelDy = NULL;
   m_Canny = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Get the count of found ellipse hypotheses
const int CzEllipseHypothesis::GetHypothesesCount()
{
   return m_Hypotheses.Size();
}

////////////////////////////////////////////////////////////////////////////////
// Get ellipse hypothesis by index
CzHypothesis* CzEllipseHypothesis::GetHypothesis( int iIndex )
{
   ASSERT( iIndex >= 0 );
   ASSERT( iIndex < (int) m_Hypotheses.Size() );
   
   return m_Hypotheses[iIndex];
}

NAMESPACE_CLASS_END()
