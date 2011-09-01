//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#include "syDebug.hpp"

#include "syEllipseDetection.hpp"

#include "syStringConversion.hpp"
#include <omp.h>
#include <iostream>
#include <iomanip>

#if defined WIN32 || defined WIN64
   #if defined ELLDETECTEXE
   #else
      #include "ipAOIMem.hpp"
   #endif
#endif

NAMESPACE_CLASS_BEGIN( RTE )

////////////////////////////////////////////////////////////////////////////////
CzEllipseDetection::CzEllipseDetection()
{
   // initialize configuration with default values
   // possible values:
   //    ELL_CONFIGMETHOD_HYPOTHESES    : no subpixel calculation (fast, most inexact)
   //    ELL_CONFIGMETHOD_DUAL_ELLIPSE  : dual ellipse method (slow, most precise)
   m_Config.eMethod = ELL_CONFIGMETHOD_DUAL_ELLIPSE;
   
   // minimum/maximum ellipse diameter in pixel
   m_Config.iMinEllipseDiameter = 5;
   m_Config.iMaxEllipseDiameter = 150;
   
   // spacing from the ellipse border in pixels used to check min. contrast and max. contrast deviation
   m_Config.dRadiusInOut = 4.0;
   
   // minimum contrast to the background; good value is 20.0
   m_Config.iMinContrast = 20;
   
   // maximum standard deviation of ellipse points
   // use values 5.0 to 15.0
   m_Config.dMaxContrastDev = 10.0;
   
   // maximum quotient long to short ellipse axis
   m_Config.dMaxQuotient = 4.0;
   
   // minimum pixel support 
   // useful range: 0.6 to 1.0
   m_Config.dMinSupport = 0.70;
   
   // maximum mean fiterror for hypotheses detection
   // useful range: 0.05 to 1.00
   m_Config.dMaxHypFiterror = 0.30;

   // maximum mean fiterror for exakt ellipse calculation
   // useful range: 0.05 to 1.00
   m_Config.dMaxFiterror = 0.30;
   
   // aperture size of sobel operator 
   // use values 3 or 5
   m_Config.iSobelApertureSize = 3;
   
   // canny edge detector, low/high border 
   // depend on sobel aperture size
   // useful default values:
   //    sobel aperture = 3: canny low/high = 70/140
   //    sobel aperture = 5: canny low/high = 200/250
   m_Config.iCannyLow = 70;
   m_Config.iCannyHigh = 140;
   
   // do a gaussian smoothing on the input image
   m_Config.bGaussSmooth = false;

   // maximum amount of pixels for a segment to grow for edge-segment-linking
   // for automatic calculation (based on min./max. ellipse diameter set to < 0)
   m_Config.iLinkingGrowCount = 50;

   // minimum gradient for a pixel to be used for dual ellipse calculation
   m_Config.dDualEllipseMinGradient = 30.0;

   // number of pixels to use around the border of the ellipse hypothesis for dual ellipse calculation
   m_Config.iDualEllipseBorder = 5;

   // find white or black ellipses
   m_Config.bEllipseWhite = true;
   
   // options for decoding the markers
   // decode the code of the marker around the ellipse
   m_Config.bDecodeMarker = true;
   // type of markers (12bit, 14bit, 20bit)
   m_Config.eMarkerCode = ELL_MARKERCODE_14BIT;
   // quotient outermost to innermost radius of marker
   m_Config.dMarkerQuotientOuterInner = 3.0;
   // inner circle radius in pixels after deskewing (good value is 12.0)
   m_Config.dMarkerDestInnerRadius = 12.0;
   // outer ring width in pixels
   m_Config.iMarkerRingWidth = 10;
   // minimal contrast needed (range: 0 ... 1.0) compared to the contrast of the inner circle to the surrounding area
   m_Config.dMarkerMinContrast = 0.5;

   // debugging options
   // directory to store debugging images and log file
   m_Config.sDebugDirectory = _T("");
   // create debugging images
   m_Config.bDebugImages = false;
   m_Config.bDebugImagesMarker = false;
   // create log file
   m_Config.bLogFiles = false;

   m_pHypothesis = new CzEllipseHypothesis();
   if (m_pHypothesis == 0)
      throw CzExcept(__HERE__, "Could not create instance of CzEllipseHypothesis! Out of memory?");
   m_pMethodDualEllipse = new CzEllMethodDualEllipse();
   if (m_pMethodDualEllipse == 0)
      throw CzExcept(__HERE__, "Could not create instance of CzEllMethodDualEllipse! Out of memory?");
   m_pMarkerDecoder = new CzMarkerDecoder();
   if (m_pMarkerDecoder == 0)
      throw CzExcept(__HERE__, "Could not create instance of CzMarkerDecoder! Out of memory?");
   
   // create unique logger instance
   m_pLogger = new CzLogger();
   if (m_pLogger == 0)
      throw CzExcept(__HERE__, "Could not create instance of CzLogger! Out of memory?");
   // set unique logger instance
   SetLogger(m_pLogger);
   m_pHypothesis->SetLogger(m_pLogger);
   m_pMethodDualEllipse->SetLogger(m_pLogger);
   m_pMarkerDecoder->SetLogger(m_pLogger);
}

////////////////////////////////////////////////////////////////////////////////
CzEllipseDetection::~CzEllipseDetection()
{
   Clear();
   delete m_pHypothesis;
   delete m_pMethodDualEllipse;
   delete m_pMarkerDecoder;
   delete m_pLogger;
}

////////////////////////////////////////////////////////////////////////////////
void CzEllipseDetection::SetLogging(bool bDoLogging)
{
   SetDoLogging(bDoLogging);
   m_pHypothesis->SetDoLogging(bDoLogging);
   m_pMethodDualEllipse->SetDoLogging(bDoLogging);
   m_pMarkerDecoder->SetDoLogging(bDoLogging);
}

////////////////////////////////////////////////////////////////////////////////
// find ellipses on given Steinbichler image
int CzEllipseDetection::FindEllipses(const CzN::CzAOI *pImage)
{
   SetLogging(m_Config.bLogFiles);
   
#if defined WIN32 || defined WIN64
#if defined ELLDETECTEXE
   return -1;
#else
   if (pImage == NULL)
      throw CzExcept(__HERE__, "parameter pImage must be <> NULL");
      
   int id;
   LogStart(id);
   const CzN::CzAOI8Mem *pAOI8 = dynamic_cast<const CzN::CzAOI8Mem*>(pImage);
   
	IplImage *img = cvCreateImage(cvSize(pImage->AOIWidth(), pImage->AOIHeight()), IPL_DEPTH_8U, 1);
   if (img == NULL)
      throw CzExcept(__HERE__, "could not create IplImage! Out of memory?");
	
   for (SHORT i=0; i<img->height; i++) {
      for (SHORT j=0; j<img->width; j++) {
         UCHAR *ptr = &CV_IMAGE_ELEM(img, UCHAR, i, j);
         UCHAR val = pAOI8->GetPoint(CzN::CzsPoint(j, i));
         ptr[0] = val;
      }
      // this would be the better choice, but is not available
      //memcpy(&img->imageData[i * img->widthStep], pAOI8->GetRowPointer((SHORT)i), img->width);
   }

   int ell_count = FindEllipses(img);
   LogEnd(id, "time total + copy image [s]: ");
   
   cvReleaseImage(&img);
   return ell_count;
#endif
#else
   return -1;
#endif
}

////////////////////////////////////////////////////////////////////////////////
// find ellipses on given OpenCV image
int CzEllipseDetection::FindEllipses(const IplImage *pImage)
{
   if (pImage == NULL)
      throw CzExcept(__HERE__, "parameter pImage must be <> NULL");
      
   int id_all, id_start;

   Clear();
   
   SetLogging(m_Config.bLogFiles);
   if (m_Config.bLogFiles)
      m_pLogger->Init(m_Config.sDebugDirectory);
      
   Log("start *FindEllipses*");
   
   Log("find hypotheses ...");
   LogStart(id_all);
   LogStart(id_start);
   // find ellipse hypotheses
   m_pHypothesis->FindHypotheses(pImage,
                      m_Config.iMinEllipseDiameter,
                      m_Config.iMaxEllipseDiameter,
                      m_Config.dMaxQuotient,
                      m_Config.dRadiusInOut,
                      m_Config.iMinContrast,
                      m_Config.dMaxContrastDev,
                      m_Config.dMinSupport,
                      m_Config.dMaxHypFiterror,
                      m_Config.iSobelApertureSize,
                      m_Config.iCannyLow,
                      m_Config.iCannyHigh,
                      m_Config.iLinkingGrowCount,
                      m_Config.bGaussSmooth,
                      m_Config.bEllipseWhite);
   Log("found %d hypotheses", m_pHypothesis->GetHypothesesCount());
   LogEnd(id_start, "time find hypotheses [s]: ");

   // process exact ellipse center calculation
   DoExactCalculation(pImage);
   
   Log("found %d exact ellipses", GetEllipsesCount());
   LogEnd(id_all, "time total [s]: ");

   LogStart(id_start);
   if (m_Config.bLogFiles) {
      WriteLogFiles();
   }
   if (m_Config.bDebugImages) {
      WriteDebugImage(pImage);
   }
   LogEnd(id_start, "time write debug information [s]: ");

   // return the count of ellipses found on the image
   return GetEllipsesCount();
}

////////////////////////////////////////////////////////////////////////////////
// finds exact ellipses on given OpenCV image and actual hypotheses
void CzEllipseDetection::DoExactCalculation(const IplImage *pImage)
{
   int id_start;

   Log("do exact ellipse calculation ...");
   LogStart(id_start);

   IplImage *dx = NULL;
   IplImage *dy = NULL;
   IplImage *edges = NULL;

   m_pHypothesis->GetSobelCanny(pImage, 
                                m_Config.bGaussSmooth,
                                m_Config.iSobelApertureSize,
                                m_Config.iCannyLow,
                                m_Config.iCannyHigh,
                                dx, dy, edges);

   // initialize marker decoder with decoding options
   m_pMarkerDecoder->Init(m_Config.bEllipseWhite, 
                          (int)m_Config.eMarkerCode, 
                          m_Config.dMarkerQuotientOuterInner, 
                          m_Config.dMarkerDestInnerRadius,
                          m_Config.iMarkerRingWidth,
                          m_Config.dMarkerMinContrast,
                          m_Config.bDebugImagesMarker,
                          m_Config.sDebugDirectory);

   // creating new ellipses
   CzArray<CzEllipse*> new_ells(m_pHypothesis->GetHypothesesCount());
   for (int i=0; i<(int)new_ells.Size(); i++) {
      new_ells[i] = new CzEllipse();
      if (new_ells[i] == 0)
         throw CzExcept(__HERE__, "Could not create instance of CzEllipse! Out of memory?");
   }
   
   
   CzHypothesis *ell = NULL;
   CzEllipse *new_ell = NULL;
   int marker_ok = 0;
   for (int i=0; i<m_pHypothesis->GetHypothesesCount(); i++) {
      ell = m_pHypothesis->GetHypothesis(i);
      new_ell = new_ells[i];
      new_ell->Set(ell->dX, ell->dY, ell->dA, ell->dB, ell->dPhi);
      new_ell->dFitError = ell->dFitError;
      new_ell->dSupport = ell->dSupport;

      new_ell->bEllipseOK = false;
      switch (m_Config.eMethod) {
         case ELL_CONFIGMETHOD_HYPOTHESES:
            new_ell->bEllipseOK = true;
            break;
         case ELL_CONFIGMETHOD_DUAL_ELLIPSE:
            new_ell->bEllipseOK = m_pMethodDualEllipse->DualEllipse(m_Config.dDualEllipseMinGradient,
                                                                    m_Config.iDualEllipseBorder,
                                                                    dx, dy, ell->m_pEdgels, new_ell);
            if (new_ell->bEllipseOK) {
               new_ell->bEllipseOK = (new_ell->dFitError < m_Config.dMaxFiterror);
            }
            break;
         default:
            throw CzExcept(__HERE__, "ellipse method %d not valid!", (int)m_Config.eMethod);
      }

      if (new_ell->bEllipseOK) {
         if (m_Config.bDecodeMarker) {
            int mark_id = m_pMarkerDecoder->DecodeEllipse(pImage, new_ell, i);
            new_ell->iMarkerID = mark_id;
            if (mark_id != 0) {
               marker_ok++;
            }
         }
      }
   }
   LogEnd(id_start, "time exact calc [s]: ");

   for (int i=0; i<(int)new_ells.Size(); i++) {
      new_ell = new_ells[i];
      if (new_ell->bEllipseOK) {
//         m_pEllipses.PushBack(new_ell);
         m_pEllipses.Append(new_ell);
      } else {
         delete new_ell;
      }
   }
   new_ells.Clear();

   Log("marker decoded: %d/%d", marker_ok, m_pEllipses.Size());
   
}

////////////////////////////////////////////////////////////////////////////////
// find exact ellipses on the given image based on the given ellispse hypotheses
int CzEllipseDetection::FindEllipses(const IplImage *pImage, CzArray<CzHypothesis*> &pHypothesis)
{
   // clear all results
   Clear();

   // set the given hypothesis
   m_pHypothesis->SetHypotheses(pHypothesis);
   
   // process exact ellipse center calculation
   DoExactCalculation(pImage);
   
   // return the count of ellipses detected on the image
   return GetEllipsesCount();
}

////////////////////////////////////////////////////////////////////////////////
// configuration for ellipse detection
CzEllipseDetection::sEllipseDetectionConfig* CzEllipseDetection::GetConfig()
{
   return &m_Config;
}
   
////////////////////////////////////////////////////////////////////////////////
// get the count of detected ellipses
const int CzEllipseDetection::GetEllipsesCount()
{
   return m_pEllipses.Size();
}

////////////////////////////////////////////////////////////////////////////////
// get detected ellipse per index
CzEllipse* CzEllipseDetection::GetEllipse( int iIndex )
{
   ASSERT( iIndex >= 0 );
   ASSERT( iIndex < (int) m_pEllipses.Size() );
   
   return m_pEllipses[iIndex];
}

////////////////////////////////////////////////////////////////////////////////
// clear all results
void CzEllipseDetection::Clear()
{
	ClearEllipses();
   m_pHypothesis->Clear();
}

////////////////////////////////////////////////////////////////////////////////
// clear results array of ellipses
void CzEllipseDetection::ClearEllipses()
{
	for (int i=0; i<(int)m_pEllipses.Size(); i++) {
	   delete m_pEllipses[i];
	}
   m_pEllipses.Clear();
}

////////////////////////////////////////////////////////////////////////////////
// interface test routine
void CzEllipseDetection::__AddEllipse(double dX, double dY, double dA, double dB, double dPhi, double dFitError)
{
   CzEllipse *aEll = new CzEllipse(dX, dY, dA, dB, dPhi);
   if (aEll == 0)
      throw CzExcept(__HERE__, "Could not create instance of CzEllipse! Out of memory?");
   aEll->dFitError = dFitError;
//   m_pEllipses.PushBack(aEll);
   m_pEllipses.Append(aEll);
}

////////////////////////////////////////////////////////////////////////////////
// write csv-file with found ellipses
void CzEllipseDetection::WriteLogFiles()
{

   CzStringConversion s(m_Config.sDebugDirectory);
   char filename[300];
   sprintf(filename, "%sellipses.csv", s.GetStr());
   std::ofstream ellsfile(filename, ios::out);

   for (int i=0; i<GetEllipsesCount(); i++) {
      CzEllipse *ell = GetEllipse(i);
      ellsfile <<"ID=" << setprecision(10)
               <<ell->iMarkerID<<" ; "
               <<ell->dX<<" ; "
               <<ell->dY<<" ; "
               <<ell->dFitError<<" ; "
               <<ell->dFitError<<" ; "
               <<ell->dA*2.<<" ; "
               <<ell->dB*2.<<" ; "
               <<ell->dPhi<<endl;
   }
}

////////////////////////////////////////////////////////////////////////////////
// write debugging image with ellipses and marker IDs
void CzEllipseDetection::WriteDebugImage(const IplImage *pImage)
{

   IplImage *img = cvCreateImage(cvGetSize(pImage), IPL_DEPTH_8U, 3);
   if (img == 0)
      throw CzExcept(__HERE__, "Could not create instance of IplImage! Out of memory?");
   cvConvertImage(pImage, img);
  
   CvFont myFont;
   cvInitFont(&myFont, CV_FONT_HERSHEY_DUPLEX, 1.5, 1.5, 0, 3);
   for (int i=0; i<GetEllipsesCount(); i++) {
      CzEllipse *ell = GetEllipse(i);
      cvEllipse(img, cvPoint((int)(ell->dX+.5),
                             (int)(ell->dY+.5)), 
                cvSize((int)(ell->dA+.5), (int)(ell->dB+.5)), 
                -ell->dPhi*180./M_PI, 0, 360, CV_RGB(255,0,0), 1, CV_AA, 0);

      std::ostringstream oss;
      oss << ell->iMarkerID;
      std::string val = oss.str();

      cvPutText(img, val.c_str(), cvPoint((int)ell->dX + 20, (int)ell->dY+20), &myFont, CV_RGB(0,255,0));
   }
   
   CzStringConversion s(m_Config.sDebugDirectory);
   char filename[300];
   sprintf(filename, "%soutput.jpg", s.GetStr());
   cvSaveImage(filename, img);
   
   cvReleaseImage(&img);
}


NAMESPACE_CLASS_END()
