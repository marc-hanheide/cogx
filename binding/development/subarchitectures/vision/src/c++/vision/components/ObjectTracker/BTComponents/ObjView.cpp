/** @file ObjectView.cpp
 *  @brief An Abstract class of object's view.
 *
 *  @author Somboon Hongeng
 *  @date march 2007
 *  @bug No known bugs.
 */
#include "ObjView.h"
#include <vision/components/common/SystemUtils/Common.h>
#include <vision/components/common/GeomUtils/GeomUtils.hpp>
#include "ViewTracker.h"

using namespace Common;
using namespace std;
using namespace TrackAux;
using namespace Geom;

const int ObjView::NUM_STATES;
const int ObjView::NUM_PTS;
const int ObjView::NUM_DIRS;
const int ObjView::MAX_CONTOURS;
const int ObjView::MAX_CONTOUR_NUM;
const int ObjView::DISTTHRESH;

// Out-dated. This can be removed.
const float ObjView::OutOfRangeCOST; 

const float ObjView::MaxColorMatchingSCORE;
const int ObjView::DOWNSAMPLE;
const int ObjView::LOSTCOUNT_THRESHOLD;

ObjView::ObjView() 
{
    m_id = -1;
    m_view = -1;
    m_shape = SH_UNDEF;

    m_type = Vision::UNDEF;
    
    // A link to parent node. No need to free memory.
    m_pViewTracker = NULL;

    m_hist = NULL;
    m_bhistvalid = false;

    m_histImage = NULL;
    m_weightImage=NULL;
    m_backprojectImage=NULL;
    m_bImagesAllocated = false;

    m_missingCounter = 0;

    //m_bReinitialize = false;

    m_detectionState = UNINITIALIZED; // m_initialCond = true

    m_maxWeight = 0.;

    for(int i=0;i<this->NUM_DIRS;i++) 
    {
	float angle = i*TrackAux::PI*2/(float)this->NUM_DIRS ; 
	m_orient[i].x = (float)cos(angle) ; 
	m_orient[i].y = (float)sin(angle) ; 
    }

}

ObjView::~ObjView() {
    if (m_weightImage != NULL)    
	cvReleaseImage(&m_weightImage);
    if (m_backprojectImage != NULL)    
	cvReleaseImage(&m_backprojectImage);
    if (m_histImage != NULL)
	cvReleaseImage(&m_histImage);
    if (m_hist!=NULL)    
	delete(m_hist);
}


void ObjView::allocateImages(CvSize _bsize) {
    if (m_bImagesAllocated)
	return;
    
    m_bsize.width = _bsize.width; 
    m_bsize.height= _bsize.height;
    
    m_backprojectImage = cvCreateImage(m_bsize,8,1) ; 
    m_weightImage = cvCreateImage(m_bsize,8,1) ;
    
    m_bImagesAllocated = true;
}


CvHistogram *ObjView::getHistogram() 
{
    if (m_bhistvalid == true)
	return m_hist;
    else
	return NULL;
}


void ObjView::initHistogram(IplImage *roiImage, IplImage *mask) 
{
    if (roiImage==NULL)
	throw user_error(__HERE__, "RoiImage does not exist");
    m_hist = createHist();

    CvRect roiRect = cvGetImageROI(roiImage);
    //scale(roiRect, 2.0/3.0);

    updateHist(m_hist, roiImage, roiRect, mask);	
    m_bhistvalid = true ;
}


void ObjView::initHistogram(char patchFilename[]) 
{
    m_histImage = cvLoadImage(patchFilename);
    
    if (m_histImage != 0) 
    {
	m_hist = createHist();
	IplImage* mask=0;
	updateHist(m_hist, m_histImage, 
		   cvRect(0,0,m_histImage->width, m_histImage->height),mask);	
	m_bhistvalid = true ;
    }
    else 
      throw user_error(__HERE__, "Patch file '%s' does not exist?",
                       patchFilename);  
}


bool ObjView::is_reliable(evidenceType basis /*=Combined*/) {
  switch(basis) {
        case (COMBINED):
            if ((colorCost() < 8.0) && (edgeCost() < 6.0)) 
		return true;
	    else 
		return false;
	    break;
	case(COLOR):
	  if (colorCost() < 8.0) // 8.5 before
		return true;
	    else
		return false;
	    break;
	case (EDGE):
	    if (edgeCost() < 6.0)
		return true;
	    else
		return false;
	    break;    
    }
    
    throw user_error(__HERE__, "Unsupported basis for reliability");
}

float ObjView::Evaluate_backProjection(float state[]) 
{
  return (DISTTHRESH * (MaxColorMatchingSCORE - averageBackProjection(state, true))
	  /MaxColorMatchingSCORE);
}


void ObjView::computePotentialImg(float max_radius) 
{
#ifdef LOG
  cout << "[ObjView::computePotentialImg] " << endl;
#endif

    cvZero(m_weightImage) ;
    m_maxWeight = 0.;
    for(int y=0; y<m_bsize.height; y+=2) 
    {
	uchar * weightimgy = (uchar*)m_weightImage->imageData+
	    y*m_weightImage->widthStep ; 
	
	for(int x=0;x<m_bsize.width;x+=2) {
	  float state[3] = {x*this->DOWNSAMPLE, y*this->DOWNSAMPLE, max_radius} ; 

	  float val = averageBackProjection(state) ; 
	  weightimgy[x] = (uchar) val;

	  m_maxWeight = std::max(m_maxWeight, val);
	}
    }
    cvSmooth(m_weightImage, m_weightImage ,CV_GAUSSIAN,3,3) ; 
}


void ObjView::CalBack(IplImage *src, IplImage *back, 
		      CvHistogram *hist_to_calback) {
  
  CvSize rectimgsize = cvGetSize(src) ; 
  IplImage * hsvimg = cvCreateImage(rectimgsize,8,3); 
  cvCvtColor(src, hsvimg, CV_BGR2HSV) ; 
  IplImage * imgplane[3] = {cvCreateImage(rectimgsize,8,1),
			    cvCreateImage(rectimgsize,8,1),
			    cvCreateImage(rectimgsize,8,1)};
  cvCvtPixToPlane( hsvimg, imgplane[0],imgplane[1],imgplane[2], 0 );
  cvCalcBackProject(imgplane,back,hist_to_calback) ; 
  
  // Release heap memory
  cvReleaseImage(&hsvimg) ;
  for(int i=0;i<3;i++){cvReleaseImage(&imgplane[i]);}  ;
}


