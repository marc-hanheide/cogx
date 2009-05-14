/** @file ViewTracker.cpp
 *  @brief A tracker that tracks views of objects.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#include "ViewTracker.h"
#include <vision/components/common/SystemUtils/Common.h>
#include <vision/components/common/GeomUtils/Model3D.h>
#include <vision/components/common/GeomUtils/Polygon2D.h>
#include <vision/components/common/GeomUtils/GeomUtils.hpp>

#include "CircleView.h"
#include "RectView.h"

//#define SHOW_EDGE_IMG
//#define DRAW_HAND

using namespace std;
using namespace Common;
using namespace Geom;
using namespace TrackAux;

ViewTracker::ViewTracker(int _viewId)
{
  viewId = _viewId;
  
  this->downsample=0;
  pCamera = NULL;

  inputimg = NULL;
  grayimg = NULL;
  cannyimg = NULL;
  distimg = NULL;
  prev_resizeimg = NULL;
  bTmpimages_allocated = false;
  m_bInitialCondition = true;
}

ViewTracker::~ViewTracker()
{
  if (grayimg != NULL)
    cvReleaseImage(&grayimg);
  if (cannyimg != NULL)
    cvReleaseImage(&cannyimg);
  if (distimg != NULL)
    cvReleaseImage(&distimg);
  if (prev_resizeimg != NULL)
      cvReleaseImage(&prev_resizeimg);

  map<int, ObjView*>::iterator iter;
  for (iter=mpObjectViews.begin();iter!=mpObjectViews.end();iter++) 
    delete(iter->second);
  mpObjectViews.clear();

  if (pCamera != NULL)
    delete(pCamera);
}

void ViewTracker::configure(int width, int height, 
			    shared_ptr<CameraProj> spProj)
{
    this->downsample = DOWNSAMPLE;     
    this->bRoiIsSet = false;
    pCamera = new CameraProj();
    *pCamera = *spProj;
    if (bTmpimages_allocated == false)
	init_tmpImages(cvSize(width,height));
}

void ViewTracker::configure(int width, int height, Vision::Camera *pcam) {
    this->downsample = DOWNSAMPLE;     
    this->bRoiIsSet = false;
     if (pcam != NULL) {
	 pCamera = new CameraProj();
	 pCamera->setDefault(this->viewId);
	 pCamera->readCamParms(*pcam);
     }
     if (bTmpimages_allocated == false)
	 init_tmpImages(cvSize(width,height));
}


void ViewTracker::configure(int width, int height,
			    TrackerConfig &trackerConfig, 
			    std::string cameraCfg) {

  this->downsample = trackerConfig.DOWNSAMPLE;

  if (trackerConfig.roi_is_set) {
    this->bRoiIsSet = true;
    this->Roi.m_center.x = trackerConfig.Roi.m_center.x;
    this->Roi.m_center.y = trackerConfig.Roi.m_center.y;
    this->Roi.m_size.x = trackerConfig.Roi.m_size.x;
    this->Roi.m_size.y = trackerConfig.Roi.m_size.y;
  }
  else
    this->bRoiIsSet = false;

  if (cameraCfg!="") {
    pCamera = new CameraProj();
    pCamera->setDefault(viewId);
    char cfgFilename[1024];
    sprintf(cfgFilename, cameraCfg.c_str(), viewId);
    pCamera->readCamParms(cfgFilename);
  }

  if (bTmpimages_allocated == false)
    init_tmpImages(cvSize(width,height));
}



ResolutionType ViewTracker::resolutionType() 
{    
    if (bTmpimages_allocated==false) 
	return RES_UNKNOWN;
    
    if ((imgsize.width ==640) && (imgsize.height==480))
	return (RES_VGA);
    else if ((imgsize.width==320) && (imgsize.height==240))
	return (RES_QVGA);
    else
	return (RES_UNKNOWN);
}

void ViewTracker::initializeObjectView(MobileObject *pMobileObject) 
{
    // Retrieve info about the object  
    int obj_id = pMobileObject->id();
    Vision::ObjType obj_type = pMobileObject->type();
    TrackAux::ObjShape obj_shape = pMobileObject->shape();	
    
    #if 0
    user_printf(__HERE__, "Initialize objview for objID %d, in view %d\n",
		obj_id, viewId);
    user_printf(__HERE__, "type: %s, sh: %s\n",
		ObjtypeToString(obj_type).c_str(),
		ShapetypeToString(obj_shape).c_str());
    #endif

    if ((obj_type==Vision::UNDEF) && (obj_shape==SH_UNDEF))
    {
	// no need to delete
	IplImage* roiImage = pMobileObject->getRoiImage(viewId);
	if (roiImage==NULL)
	    throw user_error(__HERE__, "RoiImage does not exist");

	CircleView* pObjView = new CircleView(this, viewId, obj_id, obj_type, 
					      obj_shape, pMobileObject->config());
	CvRect rectRoi = cvGetImageROI(roiImage);

	int radius = (int)((rectRoi.width + rectRoi.height)/4.0);

	pObjView->initRadius(radius);

	// no need to delete
	IplImage* mask = pMobileObject->getMaskImage(viewId);

	pObjView->initHistogram(roiImage, mask);

	mpObjectViews[obj_id] = pObjView;
    }
    else 
    {
	char histFilename[1024];
	snprintf(histFilename, 1024, pMobileObject->m_patchFileformat.c_str(), this->viewId);

	// Create an object view according to 2D shape, initialize the color histogram 
	if (obj_shape == SH_CIRCLE) {
	    CircleView* pObjView = new CircleView(this, viewId, obj_id, obj_type, 
						  obj_shape, pMobileObject->config());
	    pObjView->initHistogram(histFilename);
	    mpObjectViews[obj_id] = pObjView;
	}
	else if (obj_shape == SH_RECT) {
	    RectView* pObjView = new RectView(this, viewId, obj_id, obj_type, 
					      obj_shape, pMobileObject->config());	
	    pObjView->initHistogram(histFilename);
	    mpObjectViews[obj_id] = pObjView;
	}
    }
}

void ViewTracker::initializeObjectViews(std::map<int,MobileObject*> &mpMobileObjects) 
{
    if (mpMobileObjects.size()==0)
	return;

    std::map<int,MobileObject*>::iterator iter;
    for (iter=mpMobileObjects.begin(); iter!=mpMobileObjects.end(); iter++) 
    {
	int obj_id = iter->second->id();
	std::map<int, ObjView*>::iterator vit=mpObjectViews.find(obj_id);
	if (vit == mpObjectViews.end()) // new object ID
	    initializeObjectView(iter->second);
    }
}


void ViewTracker::drawResults(void) 
{
    DrawASample(inputimg);
   //drawDefects(inputimg);
}



void ViewTracker::DrawAllSamples(IplImage *img) {    
    std::map<int, ObjView*>::iterator iter;
    for (iter=mpObjectViews.begin(); iter!=mpObjectViews.end(); iter++) 
	iter->second->drawAllSamples(img);	
}

void ViewTracker::DrawASample(IplImage *img) 
{    
    std::map<int, ObjView*>::iterator iter;
    for (iter=mpObjectViews.begin(); iter!=mpObjectViews.end(); iter++) {
	
	ObjView* pObjectView = iter->second;	
	
	if (pObjectView->is_reliable()) { 
	    
	    pObjectView->drawASample(img);
	    
	    
// 	    if (pObjectView->type == TrackAux::TYPE_HAND) {
// 		color = getRandomColor(pObjectView->id);
// #ifdef DRAW_HAND 
// 		if (pObjectView->h_bestContour != -1) {
// 		    if (pObjectView->h_bestDefect !=-1) {
// 			cvLine(img, 
// 			       cvPoint((int)(pObjectView->beststate[0]),
// 				       (int)(pObjectView->beststate[1])),
// 			       cvPoint((int)(pObjectView->h_beststate[0]),
// 				       (int)(pObjectView->h_beststate[1])),
// 			       color);
// 			cvLine(img, 
// 			       cvPoint((int)(pObjectView->beststate[0]), 
// 				       (int)(pObjectView->beststate[1])),
// 			       cvPoint((int)(pObjectView->h_beststate[2]), 
// 				       (int)(pObjectView->h_beststate[3])),
// 			       color);
// 		    }
// 		}
// 		else {
// 		    x = cvRound(pObjectView->beststate[0]);
// 		    y = cvRound(pObjectView->beststate[1]);
// 		    r = cvRound(pObjectView->beststate[2]);
// 		    cvCircle(img,cvPoint(x,y),r, color, 2);
// 		}
// #else // draw just circle
// 		x = cvRound(pObjectView->beststate[0]);
// 		y = cvRound(pObjectView->beststate[1]);
// 		r = cvRound(pObjectView->beststate[2]);
// 		cvCircle(img,cvPoint(x,y),r, color, 2);
// #endif		
// 	    }
	    
	}
	
    }
}


void ViewTracker::track(IplImage* newimg) 
{
    static int delay=0;
    
#ifdef LOG
    cout << "[ViewTracker::track] " << endl;
#endif

    inputimg = newimg;
    
    if (bTmpimages_allocated == false)
	init_tmpImages(cvGetSize(newimg));
    
    cvCvtColor(inputimg, grayimg, CV_BGR2GRAY)	;
    cvCanny(grayimg,cannyimg,50,200,3) ;
    cvNot(cannyimg,cannyimg) ; 
    cvDistTransform(cannyimg,this->distimg) ;
    CvSize rectimgsize = bsize ; 
    IplImage * resizeimg = cvCreateImage(rectimgsize,8,3); 
    cvResize(inputimg,resizeimg) ; 
    
//     double min_val, max_val;
//     cvMinMaxLoc(distimg, &min_val, &max_val);
//     cout << "distImage: min " << min_val << ", max " << max_val << endl;



// #ifdef  MOTION_FILTER_ENABLED
//   if (bMotionDetectorInitialized == false) {
//     pMotionDetector->init(cvGetSize(resizeimg));
//     prev_resizeimg = resizeimg;
//     bMotionDetectorInitialized = true;
//   }
//   else {
//     pMotionDetector->detectOpticalFlow(resizeimg, prev_resizeimg);
//     cvReleaseImage(&prev_resizeimg);
//     prev_resizeimg = resizeimg;
//   }
// #endif

   
    if (m_bInitialCondition == true)
	delay = 2;
    else {
	delay--;
	delay = std::max(0,delay);
    }  

  map<int, ObjView*>::iterator iter;
  for (iter=mpObjectViews.begin(); iter!=mpObjectViews.end(); iter++) {
 
      ObjView* pObjView = iter->second;

      
// #ifdef MOTION_FILTER_ENABLED
//     if (m_bInitialCondition == true) {
//       if (pObjView->bhistvalid == true) 
// 	pObjView->detect(resizeimg, m_bInitialCondition);
//       else
// 	cerr << "ViewTracker::ERROR" << endl;
//     }
//     else {
//       if (delay==0 && pObjView->is_textured()) { 
// 	CvRect rect;
// 	pObjView->getResizedEnclosingBox(rect, 15, true); 

// #ifdef SHOW_ALL
// 	drawCvRect(resizeimg,rect);
// #endif
// 	cerr << "rect.x " << rect.x
// 	     << ",rect.y " << rect.y
// 	     << ",rect.width " << rect.width
// 	     << ",rect.height " << rect.height << endl;

// 	double angle=0.0;
// 	double hypotenous=0.0;
// 	cout << "measuring motion " << endl;
// 	pMotionDetector->measureFlowInRoi(rect,angle,hypotenous);
// 	cout << "motion: " << hypotenous << endl;
// 	if (hypotenous > 1.5) {
// 	  if (pObjView->bhistvalid == true)
// 	    pObjView->detect(resizeimg, m_bInitialCondition);
// 	  else
// 	    cerr << "ViewTracker::ERROR" << endl;
// 	}
//       }
//       else {
// 	if (pObjView->bhistvalid == true) 
// 	  pObjView->detect(resizeimg, m_bInitialCondition);
// 	else
// 	  cerr << "ViewTracker::ERROR" << endl;
//       }
//     }
// #else
//     if (pObjView->bhistvalid == true) 
//       pObjView->detect(resizeimg, m_bInitialCondition);
// #endif 
  

      
      if ((pObjView->ID()==TrackAux::HAND_ID) ||
	  (pObjView->GetDetectionState()  != DISAPPEARED))
	  pObjView->detect(resizeimg, m_bInitialCondition);
      
  }

  drawResults();
  
#ifndef MOTION_FILTER_ENABLED
  cvReleaseImage(&resizeimg);
#endif  

  if (m_bInitialCondition == true) {
      m_bInitialCondition = false;
  }

}


void ViewTracker::init_tmpImages(CvSize viewSize) 
{  
  if (this->bTmpimages_allocated==true || this->downsample==0)
    return;  
  this->imgsize = viewSize;
  this->bsize.width = this->imgsize.width/this->downsample; 
  this->bsize.height= this->imgsize.height/this->downsample; 
  this->grayimg = cvCreateImage(imgsize,8,1) ; 
  this->cannyimg = cvCreateImage(imgsize,8,1) ; 
  this->distimg = cvCreateImage(imgsize,IPL_DEPTH_32F,1) ;
  this->bTmpimages_allocated = true;
}


bool ViewTracker::is_in_ROI(int _x, int _y) {
    double x = (double) _x;
    double y = (double) _y;
    
    if (this->bRoiIsSet) {
	if((x > (Roi.m_center.x - (Roi.m_size.x/2.0))) &&
	   (x < (Roi.m_center.x + (Roi.m_size.x/2.0))) &&
	   (y > (Roi.m_center.y - (Roi.m_size.y/2.0))) &&
	   (y < (Roi.m_center.y + (Roi.m_size.y/2.0)))) {
	    return true;
	}
	else
	    return false;
    }
    else {
	if((x >= 0) &&
	   (x < imgsize.width) &&
	   (y >= 0) &&
	   (y < imgsize.height)) {
	    return true;
	}
	else 
	    return false;
    }
}

void ViewTracker::getGroundPosition(MobileObject* pMobileObject) {
  Vector2D ipnt;
  Vector3D gpnt;
  float x, y;

  map<int, ObjView*>::iterator iter = mpObjectViews.find(pMobileObject->id());
  if (iter != mpObjectViews.end()) {
    ObjView* pObjView = iter->second;

    pObjView->get2DGroundPoint(x,y);
    ipnt.x = (double)x;
    ipnt.y = (double)y;
    
    gpnt = pCamera->projectImagePointToGroundplane(ipnt);
    
    cout << "---ground pnt: " << gpnt.x() << "," << gpnt.y() << endl;
  }
}


void ViewTracker::updatePoseProperties(MobileObject* pMobileObject) 
{
  map<int, ObjView*>::iterator iter 
      = mpObjectViews.find(pMobileObject->id());
  if (iter != mpObjectViews.end()) 
  {
      ObjView* pObjView = iter->second;    

      // update 3D pose with confidence based on ROI
      CvBox2D roi;
      float confidence;      
      pObjView->getROI(roi, confidence);
      pCamera->projectRoiToWorld(roi, pMobileObject->m_bbox, 
				 pMobileObject->m_pose); 
      pMobileObject->putConfidence(confidence);
      
      pMobileObject->SetDetectionState(pObjView->GetDetectionState());

      //pMobileObject->lost(pObjView->is_lost());
      //pMobileObject->active(pObjView->is_reliable());


#ifdef LOG
      cout << "[ViewTracker::updatePose] Object "
	   << pMobileObject->id() << " pose: " 
	   << pMobileObject->m_bbox.m_centroid.x << ","
	   << pMobileObject->m_bbox.m_centroid.y << ","
	   << pMobileObject->m_bbox.m_centroid.z << ": "
	   << "conf " << confidence << endl;
#endif
      
  }
}


bool ViewTracker::getRoiBox2D(int objId, CvBox2D &roiBox2D)
{
    map<int, ObjView*>::iterator vit = mpObjectViews.find(objId);
    if (vit != mpObjectViews.end()) {
	ObjView* pObjView = vit->second;
	float confidence;      
	pObjView->getROI(roiBox2D, confidence);
	return true;
    } 
    else
	return false;
}


bool ViewTracker::matchDatabase(int objID, IplImage *histImage, IplImage *mask)
{
    map<int, ObjView*>::iterator oit = mpObjectViews.find(objID);
    if (oit != mpObjectViews.end())
    {
	CvHistogram * objHist = mpObjectViews[objID]->getHistogram();
		
	if (objHist != NULL)
	{
	    CvHistogram *imgHist = createHist();	

	    CvRect region = cvGetImageROI(histImage);
	    //scale(region, 2.0/3.0);
	    
	    updateHist(imgHist, histImage, region, mask);	  

	    if (cvCompareHist(imgHist,objHist,CV_COMP_BHATTACHARYYA) < 0.6)
		return true;
	    else 
		return false;
	}
	else
	    return false;
    }
    return false;
}


bool ViewTracker::check_overlap(CvRect rectRoi)
{
    std::map<int, ObjView*>::iterator vit;
    for (vit=mpObjectViews.begin(); vit!=mpObjectViews.end(); vit++)
    {
	ObjView *pObjView = vit->second;
	if (pObjView->GetDetectionState() != DISAPPEARED)
	    if (check_overlap(rectRoi, pObjView->ID()))
		return true;	
    }
    return false;
}

bool ViewTracker::check_overlap(CvRect rectRoi, int objID)
{
    CvBox2D roiBox2D;
    bool res = getRoiBox2D(objID, roiBox2D);
    if (res == false)
	return false; // objID doesn't exist
    else {
	CvPoint2D32f pt[4];
	cvBoxPoints(roiBox2D, pt);
	vector<Vector2D> points;
	for (unsigned i=0; i<4; i++)
	    points.push_back(Vector2D(pt[i].x, pt[i].y));
	
	Polygon2D poly;
	poly.init(points);
	vector<Vector2D> otherPoints;
	Geom::convert2Points(rectRoi, otherPoints);
	for (unsigned i=0; i<4; i++) 
	    if (poly.is_pt_in_zone(&(otherPoints[i])))
		return true;

	poly.init(otherPoints);
	for (unsigned i=0; i<4; i++) 
	    if (poly.is_pt_in_zone(&(points[i])))
		return true;	
	return false;
    }
}
